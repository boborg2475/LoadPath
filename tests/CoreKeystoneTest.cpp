#include "JsonReader.h"
#include "KeystoneOracle.h"
#include "ScenarioTestSupport.h"
#include "VisualSupport.h"

#include "loadpath/Scenario.h"

#include "doctest.h"

#include <algorithm>
#include <initializer_list>
#include <cmath>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;
using namespace loadpath::test::oracle;

namespace
{
constexpr double SoldierWidthM = 0.065;
constexpr double SoldierBottomM = 0.24;
constexpr double SoldierHeightM = 0.215;
constexpr double RingThicknessM = 0.215;
constexpr double RingDepthM = 0.1025;
constexpr double SpringingZM = 0.715;
constexpr double A2LimitBelow = 0.98;
constexpr double EngineTolerance = 1e-6;

const StrengthProfile& Bed() { return JointOrFail(ScenarioRows(), "clay_gp_mortar_bed_ec6").Profile; }

const StrengthProfile& Perpend() { return JointOrFail(ScenarioRows(), "clay_gp_mortar_perpend_ec6").Profile; }

double BrickDensity() { return ScenarioRows().FindMaterial("clay_brick")->DensityKgPerM3; }

struct FileGeometry
{
	bool bRead = false;
	double LeftOriginXM = 0.0, RightOriginXM = 0.0, RadiusM = 0.0;
	int Soldiers = 0, Voussoirs = 0;
	std::vector<std::string> Removed;
	std::string Text;
};

FileGeometry ReadGeometry(std::string_view Slug)
{
	FileGeometry G;
	const DiskFiles Files;
	const std::optional<std::string> Text = Files.Read(std::string(LOADPATH_SCENARIO_DIR) + "/" + std::string(Slug) + ".json");
	INFO("scenario file " << std::string(Slug));
	CHECK(Text.has_value());
	if (!Text)
	{
		return G;
	}
	G.Text = *Text;
	detail::JsonParser Parser(*Text);
	detail::JsonValue Root;
	CHECK(Parser.ParseDocument(Root));
	const detail::JsonValue* Build = detail::Find(Root, "build");
	const detail::JsonValue* Steps = detail::Find(Root, "steps");
	CHECK(Build != nullptr);
	if (Build == nullptr)
	{
		return G;
	}
	for (const detail::JsonValue& Item : Build->Items)
	{
		const detail::JsonValue* Wall = detail::Find(Item, "wall");
		if (Wall != nullptr)
		{
			(Wall->Text == "pier_left" ? G.LeftOriginXM : G.RightOriginXM) = detail::Find(Item, "origin")->Items[0].Number;
		}
		if (const detail::JsonValue* Repeat = detail::Find(Item, "repeat"))
		{
			G.Soldiers = static_cast<int>(detail::Find(*Repeat, "count")->Number);
		}
		if (detail::Find(Item, "arch") != nullptr)
		{
			G.RadiusM = detail::Find(Item, "radius")->Number;
			G.Voussoirs = static_cast<int>(detail::Find(Item, "voussoirs")->Number);
		}
	}
	for (const detail::JsonValue& Step : Steps != nullptr ? Steps->Items : std::vector<detail::JsonValue>{})
	{
		if (const detail::JsonValue* Remove = detail::Find(Step, "remove"))
		{
			for (const detail::JsonValue& Name : Remove->Items)
			{
				G.Removed.push_back(Name.Text);
			}
		}
	}
	G.bRead = true;
	return G;
}

SoldierCourse SoldierSpec(int Count)
{
	SoldierCourse S;
	S.Left = Pier2{"pier_left", -0.645, 9, 3};
	S.Right = Pier2{"pier_right", SoldierWidthM * Count, 9, 3};
	S.Count = Count;
	S.BottomM = SoldierBottomM;
	S.HeightM = SoldierHeightM;
	S.WidthM = SoldierWidthM;
	S.DensityKgPerM3 = BrickDensity();
	return S;
}

BrickRing RingSpec(double RadiusM, int Voussoirs)
{
	BrickRing A;
	A.Left = Pier2{"pier_left", -RadiusM - 0.645, 11, 4};
	A.Right = Pier2{"pier_right", RadiusM - 0.215, 11, 4};
	A.SpringingZM = SpringingZM;
	A.RadiusM = RadiusM;
	A.ThicknessM = RingThicknessM;
	A.DepthM = RingDepthM;
	A.Voussoirs = Voussoirs;
	A.DensityKgPerM3 = BrickDensity();
	return A;
}

Model2 OracleOf(const FileGeometry& G)
{
	Model2 M = G.Voussoirs > 0 ? BuildBrickRing(RingSpec(G.RadiusM, G.Voussoirs), Bed(), Perpend()) : BuildSoldierCourse(SoldierSpec(G.Soldiers), Bed(), Perpend());
	for (const std::string& Name : G.Removed)
	{
		RemoveBody2(M, Name);
	}
	return M;
}

Reading2 OracleWorst(Model2 M)
{
	const Solve2 S = Solve(M);
	CHECK(S.End == End2::Solved);
	const Reading2* W = Worst(S);
	CHECK(W != nullptr);
	return W != nullptr ? *W : Reading2{};
}

bool StartsWithAny(const std::string& Joint, std::initializer_list<std::string> Prefixes)
{
	return std::any_of(Prefixes.begin(), Prefixes.end(), [&](const std::string& P) { return Joint.starts_with(P); });
}

void CheckSoldierFile(const FileGeometry& G)
{
	INFO("left origin " << G.LeftOriginXM << " right origin " << G.RightOriginXM << " soldiers " << G.Soldiers);
	CHECK(G.LeftOriginXM == -0.645);
	CHECK(std::abs(G.RightOriginXM - SoldierWidthM * G.Soldiers) <= 1e-12);
}

const char* const SoldierFiles[] = {"core-a2-soldier-course-17-at-its-limit", "core-a2-soldier-course-17-minus-keystone", "core-a2-soldier-course-15-stands", "core-a2-soldier-course-19-mortar-fails"};

bool AllRead(std::initializer_list<const FileGeometry*> Files)
{
	return std::all_of(Files.begin(), Files.end(), [](const FileGeometry* G) { return G->bRead; });
}

void CheckEngineAgrees(const char* Slug)
{
	INFO("scenario " << std::string(Slug));
	const FileGeometry G = ReadGeometry(Slug);
	CHECK(G.bRead);
	if (!G.bRead)
	{
		return;
	}
	ScenarioLoadResult Loaded = LoadWithRows(G.Text);
	CHECK(Loaded.Ok());
	if (!Loaded.Ok())
	{
		return;
	}
	Scenario& S = Loaded.Value;
	std::erase_if(S.Steps, [](const ScenarioStep& Step) { return Step.Kind != ScenarioStepKind::Remove; });
	const ScenarioOutcome O = RunScenario(S);
	CHECK_FALSE(O.BeforeReadings.empty());
	double Engine = 0.0;
	std::string EngineJoint;
	for (const ScenarioJointReading& R : O.BeforeReadings)
	{
		if (R.Reading.Utilisation > Engine)
		{
			Engine = R.Reading.Utilisation;
			EngineJoint = R.Joint;
		}
	}
	const Reading2 Oracle = OracleWorst(OracleOf(G));
	const double Difference = std::abs(Engine - Oracle.Utilisation) / Oracle.Utilisation;
	MESSAGE(std::string(Slug) << ": engine " << Engine << " at " << EngineJoint << ", oracle " << Oracle.Utilisation << " at " << Oracle.Joint << ", difference " << Difference);
	CHECK(Difference <= EngineTolerance);
}
}

TEST_CASE("Core.Keystone.TheSoldierCourseFilesSitOnTheOracleEdges")
{
	const FileGeometry Limit = ReadGeometry(SoldierFiles[0]);
	const FileGeometry Minus = ReadGeometry(SoldierFiles[1]);
	const FileGeometry Inside = ReadGeometry(SoldierFiles[2]);
	const FileGeometry Past = ReadGeometry(SoldierFiles[3]);
	CHECK(AllRead({&Limit, &Minus, &Inside, &Past}));
	if (!AllRead({&Limit, &Minus, &Inside, &Past}))
	{
		return;
	}
	for (const FileGeometry* G : {&Limit, &Minus, &Inside, &Past})
	{
		CheckSoldierFile(*G);
	}
	const int N = Limit.Soldiers;
	const std::string Last = "s." + std::to_string(N - 1);
	const Reading2 AtLimit = OracleWorst(OracleOf(Limit));
	const Reading2 Below = OracleWorst(OracleOf(Inside));
	const Reading2 Above = OracleWorst(OracleOf(Past));
	MESSAGE("A2 oracle: n " << N - 2 << " " << Below.Utilisation << ", n " << N << " " << AtLimit.Utilisation << " at " << AtLimit.Joint << ", n " << N + 2 << " " << Above.Utilisation);
	CHECK(N % 2 == 1);
	CHECK(Minus.Soldiers == N);
	CHECK(Minus.Removed == std::vector<std::string>{"s." + std::to_string(N / 2)});
	CHECK(Inside.Soldiers == N - 2);
	CHECK(Past.Soldiers == N + 2);
	CHECK(AtLimit.Utilisation < A2LimitBelow);
	CHECK(AtLimit.Axis == GoverningAxis::Tension);
	CHECK(StartsWithAny(AtLimit.Joint, {"s.0/pier_left.c6.", Last + "/pier_right.c6."}));
	CHECK(Below.Utilisation < 1.0);
	CHECK(Above.Utilisation >= A2LimitBelow);
	CHECK(Above.Utilisation > 1.0);
}

TEST_CASE("Core.Keystone.TheEngineAgreesWithTheOracleOnTheSoldierCourses")
{
	for (const char* Slug : SoldierFiles)
	{
		CheckEngineAgrees(Slug);
	}
}
