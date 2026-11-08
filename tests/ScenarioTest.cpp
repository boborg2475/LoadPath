#include "JsonReader.h"
#include "ScenarioTestSupport.h"

#include "loadpath/Cascade.h"
#include "loadpath/Catalogue.h"
#include "loadpath/Scenario.h"
#include "loadpath/Units.h"
#include "loadpath/Viewer.h"
#include "loadpath/geometry/BoxContacts.h"
#include "loadpath/geometry/RunningBondWall.h"

#include "doctest.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double BrickXM = 0.215;
constexpr double BrickYM = 0.1025;
constexpr double BrickZM = 0.065;
constexpr double BrickDensity = 2000.0;
constexpr double TimberDensity = 420.0;
constexpr double PaPerMPa = 1e6;

std::string Lines(const std::vector<std::string>& Rows)
{
	std::string Out;
	for (const std::string& Row : Rows)
	{
		Out += Row + "\n";
	}
	return Out;
}

std::string Number(double Value)
{
	char Text[64];
	std::snprintf(Text, sizeof Text, "%.17g", Value);
	return Text;
}

std::string Format(const Vec3& V) { return Number(V.X) + "," + Number(V.Y) + "," + Number(V.Z); }

std::string WallJson(int Courses, int Bricks, const std::string& Omit, const std::string& Extra = "", const std::string& Joints = "")
{
	return "{\n"
		   "  \"title\": \"wall\",\n"
		   "  \"build\": [\n"
		   "    { \"name\": \"ground\", \"size\": [3, 0.6, 0.2], \"centre\": [0.8, 0, -0.1], \"grounded\": true },\n"
		   "    { \"wall\": \"wall\", \"courses\": " +
		std::to_string(Courses) + ", \"bricks\": " + std::to_string(Bricks) +
		", \"on\": \"ground\", \"material\": \"clay_brick\", \"bed\": \"clay_gp_mortar_bed_ec6\", \"perpend\": \"clay_gp_mortar_perpend_ec6\"" +
		(Omit.empty() ? std::string() : ", \"omit\": " + Omit) + " }" + Extra +
		"\n"
		"  ],\n"
		"  \"joints\": [" +
		Joints +
		"],\n"
		"  \"steps\": [\"cascade\"]\n"
		"}\n";
}

struct DirectWall
{
	Structure Model;
	geometry::Wall Wall;
	std::vector<std::string> Names;
};

DirectWall BuildDirectWall(int Courses, int Bricks)
{
	DirectWall D;
	const Result<BodyId> Ground = D.Model.AddBody(BodyDesc{0.0, Vec3{0.8, 0.0, -0.1}, true});
	REQUIRE(Ground.Ok());
	geometry::WallSpec Spec;
	Spec.Courses = Courses;
	Spec.BricksPerCourse = Bricks;
	Spec.DensityKgPerM3 = BrickDensity;
	Spec.Ground = Ground.Value;
	Spec.Bed = D.Model.AddProfile(JointOrFail(ScenarioRows(), "clay_gp_mortar_bed_ec6").Profile).Value;
	Spec.Perpend = D.Model.AddProfile(JointOrFail(ScenarioRows(), "clay_gp_mortar_perpend_ec6").Profile).Value;
	D.Wall = geometry::BuildRunningBondWall(D.Model, Spec);
	REQUIRE(D.Wall.bComplete);
	D.Names.assign(D.Model.LiveBodyCount(), std::string());
	D.Names[Ground.Value.Index] = "ground";
	for (std::size_t C = 0; C < D.Wall.Courses.size(); ++C)
	{
		for (std::size_t K = 0; K < D.Wall.Courses[C].size(); ++K)
		{
			D.Names[D.Wall.Courses[C][K].Index] = "wall.c" + std::to_string(C) + ".b" + std::to_string(K);
		}
	}
	return D;
}

std::string RecordKey(const std::string& A, const std::string& B, const InterfaceRecord& R)
{
	const bool bSwap = B < A;
	const Vec3 N = bSwap ? Vec3{-R.NormalAToB.X, -R.NormalAToB.Y, -R.NormalAToB.Z} : R.NormalAToB;
	return (bSwap ? B + "/" + A : A + "/" + B) + " c " + Format(R.CentreM) + " n " + Format(N) + " h " + Number(R.HalfExtentUM) + "," + Number(R.HalfExtentVM);
}

std::vector<std::string> ScenarioRecordKeys(const Scenario& S)
{
	std::vector<std::string> Keys;
	for (const InterfaceId Id : S.Model.LiveInterfaces())
	{
		const InterfaceRecord& R = S.Model.Interface(Id);
		Keys.push_back(RecordKey(NameOf(S, R.BodyA), NameOf(S, R.BodyB), R));
	}
	return SortedNames(std::move(Keys));
}

std::vector<std::string> DirectRecordKeys(const DirectWall& D)
{
	std::vector<std::string> Keys;
	for (const InterfaceId Id : D.Model.LiveInterfaces())
	{
		const InterfaceRecord& R = D.Model.Interface(Id);
		Keys.push_back(RecordKey(D.Names[R.BodyA.Index], D.Names[R.BodyB.Index], R));
	}
	return SortedNames(std::move(Keys));
}

bool IsBed(const InterfaceRecord& R) { return std::abs(R.NormalAToB.Z) > 0.5; }

std::string ProfileName(const Scenario& S, const InterfaceRecord& R) { return R.Profile.Index < S.ProfileNames.size() ? S.ProfileNames[R.Profile.Index] : std::string("?"); }

const InterfaceRecord* FindJoint(const Scenario& S, const std::string& Pair)
{
	for (const InterfaceId Id : S.Model.LiveInterfaces())
	{
		if (PairName(S, S.Model.Interface(Id)) == Pair)
		{
			return &S.Model.Interface(Id);
		}
	}
	return nullptr;
}

const std::string ContactsJson = R"({
  "title": "contacts",
  "build": [
    { "name": "ground", "size": [1.4, 0.8, 0.2], "centre": [0.4, 0.1, -0.1], "grounded": true },
    { "name": "a", "size": [0.215, 0.1025, 0.065], "centre": [0.1075, 0, 0.0325], "material": "clay_brick" },
    { "name": "b", "size": [0.215, 0.1025, 0.065], "centre": [0.3225, 0, 0.0325], "material": "clay_brick" },
    { "name": "c", "size": [0.215, 0.1025, 0.065], "centre": [0.215, 0, 0.0975], "material": "clay_brick" },
    { "name": "d", "size": [0.215, 0.1025, 0.065], "centre": [0.5385, 0, 0.0325], "material": "clay_brick" },
    { "name": "e", "size": [0.215, 0.1025, 0.065], "centre": [0.7535, 0.1025, 0.0325], "material": "clay_brick" }
  ],
  "joints": [
    { "materials": ["clay_brick", "clay_brick"], "plane": "side", "row": "clay_gp_mortar_perpend_ec6" },
    { "materials": ["clay_brick", "clay_brick"], "plane": "bed", "row": "clay_gp_mortar_bed_ec6" },
    { "materials": ["clay_brick", "ground"], "plane": "bed", "row": "clay_gp_mortar_bed_ec6" }
  ]
}
)";

std::string BayJson(const std::string& Rules)
{
	return R"({
  "title": "bay",
  "build": [
    { "name": "ground", "size": [3.8, 2, 0.2], "centre": [1.1, 0, -0.1], "grounded": true },
    { "name": "post", "size": [0.2, 0.2, 2.0], "centre": [0, 0, 1.0], "material": "c24",
      "repeat": { "count": 2, "step": [2.2, 0, 0] } },
    { "name": "beam", "size": [2.199, 0.2, 0.3], "centre": [1.1, 0, 2.15], "material": "c24",
      "repeat": { "count": 1, "step": [2.2, 0, 0] } }
  ],
  "joints": [
)" + Rules + R"(,
    { "materials": ["c24", "ground"], "row": "c24_bearing_perp_to_grain" }
  ]
}
)";
}

const std::string BearingRule = R"(    { "materials": ["c24", "c24"], "row": "c24_bearing_perp_to_grain" })";
const std::string ScrewRule = R"(    { "between": ["beam.0", "post.1"], "row": "ec5_screw_8x60_steel_to_glulam", "count": 3 })";
const std::string MortarRule = R"(    { "materials": ["c24", "c24"], "row": "clay_gp_mortar_bed_ec6" })";

const std::string CantileverJson = R"({
  "title": "cantilever",
  "materials": [
    { "name": "light_block", "density_kg_per_m3": 500 }
  ],
  "build": [
    { "name": "ground", "size": [2.4, 1, 0.2], "centre": [0.4, 0, -0.1], "grounded": true },
    { "name": "post", "size": [0.2, 0.2, 1.0], "centre": [0.1, 0, 0.5], "material": "c24" },
    { "name": "beam", "size": [1.6, 0.2, 0.1], "centre": [0.4, 0, 1.05], "material": "c24" },
    { "name": "counterweight", "size": [0.2, 0.2, 0.2], "centre": [-0.3, 0, 1.2], "material": "light_block" }
  ],
  "joints": [
    { "materials": ["c24", "ground"], "row": "c24_bearing_perp_to_grain" },
    { "materials": ["c24", "c24"], "row": "c24_bearing_perp_to_grain" },
    { "materials": ["light_block", "c24"], "row": "c24_bearing_perp_to_grain" }
  ]
}
)";

class RecordingCascade final : public ICascade
{
public:
	struct Call
	{
		std::vector<std::string> Live;
		Vec3 Gravity;
		std::vector<BodyLoad> Loads;
	};

	explicit RecordingCascade(const Scenario& S)
		: Names(S.BodyNames)
		, Inner(MakeCascade(StandardAuthority()))
	{
	}

	CascadeVerdict Run(Structure& Model, std::span<const BodyLoad> Loads, const CascadeOptions& Options, const AuthorityResult* Previous) const override
	{
		Call C;
		for (const BodyId Id : Model.LiveBodies())
		{
			C.Live.push_back(Id.Index < Names.size() ? Names[Id.Index] : std::string("?"));
		}
		C.Gravity = Options.Authority.Standing.Gravity;
		C.Loads.assign(Loads.begin(), Loads.end());
		Calls.push_back(std::move(C));
		return Inner->Run(Model, Loads, Options, Previous);
	}

	std::vector<std::string> Names;
	mutable std::vector<Call> Calls;

private:
	std::unique_ptr<ICascade> Inner;
};

bool Has(const std::vector<std::string>& Names, const std::string& Name) { return std::find(Names.begin(), Names.end(), Name) != Names.end(); }

std::vector<std::vector<std::string>> ClustersBySlot(const CascadeVerdict& V)
{
	std::vector<std::vector<std::string>> Out;
	for (const ReleasedCluster& C : V.Clusters)
	{
		std::vector<std::string> Set;
		for (const BodyId Id : C.Bodies)
		{
			Set.push_back(std::to_string(Id.Index));
		}
		Set.insert(Set.begin(), "p" + std::to_string(C.Pass));
		Out.push_back(std::move(Set));
	}
	return Out;
}

std::vector<std::string> FailuresBySlot(const Structure& Model, const CascadeVerdict& V)
{
	std::vector<std::string> Out;
	for (const FailedInterface& F : V.Failures)
	{
		const InterfaceRecord& R = Model.Interface(F.Id);
		const std::uint32_t Lo = std::min(R.BodyA.Index, R.BodyB.Index);
		const std::uint32_t Hi = std::max(R.BodyA.Index, R.BodyB.Index);
		char Ratio[32];
		std::snprintf(Ratio, sizeof Ratio, "%.9g", F.Ratio);
		Out.push_back("p" + std::to_string(F.Pass) + " " + std::to_string(Lo) + "/" + std::to_string(Hi) + " axis " + std::to_string(static_cast<int>(F.Axis)) +
			(F.bSevered ? " severed" : " cracked") + " ratio " + Ratio);
	}
	return SortedNames(std::move(Out));
}

void CheckSameVerdict(const Structure& GotModel, const CascadeVerdict& Got, const Structure& WantModel, const CascadeVerdict& Want)
{
	CHECK(static_cast<int>(Got.State) == static_cast<int>(Want.State));
	CHECK(Got.Passes == Want.Passes);
	CHECK(FailuresBySlot(GotModel, Got) == FailuresBySlot(WantModel, Want));
	CHECK(ClustersBySlot(Got) == ClustersBySlot(Want));
}

struct DirectBox
{
	std::string Material;
	geometry::Box Shape;
	double DensityKgPerM3 = 0.0;
	bool bGrounded = false;
};

Structure BuildDirectBoxes(const std::vector<DirectBox>& Boxes, const std::function<StrengthProfile(const std::string&, const std::string&)>& Pick, std::vector<BodyId>& Ids)
{
	Structure Model;
	for (const DirectBox& B : Boxes)
	{
		const Vec3 H = B.Shape.HalfExtentsM;
		const double MassKg = B.bGrounded ? 0.0 : B.DensityKgPerM3 * 8.0 * H.X * H.Y * H.Z;
		const Result<BodyId> Added = Model.AddBody(BodyDesc{MassKg, B.Shape.CentreM, B.bGrounded});
		REQUIRE(Added.Ok());
		Ids.push_back(Added.Value);
	}
	for (std::size_t I = 0; I < Boxes.size(); ++I)
	{
		for (std::size_t J = I + 1; J < Boxes.size(); ++J)
		{
			std::optional<InterfaceDesc> Desc = geometry::FaceContact(Ids[I], Boxes[I].Shape, Ids[J], Boxes[J].Shape, ProfileId{});
			if (!Desc)
			{
				continue;
			}
			const Result<ProfileId> Profile = Model.AddProfile(Pick(Boxes[I].Material, Boxes[J].Material));
			REQUIRE(Profile.Ok());
			Desc->Profile = Profile.Value;
			REQUIRE(Model.AddInterface(*Desc).Ok());
		}
	}
	return Model;
}

ScenarioOutcome Synthetic(std::vector<std::string> Names, std::vector<std::vector<std::string>> Clusters, std::string State = "Stands")
{
	ScenarioOutcome O;
	O.Names = std::move(Names);
	O.Clusters = std::move(Clusters);
	O.State = std::move(State);
	return O;
}

std::string Joined(const std::vector<std::string>& Rows)
{
	std::string Out;
	for (const std::string& Row : Rows)
	{
		Out += "\n  " + Row;
	}
	return Out;
}

struct Parsed
{
	detail::JsonValue Root;
	bool bOk = false;
};

Parsed Parse(const std::string& Json)
{
	Parsed P;
	detail::JsonParser Parser(Json);
	P.bOk = Parser.ParseDocument(P.Root);
	INFO("viewer json parse error line " << Parser.Error.Line << ": " << Parser.Error.Message);
	CHECK(P.bOk);
	return P;
}

const detail::JsonValue& Field(const detail::JsonValue& Object, std::string_view Key)
{
	static const detail::JsonValue Missing{};
	const detail::JsonValue* Value = detail::Find(Object, Key);
	INFO("viewer json key " << std::string(Key));
	CHECK(Value != nullptr);
	return Value != nullptr ? *Value : Missing;
}

bool HasKey(const detail::JsonValue& Object, std::string_view Key) { return detail::Find(Object, Key) != nullptr; }

std::vector<double> Numbers(const detail::JsonValue& Array)
{
	std::vector<double> Out;
	for (const detail::JsonValue& Item : Array.Items)
	{
		Out.push_back(Item.Number);
	}
	return Out;
}

std::vector<std::vector<double>> ClusterBodies(const detail::JsonValue& Clusters)
{
	std::vector<std::vector<double>> Out;
	for (const detail::JsonValue& C : Clusters.Items)
	{
		std::vector<double> Bodies = Numbers(Field(C, "bodies"));
		std::sort(Bodies.begin(), Bodies.end());
		Out.push_back(std::move(Bodies));
	}
	std::sort(Out.begin(), Out.end());
	return Out;
}

const detail::JsonValue* BodyNamed(const detail::JsonValue& Root, std::string_view Name)
{
	for (const detail::JsonValue& Body : Field(Root, "bodies").Items)
	{
		const detail::JsonValue* N = detail::Find(Body, "name");
		if (N != nullptr && N->Text == Name)
		{
			return &Body;
		}
	}
	return nullptr;
}

void CheckNear(const Vec3& Got, const Vec3& Want, double Tolerance)
{
	INFO("got " << Format(Got) << " want " << Format(Want));
	CHECK(std::abs(Got.X - Want.X) <= Tolerance);
	CHECK(std::abs(Got.Y - Want.Y) <= Tolerance);
	CHECK(std::abs(Got.Z - Want.Z) <= Tolerance);
}

bool Mentions(std::string_view Text, std::string_view Needle) { return Text.find(Needle) != std::string_view::npos; }
}

TEST_CASE("Scenario.Load.TheColonnadeExampleParses")
{
	const Scenario S = LoadScenarioOk(ColonnadeJson);
	const std::vector<std::string> Want = {"ground", "post.0", "post.1", "post.2", "post.3", "post.4", "beam.0", "beam.1", "beam.2", "beam.3"};
	CHECK(S.BodyNames == Want);
	CHECK(S.Model.LiveBodyCount() == 10);
	const BodyId Post3 = BodyOrFail(S, "post.3");
	CheckNear(S.Model.Body(Post3).CentroidM, Vec3{6.6, 0.0, 1.0}, 1e-12);
	const BodyId Ground = BodyOrFail(S, "ground");
	CHECK(S.Model.Body(Ground).bGrounded);
	CHECK(S.Model.Body(Ground).MassKg == 0.0);
	CHECK(S.Model.LiveInterfaceCount() == 13);
	const StrengthProfile Bearing = JointOrFail(ScenarioRows(), "c24_bearing_perp_to_grain").Profile;
	for (const InterfaceId Id : S.Model.LiveInterfaces())
	{
		const InterfaceRecord& R = S.Model.Interface(Id);
		INFO("joint " << PairName(S, R));
		CHECK(SameProfile(S.Model.Profile(R.Profile), Bearing));
		CHECK_FALSE((NameOf(S, R.BodyA).starts_with("beam.") && NameOf(S, R.BodyB).starts_with("beam.")));
	}
}

TEST_CASE("Scenario.Load.ErrorsNameTheLineAndTheProblem")
{
	const std::vector<std::string> Base = {
		"{",
		"  \"title\": \"t\",",
		"  \"build\": [",
		"    { \"name\": \"ground\", \"size\": [2, 2, 0.2], \"centre\": [0, 0, -0.1], \"grounded\": true },",
		"    { \"name\": \"block\", \"size\": [0.2, 0.2, 0.2], \"centre\": [0, 0, 0.1], \"material\": \"c24\" }",
		"  ],",
		"  \"joints\": [",
		"    { \"materials\": [\"c24\", \"ground\"], \"row\": \"c24_bearing_perp_to_grain\" }",
		"  ],",
		"  \"steps\": [\"cascade\"],",
		"  \"expect\": {}",
		"}",
	};
	const std::string Block = "    { \"name\": \"block\", \"size\": [0.2, 0.2, 0.2], \"centre\": [0, 0, 0.1], \"material\": \"c24\"";
	const std::string Wall1x1 = "\"courses\": 1, \"bricks\": 1, \"material\": \"c24\", \"bed\": \"c24_bearing_perp_to_grain\", \"perpend\": \"c24_bearing_perp_to_grain\"";
	struct Fault
	{
		const char* Name;
		int Line;
		std::string Text;
		int Line2;
		std::string Text2;
		int ErrorLine;
		const char* Phrase;
	};
	const Fault Faults[] = {
		{"unknown key", 5, Block + ", \"colour\": \"red\" }", 0, "", 5, "unknown key 'colour'"},
		{"missing size", 5, "    { \"name\": \"block\", \"centre\": [0, 0, 0.1], \"material\": \"c24\" }", 0, "", 5, "missing 'size'"},
		{"zero size", 5, "    { \"name\": \"block\", \"size\": [0, 0.2, 0.2], \"centre\": [0, 0, 0.1], \"material\": \"c24\" }", 0, "", 5, "size must be finite and positive"},
		{"negative size", 5, "    { \"name\": \"block\", \"size\": [0.2, -0.2, 0.2], \"centre\": [0, 0, 0.1], \"material\": \"c24\" }", 0, "", 5, "size must be finite and positive"},
		{"overflowing size", 5, "    { \"name\": \"block\", \"size\": [0.2, 0.2, 1e999], \"centre\": [0, 0, 0.1], \"material\": \"c24\" }", 0, "", 5, "out of range"},
		{"unknown material", 5, "    { \"name\": \"block\", \"size\": [0.2, 0.2, 0.2], \"centre\": [0, 0, 0.1], \"material\": \"oak\" }", 0, "", 5, "unknown material 'oak'"},
		{"both centre and min", 5, Block + ", \"min\": [-0.1, -0.1, 0] }", 0, "", 5, "exactly one of 'centre' and 'min'"},
		{"neither centre nor min", 5, "    { \"name\": \"block\", \"size\": [0.2, 0.2, 0.2], \"material\": \"c24\" }", 0, "", 5, "exactly one of 'centre' and 'min'"},
		{"unknown row", 8, "    { \"materials\": [\"c24\", \"ground\"], \"row\": \"no_such_row\" }", 0, "", 8, "unknown row 'no_such_row'"},
		{"fastener without count", 8, "    { \"materials\": [\"c24\", \"ground\"], \"row\": \"ec5_screw_8x60_steel_to_glulam\" }", 0, "", 8, "needs a count"},
		{"joint row with count", 8, "    { \"materials\": [\"c24\", \"ground\"], \"row\": \"c24_bearing_perp_to_grain\", \"count\": 2 }", 0, "", 8, "takes no count"},
		{"count 0", 8, "    { \"materials\": [\"c24\", \"ground\"], \"row\": \"ec5_screw_8x60_steel_to_glulam\", \"count\": 0 }", 0, "", 8, "count must be a whole number of at least 1"},
		{"count 2.5", 8, "    { \"materials\": [\"c24\", \"ground\"], \"row\": \"ec5_screw_8x60_steel_to_glulam\", \"count\": 2.5 }", 0, "", 8, "count must be a whole number of at least 1"},
		{"between and materials", 8, "    { \"between\": [\"block\", \"ground\"], \"materials\": [\"c24\", \"ground\"], \"row\": \"c24_bearing_perp_to_grain\" }", 0, "", 8,
			"exactly one of 'between' and 'materials'"},
		{"repeat count 0", 5, Block + ", \"repeat\": { \"count\": 0, \"step\": [0.5, 0, 0] } }", 0, "", 5, "count must be a whole number of at least 1"},
		{"repeat count -1", 5, Block + ", \"repeat\": { \"count\": -1, \"step\": [0.5, 0, 0] } }", 0, "", 5, "count must be a whole number of at least 1"},
		{"repeat count 2.5", 5, Block + ", \"repeat\": { \"count\": 2.5, \"step\": [0.5, 0, 0] } }", 0, "", 5, "count must be a whole number of at least 1"},
		{"non-finite repeat step", 5, Block + ", \"repeat\": { \"count\": 2, \"step\": [1e999, 0, 0] } }", 0, "", 5, "out of range"},
		{"name with a dot", 5, "    { \"name\": \"block.a\", \"size\": [0.2, 0.2, 0.2], \"centre\": [0, 0, 0.1], \"material\": \"c24\" }", 0, "", 5, "contains a dot"},
		{"duplicate name", 5, "    { \"name\": \"ground\", \"size\": [0.2, 0.2, 0.2], \"centre\": [0, 0, 0.1], \"material\": \"c24\" }", 0, "", 5, "duplicate name"},
		{"duplicate repeat name", 5,
			"    { \"name\": \"post\", \"size\": [0.2, 0.2, 0.2], \"centre\": [0, 0, 0.1], \"material\": \"c24\", \"repeat\": { \"count\": 2, \"step\": [0.5, 0, 0] } }, "
			"{ \"name\": \"post\", \"size\": [0.2, 0.2, 0.2], \"centre\": [0, 0.5, 0.1], \"material\": \"c24\", \"repeat\": { \"count\": 2, \"step\": [0.5, 0, 0] } }",
			0, "", 5, "duplicate name"},
		{"wall on a box that is not grounded", 5, Block + " },", 6, "    { \"wall\": \"w\", \"origin\": [-0.1, 0, 0.2], \"on\": \"block\", " + Wall1x1 + " }\n  ],", 6, "not grounded"},
		{"wall not at the top of its ground", 5, "    { \"wall\": \"w\", \"origin\": [0.3, 0, 0.1], \"on\": \"ground\", " + Wall1x1 + " }", 0, "", 5, "base level"},
		{"omit matching nothing", 5, Block + " }, { \"wall\": \"w\", \"origin\": [0.5, 0, 0], \"on\": \"ground\", " + Wall1x1 + ", \"omit\": [\"w.c9.*\"] }", 0, "", 5,
			"matches no brick"},
		{"step with two keys", 10, "  \"steps\": [{ \"remove\": [\"block\"], \"lateral_g\": [0, 0.1] }],", 0, "", 10, "exactly one key"},
		{"unknown step", 10, "  \"steps\": [\"explode\"],", 0, "", 10, "unknown step 'explode'"},
		{"remove naming no body", 10, "  \"steps\": [{ \"remove\": [\"nothing\"] }, \"cascade\"],", 0, "", 10, "no body matches 'nothing'"},
		{"expect pattern matching nothing", 11, "  \"expect\": { \"released\": [\"ghost.*\"] }", 0, "", 11, "no body matches 'ghost.*'"},
		{"expect naming a removed body", 10, "  \"steps\": [{ \"remove\": [\"block\"] }, \"cascade\"],", 11, "  \"expect\": { \"released\": [\"block\"] }", 11, "is removed"},
		{"file material shadowing the catalogue", 2, "  \"title\": \"t\", \"materials\": [{ \"name\": \"c24\", \"density_kg_per_m3\": 500 }],", 0, "", 2,
			"already in the catalogue"},
		{"file profile shadowing the catalogue", 2,
			"  \"title\": \"t\", \"profiles\": [{ \"name\": \"c24_bearing_perp_to_grain\", \"from\": \"c24_bearing_perp_to_grain\" }],", 0, "", 2, "already in the catalogue"},
		{"profile from an unknown row", 2, "  \"title\": \"t\", \"profiles\": [{ \"name\": \"p\", \"from\": \"no_such_row\" }],", 0, "", 2, "unknown row 'no_such_row'"},
		{"empty deferred", 2, "  \"title\": \"t\", \"deferred\": \"\",", 0, "", 2, "deferred must not be empty"},
		{"unknown tier", 2, "  \"title\": \"t\", \"tier\": \"fast\",", 0, "", 2, "unknown tier 'fast'"},
		{"wall key along", 5, "    { \"wall\": \"w\", \"origin\": [0.5, 0, 0], \"on\": \"ground\", " + Wall1x1 + ", \"along\": \"y\" }", 0, "", 5, "unknown key 'along'"},
		{"wall key brick", 5, "    { \"wall\": \"w\", \"origin\": [0.5, 0, 0], \"on\": \"ground\", " + Wall1x1 + ", \"brick\": [0.215, 0.1025, 0.065] }", 0, "", 5, "unknown key 'brick'"},
		{"wall key bond", 5, "    { \"wall\": \"w\", \"origin\": [0.5, 0, 0], \"on\": \"ground\", " + Wall1x1 + ", \"bond\": \"stack\" }", 0, "", 5, "unknown key 'bond'"},
		{"profile key compressive_mpa", 2, "  \"title\": \"t\", \"profiles\": [{ \"name\": \"p\", \"from\": \"c24_bearing_perp_to_grain\", \"compressive_mpa\": 3 }],", 0, "", 2,
			"unknown key 'compressive_mpa'"},
		{"profile key normal_stiffness_gpa_per_m", 2,
			"  \"title\": \"t\", \"profiles\": [{ \"name\": \"p\", \"from\": \"c24_bearing_perp_to_grain\", \"normal_stiffness_gpa_per_m\": 3 }],", 0, "", 2,
			"unknown key 'normal_stiffness_gpa_per_m'"},
		{"profile key tangential_stiffness_gpa_per_m", 2,
			"  \"title\": \"t\", \"profiles\": [{ \"name\": \"p\", \"from\": \"c24_bearing_perp_to_grain\", \"tangential_stiffness_gpa_per_m\": 3 }],", 0, "", 2,
			"unknown key 'tangential_stiffness_gpa_per_m'"},
		{"load step key point", 10, "  \"steps\": [{ \"load\": { \"body\": \"block\", \"force_n\": [0, 0, -10], \"point\": [0, 0, 0.1] } }, \"cascade\"],", 0, "", 10, "unknown key 'point'"},
		{"file material named ground", 2, "  \"title\": \"t\", \"materials\": [{ \"name\": \"ground\", \"density_kg_per_m3\": 500 }],", 0, "", 2, "already in the catalogue"},
		{"duplicate file material", 2, "  \"title\": \"t\", \"materials\": [{ \"name\": \"m1\", \"density_kg_per_m3\": 500 },", 3,
			"    { \"name\": \"m1\", \"density_kg_per_m3\": 600 }], \"build\": [", 3, "duplicate material 'm1'"},
		{"duplicate file profile", 2, "  \"title\": \"t\", \"profiles\": [{ \"name\": \"p\", \"from\": \"c24_bearing_perp_to_grain\" },", 3,
			"    { \"name\": \"p\", \"from\": \"c24_bearing_perp_to_grain\" }], \"build\": [", 3, "duplicate profile 'p'"},
		{"negative override", 2, "  \"title\": \"t\", \"profiles\": [{ \"name\": \"p\", \"from\": \"c24_bearing_perp_to_grain\",", 3,
			"    \"tensile_mpa\": -1 }], \"build\": [", 3, "tensile_mpa must be a finite number, not negative"},
		{"ground material on an ungrounded box", 5, "    { \"name\": \"block\", \"size\": [0.2, 0.2, 0.2], \"centre\": [0, 0, 0.1], \"material\": \"ground\" }", 0, "", 5,
			"material 'ground' is for grounded boxes only"},
		{"wall missing courses", 5, "    { \"wall\": \"w\", \"origin\": [0.5, 0, 0], \"on\": \"ground\", \"bricks\": 1, \"material\": \"c24\", \"bed\": \"c24_bearing_perp_to_grain\", \"perpend\": \"c24_bearing_perp_to_grain\" }",
			0, "", 5, "missing 'courses'"},
		{"wall courses 0", 5, "    { \"wall\": \"w\", \"origin\": [0.5, 0, 0], \"on\": \"ground\", \"courses\": 0, \"bricks\": 1, \"material\": \"c24\", \"bed\": \"c24_bearing_perp_to_grain\", \"perpend\": \"c24_bearing_perp_to_grain\" }",
			0, "", 5, "courses must be a whole number of at least 1"},
		{"wall missing on", 5, "    { \"wall\": \"w\", \"origin\": [0.5, 0, 0], \"courses\": 1, \"bricks\": 1, \"material\": \"c24\", \"bed\": \"c24_bearing_perp_to_grain\", \"perpend\": \"c24_bearing_perp_to_grain\" }",
			0, "", 5, "missing 'on'"},
		{"wall origin of two numbers", 5, "    { \"wall\": \"w\", \"origin\": [0.5, 0], \"on\": \"ground\", " + Wall1x1 + " }", 0, "", 5, "'origin' must hold 3 numbers"},
		{"wall name with a dot", 5, "    { \"wall\": \"w.a\", \"origin\": [0.5, 0, 0], \"on\": \"ground\", " + Wall1x1 + " }", 0, "", 5, "contains a dot"},
		{"wall unknown material", 5, "    { \"wall\": \"w\", \"origin\": [0.5, 0, 0], \"on\": \"ground\", \"courses\": 1, \"bricks\": 1, \"material\": \"oak\", \"bed\": \"c24_bearing_perp_to_grain\", \"perpend\": \"c24_bearing_perp_to_grain\" }",
			0, "", 5, "unknown material 'oak'"},
		{"wall unknown joint row", 5, "    { \"wall\": \"w\", \"origin\": [0.5, 0, 0], \"on\": \"ground\", \"courses\": 1, \"bricks\": 1, \"material\": \"c24\", \"bed\": \"no_such_row\", \"perpend\": \"c24_bearing_perp_to_grain\" }",
			0, "", 5, "unknown row 'no_such_row' for a wall joint"},
		{"wall omit holding a number", 5, "    { \"wall\": \"w\", \"origin\": [0.5, 0, 0], \"on\": \"ground\", " + Wall1x1 + ", \"omit\": [3] }", 0, "", 5, "omit must hold names"},
		{"wall on naming no box", 5, "    { \"wall\": \"w\", \"origin\": [0.5, 0, 0], \"on\": \"nothing\", " + Wall1x1 + " }", 0, "", 5, "'on' names no box: 'nothing'"},
		{"wall listed before its on box", 4, "    { \"wall\": \"w\", \"origin\": [0.5, 0, 0], \"on\": \"ground\", " + Wall1x1 + " },", 5,
			"    { \"name\": \"ground\", \"size\": [2, 2, 0.2], \"centre\": [0, 0, -0.1], \"grounded\": true }", 4, "must come before the wall"},
		{"bad range in a between pattern", 8, "    { \"between\": [\"block[1-]\", \"ground\"], \"row\": \"c24_bearing_perp_to_grain\" }", 0, "", 8, "bad range in a 'between' pattern"},
		{"unknown expect state", 11, "  \"expect\": { \"state\": \"Falls\" }", 0, "", 11, "unknown state 'Falls'"},
		{"badly written cracked joint", 11, "  \"expect\": { \"cracked\": [\"block\"] }", 0, "", 11, "a cracked joint is written '<a>/<b>'"},
	};
	{
		const ScenarioLoadResult Valid = LoadWithRows(Lines(Base));
		INFO("the base document must load; line " << Valid.Error.Line << ": " << Valid.Error.Message);
		CHECK(Valid.Ok());
	}
	for (const Fault& F : Faults)
	{
		std::vector<std::string> Doc = Base;
		Doc[static_cast<std::size_t>(F.Line - 1)] = F.Text;
		if (F.Line2 > 0)
		{
			Doc[static_cast<std::size_t>(F.Line2 - 1)] = F.Text2;
		}
		const std::string Json = Lines(Doc);
		const ScenarioLoadResult Loaded = LoadWithRows(Json);
		INFO(std::string(F.Name) << "\n" << Json << "line " << Loaded.Error.Line << ": " << Loaded.Error.Message);
		CHECK_FALSE(Loaded.Ok());
		CHECK(Loaded.Error.Line == F.ErrorLine);
		CHECK_MESSAGE(Mentions(Loaded.Error.Message, F.Phrase), "the message must contain: " << F.Phrase);
		CHECK(Loaded.Value.BodyNames.empty());
		CHECK(Loaded.Value.Model.LiveBodyCount() == 0);
	}
}

TEST_CASE("Scenario.Load.OverlappingBoxesAreAnErrorNamingBothBoxes")
{
	auto Doc = [](const std::string& BCentreX) {
		return "{\n"
			   "  \"title\": \"overlap\",\n"
			   "  \"build\": [\n"
			   "    { \"name\": \"ground\", \"size\": [0.8, 0.5, 0.2], \"centre\": [0.15, 0, -0.1], \"grounded\": true },\n"
			   "    { \"name\": \"a\", \"size\": [0.215, 0.1025, 0.065], \"centre\": [0.1075, 0, 0.0325], \"material\": \"clay_brick\" },\n"
			   "    { \"name\": \"b\", \"size\": [0.215, 0.1025, 0.065], \"centre\": [" +
			BCentreX +
			", 0, 0.0325], \"material\": \"clay_brick\" }\n"
			"  ],\n"
			"  \"joints\": [\n"
			"    { \"materials\": [\"clay_brick\", \"clay_brick\"], \"row\": \"clay_gp_mortar_perpend_ec6\" },\n"
			"    { \"materials\": [\"clay_brick\", \"ground\"], \"row\": \"clay_gp_mortar_bed_ec6\" }\n"
			"  ]\n"
			"}\n";
	};
	const ScenarioLoadResult Overlap = LoadWithRows(Doc("0.3125"));
	INFO("line " << Overlap.Error.Line << ": " << Overlap.Error.Message);
	CHECK_FALSE(Overlap.Ok());
	CHECK(Overlap.Error.Line == 6);
	CHECK(Mentions(Overlap.Error.Message, "'a'"));
	CHECK(Mentions(Overlap.Error.Message, "'b'"));
	CHECK(Mentions(Overlap.Error.Message, "overlap"));

	const ScenarioLoadResult Touching = LoadWithRows(Doc(Number(0.3225 - 0.5e-9)));
	INFO("within the tolerance; line " << Touching.Error.Line << ": " << Touching.Error.Message);
	REQUIRE(Touching.Ok());
	CHECK(FindJoint(Touching.Value, "a/b") != nullptr);
}

TEST_CASE("Scenario.Load.ATouchingPairWithNoRuleIsAnErrorNamingThePair")
{
	const std::string Json = R"({
  "title": "missing rule",
  "build": [
    { "name": "ground", "size": [0.8, 0.5, 0.2], "centre": [0.1, 0, -0.1], "grounded": true },
    { "name": "block", "size": [0.3, 0.2, 0.1], "centre": [0.1, 0, 0.05], "material": "c24" },
    { "name": "brick", "size": [0.215, 0.1025, 0.065], "centre": [0.1, 0, 0.1325], "material": "clay_brick" }
  ],
  "joints": [
    { "materials": ["c24", "ground"], "row": "c24_bearing_perp_to_grain" }
  ]
}
)";
	const ScenarioLoadResult Loaded = LoadWithRows(Json);
	INFO("line " << Loaded.Error.Line << ": " << Loaded.Error.Message);
	CHECK_FALSE(Loaded.Ok());
	CHECK(Loaded.Error.Line == 6);
	CHECK(Mentions(Loaded.Error.Message, "block / brick (c24 / clay_brick)"));
}

TEST_CASE("Scenario.Load.RepeatMakesNumberedCopiesAlongItsStep")
{
	const Scenario S = LoadScenarioOk(ColonnadeJson);
	const double PostMassKg = TimberDensity * 0.2 * 0.2 * 2.0;
	for (int K = 0; K < 5; ++K)
	{
		const std::string Name = "post." + std::to_string(K);
		INFO(Name);
		const BodyId Id = BodyOrFail(S, Name);
		CheckNear(S.Model.Body(Id).CentroidM, Vec3{2.2 * K, 0.0, 1.0}, 1e-12);
		CheckNear(S.BodyBoxes[Id.Index].HalfExtentsM, Vec3{0.1, 0.1, 1.0}, 0.0);
		CHECK(S.BodyMaterials[Id.Index] == "c24");
		CHECK(std::abs(S.Model.Body(Id).MassKg - PostMassKg) <= 1e-12 * PostMassKg);
	}
	for (int K = 0; K < 4; ++K)
	{
		const std::string Name = "beam." + std::to_string(K);
		INFO(Name);
		const BodyId Id = BodyOrFail(S, Name);
		CheckNear(S.Model.Body(Id).CentroidM, Vec3{1.1 + 2.2 * K, 0.0, 2.15}, 1e-12);
		CHECK(S.BodyMaterials[Id.Index] == "c24");
	}
	CHECK(S.BodyNames[0] == "ground");
	CHECK_FALSE(Has(S.BodyNames, "post"));
	CHECK_FALSE(Has(S.BodyNames, "beam"));

	const Scenario ByCorner = LoadScenarioOk(R"({
  "title": "min corner",
  "build": [
    { "name": "ground", "size": [12, 2, 0.2], "min": [-1.6, -1, -0.2], "grounded": true },
    { "name": "post", "size": [0.2, 0.2, 2.0], "min": [-0.1, -0.1, 0], "material": "c24",
      "repeat": { "count": 2, "step": [2.2, 0, 0] } }
  ],
  "joints": [
    { "materials": ["c24", "ground"], "row": "c24_bearing_perp_to_grain" }
  ]
}
)");
	CheckNear(ByCorner.Model.Body(BodyOrFail(ByCorner, "ground")).CentroidM, Vec3{4.4, 0.0, -0.1}, 1e-12);
	CheckNear(ByCorner.Model.Body(BodyOrFail(ByCorner, "post.1")).CentroidM, Vec3{2.2, 0.0, 1.0}, 1e-12);
}

TEST_CASE("Scenario.Load.TheWallGivesItsBricksCourseAndBrickNames")
{
	const Scenario S = LoadScenarioOk(WallJson(3, 4, ""));
	const DirectWall D = BuildDirectWall(3, 4);
	CHECK(S.Model.LiveBodyCount() == 14);
	const double FullKg = BrickDensity * BrickXM * BrickYM * BrickZM;
	for (const char* Half : {"wall.c1.b0", "wall.c1.b4"})
	{
		INFO(Half);
		CHECK(std::abs(S.Model.Body(BodyOrFail(S, Half)).MassKg - 0.5 * FullKg) <= 1e-12 * FullKg);
	}
	for (std::size_t C = 0; C < D.Wall.Courses.size(); ++C)
	{
		for (std::size_t K = 0; K < D.Wall.Courses[C].size(); ++K)
		{
			const BodyId Direct = D.Wall.Courses[C][K];
			const std::string Name = D.Names[Direct.Index];
			INFO(Name);
			const BodyId Id = BodyOrFail(S, Name);
			CHECK(S.Model.Body(Id).CentroidM.X == D.Model.Body(Direct).CentroidM.X);
			CHECK(S.Model.Body(Id).CentroidM.Y == D.Model.Body(Direct).CentroidM.Y);
			CHECK(S.Model.Body(Id).CentroidM.Z == D.Model.Body(Direct).CentroidM.Z);
			CHECK(S.Model.Body(Id).MassKg == D.Model.Body(Direct).MassKg);
		}
	}
	int Beds = 0;
	int Perpends = 0;
	for (const InterfaceId Id : S.Model.LiveInterfaces())
	{
		(IsBed(S.Model.Interface(Id)) ? Beds : Perpends) += 1;
	}
	CHECK(Beds == 20);
	CHECK(Perpends == 10);
	CHECK(ScenarioRecordKeys(S) == DirectRecordKeys(D));
}

TEST_CASE("Scenario.Load.OmittedWallBricksAreNeverBuilt")
{
	const std::string Omit = R"(["wall.c[0-2].b[2-3]"])";
	const Scenario S = LoadScenarioOk(WallJson(6, 6, Omit));
	CHECK(S.Model.LiveBodyCount() == 34);
	const char* const Omitted[] = {"wall.c0.b2", "wall.c0.b3", "wall.c1.b2", "wall.c1.b3", "wall.c2.b2", "wall.c2.b3"};
	for (const char* Name : Omitted)
	{
		INFO(Name);
		CHECK_FALSE(S.Model.IsKnown(S.Find(Name)));
		CHECK_FALSE(Has(S.BodyNames, Name));
	}
	for (const InterfaceId Id : S.Model.LiveInterfaces())
	{
		const InterfaceRecord& R = S.Model.Interface(Id);
		CHECK(S.Model.IsLive(R.BodyA));
		CHECK(S.Model.IsLive(R.BodyB));
	}
	DirectWall D = BuildDirectWall(6, 6);
	for (const char* Name : Omitted)
	{
		for (std::size_t Slot = 0; Slot < D.Names.size(); ++Slot)
		{
			if (D.Names[Slot] == Name)
			{
				for (const BodyId Id : D.Model.LiveBodies())
				{
					if (Id.Index == Slot)
					{
						CHECK(D.Model.RemoveBody(Id));
					}
				}
			}
		}
	}
	CHECK(S.Model.LiveInterfaceCount() == D.Model.LiveInterfaceCount());

	const std::string NamingOmitted = Replaced(WallJson(6, 6, Omit), "  \"steps\": [\"cascade\"]\n", "  \"steps\": [\"cascade\"],\n  \"expect\": { \"released\": [\"wall.c1.b2\"] }\n");
	const ScenarioLoadResult Loaded = LoadWithRows(NamingOmitted);
	INFO("line " << Loaded.Error.Line << ": " << Loaded.Error.Message);
	CHECK_FALSE(Loaded.Ok());
	CHECK(Mentions(Loaded.Error.Message, "wall.c1.b2"));
}

TEST_CASE("Scenario.Joints.EveryFaceTouchingPairGetsOneJointWithFaceContactsRectangle")
{
	const Scenario S = LoadScenarioOk(ContactsJson);
	std::vector<std::string> Pairs;
	for (const InterfaceId Id : S.Model.LiveInterfaces())
	{
		const InterfaceRecord& R = S.Model.Interface(Id);
		Pairs.push_back(PairName(S, R));
		INFO("joint " << PairName(S, R));
		const std::optional<InterfaceDesc> Want =
			geometry::FaceContact(R.BodyA, S.BodyBoxes[R.BodyA.Index], R.BodyB, S.BodyBoxes[R.BodyB.Index], R.Profile);
		REQUIRE(Want.has_value());
		CHECK(Format(R.NormalAToB) == Format(Want->NormalAToB));
		CHECK(Format(R.CentreM) == Format(Want->CentreM));
		CHECK(Format(R.AxisU) == Format(Want->AxisU));
		CHECK(Format(R.AxisV) == Format(Want->AxisV));
		CHECK(R.HalfExtentUM == Want->HalfExtentUM);
		CHECK(R.HalfExtentVM == Want->HalfExtentVM);
	}
	const std::vector<std::string> Want = SortedNames({"a/ground", "b/ground", "d/ground", "e/ground", "a/b", "a/c", "b/c"});
	CHECK(SortedNames(Pairs) == Want);
}

TEST_CASE("Scenario.Joints.APlaneRuleSeparatesBedsFromPerpends")
{
	const Scenario S = LoadScenarioOk(ContactsJson);
	REQUIRE(S.Model.LiveInterfaceCount() == 7);
	for (const InterfaceId Id : S.Model.LiveInterfaces())
	{
		const InterfaceRecord& R = S.Model.Interface(Id);
		INFO("joint " << PairName(S, R));
		CHECK(ProfileName(S, R) == (PairName(S, R) == "a/b" ? "clay_gp_mortar_perpend_ec6" : "clay_gp_mortar_bed_ec6"));
	}
}

TEST_CASE("Scenario.Joints.ABodyPairRuleBeatsAMaterialRuleWhateverTheirOrder")
{
	const StrengthProfile Bearing = JointOrFail(ScenarioRows(), "c24_bearing_perp_to_grain").Profile;
	const FastenerRow& Screw = FastenerOrFail(ScenarioRows(), "ec5_screw_8x60_steel_to_glulam");
	for (const std::string& Rules : {BearingRule + ",\n" + ScrewRule, ScrewRule + ",\n" + BearingRule})
	{
		const Scenario S = LoadScenarioOk(BayJson(Rules));
		INFO(Rules);
		REQUIRE(S.Model.LiveInterfaceCount() == 4);
		for (const InterfaceId Id : S.Model.LiveInterfaces())
		{
			const InterfaceRecord& R = S.Model.Interface(Id);
			INFO("joint " << PairName(S, R));
			const bool bScrewed = PairName(S, R) == "beam.0/post.1";
			const StrengthProfile Want = bScrewed ? FastenerProfile(Screw, 3, 4.0 * R.HalfExtentUM * R.HalfExtentVM) : Bearing;
			CHECK(SameProfile(S.Model.Profile(R.Profile), Want));
		}
	}
	const Scenario First = LoadScenarioOk(BayJson(BearingRule + ",\n" + MortarRule));
	const Scenario Reversed = LoadScenarioOk(BayJson(MortarRule + ",\n" + BearingRule));
	for (const InterfaceId Id : First.Model.LiveInterfaces())
	{
		const InterfaceRecord& R = First.Model.Interface(Id);
		INFO("joint " << PairName(First, R));
		CHECK(ProfileName(First, R) == "c24_bearing_perp_to_grain");
	}
	for (const InterfaceId Id : Reversed.Model.LiveInterfaces())
	{
		const InterfaceRecord& R = Reversed.Model.Interface(Id);
		INFO("joint " << PairName(Reversed, R));
		CHECK(ProfileName(Reversed, R) == (Mentions(PairName(Reversed, R), "ground") ? "c24_bearing_perp_to_grain" : "clay_gp_mortar_bed_ec6"));
	}
}

TEST_CASE("Scenario.Joints.AFastenerRuleGivesFastenerProfileForTheContactArea")
{
	const Scenario S = LoadScenarioOk(BayJson(BearingRule + ",\n" + ScrewRule));
	const InterfaceRecord* R = FindJoint(S, "beam.0/post.1");
	REQUIRE(R != nullptr);
	const double AreaM2 = 4.0 * R->HalfExtentUM * R->HalfExtentVM;
	CHECK(std::abs(AreaM2 - 0.0995 * 0.2) <= 1e-12);
	const StrengthProfile Want = FastenerProfile(FastenerOrFail(ScenarioRows(), "ec5_screw_8x60_steel_to_glulam"), 3, AreaM2);
	CHECK(SameProfile(S.Model.Profile(R->Profile), Want));
}

TEST_CASE("Scenario.Joints.WallBricksKeepTheGeneratorsJointsAndMeetOtherBoxesByRule")
{
	const std::string Beam = ",\n    { \"name\": \"beam\", \"size\": [0.86, 0.1025, 0.15], \"centre\": [0.43, 0, 0.335], \"material\": \"c24\" }";
	const std::string Rule = " { \"materials\": [\"c24\", \"clay_brick\"], \"row\": \"c24_bearing_perp_to_grain\" } ";
	const Scenario S = LoadScenarioOk(WallJson(4, 4, "", Beam, Rule));
	int Beds = 0;
	int Perpends = 0;
	int BeamJoints = 0;
	std::vector<std::string> Pairs;
	for (const InterfaceId Id : S.Model.LiveInterfaces())
	{
		const InterfaceRecord& R = S.Model.Interface(Id);
		const std::string Pair = PairName(S, R);
		INFO("joint " << Pair);
		Pairs.push_back(Pair);
		if (Mentions(Pair, "beam"))
		{
			++BeamJoints;
			CHECK(ProfileName(S, R) == "c24_bearing_perp_to_grain");
		}
		else if (IsBed(R))
		{
			++Beds;
			CHECK(ProfileName(S, R) == "clay_gp_mortar_bed_ec6");
		}
		else
		{
			++Perpends;
			CHECK(ProfileName(S, R) == "clay_gp_mortar_perpend_ec6");
		}
	}
	CHECK(Beds == 28);
	CHECK(Perpends == 14);
	CHECK(BeamJoints == 5);
	std::vector<std::string> Unique = SortedNames(Pairs);
	CHECK(std::adjacent_find(Unique.begin(), Unique.end()) == Unique.end());
}

TEST_CASE("Scenario.Joints.ProfilesAreSharedPerRow")
{
	CHECK(LoadScenarioOk(ColonnadeJson).ProfileNames.size() == 1);
	CHECK(LoadScenarioOk(BayJson(BearingRule + ",\n" + ScrewRule)).ProfileNames.size() == 2);
}

TEST_CASE("Scenario.Build.MassIsDensityTimesBoxVolume")
{
	const Scenario S = LoadScenarioOk(CantileverJson);
	CHECK(S.Model.Body(BodyOrFail(S, "ground")).MassKg == 0.0);
	struct Row
	{
		const char* Name;
		double Want;
		double Published;
	};
	for (const Row& R : {Row{"post", TimberDensity * 0.2 * 0.2 * 1.0, 16.8}, Row{"beam", TimberDensity * 1.6 * 0.2 * 0.1, 13.44}, Row{"counterweight", 500.0 * 0.2 * 0.2 * 0.2, 4.0}})
	{
		INFO(R.Name);
		const double Got = S.Model.Body(BodyOrFail(S, R.Name)).MassKg;
		CHECK(std::abs(Got - R.Want) <= 1e-12 * R.Want);
		CHECK(std::abs(Got - R.Published) <= 1e-12 * R.Published);
	}
}

TEST_CASE("Scenario.Build.FileLocalProfilesDeriveFromACatalogueRow")
{
	const Scenario S = LoadScenarioOk(R"({
  "title": "dry brick",
  "profiles": [
    { "name": "dry_brick", "from": "clay_gp_mortar_bed_ec6", "cohesion_mpa": 0, "tensile_mpa": 0, "friction": 0.7, "shear_cap_mpa": null }
  ],
  "build": [
    { "name": "ground", "size": [1, 0.6, 0.2], "centre": [0.1075, 0, -0.1], "grounded": true },
    { "name": "low", "size": [0.215, 0.1025, 0.065], "centre": [0.1075, 0, 0.0325], "material": "clay_brick" },
    { "name": "high", "size": [0.215, 0.1025, 0.065], "centre": [0.1075, 0, 0.0975], "material": "clay_brick" }
  ],
  "joints": [
    { "materials": ["clay_brick", "clay_brick"], "row": "dry_brick" },
    { "materials": ["clay_brick", "ground"], "row": "dry_brick" }
  ]
}
)");
	const StrengthProfile Row = JointOrFail(ScenarioRows(), "clay_gp_mortar_bed_ec6").Profile;
	REQUIRE(S.Model.LiveInterfaceCount() == 2);
	for (const InterfaceId Id : S.Model.LiveInterfaces())
	{
		const StrengthProfile& P = S.Model.Profile(S.Model.Interface(Id).Profile);
		CHECK(P.ShearCohesionPa == 0.0);
		CHECK(P.TensileStrengthPa == 0.0);
		CHECK(P.FrictionCoefficient == 0.7);
		CHECK(P.MaxShearStrengthPa == std::numeric_limits<double>::infinity());
		CHECK(P.CompressiveStrengthPa == Row.CompressiveStrengthPa);
		CHECK(P.CompressiveStrengthPa == 8.93 * PaPerMPa);
		CHECK(P.NormalStiffnessPaPerM == Row.NormalStiffnessPaPerM);
		CHECK(P.TangentialStiffnessPaPerM == Row.TangentialStiffnessPaPerM);
	}
}

TEST_CASE("Scenario.Steps.RunInFileOrder")
{
	Scenario S = LoadScenarioOk(ColonnadeJson);
	const RecordingCascade Recorder(S);
	const std::unique_ptr<IStructuralAuthority> Authority = StandardAuthority();
	const ScenarioOutcome O = RunScenario(S, Recorder, *Authority);
	INFO("run error: " << O.RunError);
	REQUIRE(Recorder.Calls.size() == 2);
	CHECK(Has(Recorder.Calls[0].Live, "post.2"));
	CHECK_FALSE(Has(Recorder.Calls[1].Live, "post.2"));
	REQUIRE(O.Steps.size() == 3);
	CHECK(O.Steps[0].Kind == "cascade");
	CHECK(O.Steps[0].Released.empty());
	CHECK(O.Steps[1].Kind == "remove");
	CHECK(O.Steps[1].Removed == std::vector<std::string>{"post.2"});
	CHECK(O.Steps[2].Kind == "cascade");
	CHECK(SortedPartition(O.Steps[2].Released) == SortedPartition({{"beam.1"}, {"beam.2"}}));

	Scenario Early = LoadScenarioOk(Replaced(ColonnadeJson, R"("steps": ["cascade", { "remove": ["post.2"] }, "cascade"])", R"("steps": [{ "remove": ["post.2"] }, "cascade"])"));
	const RecordingCascade EarlyRecorder(Early);
	const ScenarioOutcome EarlyOutcome = RunScenario(Early, EarlyRecorder, *Authority);
	REQUIRE(EarlyRecorder.Calls.size() == 1);
	CHECK_FALSE(Has(EarlyRecorder.Calls[0].Live, "post.2"));
	REQUIRE(EarlyOutcome.Steps.size() == 2);
	CHECK(SortedPartition(EarlyOutcome.Steps[1].Released) == SortedPartition({{"beam.1"}, {"beam.2"}}));
}

TEST_CASE("Scenario.Steps.LoadAndLateralStepsReachTheCascadeExactlyAsTheApiWould")
{
	const std::unique_ptr<IStructuralAuthority> Authority = StandardAuthority();
	SUBCASE("lateral_g on a bonded wall")
	{
		const double HeightM = 5 * BrickZM;
		const double BaseAreaM2 = 4 * BrickXM * BrickYM;
		const double WeightN = BrickDensity * 20 * BrickXM * BrickYM * BrickZM * GravityMPerS2;
		const double TensilePa = JointOrFail(ScenarioRows(), "clay_gp_mortar_bed_ec6").Profile.TensileStrengthPa;
		const double LambdaC = (BrickYM / HeightM) * (1.0 + TensilePa * BaseAreaM2 / WeightN);
		CHECK(std::abs(LambdaC - 1.9646) <= 1e-4);
		const double Lateral = 1.02 * LambdaC;
		const std::string Json =
			Replaced(WallJson(5, 4, ""), "  \"steps\": [\"cascade\"]\n", "  \"steps\": [{ \"lateral_g\": [0, " + Number(Lateral) + "] }, \"cascade\"]\n");
		Scenario S = LoadScenarioOk(Json);
		const RecordingCascade Recorder(S);
		const ScenarioOutcome O = RunScenario(S, Recorder, *Authority);
		INFO("run error: " << O.RunError);
		REQUIRE(Recorder.Calls.size() == 1);
		const Vec3 Gravity{0.0, Lateral * GravityMPerS2, -GravityMPerS2};
		CHECK(Recorder.Calls[0].Gravity.X == Gravity.X);
		CHECK(Recorder.Calls[0].Gravity.Y == Gravity.Y);
		CHECK(Recorder.Calls[0].Gravity.Z == Gravity.Z);
		CHECK(Recorder.Calls[0].Loads.empty());

		DirectWall D = BuildDirectWall(5, 4);
		CascadeOptions Options;
		Options.Authority.Standing.Gravity = Gravity;
		const CascadeVerdict Want = MakeCascade(StandardAuthority())->Run(D.Model, {}, Options, nullptr);
		REQUIRE(O.Verdicts.size() == 1);
		CheckSameVerdict(S.Model, O.Verdicts.back(), D.Model, Want);
	}
	SUBCASE("load on a beam over two piers")
	{
		const StrengthProfile Bearing = JointOrFail(ScenarioRows(), "c24_bearing_perp_to_grain").Profile;
		const StrengthProfile Bed = JointOrFail(ScenarioRows(), "clay_gp_mortar_bed_ec6").Profile;
		const double BeamWeightN = TimberDensity * 1.215 * BrickYM * 0.15 * GravityMPerS2;
		const double LoadN = 2.0 * (1.2 * Bearing.CompressiveStrengthPa * BrickXM * BrickYM - 0.5 * BeamWeightN);
		const std::string Json = R"({
  "title": "beam on piers",
  "build": [
    { "name": "ground", "size": [2, 0.6, 0.2], "centre": [0.6, 0, -0.1], "grounded": true },
    { "name": "pier0", "size": [0.215, 0.1025, 0.065], "centre": [0.1075, 0, 0.0325], "material": "clay_brick",
      "repeat": { "count": 5, "step": [0, 0, 0.065] } },
    { "name": "pier1", "size": [0.215, 0.1025, 0.065], "centre": [1.1075, 0, 0.0325], "material": "clay_brick",
      "repeat": { "count": 5, "step": [0, 0, 0.065] } },
    { "name": "beam", "size": [1.215, 0.1025, 0.15], "centre": [0.6075, 0, 0.4], "material": "c24" }
  ],
  "joints": [
    { "materials": ["clay_brick", "ground"], "row": "clay_gp_mortar_bed_ec6" },
    { "materials": ["clay_brick", "clay_brick"], "row": "clay_gp_mortar_bed_ec6" },
    { "materials": ["c24", "clay_brick"], "row": "c24_bearing_perp_to_grain" }
  ],
  "steps": [{ "load": { "body": "beam", "force_n": [0, 0, -)" + Number(LoadN) + R"(] } }, "cascade"]
}
)";
		Scenario S = LoadScenarioOk(Json);
		const RecordingCascade Recorder(S);
		const ScenarioOutcome O = RunScenario(S, Recorder, *Authority);
		INFO("run error: " << O.RunError);
		REQUIRE(Recorder.Calls.size() == 1);
		REQUIRE(Recorder.Calls[0].Loads.size() == 1);
		const BodyLoad& Got = Recorder.Calls[0].Loads[0];
		CHECK(NameOf(S, Got.Body) == "beam");
		CHECK(Format(Got.ForceN) == Format(Vec3{0.0, 0.0, -LoadN}));
		CHECK(Format(Got.PointM) == Format(Vec3{0.6075, 0.0, 0.4}));
		CHECK(Format(Recorder.Calls[0].Gravity) == Format(Vec3{0.0, 0.0, -GravityMPerS2}));

		std::vector<DirectBox> Boxes = {{"ground", geometry::Box{Vec3{0.6, 0.0, -0.1}, Vec3{1.0, 0.3, 0.1}}, 0.0, true}};
		for (const double X : {0.1075, 1.1075})
		{
			for (int C = 0; C < 5; ++C)
			{
				Boxes.push_back({"clay_brick", geometry::Box{Vec3{X, 0.0, 0.0325 + 0.065 * C}, Vec3{BrickXM / 2, BrickYM / 2, BrickZM / 2}}, BrickDensity, false});
			}
		}
		Boxes.push_back({"c24", geometry::Box{Vec3{0.6075, 0.0, 0.4}, Vec3{0.6075, BrickYM / 2, 0.075}}, TimberDensity, false});
		std::vector<BodyId> Ids;
		Structure Direct = BuildDirectBoxes(Boxes, [&](const std::string& A, const std::string& B) { return A == "c24" || B == "c24" ? Bearing : Bed; }, Ids);
		const std::vector<BodyLoad> Loads = {BodyLoad{Ids.back(), Vec3{0.0, 0.0, -LoadN}, Vec3{0.6075, 0.0, 0.4}}};
		const CascadeVerdict Want = MakeCascade(StandardAuthority())->Run(Direct, Loads, CascadeOptions{}, nullptr);
		REQUIRE(O.Verdicts.size() == 1);
		CheckSameVerdict(S.Model, O.Verdicts.back(), Direct, Want);
	}
}

TEST_CASE("Scenario.Steps.TheBeforeVerdictJudgesTheStructureAsTheBeforeViewDrawsIt")
{
	const auto FileText = [](std::string_view Name) {
		std::ifstream File(std::string(LOADPATH_SCENARIO_DIR) + "/" + std::string(Name), std::ios::binary);
		REQUIRE(File.good());
		return std::string(std::istreambuf_iterator<char>(File), std::istreambuf_iterator<char>());
	};
	const auto BeforeOf = [](const std::string& Json) {
		Scenario S = LoadScenarioOk(Json);
		const ScenarioOutcome O = RunScenario(S);
		INFO("run error: " << O.RunError);
		REQUIRE(O.RunError.empty());
		REQUIRE(O.Before.has_value());
		return *O.Before;
	};
	SUBCASE("cascade, post.2 removed, cascade: the file as written")
	{
		CHECK(BeforeOf(FileText("colonnade-middle-post.json")) == ComponentVerdict::Collapses);
	}
	SUBCASE("post.2 removed, then cascade")
	{
		const std::string Json =
			Replaced(FileText("colonnade-middle-post.json"), R"("steps": ["cascade", { "remove": ["post.2"] }, "cascade"])", R"("steps": [{ "remove": ["post.2"] }, "cascade"])");
		CHECK(BeforeOf(Json) == ComponentVerdict::Collapses);
	}
	SUBCASE("lateral_g at 1.02 of the bonded factor: overstressed before the cascade cracks the base")
	{
		CHECK(BeforeOf(FileText("bonded-wall-lateral-step.json")) == ComponentVerdict::Overstressed);
	}
	SUBCASE("a load step: overloaded before the cascade releases the beam")
	{
		CHECK(BeforeOf(FileText("beam-on-piers-load-step.json")) == ComponentVerdict::Collapses);
	}
	SUBCASE("intact wall, cascade only")
	{
		CHECK(BeforeOf(FileText("wall-brick-names.json")) == ComponentVerdict::Stands);
	}
}

TEST_CASE("Scenario.Steps.RemovingABodyAlreadyGoneIsARunError")
{
	Scenario S = LoadScenarioOk(Replaced(ColonnadeJson, R"(, "cascade"],)", R"(, "cascade", { "remove": ["beam.1"] }],)"));
	const ScenarioOutcome O = RunScenario(S);
	INFO("run error: " << O.RunError);
	CHECK(Mentions(O.RunError, "beam.1"));
	CHECK(Mentions(O.RunError, "step 4"));
	REQUIRE(S.Expect.has_value());
	CHECK_FALSE(CompareExpectation(*S.Expect, O).bMet);
}

TEST_CASE("Scenario.Steps.AJudgeStepReleasesNothing")
{
	Scenario S = LoadScenarioOk(Replaced(ColonnadeJson, R"("steps": ["cascade", { "remove": ["post.2"] }, "cascade"])", R"("steps": [{ "remove": ["post.2"] }, "judge"])"));
	const ScenarioOutcome O = RunScenario(S);
	INFO("run error: " << O.RunError);
	REQUIRE(O.Steps.size() == 2);
	CHECK(O.Steps[1].Kind == "judge");
	CHECK(O.Steps[1].State == "Collapses");
	CHECK(O.Steps[1].Released.empty());
	CHECK(O.Clusters.empty());
	for (const std::string& Name : S.BodyNames)
	{
		INFO(Name);
		CHECK(S.Model.IsLive(S.Find(Name)) == (Name != "post.2"));
	}
	for (const InterfaceId Id : S.Model.LiveInterfaces())
	{
		const InterfaceRecord& R = S.Model.Interface(Id);
		INFO("joint " << PairName(S, R));
		CHECK(R.State == InterfaceState::Intact);
		CHECK(R.CrackPass == 0u);
		CHECK(R.SeverPass == 0u);
	}
}

TEST_CASE("Scenario.Expect.TheColonnadeMeetsItsExpectation")
{
	Scenario S = LoadScenarioOk(ColonnadeJson);
	const ScenarioOutcome O = RunScenario(S);
	REQUIRE(S.Expect.has_value());
	const ExpectationReport Report = CompareExpectation(*S.Expect, O);
	INFO("run error: " << O.RunError << "; report:" << Joined(Report.Lines));
	CHECK(Report.bMet);
	CHECK(Report.Lines.empty());
}

TEST_CASE("Scenario.Expect.AMismatchNamesEveryBodyThatDiffers")
{
	Scenario S = LoadScenarioOk(WrongExpectationJson());
	const ScenarioOutcome O = RunScenario(S);
	REQUIRE(S.Expect.has_value());
	const ExpectationReport Report = CompareExpectation(*S.Expect, O);
	INFO("report:" << Joined(Report.Lines));
	CHECK_FALSE(Report.bMet);
	const std::vector<std::string> Want = SortedNames(
		{"released, not expected: beam.2", "expected to fall, still standing: post.1", "expected {beam.1} {post.1}; engine {beam.1} {beam.2}"});
	CHECK(SortedNames(Report.Lines) == Want);
}

TEST_CASE("Scenario.Expect.ClustersAreComparedAsAPartition")
{
	const ScenarioOutcome Engine = Synthetic({"a", "b"}, {{"a"}, {"b"}});
	ScenarioExpect Together;
	Together.bPartition = true;
	Together.Released = {{"a", "b"}};
	const ExpectationReport NotMet = CompareExpectation(Together, Engine);
	INFO("report:" << Joined(NotMet.Lines));
	CHECK_FALSE(NotMet.bMet);
	CHECK(NotMet.Lines == std::vector<std::string>{"expected {a, b}; engine {a} {b}"});

	ScenarioExpect Flat;
	Flat.bPartition = false;
	Flat.Released = {{"a", "b"}};
	CHECK(CompareExpectation(Flat, Engine).bMet);

	ScenarioExpect Reordered;
	Reordered.bPartition = true;
	Reordered.Released = {{"b"}, {"a"}};
	CHECK(CompareExpectation(Reordered, Engine).bMet);
}

TEST_CASE("Scenario.Expect.PatternsExpandOverNames")
{
	std::vector<std::string> Names;
	for (int C = 0; C < 10; ++C)
	{
		for (int K = 0; K < (C % 2 == 0 ? 8 : 9); ++K)
		{
			Names.push_back("wall.c" + std::to_string(C) + ".b" + std::to_string(K));
		}
	}
	for (int K = 0; K < 5; ++K)
	{
		Names.push_back("post." + std::to_string(K));
	}
	Names.push_back("post");
	Names.push_back("postbox");

	const PatternExpansion Posts = ExpandPattern("post.*", Names);
	CHECK(Posts.Ok());
	CHECK(SortedNames(Posts.Names) == std::vector<std::string>{"post.0", "post.1", "post.2", "post.3", "post.4"});
	CHECK(ExpandPattern("wall.c3.*", Names).Names.size() == 9);
	CHECK(ExpandPattern("wall.c*.b0", Names).Names.size() == 10);
	const PatternExpansion Block = ExpandPattern("wall.c[6-9].b[2-5]", Names);
	CHECK(Block.Names.size() == 16);
	CHECK(Has(Block.Names, "wall.c6.b2"));
	CHECK(Has(Block.Names, "wall.c9.b5"));
	CHECK_FALSE(Has(Block.Names, "wall.c5.b2"));
	CHECK_FALSE(Has(Block.Names, "wall.c6.b6"));
	CHECK(ExpandPattern("postbox", Names).Names == std::vector<std::string>{"postbox"});
	CHECK_FALSE(ExpandPattern("wall.c[9-6].*", Names).Ok());
	CHECK_FALSE(ExpandPattern("nothing.*", Names).Ok());
}

TEST_CASE("Scenario.Expect.DefaultsAreStandsAndNothingElseFalls")
{
	const ScenarioExpect Default{};
	CHECK(Default.State == "Stands");
	CHECK(CompareExpectation(Default, Synthetic({"x"}, {})).bMet);

	const ExpectationReport Undecided = CompareExpectation(Default, Synthetic({"x"}, {}, "Undecided"));
	INFO("report:" << Joined(Undecided.Lines));
	CHECK_FALSE(Undecided.bMet);
	CHECK(Undecided.Lines == std::vector<std::string>{"state: expected Stands, engine Undecided"});

	const ExpectationReport Released = CompareExpectation(Default, Synthetic({"x"}, {{"x"}}));
	INFO("report:" << Joined(Released.Lines));
	CHECK_FALSE(Released.bMet);
	CHECK(Released.Lines == std::vector<std::string>{"released, not expected: x"});
}

TEST_CASE("Scenario.Expect.EitherBodiesAreNotAsserted")
{
	ScenarioExpect Expect;
	Expect.Either = {"w.c6.b0"};
	CHECK(CompareExpectation(Expect, Synthetic({"w.c6.b0", "w.c6.b1"}, {})).bMet);
	CHECK(CompareExpectation(Expect, Synthetic({"w.c6.b0", "w.c6.b1"}, {{"w.c6.b0"}})).bMet);

	const std::string Json = R"({
  "title": "either overlap",
  "build": [
    { "name": "ground", "size": [1, 0.6, 0.2], "centre": [0.1, 0, -0.1], "grounded": true },
    { "wall": "w", "courses": 7, "bricks": 1, "on": "ground", "material": "clay_brick",
      "bed": "clay_gp_mortar_bed_ec6", "perpend": "clay_gp_mortar_perpend_ec6" }
  ],
  "expect": { "released": [["w.c6.b0"]], "either": ["w.c6.b0"] }
}
)";
	const ScenarioLoadResult Loaded = LoadWithRows(Json);
	INFO("line " << Loaded.Error.Line << ": " << Loaded.Error.Message);
	CHECK_FALSE(Loaded.Ok());
	CHECK(Loaded.Error.Line == 8);
	CHECK(Mentions(Loaded.Error.Message, "w.c6.b0"));
}

TEST_CASE("Scenario.Expect.PassesAndCrackedAreCheckedOnlyWhenGiven")
{
	ScenarioOutcome Engine = Synthetic({"w.c0.b1", "ground"}, {});
	Engine.Passes = 2;
	Engine.Cracked = {"w.c0.b1/ground"};

	const ExpectationReport Absent = CompareExpectation(ScenarioExpect{}, Engine);
	INFO("report:" << Joined(Absent.Lines));
	CHECK(Absent.bMet);
	CHECK(Absent.Lines.empty());

	ScenarioExpect Two;
	Two.Passes = 2;
	CHECK(CompareExpectation(Two, Engine).bMet);

	ScenarioExpect One;
	One.Passes = 1;
	const ExpectationReport Wrong = CompareExpectation(One, Engine);
	INFO("report:" << Joined(Wrong.Lines));
	CHECK_FALSE(Wrong.bMet);
	REQUIRE(Wrong.Lines.size() == 1);
	CHECK(Wrong.Lines[0].starts_with("passes"));

	ScenarioExpect Cracked;
	Cracked.Cracked = std::vector<std::string>{"w.c0.*/ground"};
	CHECK(CompareExpectation(Cracked, Engine).bMet);
}

TEST_CASE("Scenario.Expect.TheDeferredFieldIsKeptAndReported")
{
	const std::string Deferred = Replaced(ColonnadeJson, "  \"title\": ", "  \"deferred\": \"Slice6 least-joint-movement selection\",\n  \"title\": ");
	Scenario WithItem = LoadScenarioOk(Deferred);
	Scenario Plain = LoadScenarioOk(ColonnadeJson);
	REQUIRE(WithItem.Deferred.has_value());
	CHECK(*WithItem.Deferred == "Slice6 least-joint-movement selection");
	CHECK_FALSE(Plain.Deferred.has_value());
	const ScenarioOutcome A = RunScenario(WithItem);
	const ScenarioOutcome B = RunScenario(Plain);
	CHECK(SortedPartition(A.Clusters) == SortedPartition(B.Clusters));
	CHECK(A.State == B.State);
	CHECK(A.Passes == B.Passes);
	REQUIRE(WithItem.Expect.has_value());
	REQUIRE(Plain.Expect.has_value());
	const ExpectationReport ReportA = CompareExpectation(*WithItem.Expect, A);
	const ExpectationReport ReportB = CompareExpectation(*Plain.Expect, B);
	CHECK(ReportA.bMet == ReportB.bMet);
	CHECK(ReportA.Lines == ReportB.Lines);
}

TEST_CASE("Scenario.Viewer.TheExportCarriesBodiesBeforeAndAfter")
{
	Scenario S = LoadScenarioOk(ColonnadeJson);
	const ScenarioOutcome O = RunScenario(S);
	const std::string Json = WriteViewerJson(S, O, ColonnadeSlug);
	const Parsed P = Parse(Json);
	REQUIRE(P.bOk);
	const detail::JsonValue& Root = P.Root;
	CHECK(Field(Root, "slug").Text == ColonnadeSlug);
	CHECK(Field(Root, "title").Text == "Colonnade losing its middle post");
	CHECK(Field(Root, "summary").Text == S.Summary);
	CHECK(Field(Root, "units").Text == "m");
	CHECK(Numbers(Field(Root, "gravity")) == std::vector<double>{0.0, 0.0, -GravityMPerS2});
	const detail::JsonValue& Bodies = Field(Root, "bodies");
	REQUIRE(Bodies.Items.size() == 10);
	for (const detail::JsonValue& Body : Bodies.Items)
	{
		const std::size_t Slot = static_cast<std::size_t>(Field(Body, "id").Number);
		REQUIRE(Slot < S.BodyNames.size());
		INFO("body " << S.BodyNames[Slot]);
		CHECK(Field(Body, "name").Text == S.BodyNames[Slot]);
		CHECK(Field(Body, "material").Text == S.BodyMaterials[Slot]);
		const geometry::Box& Box = S.BodyBoxes[Slot];
		CHECK(Numbers(Field(Body, "c")) == std::vector<double>{Box.CentreM.X, Box.CentreM.Y, Box.CentreM.Z});
		CHECK(Numbers(Field(Body, "h")) == std::vector<double>{Box.HalfExtentsM.X, Box.HalfExtentsM.Y, Box.HalfExtentsM.Z});
		CHECK(Numbers(Field(Body, "q")) == std::vector<double>{1.0, 0.0, 0.0, 0.0});
		const bool bGround = S.BodyNames[Slot] == "ground";
		CHECK(Field(Body, "grounded").Bool == bGround);
		const double Volume = 8.0 * Box.HalfExtentsM.X * Box.HalfExtentsM.Y * Box.HalfExtentsM.Z;
		const double WantKg = bGround ? 0.0 : TimberDensity * Volume;
		CHECK(std::abs(Field(Body, "mass_kg").Number - WantKg) <= 1e-9 * (WantKg + 1.0));
	}
	const detail::JsonValue& Interfaces = Field(Root, "interfaces");
	CHECK(Interfaces.Items.size() == 13);
	for (const detail::JsonValue& I : Interfaces.Items)
	{
		CHECK(HasKey(I, "a"));
		CHECK(HasKey(I, "b"));
		CHECK(Field(I, "p").Items.size() == 3);
		CHECK(Field(I, "n").Items.size() == 3);
	}
	const double Post2 = static_cast<double>(BodyOrFail(S, "post.2").Index);
	CHECK(Numbers(Field(Root, "removed_before_run")) == std::vector<double>{Post2});
	CHECK(Field(Field(Root, "before"), "verdict").Text == "Collapses");
	const detail::JsonValue& After = Field(Root, "after");
	CHECK(Field(After, "state").Text == "Stands");
	CHECK(Field(After, "passes").Number == 1.0);
	const detail::JsonValue& Clusters = Field(After, "clusters");
	const std::vector<std::vector<double>> Want = {{static_cast<double>(BodyOrFail(S, "beam.1").Index)}, {static_cast<double>(BodyOrFail(S, "beam.2").Index)}};
	CHECK(ClusterBodies(Clusters) == Want);
	for (const detail::JsonValue& C : Clusters.Items)
	{
		CHECK(HasKey(C, "motion"));
	}
	CHECK_FALSE(HasKey(Root, "expected"));
}

TEST_CASE("Scenario.Viewer.TheExpectedAfterAppearsOnlyWhenItDiffers")
{
	Scenario Wrong = LoadScenarioOk(WrongExpectationJson());
	const ScenarioOutcome WrongOutcome = RunScenario(Wrong);
	const Parsed P = Parse(WriteViewerJson(Wrong, WrongOutcome, WrongExpectationSlug));
	REQUIRE(P.bOk);
	REQUIRE(HasKey(P.Root, "expected"));
	const detail::JsonValue& Expected = Field(P.Root, "expected");
	CHECK(Field(Expected, "state").Text == "Stands");
	const std::vector<std::vector<double>> Want =
		std::vector<std::vector<double>>{{static_cast<double>(BodyOrFail(Wrong, "post.1").Index)}, {static_cast<double>(BodyOrFail(Wrong, "beam.1").Index)}};
	std::vector<std::vector<double>> SortedWant = Want;
	std::sort(SortedWant.begin(), SortedWant.end());
	CHECK(ClusterBodies(Field(Expected, "clusters")) == SortedWant);

	Scenario Right = LoadScenarioOk(ColonnadeJson);
	const ScenarioOutcome RightOutcome = RunScenario(Right);
	const Parsed Q = Parse(WriteViewerJson(Right, RightOutcome, ColonnadeSlug));
	REQUIRE(Q.bOk);
	CHECK_FALSE(HasKey(Q.Root, "expected"));
}

TEST_CASE("Scenario.Viewer.WallBricksExportTheirBoxes")
{
	Scenario S = LoadScenarioOk(WallJson(3, 4, ""));
	const ScenarioOutcome O = RunScenario(S);
	const Parsed P = Parse(WriteViewerJson(S, O, "wall-brick-names"));
	REQUIRE(P.bOk);
	struct Row
	{
		const char* Name;
		Vec3 Half;
	};
	for (const Row& R : {Row{"wall.c1.b0", Vec3{0.05375, 0.05125, 0.0325}}, Row{"wall.c0.b0", Vec3{0.1075, 0.05125, 0.0325}}})
	{
		INFO(R.Name);
		const detail::JsonValue* Body = BodyNamed(P.Root, R.Name);
		REQUIRE(Body != nullptr);
		const std::vector<double> H = Numbers(Field(*Body, "h"));
		REQUIRE(H.size() == 3);
		CheckNear(Vec3{H[0], H[1], H[2]}, R.Half, 1e-12);
		const Vec3 Centroid = S.Model.Body(BodyOrFail(S, R.Name)).CentroidM;
		CHECK(Numbers(Field(*Body, "c")) == std::vector<double>{Centroid.X, Centroid.Y, Centroid.Z});
	}
}

TEST_CASE("Scenario.Viewer.OmittedBricksAreNotExported")
{
	Scenario S = LoadScenarioOk(WallJson(6, 6, R"(["wall.c[0-2].b[2-3]"])"));
	const ScenarioOutcome O = RunScenario(S);
	const Parsed P = Parse(WriteViewerJson(S, O, "wall-with-opening"));
	REQUIRE(P.bOk);
	CHECK(Field(P.Root, "bodies").Items.size() == 34);
	for (const char* Name : {"wall.c0.b2", "wall.c0.b3", "wall.c1.b2", "wall.c1.b3", "wall.c2.b2", "wall.c2.b3"})
	{
		INFO(Name);
		CHECK(BodyNamed(P.Root, Name) == nullptr);
	}
	CHECK(Field(P.Root, "removed_before_run").Items.empty());
}

TEST_CASE("Scenario.Viewer.TheExportIsByteIdenticalRunToRun")
{
	Scenario First = LoadScenarioOk(ColonnadeJson);
	Scenario Second = LoadScenarioOk(ColonnadeJson);
	const std::string A = WriteViewerJson(First, RunScenario(First), ColonnadeSlug);
	const std::string B = WriteViewerJson(Second, RunScenario(Second), ColonnadeSlug);
	CHECK(A == B);
	CHECK(A.find("elapsed_ms") == std::string::npos);
}

TEST_CASE("Scenario.Viewer.TheScenarioExportIsTheSceneExport")
{
	Scenario S = LoadScenarioOk(ColonnadeJson);
	const Scenario Fresh = LoadScenarioOk(ColonnadeJson);
	const ScenarioOutcome O = RunScenario(S);
	ViewerScene V;
	V.Slug = std::string(ColonnadeSlug);
	V.Title = S.Title;
	V.Summary = S.Summary;
	V.Model = S.Model;
	for (std::size_t Slot = 0; Slot < S.BodyNames.size(); ++Slot)
	{
		const BodyId Id = Fresh.Find(Fresh.BodyNames[Slot]);
		V.Boxes.push_back(ViewerBox{Id, S.BodyNames[Slot], S.BodyMaterials[Slot], S.BodyBoxes[Slot].CentreM, S.BodyBoxes[Slot].HalfExtentsM, {1.0, 0.0, 0.0, 0.0},
			Fresh.Model.Body(Id).bGrounded, Fresh.Model.Body(Id).MassKg});
	}
	V.RemovedBeforeRun = {BodyOrFail(S, "post.2")};
	V.Gravity = Vec3{0.0, 0.0, -GravityMPerS2};
	V.Before = O.Before;
	V.After = O.Verdicts;
	CHECK(WriteViewerJson(S, O, ColonnadeSlug) == WriteViewerJson(V));
}

TEST_CASE("Scenario.Viewer.ARotatedBoxExportsItsRotation")
{
	ViewerScene V;
	V.Slug = "rotated-box";
	V.Title = "rotated box";
	const Result<BodyId> Body = V.Model.AddBody(BodyDesc{1.0, Vec3{0.0, 0.0, 1.0}, false});
	REQUIRE(Body.Ok());
	const double HalfAngleRad = 0.5 * 30.0 * 3.14159265358979323846 / 180.0;
	V.Boxes.push_back(ViewerBox{Body.Value, "voussoir", "stone", Vec3{0.0, 0.0, 1.0}, Vec3{0.2, 0.1, 0.05}, {std::cos(HalfAngleRad), 0.0, std::sin(HalfAngleRad), 0.0}, false, 1.0});
	const Parsed P = Parse(WriteViewerJson(V));
	REQUIRE(P.bOk);
	const detail::JsonValue* Box = BodyNamed(P.Root, "voussoir");
	REQUIRE(Box != nullptr);
	const std::vector<double> Q = Numbers(Field(*Box, "q"));
	REQUIRE(Q.size() == 4);
	CHECK(std::abs(Q[0] - std::cos(15.0 * 3.14159265358979323846 / 180.0)) <= 1e-12);
	CHECK(std::abs(Q[1]) <= 1e-12);
	CHECK(std::abs(Q[2] - std::sin(15.0 * 3.14159265358979323846 / 180.0)) <= 1e-12);
	CHECK(std::abs(Q[3]) <= 1e-12);
	CHECK(Numbers(Field(*Box, "h")) == std::vector<double>{0.2, 0.1, 0.05});
}

TEST_CASE("Scenario.Viewer.EveryClusterBodyCarriesTheRigidMotionOfItsCluster")
{
	std::ifstream File(std::string(LOADPATH_DATA_PATH) + "/scenarios/cantilever-light-counterweight.json", std::ios::binary);
	const std::string Text((std::istreambuf_iterator<char>(File)), std::istreambuf_iterator<char>());
	Scenario S = LoadScenarioOk(Text);
	const ScenarioOutcome O = RunScenario(S);
	const Parsed P = Parse(WriteViewerJson(S, O, "cantilever-light-counterweight"));
	REQUIRE(P.bOk);
	const auto CentreOf = [&](const std::string& Id) {
		for (const detail::JsonValue& Body : Field(P.Root, "bodies").Items)
		{
			if (std::to_string(static_cast<long long>(Field(Body, "id").Number)) == Id)
			{
				const std::vector<double> C = Numbers(Field(Body, "c"));
				return Vec3{C[0], C[1], C[2]};
			}
		}
		FAIL_CHECK("no body with id " << Id);
		return Vec3{};
	};
	const auto VecOf = [](const detail::JsonValue& Array) {
		const std::vector<double> V = Numbers(Array);
		REQUIRE(V.size() == 3);
		return Vec3{V[0], V[1], V[2]};
	};
	// T and r come from the run, not the export, so a writer spinning about the wrong reference point fails.
	const auto MotionOf = [&](const std::string& Ref) {
		for (const CascadeVerdict& Verdict : O.Verdicts)
		{
			for (const ReleasedCluster& Released : Verdict.Clusters)
			{
				if (std::to_string(Released.Motion.Body.Index) == Ref)
				{
					return Released.Motion;
				}
			}
		}
		FAIL_CHECK("no released cluster moves by body " << Ref);
		return MechanismMotion{};
	};
	int MultiBodyClusters = 0;
	for (const detail::JsonValue& Cluster : Field(Field(P.Root, "after"), "clusters").Items)
	{
		const detail::JsonValue& Motion = Field(Cluster, "motion");
		if (Motion.Items.size() < 2)
		{
			continue;
		}
		++MultiBodyClusters;
		const std::string Ref = std::to_string(static_cast<long long>(Field(Cluster, "motion_body").Number));
		const MechanismMotion Truth = MotionOf(Ref);
		const Vec3 TRef = Truth.TranslationM;
		const Vec3 R = Truth.RotationRad;
		const Vec3 CRef = CentreOf(Ref);
		for (std::size_t K = 0; K < Motion.Items.size(); ++K)
		{
			const std::string& Id = Motion.Keys[K];
			const detail::JsonValue& Entry = Motion.Items[K];
			const Vec3 C = CentreOf(Id);
			const Vec3 D{C.X - CRef.X, C.Y - CRef.Y, C.Z - CRef.Z};
			const Vec3 Want{TRef.X + R.Y * D.Z - R.Z * D.Y, TRef.Y + R.Z * D.X - R.X * D.Z, TRef.Z + R.X * D.Y - R.Y * D.X};
			const Vec3 T = VecOf(Field(Entry, "t"));
			const Vec3 Ri = VecOf(Field(Entry, "r"));
			INFO("body " << Id << " in the cluster of " << Ref << ": t " << Format(T) << " want " << Format(Want));
			CheckNear(T, Want, 1e-12);
			CheckNear(Ri, R, 1e-12);
		}
	}
	CHECK(MultiBodyClusters == 1);
}

TEST_CASE("Scenario.Viewer.PassesAreWrittenWhenRecordedAndOmittedWhenNot")
{
	Scenario Plain = LoadScenarioOk(ColonnadeJson);
	const ScenarioOutcome PlainOutcome = RunScenario(Plain);
	const Parsed P = Parse(WriteViewerJson(Plain, PlainOutcome, ColonnadeSlug));
	REQUIRE(P.bOk);
	CHECK_FALSE(HasKey(Field(P.Root, "after"), "pass_records"));
	CHECK(Field(Field(P.Root, "after"), "passes").Number == 1.0);

	Settings Tunables;
	Tunables.Debug.bRecordPasses = true;
	Scenario Recorded = LoadScenarioOk(ColonnadeJson);
	const ScenarioOutcome RecordedOutcome = RunScenario(Recorded, Tunables);
	const Parsed Q = Parse(WriteViewerJson(Recorded, RecordedOutcome, ColonnadeSlug));
	REQUIRE(Q.bOk);
	const detail::JsonValue& After = Field(Q.Root, "after");
	CHECK(Field(After, "passes").Number == 1.0);
	REQUIRE(HasKey(After, "pass_records"));
	const detail::JsonValue& Records = Field(After, "pass_records");
	std::size_t WantRecords = 0;
	for (const CascadeVerdict& Verdict : RecordedOutcome.Verdicts)
	{
		WantRecords += Verdict.PassRecords.size();
	}
	CHECK(WantRecords >= 3u);
	REQUIRE(Records.Items.size() == WantRecords);
	if (Records.Items.size() != WantRecords)
	{
		return;
	}
	std::vector<std::vector<double>> RecordedClusters;
	for (const detail::JsonValue& Record : Records.Items)
	{
		INFO("pass " << Field(Record, "pass").Number);
		CHECK(Field(Record, "pass").Number >= 1.0);
		const detail::JsonValue& Components = Field(Record, "components");
		REQUIRE_FALSE(Components.Items.empty());
		for (const detail::JsonValue& Component : Components.Items)
		{
			CHECK_FALSE(Field(Component, "bodies").Items.empty());
			CHECK_FALSE(Field(Component, "verdict").Text.empty());
			CHECK(Field(Component, "end").Text != "None");
			const detail::JsonValue& Inner = Field(Component, "inner");
			CHECK(Field(Component, "iterations").Number == static_cast<double>(Inner.Items.size()));
			for (const detail::JsonValue& Iteration : Inner.Items)
			{
				CHECK(Field(Iteration, "iteration").Number >= 1.0);
				for (const detail::JsonValue& Joint : Field(Iteration, "joints").Items)
				{
					CHECK(HasKey(Joint, "interface"));
					CHECK(Field(Joint, "corners").Items.size() == 4u);
					for (const detail::JsonValue& Corner : Field(Joint, "corners").Items)
					{
						CHECK((Corner.Text == "closed" || Corner.Text == "open" || Corner.Text == "slipping"));
					}
					CHECK(Field(Joint, "flipped").Items.size() == 4u);
					CHECK(Field(Joint, "corner_n").Items.size() == 4u);
					CHECK(Field(Joint, "corner_shear").Items.size() == 4u);
					CHECK(Field(Joint, "corner_shear").Items[0].Items.size() == 3u);
					CHECK(HasKey(Joint, "n"));
					CHECK(Field(Joint, "shear").Items.size() == 3u);
					CHECK(Field(Joint, "moment").Items.size() == 3u);
					CHECK(HasKey(Joint, "utilisation"));
					CHECK_FALSE(Field(Joint, "axis").Text.empty());
				}
			}
		}
		for (const detail::JsonValue& Reading : Field(Record, "readings").Items)
		{
			CHECK(HasKey(Reading, "interface"));
			CHECK(Field(Reading, "corners").Items.size() == 4u);
			CHECK(HasKey(Reading, "utilisation"));
		}
		for (const detail::JsonValue& Cluster : Field(Record, "clusters").Items)
		{
			CHECK(HasKey(Cluster, "motion"));
			std::vector<double> Bodies = Numbers(Field(Cluster, "bodies"));
			std::sort(Bodies.begin(), Bodies.end());
			RecordedClusters.push_back(std::move(Bodies));
		}
		for (const detail::JsonValue& Failure : Field(Record, "failures").Items)
		{
			CHECK(HasKey(Failure, "interface"));
			CHECK(HasKey(Failure, "severed"));
		}
	}
	std::sort(RecordedClusters.begin(), RecordedClusters.end());
	CHECK(RecordedClusters == ClusterBodies(Field(After, "clusters")));
}
