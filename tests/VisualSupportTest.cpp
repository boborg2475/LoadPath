#include "JsonReader.h"
#include "ScenarioFilesSupport.h"
#include "ScenarioTestSupport.h"
#include "VisualSupport.h"

#include "loadpath/Scenario.h"
#include "loadpath/Viewer.h"

#include "doctest.h"

#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
struct ColonnadeRun
{
	Scenario Value;
	ScenarioOutcome Outcome;
};

ColonnadeRun RunColonnade()
{
	ColonnadeRun R{LoadScenarioOk(ColonnadeJson), {}};
	R.Outcome = RunScenario(R.Value);
	return R;
}

ViewerScene ColonnadeScene(const ColonnadeRun& R, std::string_view Slug, const std::vector<std::string>& SecondCluster)
{
	ViewerScene Scene = ViewerSceneOf(R.Value, R.Outcome, Slug);
	ViewerExpected Expected;
	Expected.State = "Stands";
	Expected.Clusters = {{BodyOrFail(R.Value, "beam.1")}};
	std::vector<BodyId> Second;
	for (const std::string& Name : SecondCluster)
	{
		Second.push_back(BodyOrFail(R.Value, Name));
	}
	Expected.Clusters.push_back(Second);
	Scene.Expected = Expected;
	return Scene;
}

bool HasKey(const std::string& Json, std::string_view Key)
{
	detail::JsonValue Root;
	detail::JsonParser Parser(Json);
	const bool bOk = Parser.ParseDocument(Root);
	CHECK(bOk);
	return bOk && detail::Find(Root, Key) != nullptr;
}

std::string TextField(const std::string& Json, std::string_view Key)
{
	detail::JsonValue Root;
	detail::JsonParser Parser(Json);
	if (!Parser.ParseDocument(Root))
	{
		return {};
	}
	const detail::JsonValue* Value = detail::Find(Root, Key);
	return Value != nullptr ? Value->Text : std::string();
}

bool AfterHasElapsed(const std::string& Json)
{
	detail::JsonValue Root;
	detail::JsonParser Parser(Json);
	if (!Parser.ParseDocument(Root))
	{
		return false;
	}
	const detail::JsonValue* After = detail::Find(Root, "after");
	return After != nullptr && detail::Find(*After, "elapsed_ms") != nullptr;
}

std::string WithTest(ViewerScene Scene, std::string_view TestName)
{
	Scene.Test = std::string(TestName);
	return WriteViewerJson(Scene);
}

bool Mentions(std::string_view Text, std::string_view Needle) { return Text.find(Needle) != std::string_view::npos; }
}

TEST_CASE("Visuals.Record.WritesNothingWhenTheVariableIsUnset")
{
	const ColonnadeRun R = RunColonnade();
	MemoryFiles Files;
	const std::string Error = RecordVisualTo(std::nullopt, Files, "E2E.A", ViewerSceneOf(R.Value, R.Outcome, ColonnadeSlug));
	CHECK(Error.empty());
	CHECK(Files.Calls() == 0);
	CHECK_FALSE(VisualsDirFrom(nullptr).has_value());
	CHECK_FALSE(VisualsDirFrom("").has_value());
	REQUIRE(VisualsDirFrom("C:\\v").has_value());
	CHECK(*VisualsDirFrom("C:\\v") == "C:\\v");
}

TEST_CASE("Visuals.Record.WritesOneEntryPerCallWithExpectedOnlyWhenItDiffers")
{
	const ColonnadeRun R = RunColonnade();
	MemoryFiles Files;
	const std::optional<std::string> Dir = "v";
	const ViewerScene Agrees = ColonnadeScene(R, "a", {"beam.2"});
	const ViewerScene Differs = ColonnadeScene(R, "b", {"post.1"});
	CHECK(RecordVisualTo(Dir, Files, "E2E.W2", Agrees).empty());
	CHECK(RecordVisualTo(Dir, Files, "E2E.W2", Differs).empty());
	REQUIRE(Files.Files.size() == 2);
	REQUIRE(Files.Files.count("v/a.json") == 1);
	REQUIRE(Files.Files.count("v/b.json") == 1);
	const std::string& A = Files.Files["v/a.json"];
	const std::string& B = Files.Files["v/b.json"];
	CHECK(A == WithTest(Agrees, "E2E.W2"));
	CHECK(B == WithTest(Differs, "E2E.W2"));
	for (const std::string* Json : {&A, &B})
	{
		CHECK(TextField(*Json, "test") == "E2E.W2");
		CHECK(HasKey(*Json, "slug"));
		CHECK(HasKey(*Json, "before"));
		CHECK(HasKey(*Json, "after"));
	}
	CHECK_FALSE(HasKey(A, "expected"));
	CHECK(HasKey(B, "expected"));
}

TEST_CASE("Visuals.Record.ASlugThatIsNotFileNameSafeIsAnError")
{
	const ColonnadeRun R = RunColonnade();
	const std::optional<std::string> Dir = "v";
	for (const char* Slug : {"", "Colonnade", "a b", "a/b", "a\\b", "..", "a_b", "a.json"})
	{
		INFO("slug '" << Slug << "'");
		MemoryFiles Files;
		const std::string Error = RecordVisualTo(Dir, Files, "E2E.Slugs", ViewerSceneOf(R.Value, R.Outcome, Slug));
		INFO("error: " << Error);
		CHECK_FALSE(Error.empty());
		CHECK(Mentions(Error, "'" + std::string(Slug) + "'"));
		CHECK(Mentions(Error, "E2E.Slugs"));
		CHECK(Files.Writes == 0);
	}
	MemoryFiles Files;
	CHECK(RecordVisualTo(Dir, Files, "E2E.Slugs", ViewerSceneOf(R.Value, R.Outcome, "arch-keystone-intact-2")).empty());
	CHECK(Files.Writes == 1);
	CHECK(Files.Files.count("v/arch-keystone-intact-2.json") == 1);
}

TEST_CASE("Visuals.Record.ASlugUsedByTwoTestsIsAnErrorNamingBoth")
{
	const ColonnadeRun R = RunColonnade();
	const std::optional<std::string> Dir = "v";
	const ViewerScene Scene = ViewerSceneOf(R.Value, R.Outcome, "x");

	MemoryFiles Shared;
	CHECK(RecordVisualTo(Dir, Shared, "E2E.A", Scene).empty());
	const std::string AsWritten = Shared.Files["v/x.json"];
	const std::string Error = RecordVisualTo(Dir, Shared, "E2E.B", Scene);
	INFO("error: " << Error);
	CHECK(Mentions(Error, "x"));
	CHECK(Mentions(Error, "E2E.A"));
	CHECK(Mentions(Error, "E2E.B"));
	CHECK(Shared.Files["v/x.json"] == AsWritten);
	CHECK(TextField(Shared.Files["v/x.json"], "test") == "E2E.A");

	MemoryFiles Repeated;
	CHECK(RecordVisualTo(Dir, Repeated, "E2E.A", Scene).empty());
	CHECK(RecordVisualTo(Dir, Repeated, "E2E.A", Scene).empty());
	CHECK(Repeated.Files.size() == 1);
}

TEST_CASE("Visuals.Record.AFailingTestStillWritesItsPicture")
{
	const ColonnadeRun R = RunColonnade();
	MemoryFiles Files;
	const std::string Error = RecordVisualTo(std::optional<std::string>("v"), Files, "E2E.Red", ColonnadeScene(R, "red-case", {"post.1"}));
	CHECK(Error.empty());
	REQUIRE(Files.Files.size() == 1);
	REQUIRE(Files.Files.count("v/red-case.json") == 1);
	CHECK(HasKey(Files.Files["v/red-case.json"], "expected"));
}

TEST_CASE("Visuals.Scenarios.TheGenericTestWritesOneEntryPerFile")
{
	MemoryFiles Folder;
	Folder.Files["scen/colonnade-middle-post.json"] = ColonnadeJson;
	Folder.Files["scen/colonnade-wrong-expectation.json"] = WrongExpectationJson();
	MemoryFiles Visuals;
	const std::vector<ScenarioFileRun> Runs =
		RunScenarioFolder(Folder, "scen", ScenarioRows(), ScenarioGroup::Default, std::optional<std::string>("v"), Visuals, DefaultScenariosTest);
	REQUIRE(Runs.size() == 2);
	REQUIRE(Visuals.Files.size() == 2);
	for (const ScenarioFileRun& Run : Runs)
	{
		INFO("file " << Run.File << ": " << Run.LoadError.Message << " " << Run.VisualError);
		REQUIRE(Run.bLoaded);
		CHECK(Run.VisualError.empty());
		const std::string Path = "v/" + Run.Slug + ".json";
		REQUIRE(Visuals.Files.count(Path) == 1);
		const std::string& Written = Visuals.Files[Path];
		ViewerScene Scene = ViewerSceneOf(Run.Value, Run.Outcome, Run.Slug);
		Scene.ElapsedMs = Run.Outcome.ElapsedMs;
		CHECK(Written == WithTest(Scene, DefaultScenariosTest));
		CHECK(TextField(Written, "test") == DefaultScenariosTest);
		CHECK(HasKey(Written, "after"));
		CHECK(AfterHasElapsed(Written));
		const bool bWrong = Run.Slug == WrongExpectationSlug;
		CHECK(HasKey(Written, "expected") == bWrong);
		CHECK(Run.Report.bMet == !bWrong);
	}
	CHECK(Runs[0].Slug == ColonnadeSlug);
	CHECK(Runs[1].Slug == WrongExpectationSlug);
}

TEST_CASE("Visuals.Record.AFreeBoxInsideAGroundedBoxIsAnErrorNamingBoth")
{
	const ColonnadeRun R = RunColonnade();
	const std::optional<std::string> Dir = "v";
	const auto Sunk = [&](std::string_view Slug, double DepthM) {
		ViewerScene Scene = ViewerSceneOf(R.Value, R.Outcome, Slug);
		for (ViewerBox& Box : Scene.Boxes)
		{
			if (Box.Name == "post.1")
			{
				Box.CentreM.Z -= DepthM;
			}
		}
		return Scene;
	};
	{
		MemoryFiles Files;
		const std::string Error = RecordVisualTo(Dir, Files, "E2E.Buried", Sunk("touching", 0.0));
		INFO(Error);
		CHECK(Error.empty());
		CHECK(Files.Files.count("v/touching.json") == 1);
	}
	{
		MemoryFiles Files;
		const std::string Error = RecordVisualTo(Dir, Files, "E2E.Buried", Sunk("within-tolerance", 0.5e-9));
		INFO(Error);
		CHECK(Error.empty());
	}
	{
		MemoryFiles Files;
		const std::string Error = RecordVisualTo(Dir, Files, "E2E.Buried", Sunk("buried", 1e-6));
		INFO(Error);
		CHECK(Mentions(Error, "post.1"));
		CHECK(Mentions(Error, "ground"));
		CHECK(Mentions(Error, "E2E.Buried"));
		CHECK(Files.Writes == 0);
	}
	/*
	 * Rotated boxes, as the arch voussoirs are. A (0.2, 0.1, 0.05) box turned 45 deg about y reaches
	 * (0.2 + 0.05) / sqrt 2 below its centre; the slab's top face is z = 0.
	 */
	constexpr double Pi = 3.14159265358979323846;
	const double S2 = std::sqrt(0.5);
	const auto Pair = [&](std::string_view Slug, const ViewerBox& Free) {
		ViewerScene Scene;
		Scene.Slug = std::string(Slug);
		Scene.Boxes.push_back(ViewerBox{BodyId{0}, "slab", "ground", Vec3{0.0, 0.0, -0.5}, Vec3{5.0, 5.0, 0.5}, {1.0, 0.0, 0.0, 0.0}, true, 0.0});
		Scene.Boxes.push_back(Free);
		return Scene;
	};
	const auto Tilted = [&](double CentreZ) {
		return ViewerBox{BodyId{1}, "tilted", "brick", Vec3{0.0, 0.0, CentreZ}, Vec3{0.2, 0.1, 0.05}, {std::cos(Pi / 8), 0.0, std::sin(Pi / 8), 0.0}, false, 1.0};
	};
	const double Reach = 0.25 * S2;
	{
		MemoryFiles Files;
		const std::string Error = RecordVisualTo(Dir, Files, "E2E.Rotated", Pair("rotated-sunk", Tilted(Reach - 0.001)));
		INFO(Error);
		CHECK(Mentions(Error, "tilted"));
		CHECK(Mentions(Error, "slab"));
	}
	{
		MemoryFiles Files;
		const std::string Error = RecordVisualTo(Dir, Files, "E2E.Rotated", Pair("rotated-lifted", Tilted(Reach + 0.002)));
		INFO(Error);
		CHECK(Error.empty());
	}
	{
		/*
		 * A grounded rod along y and a free rod along x, each a 0.2 m square turned 45 deg about its length, so
		 * the facing edges are 0.3 - 0.2 sqrt 2 = 17 mm apart along x cross y. The pair is then turned 50 deg
		 * about (1, 1, 1): only the edge-edge axis separates them, and their axis-aligned bounds overlap.
		 */
		using Quat = std::array<double, 4>;
		const auto Mul = [](const Quat& A, const Quat& B) {
			return Quat{A[0] * B[0] - A[1] * B[1] - A[2] * B[2] - A[3] * B[3], A[0] * B[1] + A[1] * B[0] + A[2] * B[3] - A[3] * B[2],
				A[0] * B[2] - A[1] * B[3] + A[2] * B[0] + A[3] * B[1], A[0] * B[3] + A[1] * B[2] - A[2] * B[1] + A[3] * B[0]};
		};
		const double Q = std::sin(Pi / 8), C = std::cos(Pi / 8);
		const double Half = 25.0 * Pi / 180.0, K = std::sin(Half) / std::sqrt(3.0);
		const Quat G{std::cos(Half), K, K, K};
		ViewerBox Lower{BodyId{0}, "lower", "ground", Vec3{0.0, 0.0, 0.0}, Vec3{0.1, 1.0, 0.1}, Mul(G, Quat{C, 0.0, Q, 0.0}), true, 0.0};
		ViewerBox Upper{BodyId{1}, "upper", "brick", Vec3{}, Vec3{1.0, 0.1, 0.1}, Mul(G, Quat{C, Q, 0.0, 0.0}), false, 1.0};
		ViewerBox Frame{BodyId{}, "", "", Vec3{}, Vec3{}, G, false, 0.0};
		const std::array<Vec3, 3> Gz = BoxAxes(Frame);
		Upper.CentreM = Vec3{0.3 * Gz[2].X, 0.3 * Gz[2].Y, 0.3 * Gz[2].Z};
		const auto Bounds = [](const ViewerBox& Box, int Axis) {
			const std::array<Vec3, 3> U = BoxAxes(Box);
			const std::array<double, 3> H{Box.HalfExtentsM.X, Box.HalfExtentsM.Y, Box.HalfExtentsM.Z};
			const auto Comp = [Axis](const Vec3& V) { return Axis == 0 ? V.X : (Axis == 1 ? V.Y : V.Z); };
			double Reach = 0.0;
			for (int I = 0; I < 3; ++I)
			{
				Reach += H[I] * std::abs(Comp(U[I]));
			}
			return std::pair<double, double>{Comp(Box.CentreM) - Reach, Comp(Box.CentreM) + Reach};
		};
		for (int Axis = 0; Axis < 3; ++Axis)
		{
			INFO("world axis " << Axis);
			CHECK(Bounds(Lower, Axis).second > Bounds(Upper, Axis).first);
			CHECK(Bounds(Upper, Axis).second > Bounds(Lower, Axis).first);
		}
		ViewerScene Scene;
		Scene.Slug = "rotated-edge-separated";
		Scene.Boxes = {Lower, Upper};
		MemoryFiles Files;
		const std::string Error = RecordVisualTo(Dir, Files, "E2E.Rotated", Scene);
		INFO(Error);
		CHECK(Error.empty());
		// The same pair 30 mm closer interpenetrates, so the separation above is the edge axis doing its job.
		Upper.CentreM = Vec3{0.27 * Gz[2].X, 0.27 * Gz[2].Y, 0.27 * Gz[2].Z};
		Scene.Boxes = {Lower, Upper};
		CHECK(Mentions(RecordVisualTo(Dir, Files, "E2E.Rotated", Scene), "upper"));
	}
}
