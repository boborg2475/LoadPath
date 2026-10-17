#include "ScenarioTestSupport.h"
#include "StructureFileTestSupport.h"
#include "VisualSupport.h"

#include "loadpath/Cli.h"
#include "loadpath/Scenario.h"
#include "loadpath/Viewer.h"

#include "doctest.h"

#include <algorithm>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
struct CliRun
{
	int Code = -1;
	std::string Out;
	std::string Err;
};

CliRun Run(std::vector<std::string_view> Args, std::optional<std::string_view> FileContents)
{
	CliRun R;
	R.Code = RunCli(Args, FileContents, R.Out, R.Err);
	return R;
}

std::string MissingMassDocument()
{
	return Build(Without(ExampleSpec(), 'b', 1, "mass_kg")).Json;
}

std::string NegativeMassDocument()
{
	return Build(WithValue(ExampleSpec(), 'b', 1, "mass_kg", "-1")).Json;
}

const char* const SyntaxErrorOnLine3 = "{\n  \"bodies\": [\n    { \"name\" \"b0\", \"mass_kg\": 1 }\n  ]\n}";
}

TEST_CASE("Cli.NoArgumentPrintsUsageAndReturns2")
{
	const CliRun R = Run({}, ExampleDocument);
	INFO("Err: " << R.Err);
	CHECK(R.Code == 2);
	CHECK(Contains(Lowercase(R.Err), "usage"));
	CHECK(R.Out.empty());
}

TEST_CASE("Cli.TwoArgumentsPrintUsageAndReturn2")
{
	const CliRun R = Run({"a.json", "b.json"}, ExampleDocument);
	INFO("Err: " << R.Err);
	CHECK(R.Code == 2);
	CHECK(Contains(Lowercase(R.Err), "usage"));
	CHECK(R.Out.empty());
}

TEST_CASE("Cli.UnreadableFileReturns2AndNamesTheFile")
{
	const CliRun R = Run({"missing_structure.json"}, std::nullopt);
	INFO("Err: " << R.Err);
	CHECK(R.Code == 2);
	CHECK(Contains(R.Err, "cannot read"));
	CHECK(Contains(R.Err, "missing_structure.json"));
	CHECK(R.Out.empty());
}

TEST_CASE("Cli.LoadErrorReturns1WithTheLinePrefixOnlyWhenLineIsPositive")
{
	const std::string Documents[] = {SyntaxErrorOnLine3, MissingMassDocument(), NegativeMassDocument(), "[]"};
	for (const std::string& Document : Documents)
	{
		const LoadResult Loaded = LoadStructure(Document);
		INFO("document:\n" << Document);
		REQUIRE_FALSE(Loaded.Ok());
		const std::string Expected =
			(Loaded.Error.Line > 0 ? "line " + std::to_string(Loaded.Error.Line) + ": " : std::string()) + Loaded.Error.Message + "\n";

		const CliRun R = Run({"structure.json"}, Document);
		CHECK(R.Code == 1);
		CHECK(R.Err == Expected);
		CHECK(R.Out.empty());
	}

	const CliRun Syntax = Run({"structure.json"}, std::string_view(SyntaxErrorOnLine3));
	INFO("Err: " << Syntax.Err);
	CHECK(Syntax.Err.starts_with("line 3: "));
}

TEST_CASE("Cli.SuccessPrintsTheCountsAndReturns0")
{
	const CliRun R = Run({"example.json"}, ExampleDocument);
	INFO("Err: " << R.Err);
	CHECK(R.Code == 0);
	CHECK(R.Out == "profiles 1  bodies 2  interfaces 1\n");
	CHECK(R.Err.empty());
}

namespace
{
const std::string ScenarioPath = "data/scenarios/colonnade-middle-post.json";
const std::string CataloguePath = "data/catalogue.json";

MemoryFiles ScenarioFiles(const std::string& Path, const std::string& Json)
{
	MemoryFiles Files;
	Files.Files[Path] = Json;
	Files.Files[CataloguePath] = ReadCatalogueFile();
	return Files;
}

CliRun RunWith(std::vector<std::string_view> Args, IFileStore& Files)
{
	CliRun R;
	R.Code = RunCli(Args, Files, R.Out, R.Err);
	return R;
}

const std::string OakOnLine7 = "{\n"
							   "  \"title\": \"oak\",\n"
							   "  \"build\": [\n"
							   "    { \"name\": \"ground\", \"size\": [1, 1, 0.2], \"centre\": [0, 0, -0.1], \"grounded\": true },\n"
							   "    { \"name\": \"one\", \"size\": [0.2, 0.2, 0.2], \"centre\": [0, 0, 5], \"material\": \"c24\" },\n"
							   "    { \"name\": \"two\", \"size\": [0.2, 0.2, 0.2], \"centre\": [0, 0, 7], \"material\": \"c24\" },\n"
							   "    { \"name\": \"plank\", \"size\": [0.2, 0.2, 0.2], \"centre\": [0, 0, 9], \"material\": \"oak\" }\n"
							   "  ]\n"
							   "}\n";
}

TEST_CASE("Cli.Scenario.AMetScenarioPrintsTheStepsAndReturns0")
{
	MemoryFiles Files = ScenarioFiles(ScenarioPath, ColonnadeJson);
	const CliRun R = RunWith({"scenario", ScenarioPath}, Files);
	INFO("Out:\n" << R.Out << "Err:\n" << R.Err);
	CHECK(R.Code == 0);
	CHECK(Contains(R.Out, "built 10 bodies (1 grounded), 13 joints"));
	CHECK(Contains(R.Out, "step 3 cascade: Stands, 1 pass, released {beam.1} {beam.2}"));
	CHECK(Contains(R.Out, "expectation met"));
	CHECK(R.Err.empty());
}

TEST_CASE("Cli.Scenario.AnUnmetScenarioPrintsTheMismatchAndReturns1")
{
	const std::string Path = "data/scenarios/colonnade-wrong-expectation.json";
	MemoryFiles Files = ScenarioFiles(Path, WrongExpectationJson());
	const CliRun R = RunWith({"scenario", Path}, Files);
	INFO("Out:\n" << R.Out << "Err:\n" << R.Err);
	CHECK(R.Code == 1);
	CHECK(Contains(R.Out, "expectation not met"));
	CHECK(Contains(R.Out, "released, not expected: beam.2"));
	CHECK(Contains(R.Out, "expected to fall, still standing: post.1"));
	CHECK(Contains(R.Out, "expected {beam.1} {post.1}; engine {beam.1} {beam.2}"));
}

TEST_CASE("Cli.Scenario.ABadFileReturns2WithTheFileAndLine")
{
	{
		MemoryFiles Files;
		Files.Files["x.json"] = OakOnLine7;
		Files.Files["cat.json"] = ReadCatalogueFile();
		const CliRun R = RunWith({"scenario", "x.json", "--catalogue", "cat.json"}, Files);
		INFO("Out:\n" << R.Out << "Err:\n" << R.Err);
		CHECK(R.Code == 2);
		CHECK(R.Err == "x.json:7: unknown material 'oak'\n");
	}
	{
		MemoryFiles Files;
		Files.Files["cat.json"] = ReadCatalogueFile();
		const CliRun R = RunWith({"scenario", "x.json", "--catalogue", "cat.json"}, Files);
		INFO("Out:\n" << R.Out << "Err:\n" << R.Err);
		CHECK(R.Code == 2);
		CHECK(R.Err == "cannot read x.json\n");
	}
	{
		MemoryFiles Files;
		Files.Files["data/scenarios/x.json"] = ColonnadeJson;
		const CliRun R = RunWith({"scenario", "data/scenarios/x.json"}, Files);
		INFO("Out:\n" << R.Out << "Err:\n" << R.Err);
		CHECK(R.Code == 2);
		CHECK(R.Err == "cannot read data/catalogue.json\n");
	}
}

TEST_CASE("Cli.Scenario.VizWritesTheViewerJson")
{
	MemoryFiles Plain = ScenarioFiles(ScenarioPath, ColonnadeJson);
	const CliRun Without = RunWith({"scenario", ScenarioPath}, Plain);
	MemoryFiles Files = ScenarioFiles(ScenarioPath, ColonnadeJson);
	const CliRun R = RunWith({"scenario", ScenarioPath, "--viz", "out.json"}, Files);
	INFO("Out:\n" << R.Out << "Err:\n" << R.Err);
	CHECK(R.Code == Without.Code);
	REQUIRE(Files.Files.count("out.json") == 1);
	Scenario S = LoadScenarioOk(ColonnadeJson);
	const ScenarioOutcome O = RunScenario(S);
	CHECK(Files.Files["out.json"] == WriteViewerJson(S, O, ColonnadeSlug));
}

TEST_CASE("Cli.Scenario.UsageErrorsReturn2")
{
	const std::vector<std::vector<std::string_view>> ArgLists = {
		{"scenario"},
		{"scenario", ScenarioPath, "--viz"},
		{"scenario", ScenarioPath, "--bogus"},
	};
	for (const std::vector<std::string_view>& Args : ArgLists)
	{
		MemoryFiles Files = ScenarioFiles(ScenarioPath, ColonnadeJson);
		const CliRun R = RunWith(Args, Files);
		INFO("args " << Args.size() << "; Err: " << R.Err);
		CHECK(R.Code == 2);
		CHECK(Contains(Lowercase(R.Err), "usage"));
		CHECK(Contains(Lowercase(R.Err), "scenario"));
	}
	MemoryFiles Files;
	Files.Files["example.json"] = ExampleDocument;
	const CliRun StructureRun = RunWith({"example.json"}, Files);
	INFO("Err: " << StructureRun.Err);
	CHECK(StructureRun.Code == 0);
	CHECK(StructureRun.Out == "profiles 1  bodies 2  interfaces 1\n");
}

TEST_CASE("Cli.Scenario.TheCatalogueDefaultsToTheDataFolder")
{
	{
		MemoryFiles Files = ScenarioFiles("data/scenarios/x.json", ColonnadeJson);
		const CliRun R = RunWith({"scenario", "data/scenarios/x.json"}, Files);
		INFO("Err: " << R.Err);
		CHECK(R.Code == 0);
		CHECK(std::find(Files.ReadPaths.begin(), Files.ReadPaths.end(), "data/catalogue.json") != Files.ReadPaths.end());
	}
	{
		MemoryFiles Files;
		Files.Files["data/scenarios/x.json"] = ColonnadeJson;
		Files.Files["elsewhere/cat.json"] = ReadCatalogueFile();
		const CliRun R = RunWith({"scenario", "data/scenarios/x.json", "--catalogue", "elsewhere/cat.json"}, Files);
		INFO("Err: " << R.Err);
		CHECK(R.Code == 0);
		CHECK(std::find(Files.ReadPaths.begin(), Files.ReadPaths.end(), "elsewhere/cat.json") != Files.ReadPaths.end());
		CHECK(std::find(Files.ReadPaths.begin(), Files.ReadPaths.end(), "data/catalogue.json") == Files.ReadPaths.end());
	}
}

TEST_CASE("Cli.Scenario.ASketchWithNoExpectationPrintsTheOutcome")
{
	const std::string Sketch = Replaced(ColonnadeJson, ",\n  \"expect\": { \"released\": [[\"beam.1\"], [\"beam.2\"]] }", "");
	{
		MemoryFiles Files = ScenarioFiles(ScenarioPath, Sketch);
		const CliRun R = RunWith({"scenario", ScenarioPath}, Files);
		INFO("Out:\n" << R.Out << "Err:\n" << R.Err);
		CHECK(R.Code == 0);
		CHECK(Contains(R.Out, "step 3 cascade: Stands, 1 pass, released {beam.1} {beam.2}"));
		CHECK(Contains(R.Out, "no expectation"));
		CHECK_FALSE(Contains(R.Out, "expectation met"));
		CHECK(R.Err.empty());
	}
	{
		const std::string Broken = Replaced(Sketch, "\"cascade\"]", "\"cascade\", { \"remove\": [\"beam.1\"] }]");
		MemoryFiles Files = ScenarioFiles(ScenarioPath, Broken);
		const CliRun R = RunWith({"scenario", ScenarioPath}, Files);
		INFO("Out:\n" << R.Out << "Err:\n" << R.Err);
		CHECK(R.Code == 1);
		CHECK(Contains(R.Out, "run error: "));
		CHECK(Contains(R.Out, "beam.1"));
		CHECK_FALSE(Contains(R.Out, "no expectation"));
	}
}

TEST_CASE("Cli.Scenario.AVizPathThatCannotBeWrittenReturns2")
{
	struct ReadOnlyFiles final : public IFileStore
	{
		MemoryFiles Inner;
		std::optional<std::string> Read(std::string_view Path) const override { return Inner.Read(Path); }
		bool Write(std::string_view, std::string_view) override { return false; }
		std::vector<std::string> List(std::string_view Dir) const override { return Inner.List(Dir); }
	};
	ReadOnlyFiles Files;
	Files.Inner = ScenarioFiles(ScenarioPath, ColonnadeJson);
	const CliRun R = RunWith({"scenario", ScenarioPath, "--viz", "locked/out.json"}, Files);
	INFO("Out:\n" << R.Out << "Err:\n" << R.Err);
	CHECK(R.Code == 2);
	CHECK(R.Err == "cannot write locked/out.json\n");
}

TEST_CASE("Cli.Scenario.ACatalogueThatDoesNotLoadReturns2WithItsPath")
{
	MemoryFiles Files;
	Files.Files["x.json"] = ColonnadeJson;
	Files.Files["cat.json"] = "{\n  \"materials\": 3\n}\n";
	const CliRun R = RunWith({"scenario", "x.json", "--catalogue", "cat.json"}, Files);
	INFO("Out:\n" << R.Out << "Err:\n" << R.Err);
	CHECK(R.Code == 2);
	CHECK(R.Err.starts_with("cat.json:"));
	CHECK(R.Out.empty());
}
