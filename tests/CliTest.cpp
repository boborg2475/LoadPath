#include "StructureFileTestSupport.h"

#include "loadpath/Cli.h"

#include "doctest.h"

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
