#include "ScenarioFilesSupport.h"
#include "ScenarioTestSupport.h"
#include "SlowInDebug.h"
#include "VisualSupport.h"

#include "doctest.h"

#include <optional>
#include <string>
#include <string_view>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
void CheckScenarioFile(const std::string& File, ScenarioGroup Group, std::string_view TestName)
{
	const DiskFiles Files;
	DiskFiles VisualFiles;
	const std::optional<ScenarioFileRun> Run =
		RunScenarioFile(Files, LOADPATH_SCENARIO_DIR, File, ScenarioRows(), Group, VisualsDirFromEnvironment(), VisualFiles, TestName);
	if (!Run)
	{
		return;
	}
	CHECK_MESSAGE(Run->bLoaded, File << ":" << Run->LoadError.Line << ": " << Run->LoadError.Message);
	if (!Run->bLoaded)
	{
		return;
	}
	CHECK_MESSAGE(Run->VisualError.empty(), Run->VisualError);
	CHECK_MESSAGE(Run->Value.Expect.has_value(), File << ": a file under data/scenarios needs an expect block");
	CHECK_MESSAGE(Run->Report.bMet, ReportText(*Run));
}

std::vector<std::string> ScenarioFileNames()
{
	const DiskFiles Files;
	return JsonFiles(Files.List(LOADPATH_SCENARIO_DIR));
}

void CheckEveryFile(ScenarioGroup Group, std::string_view TestName)
{
	for (const std::string& File : ScenarioFileNames())
	{
		SUBCASE(File.c_str())
		{
			CheckScenarioFile(File, Group, TestName);
		}
	}
}
}

TEST_CASE("E2E.Scenarios.EveryScenarioFileMeetsItsExpectation")
{
	CHECK_MESSAGE(!ScenarioFileNames().empty(), "no scenario files in " LOADPATH_SCENARIO_DIR);
	CheckEveryFile(ScenarioGroup::Default, DefaultScenariosTest);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "E2E.Scenarios.EverySlowInDebugScenarioFileMeetsItsExpectation")
{
	CheckEveryFile(ScenarioGroup::SlowInDebug, CurrentTestName());
}

TEST_CASE("Slow.Scenarios.EverySlowScenarioFileMeetsItsExpectation")
{
	CheckEveryFile(ScenarioGroup::Slow, SlowScenariosTest);
}

TEST_CASE("Deferred.Scenarios.EveryDeferredScenarioFileMeetsItsExpectation")
{
	CheckEveryFile(ScenarioGroup::Deferred, DeferredScenariosTest);
}

TEST_CASE("E2E.Scenarios.FileNamesAreTheirSlugs")
{
	const DiskFiles Files;
	const std::vector<std::string> Names = Files.List(LOADPATH_SCENARIO_DIR);
	CHECK_MESSAGE(!Names.empty(), "no files in " LOADPATH_SCENARIO_DIR);
	for (const std::string& Name : Names)
	{
		INFO("file " << Name);
		CHECK(Name.ends_with(".json"));
		CHECK(Name.size() > 5);
		CHECK(IsSlug(std::string_view(Name).substr(0, Name.size() > 5 ? Name.size() - 5 : 0)));
	}
}
