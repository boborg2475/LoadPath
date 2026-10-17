#pragma once

#include "VisualSupport.h"

#include "loadpath/Catalogue.h"
#include "loadpath/Cli.h"
#include "loadpath/Scenario.h"
#include "loadpath/StructureFile.h"
#include "loadpath/Viewer.h"

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace loadpath::test
{
inline constexpr std::string_view DefaultScenariosTest = "E2E.Scenarios.EveryScenarioFileMeetsItsExpectation";
inline constexpr std::string_view SlowInDebugScenariosTest = "E2E.Scenarios.EverySlowInDebugScenarioFileMeetsItsExpectation";
inline constexpr std::string_view SlowScenariosTest = "Slow.Scenarios.EverySlowScenarioFileMeetsItsExpectation";
inline constexpr std::string_view DeferredScenariosTest = "Deferred.Scenarios.EveryDeferredScenarioFileMeetsItsExpectation";

enum class ScenarioGroup
{
	Default,
	SlowInDebug,
	Slow,
	Deferred
};

struct ScenarioFileRun
{
	std::string File;
	std::string Slug;
	bool bLoaded = false;
	FileError LoadError;
	Scenario Value;
	ScenarioOutcome Outcome;
	ExpectationReport Report;
	std::string VisualError;
};

inline bool InGroup(const Scenario& S, ScenarioGroup Group)
{
	if (S.Deferred)
	{
		return Group == ScenarioGroup::Deferred;
	}
	switch (Group)
	{
	case ScenarioGroup::Default: return S.Tier == ScenarioTier::Default;
	case ScenarioGroup::SlowInDebug: return S.Tier == ScenarioTier::SlowInDebug;
	case ScenarioGroup::Slow: return S.Tier == ScenarioTier::Slow;
	case ScenarioGroup::Deferred: return false;
	}
	return false;
}

inline std::vector<std::string> JsonFiles(const std::vector<std::string>& Names)
{
	std::vector<std::string> Out;
	for (const std::string& Name : Names)
	{
		if (Name.size() > 5 && Name.ends_with(".json"))
		{
			Out.push_back(Name);
		}
	}
	return Out;
}

// Nullopt when the file loads but belongs to another group; a file that does not load is reported only to the default group.
inline std::optional<ScenarioFileRun> RunScenarioFile(const IFileStore& Files, std::string_view Dir, const std::string& File, const Catalogue& Rows, ScenarioGroup Group,
	const std::optional<std::string>& VisualsDir, IFileStore& VisualFiles, std::string_view TestName)
{
	ScenarioFileRun Run;
	Run.File = File;
	Run.Slug = File.substr(0, File.size() - 5);
	const std::optional<std::string> Text = Files.Read(std::string(Dir) + "/" + File);
	if (!Text)
	{
		Run.LoadError = {0, "cannot read " + std::string(Dir) + "/" + File};
		return Group == ScenarioGroup::Default ? std::optional<ScenarioFileRun>(std::move(Run)) : std::nullopt;
	}
	ScenarioLoadResult Loaded = LoadScenario(*Text, Rows);
	if (!Loaded.Ok())
	{
		Run.LoadError = Loaded.Error;
		return Group == ScenarioGroup::Default ? std::optional<ScenarioFileRun>(std::move(Run)) : std::nullopt;
	}
	if (!InGroup(Loaded.Value, Group))
	{
		return std::nullopt;
	}
	Run.bLoaded = true;
	Run.Value = std::move(Loaded.Value);
	Run.Outcome = RunScenario(Run.Value);
	if (Run.Value.Expect)
	{
		Run.Report = CompareExpectation(*Run.Value.Expect, Run.Outcome);
	}
	ViewerScene Scene = ViewerSceneOf(Run.Value, Run.Outcome, Run.Slug);
	Scene.ElapsedMs = Run.Outcome.ElapsedMs;
	Run.VisualError = RecordVisualTo(VisualsDir, VisualFiles, TestName, std::move(Scene));
	return Run;
}

inline std::vector<ScenarioFileRun> RunScenarioFolder(const IFileStore& Files, std::string_view Dir, const Catalogue& Rows, ScenarioGroup Group,
	const std::optional<std::string>& VisualsDir, IFileStore& VisualFiles, std::string_view TestName)
{
	std::vector<ScenarioFileRun> Runs;
	for (const std::string& File : JsonFiles(Files.List(Dir)))
	{
		std::optional<ScenarioFileRun> Run = RunScenarioFile(Files, Dir, File, Rows, Group, VisualsDir, VisualFiles, TestName);
		if (Run)
		{
			Runs.push_back(std::move(*Run));
		}
	}
	return Runs;
}

inline std::string ReportText(const ScenarioFileRun& Run)
{
	std::string Out = Run.File;
	if (Run.Value.Deferred)
	{
		Out += " (deferred: " + *Run.Value.Deferred + ")";
	}
	if (!Run.Outcome.RunError.empty())
	{
		Out += "\n  run error: " + Run.Outcome.RunError;
	}
	for (const std::string& Line : Run.Report.Lines)
	{
		Out += "\n  " + Line;
	}
	return Out;
}
}
