#include "loadpath/Cli.h"

#include "loadpath/Catalogue.h"
#include "loadpath/Scenario.h"
#include "loadpath/StructureFile.h"

#include <algorithm>
#include <cstddef>

namespace loadpath
{
namespace
{
constexpr std::string_view Usage = "usage: loadpath_cli <structure.json>\n"
								   "       loadpath_cli scenario <scenario.json> [--viz <out.json>] [--catalogue <catalogue.json>]\n";

std::string DefaultCataloguePath(std::string_view ScenarioPath)
{
	const std::size_t Slash = ScenarioPath.find_last_of("/\\");
	if (Slash == std::string_view::npos)
	{
		return "../catalogue.json";
	}
	const std::string_view Dir = ScenarioPath.substr(0, Slash);
	const std::size_t Parent = Dir.find_last_of("/\\");
	return Parent == std::string_view::npos ? std::string("catalogue.json") : std::string(Dir.substr(0, Parent + 1)) + "catalogue.json";
}

std::string SlugOf(std::string_view Path)
{
	const std::size_t Slash = Path.find_last_of("/\\");
	std::string_view Name = Slash == std::string_view::npos ? Path : Path.substr(Slash + 1);
	if (Name.ends_with(".json"))
	{
		Name.remove_suffix(5);
	}
	return std::string(Name);
}

std::string ClustersText(const std::vector<std::vector<std::string>>& Clusters)
{
	std::string Out;
	for (const std::vector<std::string>& Cluster : Clusters)
	{
		Out += Out.empty() ? "{" : " {";
		for (std::size_t I = 0; I < Cluster.size(); ++I)
		{
			Out += (I == 0 ? "" : ", ") + Cluster[I];
		}
		Out += "}";
	}
	return Out;
}

std::string StepText(const ScenarioStepRecord& Step)
{
	if (Step.Kind == "cascade")
	{
		std::string Text = Step.State + ", " + std::to_string(Step.Passes) + (Step.Passes == 1 ? " pass" : " passes");
		return Step.Released.empty() ? Text : Text + ", released " + ClustersText(Step.Released);
	}
	if (Step.Kind == "judge")
	{
		return Step.State;
	}
	std::string Text;
	for (const std::string& Name : Step.Removed)
	{
		Text += (Text.empty() ? "" : ", ") + Name;
	}
	return Text.empty() ? "done" : Text;
}

struct ScenarioArgs
{
	std::string File, Viz, Catalogue;
	bool bOk = false;
};

ScenarioArgs ParseScenarioArgs(std::span<const std::string_view> Args)
{
	ScenarioArgs Parsed;
	if (Args.size() < 2 || Args[1].starts_with("--"))
	{
		return Parsed;
	}
	Parsed.File = std::string(Args[1]);
	for (std::size_t I = 2; I < Args.size(); I += 2)
	{
		if (I + 1 >= Args.size() || (Args[I] != "--viz" && Args[I] != "--catalogue"))
		{
			return Parsed;
		}
		(Args[I] == "--viz" ? Parsed.Viz : Parsed.Catalogue) = std::string(Args[I + 1]);
	}
	if (Parsed.Catalogue.empty())
	{
		Parsed.Catalogue = DefaultCataloguePath(Parsed.File);
	}
	Parsed.bOk = true;
	return Parsed;
}

std::string Located(const std::string& Path, const FileError& Error)
{
	return Path + (Error.Line > 0 ? ":" + std::to_string(Error.Line) : std::string()) + ": " + Error.Message + "\n";
}

int RunScenarioCommand(std::span<const std::string_view> Args, IFileStore& Files, std::string& Out, std::string& Err)
{
	const ScenarioArgs Parsed = ParseScenarioArgs(Args);
	if (!Parsed.bOk)
	{
		Err += Usage;
		return 2;
	}
	const std::optional<std::string> Text = Files.Read(Parsed.File);
	if (!Text)
	{
		Err += "cannot read " + Parsed.File + "\n";
		return 2;
	}
	const std::optional<std::string> CatalogueText = Files.Read(Parsed.Catalogue);
	if (!CatalogueText)
	{
		Err += "cannot read " + Parsed.Catalogue + "\n";
		return 2;
	}
	const CatalogueResult Rows = LoadCatalogue(*CatalogueText);
	if (!Rows.Ok())
	{
		Err += Located(Parsed.Catalogue, Rows.Error);
		return 2;
	}
	ScenarioLoadResult Loaded = LoadScenario(*Text, Rows.Value);
	if (!Loaded.Ok())
	{
		Err += Located(Parsed.File, Loaded.Error);
		return 2;
	}
	Scenario& Value = Loaded.Value;
	const auto Grounded = std::count(Value.BodyGrounded.begin(), Value.BodyGrounded.end(), true);
	Out += Value.Title + "\n";
	Out += "built " + std::to_string(Value.BodyNames.size()) + " bodies (" + std::to_string(Grounded) + " grounded), " +
		std::to_string(Value.Model.LiveInterfaceCount()) + " joints\n";
	const ScenarioOutcome Outcome = RunScenario(Value);
	for (std::size_t I = 0; I < Outcome.Steps.size(); ++I)
	{
		Out += "step " + std::to_string(I + 1) + " " + Outcome.Steps[I].Kind + ": " + StepText(Outcome.Steps[I]) + "\n";
	}
	if (!Parsed.Viz.empty() && !Files.Write(Parsed.Viz, WriteViewerJson(Value, Outcome, SlugOf(Parsed.File))))
	{
		Err += "cannot write " + Parsed.Viz + "\n";
		return 2;
	}
	if (!Value.Expect)
	{
		if (!Outcome.RunError.empty())
		{
			Out += "run error: " + Outcome.RunError + "\n";
			return 1;
		}
		Out += "no expectation\n";
		return 0;
	}
	const ExpectationReport Report = CompareExpectation(*Value.Expect, Outcome);
	Out += Report.bMet ? "expectation met\n" : "expectation not met\n";
	for (const std::string& Line : Report.Lines)
	{
		Out += "  " + Line + "\n";
	}
	return Report.bMet ? 0 : 1;
}
}

int RunCli(std::span<const std::string_view> Args, std::optional<std::string_view> FileContents, std::string& Out, std::string& Err)
{
	if (Args.size() != 1)
	{
		Err += "usage: loadpath_cli <structure.json>\n";
		return 2;
	}
	if (!FileContents)
	{
		Err += "cannot read " + std::string(Args[0]) + "\n";
		return 2;
	}
	const LoadResult Loaded = LoadStructure(*FileContents);
	if (!Loaded.Ok())
	{
		if (Loaded.Error.Line > 0)
		{
			Err += "line " + std::to_string(Loaded.Error.Line) + ": ";
		}
		Err += Loaded.Error.Message + "\n";
		return 1;
	}
	const Structure& Model = Loaded.Value.Model;
	Out += "profiles " + std::to_string(Loaded.Value.ProfileNames.size()) + "  bodies " + std::to_string(Model.LiveBodyCount()) +
		"  interfaces " + std::to_string(Model.LiveInterfaceCount()) + "\n";
	return 0;
}

int RunCli(std::span<const std::string_view> Args, IFileStore& Files, std::string& Out, std::string& Err)
{
	if (!Args.empty() && Args[0] == "scenario")
	{
		return RunScenarioCommand(Args, Files, Out, Err);
	}
	if (Args.size() != 1)
	{
		Err += Usage;
		return 2;
	}
	const std::optional<std::string> Contents = Files.Read(Args[0]);
	return RunCli(Args, Contents ? std::optional<std::string_view>(*Contents) : std::nullopt, Out, Err);
}
}
