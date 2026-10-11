#include "loadpath/Cli.h"

#include "loadpath/StructureFile.h"

namespace loadpath
{
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
}
