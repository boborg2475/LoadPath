#pragma once

#include "loadpath/Structure.h"

#include <string>
#include <string_view>
#include <vector>

namespace loadpath
{
// Line is 1-based; 0 means the error has no location (the document is not an object).
struct FileError
{
	int Line = 0;
	std::string Message;
};

// Names are indexed by slot: BodyNames[Id.Index] names body Id, and likewise for interfaces and profiles.
struct LoadedStructure
{
	Structure Model;
	std::vector<std::string> BodyNames;
	std::vector<std::string> InterfaceNames;
	std::vector<std::string> ProfileNames;
};

struct LoadResult
{
	LoadedStructure Value;
	FileError Error;

	bool Ok() const { return Error.Message.empty(); }
};

LoadResult LoadStructure(std::string_view Json);

// Removed bodies are not written; an interface is written, with its state and stamps, while both its bodies are live.
std::string WriteStructure(const LoadedStructure& Loaded);
}
