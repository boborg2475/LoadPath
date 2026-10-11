#include "loadpath/StructureFile.h"

namespace loadpath
{
LoadResult LoadStructure(std::string_view)
{
	LoadResult Result;
	Result.Error.Message = "not implemented";
	return Result;
}

std::string WriteStructure(const LoadedStructure&)
{
	return {};
}
}
