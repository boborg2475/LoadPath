#include "loadpath/Catalogue.h"

namespace loadpath
{
const JointRow* Catalogue::FindJoint(std::string_view) const
{
	return nullptr;
}

const FastenerRow* Catalogue::FindFastener(std::string_view) const
{
	return nullptr;
}

CatalogueResult LoadCatalogue(std::string_view)
{
	CatalogueResult Result;
	Result.Error.Message = "not implemented";
	return Result;
}

StrengthProfile FastenerProfile(const FastenerRow&, int, double)
{
	return StrengthProfile{};
}
}
