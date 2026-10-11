#include "loadpath/geometry/BoxContacts.h"

namespace loadpath::geometry
{
std::optional<InterfaceDesc> FaceContact(BodyId, const Box&, BodyId, const Box&, ProfileId, double)
{
	return std::nullopt;
}
}
