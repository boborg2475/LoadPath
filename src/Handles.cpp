#include "loadpath/Handles.h"

namespace loadpath
{
bool BodyId::operator==(const BodyId&) const
{
	return false;
}

bool InterfaceId::operator==(const InterfaceId&) const
{
	return false;
}

bool ProfileId::operator==(const ProfileId&) const
{
	return false;
}
}
