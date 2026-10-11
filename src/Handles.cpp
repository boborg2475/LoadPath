#include "loadpath/Handles.h"

namespace loadpath
{
bool BodyId::operator==(const BodyId& Other) const
{
	return Index == Other.Index && Generation == Other.Generation;
}

bool InterfaceId::operator==(const InterfaceId& Other) const
{
	return Index == Other.Index && Generation == Other.Generation;
}

bool ProfileId::operator==(const ProfileId& Other) const
{
	return Index == Other.Index && Generation == Other.Generation;
}
}
