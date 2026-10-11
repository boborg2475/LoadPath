#include "loadpath/Structure.h"

namespace loadpath
{
Result<ProfileId> Structure::AddProfile(const StrengthProfile&)
{
	return {};
}

Result<BodyId> Structure::AddBody(const BodyDesc&)
{
	return {};
}

Result<InterfaceId> Structure::AddInterface(const InterfaceDesc&)
{
	return {};
}

bool Structure::RemoveBody(BodyId)
{
	return false;
}

bool Structure::IsLive(BodyId) const
{
	return false;
}

bool Structure::IsLive(InterfaceId) const
{
	return false;
}

bool Structure::IsValid(ProfileId) const
{
	return false;
}

const BodyDesc& Structure::Body(BodyId) const
{
	return DeadBody;
}

const InterfaceRecord& Structure::Interface(InterfaceId) const
{
	return DeadInterface;
}

const StrengthProfile& Structure::Profile(ProfileId) const
{
	return DeadProfile;
}

std::size_t Structure::LiveBodyCount() const
{
	return 0;
}

std::size_t Structure::LiveInterfaceCount() const
{
	return 0;
}

std::vector<BodyId> Structure::LiveBodies() const
{
	return {};
}

std::vector<InterfaceId> Structure::LiveInterfaces() const
{
	return {};
}

std::vector<InterfaceId> Structure::InterfacesOf(BodyId) const
{
	return {};
}
}
