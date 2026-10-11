#pragma once

#include "loadpath/Body.h"
#include "loadpath/Handles.h"
#include "loadpath/Interface.h"
#include "loadpath/Result.h"
#include "loadpath/StrengthProfile.h"

#include <cstddef>
#include <vector>

namespace loadpath
{
// Handles stay valid across removal: removal tombstones the slot and bumps its generation, nothing renumbers.
class Structure
{
public:
	Result<ProfileId> AddProfile(const StrengthProfile& Desc);
	Result<BodyId> AddBody(const BodyDesc& Desc);
	Result<InterfaceId> AddInterface(const InterfaceDesc& Desc);

	// False for an unknown or already removed id.
	bool RemoveBody(BodyId Id);

	bool IsLive(BodyId Id) const;
	bool IsLive(InterfaceId Id) const;
	bool IsValid(ProfileId Id) const;

	// Precondition IsLive; a dead id reads a zero desc.
	const BodyDesc& Body(BodyId Id) const;
	// A removed body's interfaces stay readable here, in state Removed with no pass stamp.
	const InterfaceRecord& Interface(InterfaceId Id) const;
	const StrengthProfile& Profile(ProfileId Id) const;

	std::size_t LiveBodyCount() const;
	std::size_t LiveInterfaceCount() const;

	// Ascending slot index order, independent of removal history.
	std::vector<BodyId> LiveBodies() const;
	std::vector<InterfaceId> LiveInterfaces() const;
	std::vector<InterfaceId> InterfacesOf(BodyId Id) const;

private:
	BodyDesc DeadBody;
	InterfaceRecord DeadInterface;
	StrengthProfile DeadProfile;
};
}
