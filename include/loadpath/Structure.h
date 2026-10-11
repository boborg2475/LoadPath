#pragma once

#include "loadpath/Body.h"
#include "loadpath/Handles.h"
#include "loadpath/Interface.h"
#include "loadpath/Result.h"
#include "loadpath/StrengthProfile.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace loadpath
{
// Handles stay valid across removal: removal tombstones the slot, slots are never reused and nothing renumbers.
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

	// A dead or unknown id reads a NaN mass, never zero, so it fails closed.
	const BodyDesc& Body(BodyId Id) const;
	// A removed body's interfaces stay readable here, in state Removed with no pass stamp.
	// An unknown id reads state Removed with zero area.
	const InterfaceRecord& Interface(InterfaceId Id) const;
	// An unknown id reads NaN strengths.
	const StrengthProfile& Profile(ProfileId Id) const;

	std::size_t LiveBodyCount() const;
	std::size_t LiveInterfaceCount() const;

	// Ascending slot index order, independent of removal history.
	std::vector<BodyId> LiveBodies() const;
	std::vector<InterfaceId> LiveInterfaces() const;
	std::vector<InterfaceId> InterfacesOf(BodyId Id) const;

private:
	struct BodySlot
	{
		BodyDesc Desc;
		bool bLive = false;
	};

	struct InterfaceSlot
	{
		InterfaceRecord Record;
	};

	// Known means the handle was issued at some point, whether or not it is still live.
	bool IsKnown(BodyId Id) const;
	bool IsKnown(InterfaceId Id) const;
	ModelError ValidateInterface(const InterfaceDesc& Desc, Vec3& UnitNormal) const;

	std::vector<StrengthProfile> Profiles;
	std::vector<BodySlot> Bodies;
	std::vector<InterfaceSlot> Interfaces;
};
}
