#pragma once

#include "loadpath/Body.h"
#include "loadpath/ComponentPartition.h"
#include "loadpath/Handles.h"
#include "loadpath/Interface.h"
#include "loadpath/Result.h"
#include "loadpath/StrengthProfile.h"

#include <cstddef>
#include <cstdint>
#include <optional>
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

	// Intact to Cracked, stamping CrackPass; false and no change otherwise or for pass 0.
	bool Crack(InterfaceId Id, std::uint32_t Pass);
	// Intact or Cracked to Severed, stamping SeverPass; false and no change otherwise, for pass 0 or below CrackPass.
	bool Sever(InterfaceId Id, std::uint32_t Pass);
	// The pass counter: the largest stamp written or pass marked, 0 when none.
	std::uint32_t HighestPass() const;
	// Raises the counter to max(counter, Pass) without stamping; true for any Pass >= 1, false for 0.
	bool MarkPass(std::uint32_t Pass);
	// Tension and cohesion zeroed unless Intact, so a dead joint reads the weaker form; an unknown id reads the NaN profile.
	StrengthProfile EffectiveProfile(InterfaceId Id) const;

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

	// Cached until the next AddBody, AddInterface, RemoveBody, Crack or Sever; repeated calls in between return the same object.
	const ComponentPartition& Components() const;

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
	// Tracked, never recomputed from records that removal may have rewritten.
	std::uint32_t HighestStamp = 0;
	mutable std::optional<ComponentPartition> CachedComponents;
};
}
