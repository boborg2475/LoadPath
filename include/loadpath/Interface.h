#pragma once

#include "loadpath/Handles.h"
#include "loadpath/Vec3.h"

#include <cstdint>

namespace loadpath
{
struct InterfaceDesc
{
	BodyId BodyA;
	BodyId BodyB;
	// The axis of separation pointing from A into B, never the centroid direction; normalised on add.
	Vec3 NormalAToB;
	Vec3 CentreM;
	// In-plane unit axes; U x V must point along the normal.
	Vec3 AxisU;
	Vec3 AxisV;
	double HalfExtentUM = 0;
	double HalfExtentVM = 0;
	ProfileId Profile;
};

enum class InterfaceState : std::uint8_t
{
	Intact,
	Cracked,
	Severed,
	Removed
};

struct InterfaceRecord
{
	BodyId BodyA;
	BodyId BodyB;
	Vec3 NormalAToB;
	Vec3 CentreM;
	Vec3 AxisU;
	Vec3 AxisV;
	double HalfExtentUM = 0;
	double HalfExtentVM = 0;
	double AreaM2 = 0;
	ProfileId Profile;
	InterfaceState State = InterfaceState::Intact;
	// Pass stamps, 0 = never; a removed body's interfaces go to Removed with no stamp.
	std::uint32_t CrackPass = 0;
	std::uint32_t SeverPass = 0;
};
}
