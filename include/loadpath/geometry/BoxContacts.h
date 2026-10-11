#pragma once

#include "loadpath/Handles.h"
#include "loadpath/Interface.h"
#include "loadpath/Vec3.h"

#include <optional>

namespace loadpath::geometry
{
struct Box
{
	Vec3 CentreM;
	Vec3 HalfExtentsM;
};

// The overlap rectangle of two axis-aligned boxes touching on one face within Tolerance; nullopt when they do not touch or overlap in volume.
std::optional<InterfaceDesc> FaceContact(BodyId A, const Box& BoxA, BodyId B, const Box& BoxB, ProfileId Profile, double ToleranceM = 1e-9);
}
