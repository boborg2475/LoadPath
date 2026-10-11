#pragma once

#include "loadpath/Handles.h"
#include "loadpath/Structure.h"
#include "loadpath/Units.h"
#include "loadpath/Vec3.h"

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace loadpath
{
// A point force; its moment about the body's centroid is (PointM - CentroidM) x ForceN.
struct BodyLoad
{
	BodyId Body;
	Vec3 ForceN;
	Vec3 PointM;
};

struct StandingOptions
{
	// A tilt test rotates this vector; a lateral load factor adds a horizontal component.
	Vec3 Gravity = {0, 0, -GravityMPerS2};
	int MaxActiveSetIterations = 50;
	// A pivot below this times the largest pivot is a mechanism.
	double PivotToleranceRelative = 1e-10;
};

enum class StandingState : std::uint8_t
{
	Stands,
	Overstressed,
	Mechanism,
	Undecided,
	InvalidInput
};

enum class GoverningAxis : std::uint8_t
{
	None,
	Compression,
	Tension,
	Shear
};

// Normal force is compression positive and acts on body B of the interface.
struct CornerReading
{
	Vec3 PointM;
	double NormalForceN = 0;
	Vec3 ShearForceN;
	bool bOpen = false;
	bool bSlipping = false;
};

struct InterfaceReading
{
	InterfaceId Id;
	double Utilisation = 0;
	GoverningAxis Axis = GoverningAxis::None;
	// Resultants of the force on B, about the rectangle centre.
	double NormalForceN = 0;
	Vec3 ShearForceN;
	Vec3 MomentNm;
	// Order: (+u,+v), (-u,+v), (-u,-v), (+u,-v).
	std::array<CornerReading, 4> Corners;
};

// One body's share of the null vector, normalised so its largest absolute component over all bodies is 1.
struct MechanismMotion
{
	BodyId Body;
	Vec3 TranslationM;
	Vec3 RotationRad;
};

struct StandingResult
{
	StandingState State = StandingState::Undecided;
	// Every live interface, ascending slot order.
	std::vector<InterfaceReading> Readings;
	// Non-empty only when State is Mechanism.
	std::vector<MechanismMotion> Mechanism;
	std::vector<BodyId> UnconstrainedBodies;
	int ActiveSetIterations = 0;
	// Set when State is InvalidInput and an interface is the cause.
	InterfaceId InvalidInterface;
};

// Solve keeps no state between calls, never mutates the structure and is deterministic for a given structure.
class IStandingAuthority
{
public:
	virtual ~IStandingAuthority() = default;
	virtual StandingResult Solve(const Structure& Model, std::span<const BodyLoad> Loads, const StandingOptions& Options) const = 0;
};

std::unique_ptr<IStandingAuthority> MakeRbsmStandingAuthority();
}
