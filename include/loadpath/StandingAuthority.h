#pragma once

#include "loadpath/Handles.h"
#include "loadpath/Structure.h"
#include "loadpath/Units.h"
#include "loadpath/Vec3.h"

#include <array>
#include <cstddef>
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
	// A component with more DoF than this is factorised sparse, falling back to the dense blocked factor on any sign of rank deficiency.
	int DenseDofLimit = 600;
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
	// Utilisation is the maximum of these and Axis names it; a non-finite ratio stays non-finite.
	double CompressionUtilisation = 0, TensionUtilisation = 0, ShearUtilisation = 0;
	// Resultants of the force on B, about the rectangle centre.
	double NormalForceN = 0;
	Vec3 ShearForceN;
	Vec3 MomentNm;
	// Order: (+u,+v), (-u,+v), (-u,-v), (+u,-v).
	std::array<CornerReading, 4> Corners;
};

// One body's share of a mechanism component's null vector, normalised so that component's largest absolute component is 1.
struct MechanismMotion
{
	BodyId Body;
	Vec3 TranslationM;
	Vec3 RotationRad;
};

struct StandingComponent
{
	// Ascending slot order, as ComponentPartition::FreeBodies lists them.
	std::vector<BodyId> Bodies;
	StandingState State = StandingState::Undecided;
	int ActiveSetIterations = 0;
	// Indices into StandingResult::Readings of this component's interfaces.
	std::vector<std::size_t> ReadingSlots;
};

struct StandingResult
{
	StandingState State = StandingState::Undecided;
	// Every live interface in the solved components plus those joining two grounded bodies, ascending slot order.
	std::vector<InterfaceReading> Readings;
	// Every mechanism component, whatever the overall State; empty when no component is a mechanism.
	std::vector<MechanismMotion> Mechanism;
	std::vector<BodyId> UnconstrainedBodies;
	// Ascending by lowest body slot; empty when State is InvalidInput.
	std::vector<StandingComponent> Components;
	int ActiveSetIterations = 0;
	// Set when State is InvalidInput and an interface is the cause.
	InterfaceId InvalidInterface;
	// Ascending indices into the input loads whose body this structure issued but has since removed; those loads were skipped.
	std::vector<std::size_t> StaleLoads;
};

// Solve keeps no state between calls, never mutates the structure and is deterministic for a given structure.
class IStandingAuthority
{
public:
	virtual ~IStandingAuthority() = default;
	// Only empty solves every component; otherwise only the components holding a listed body, and any entry that is not a live free body is InvalidInput.
	// A component with no live interface to a grounded body, no host load and a non-zero gravity reads Mechanism without a solve: every body unconstrained, the gravity direction as its motion, zero iterations, every reading zero with axis None; a zero-force load counts as no load, and an assembled stiffness that overflows reads InvalidInput, as on the full path.
	virtual StandingResult Solve(const Structure& Model, std::span<const BodyLoad> Loads, const StandingOptions& Options, std::span<const BodyId> Only) const = 0;
};

std::unique_ptr<IStandingAuthority> MakeRbsmStandingAuthority();
}
