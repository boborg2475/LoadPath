#pragma once

#include "loadpath/Handles.h"
#include "loadpath/ParallelFor.h"
#include "loadpath/PhaseTimings.h"
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
struct DebugSettings;

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
	// A slipping corner re-sticks when its shear falls below capacity times (1 - this).
	double StickMarginRelative = 1e-9;
	// A slipping corner's held force has settled when it moves less than this times the component's force scale.
	double HeldForceToleranceRelative = 1e-12;
	// A dry corner inside plus or minus this times the force scale is open; non-dry corners use OpenTensionToleranceRelative instead.
	double DryNoiseFloorRelative = 1e-12;
	// An open dry corner re-closes only above this times the noise floor; above 1 it is the floor's hysteresis (DESIGN §10, 2026-10-11).
	double DryRecloseFactor = 2.0;
	// The sparse factor is accepted only when its smallest eigenvalue estimate exceeds this times the pivot tolerance times the largest pivot.
	double SparseAcceptanceMargin = 1.0;
	// A closed non-dry tensionless corner opens only below minus this times the force scale; dry corners ignore it.
	double OpenTensionToleranceRelative = 1e-12;
	// An open non-dry tensionless corner re-closes only above this times the force scale; dry corners ignore it.
	double CloseCompressionToleranceRelative = 1e-12;
	// Above 0, every free unknown gets a spring to the world of this times its all-closed diagonal stiffness; 0 is off.
	double SoftSpringRelative = 0.0;
	// With soft springs on, a body is unconstrained when softening the springs tenfold grows its displacement by more than this.
	double SoftSpringMechanismRatio = 3.1622776601683795;
	// A rank-deficient component is a mechanism only when the load's null-space projection exceeds this times the load.
	double NeutralLoadToleranceRelative = 1e-12;
	// A corner closes or slides against friction in a null motion when it moves more than this times the motion's scale.
	double AdmissibilityToleranceRelative = 1e-9;
	// A body is unconstrained when its share of the unit null vector exceeds this.
	double UnconstrainedComponent = 1e-6;
	// The host's debug hook; null means no debug work.
	const DebugSettings* Debug = nullptr;
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

enum class CornerState : std::uint8_t
{
	Closed,
	Open,
	Slipping
};

// Why a component's solve stopped; None only when no solve ran.
enum class SolveEnd : std::uint8_t
{
	None,
	Converged,
	Mechanism,
	RestoreFailedClosed,
	PowerCheckFailed,
	IterationCap,
	InvalidInput
};

// One joint after one active-set iteration: states after the update, forces from that iteration's solution.
struct JointPassRecord
{
	InterfaceId Id;
	std::array<CornerState, 4> Corners{};
	// A corner whose state differs from the previous iteration's; all false on iteration 1.
	std::array<bool, 4> bFlipped{};
	// Compression positive.
	std::array<double, 4> CornerNormalN{};
	std::array<Vec3, 4> CornerShearN{};
	double NormalForceN = 0;
	Vec3 ShearForceN;
	Vec3 MomentNm;
	double Utilisation = 0;
	GoverningAxis Axis = GoverningAxis::None;
};

struct ElasticPassRecord
{
	int Iteration = 0;
	std::vector<JointPassRecord> Joints;
};

struct StandingComponent
{
	// Ascending slot order, as ComponentPartition::FreeBodies lists them.
	std::vector<BodyId> Bodies;
	StandingState State = StandingState::Undecided;
	int ActiveSetIterations = 0;
	// Indices into StandingResult::Readings of this component's interfaces.
	std::vector<std::size_t> ReadingSlots;
	PhaseTimings Timings;
	// Empty unless Options.Debug->bRecordPasses; recording changes no answer.
	std::vector<ElasticPassRecord> Passes;
	SolveEnd End = SolveEnd::None;
	// RestoreFailedClosed only: the interfaces whose corners blocked the restore.
	std::vector<InterfaceId> Blocking;
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
	// Field-wise sum of the Components timings, in component order.
	PhaseTimings Timings;
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

// Workers is non-owning and the host keeps it alive; null solves serially, and the answer is bit-identical either way when the hook keeps the caller's floating-point environment.
std::unique_ptr<IStandingAuthority> MakeRbsmStandingAuthority(const IParallelFor* Workers = nullptr);
}
