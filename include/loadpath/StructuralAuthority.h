#pragma once

#include "loadpath/Handles.h"
#include "loadpath/PhaseTimings.h"
#include "loadpath/StandingAuthority.h"
#include "loadpath/Structure.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace loadpath
{
enum class ComponentVerdict : std::uint8_t
{
	Stands,
	Overstressed,
	Collapses,
	Undecided,
	InvalidInput
};

struct ComponentReport
{
	// The free bodies of this connected component, ascending slot order.
	std::vector<BodyId> Bodies;
	ComponentVerdict Verdict = ComponentVerdict::Undecided;
	StandingState Elastic = StandingState::Undecided;
	std::vector<MechanismMotion> Mechanism;
	// On Collapses the elastic UnconstrainedBodies of this component and Mechanism its motions; else both empty.
	std::vector<BodyId> Release;
	// Copied from the previous verdict rather than solved.
	bool bReused = false;
	// The loads on this component's bodies as judged, in the order given; a change to them dirties the component.
	std::vector<BodyLoad> Loads;
	// Carried from the elastic component; Passes is empty unless recording.
	SolveEnd End = SolveEnd::None;
	std::vector<ElasticPassRecord> Passes;
	int PassCount = 0;
	// Set by the cascade: the verdict came from resolving an elastic Undecided, not from a proven mechanism.
	bool Forced = false;
	// The elastic end behind a forced verdict; None when the solve stopped on a non-finite force scale or nothing was forced.
	SolveEnd ForcedReason = SolveEnd::None;
	// RestoreFailedClosed: the blocking joints; IterationCap with recording on: the joints with a corner flipped in the last iteration.
	std::vector<InterfaceId> UnsettledJoints;
};

struct AuthorityOptions
{
	StandingOptions Standing;
};

struct AuthorityResult
{
	// InvalidInput > Undecided > Collapses > Overstressed > Stands.
	ComponentVerdict Worst = ComponentVerdict::Undecided;
	// The elastic authority's readings, every live interface, ascending slot order.
	std::vector<InterfaceReading> Readings;
	// Ascending by lowest body slot.
	std::vector<ComponentReport> Components;
	InterfaceId InvalidInterface{};
	// The structure's Revision() when judged.
	std::uint64_t Revision = 0;
	// The structure's Lineage() when judged; a Previous from another lineage reuses nothing.
	std::uint64_t Lineage = 0;
	// The options this result was judged under; a Previous judged under any other options reuses nothing.
	AuthorityOptions Options;
	// Ascending indices into the input loads whose body this structure issued but has since removed; those loads were skipped.
	std::vector<std::size_t> StaleLoads;
	// The components solved in this call; a reused component adds nothing.
	// An abandoned reuse attempt (merging readings failed and the judge re-solved) counts its solve too, since that time was spent in this call.
	PhaseTimings ElasticTimings;
	// The whole call, so at least ElasticTimings.TotalSeconds when profiled.
	double JudgeSeconds = 0.0;
};

// Judge keeps no state between calls, never mutates the structure and is deterministic for a given structure.
class IStructuralAuthority
{
public:
	virtual ~IStructuralAuthority() = default;
	// Previous may be any earlier verdict or nullptr; components it matches unchanged are copied, an unmatched one reuses nothing.
	virtual AuthorityResult Judge(const Structure& Model, std::span<const BodyLoad> Loads, const AuthorityOptions& Options, const AuthorityResult* Previous) const = 0;
};

// Elastic-only: Mechanism reads Collapses and releases the elastic UnconstrainedBodies; Undecided releases nothing.
std::unique_ptr<IStructuralAuthority> MakeElasticAuthority(std::unique_ptr<IStandingAuthority> Standing);
}
