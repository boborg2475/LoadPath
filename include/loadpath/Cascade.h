#pragma once

#include "loadpath/Handles.h"
#include "loadpath/StandingAuthority.h"
#include "loadpath/StructuralAuthority.h"
#include "loadpath/Structure.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace loadpath
{
enum class UndecidedResolution : std::uint8_t
{
	Collapse,
	Stands
};

inline constexpr std::size_t SolveEndCount = 7;

enum class CascadeState : std::uint8_t
{
	Stands,
	Overstressed,
	Undecided,
	InvalidInput,
	PassCapReached
};

struct FailedInterface
{
	InterfaceId Id;
	std::uint32_t Pass = 0;
	// The crack: the larger of tension and shear. The sever: Tension if the elastic mechanism motion opens it beyond friction dilatancy, else Shear if it slides, else the reading's axis (None when unread).
	GoverningAxis Axis = GoverningAxis::None;
	// The crack ratio, or the elastic reading's utilisation at release.
	double Ratio = 0.0;
	// False: cracked (bond lost, friction kept); true: severed with its cluster.
	bool bSevered = false;
};

struct ReleasedCluster
{
	std::uint32_t Pass = 0;
	// Ascending slot order; released as one rigid body by the host.
	std::vector<BodyId> Bodies;
	// The elastic MechanismMotion of the lowest-slot body; the others share it.
	MechanismMotion Motion;
	// Every live interface that crossed the cluster boundary, now Severed.
	std::vector<InterfaceId> Severed;
	// Released by resolving an Undecided component; its Motion is zero.
	bool bForced = false;
};

struct CascadeOptions
{
	AuthorityOptions Authority;
	// Joints with ratio >= R_max (1 - delta) crack this pass; 0 is one event per pass (ties together), 1 all at once.
	double CrackBandFraction = 0.5;
	// Counted passes before PassCapReached.
	int MaxPasses = 32;
	// Collapse releases a whole Undecided component; Stands keeps it standing; either marks it Forced.
	UndecidedResolution OnUndecided = UndecidedResolution::Collapse;
};

// One pass's judge and what it changed; the final judge that changed nothing is recorded too.
struct CascadePassRecord
{
	std::uint32_t Pass = 0;
	std::vector<ComponentReport> Components;
	std::vector<InterfaceReading> Readings;
	std::vector<ReleasedCluster> Clusters;
	std::vector<FailedInterface> Failures;
};

struct CascadeVerdict
{
	CascadeState State = CascadeState::Undecided;
	// Stamps written by this run; FirstPass is the previous run's LastPass + 1.
	std::uint32_t FirstPass = 0, LastPass = 0;
	// Counted passes: those that cracked or released something.
	int Passes = 0;
	// Ascending (pass, slot).
	std::vector<FailedInterface> Failures;
	// Ascending (pass, lowest body slot).
	std::vector<ReleasedCluster> Clusters;
	// The final judge's readings, every live interface.
	std::vector<InterfaceReading> Readings;
	// The final judge's components.
	std::vector<ComponentReport> Components;
	InterfaceId InvalidInterface{};
	// Ascending indices into the input loads whose body this structure issued but has since removed; those loads were skipped.
	std::vector<std::size_t> StaleLoads;
	// Empty unless Options.Authority.Standing.Debug->bRecordPasses.
	std::vector<CascadePassRecord> PassRecords;
	// Forced components solved this run, indexed by their SolveEnd.
	std::array<std::uint32_t, SolveEndCount> ForcedByReason{};
};

// Run mutates the structure it is given (stamps, removals) and lists every change in the verdict.
class ICascade
{
public:
	virtual ~ICascade() = default;
	// Previous seeds the first pass's judge; each later pass judges against the pass before it.
	virtual CascadeVerdict Run(Structure& Model, std::span<const BodyLoad> Loads, const CascadeOptions& Options, const AuthorityResult* Previous) const = 0;
};

std::unique_ptr<ICascade> MakeCascade(std::unique_ptr<IStructuralAuthority> Authority);
}
