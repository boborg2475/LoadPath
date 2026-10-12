#pragma once

#include "loadpath/CollapseCertificate.h"
#include "loadpath/Handles.h"
#include "loadpath/StandingAuthority.h"
#include "loadpath/Structure.h"

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
	// Undecided when the certificate was not consulted.
	CertificateState Kinematic = CertificateState::Undecided;
	bool bConsultedCertificate = false;
	// Elastic Mechanism overruled by a certificate Stands.
	bool bDisagreed = false;
	// From the certificate when consulted, else NaN.
	double LoadFactorLower = 0.0;
	double LoadFactorUpper = 0.0;
	std::vector<MechanismMotion> Mechanism;
	// The certificate's UnconstrainedBodies on Collapses, else empty; the elastic lists are never released.
	std::vector<BodyId> Release;
	std::vector<InterfaceId> Hinging;
	// Copied from the previous verdict rather than solved.
	bool bReused = false;
	// The loads on this component's bodies as judged, in the order given; a change to them dirties the component.
	std::vector<BodyLoad> Loads;
};

struct AuthorityOptions
{
	StandingOptions Standing;
	// Gravity must equal Standing.Gravity or the call is InvalidInput.
	CertificateOptions Certificate;
};

struct AuthorityResult
{
	// InvalidInput > Undecided > Collapses > Overstressed > Stands.
	ComponentVerdict Worst = ComponentVerdict::Undecided;
	// The elastic authority's readings, every live interface, ascending slot order.
	std::vector<InterfaceReading> Readings;
	// Ascending by lowest body slot.
	std::vector<ComponentReport> Components;
	int Disagreements = 0;
	InterfaceId InvalidInterface{};
	// The structure's Revision() when judged.
	std::uint64_t Revision = 0;
};

// Judge keeps no state between calls, never mutates the structure and is deterministic for a given structure.
class IStructuralAuthority
{
public:
	virtual ~IStructuralAuthority() = default;
	// Previous may be any earlier verdict or nullptr; components it matches unchanged are copied, an unmatched one reuses nothing.
	virtual AuthorityResult Judge(const Structure& Model, std::span<const BodyLoad> Loads, const AuthorityOptions& Options, const AuthorityResult* Previous) const = 0;
};

std::unique_ptr<IStructuralAuthority> MakeCombinedAuthority(std::unique_ptr<IStandingAuthority> Standing, std::unique_ptr<ICollapseCertificate> Certificate);
}
