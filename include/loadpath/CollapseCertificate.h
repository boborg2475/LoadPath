#pragma once

#include "loadpath/Handles.h"
#include "loadpath/StandingAuthority.h"
#include "loadpath/Structure.h"
#include "loadpath/Units.h"
#include "loadpath/Vec3.h"

#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace loadpath
{
enum class CertificateState : std::uint8_t
{
	Stands,
	Collapses,
	Undecided,
	InvalidInput
};

struct CertificateOptions
{
	Vec3 Gravity = {0.0, 0.0, -GravityMPerS2};
	// Even and at least 4; the friction circle is approximated by this polygon.
	int ConeGenerators = 16;
	// The upper bound on the load factor; reaching it means the structure stands with at least this margin.
	double LoadFactorCap = 1.0e3;
	// Relative, on the equilibrium residual and complementarity.
	double FeasibilityTolerance = 1.0e-9;
	// Interior-point iterations allowed per solve (inscribed, circumscribed, selection); a failed solve reads Undecided unless t_in >= 1.
	int MaxIterations = 200;
	// Components with more equilibrium rows than this factor the normal matrix sparsely; at or below it, dense.
	int DenseRowLimit = 600;
};

struct CertificateResult
{
	CertificateState State = CertificateState::Undecided;
	// The inscribed-cone collapse multiplier: a statically admissible field exists up to here.
	double LoadFactorLower = 0.0;
	// The circumscribed-cone multiplier: no statically admissible field exists beyond here (the cap when t_in >= 1 and the circumscribed solve failed).
	double LoadFactorUpper = 0.0;
	// The selected mechanism of every Collapses component whatever State, per moving body, normalised per component so its largest component is 1.
	std::vector<MechanismMotion> Mechanism;
	std::vector<BodyId> UnconstrainedBodies;
	// Interfaces the selected mechanism moves: a corner's relative velocity above 1e-3 of the component's fastest corner on an interface with a listed body; empty on Stands.
	std::vector<InterfaceId> HingingInterfaces;
	// Maximum over components of the cone solves' iterations; the selection solve's count is not included.
	int Iterations = 0;
	InterfaceId InvalidInterface{};
};

// Decide keeps no state between calls, never mutates the structure and is deterministic for a given structure.
class ICollapseCertificate
{
public:
	virtual ~ICollapseCertificate() = default;
	// Only names bodies whose components are decided, as if no other component existed (empty: all); an entry that is not a live free body reads InvalidInput.
	virtual CertificateResult Decide(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, std::span<const BodyId> Only) const = 0;
};

std::unique_ptr<ICollapseCertificate> MakeLimitAnalysisCertificate();
}
