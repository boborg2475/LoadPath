#pragma once

#include "loadpath/CollapseCertificate.h"

#include <cstddef>
#include <span>
#include <vector>

namespace loadpath::detail
{
// One interior-point solve of a component's inscribed or circumscribed program, without stall recovery or the rigid-fall shortcut.
struct ProgramProbe
{
	bool bConverged = false;
	double LoadFactor = 0.0;
	double DualBound = 0.0;
	std::vector<double> Primal;
	std::vector<double> Duals;
	std::vector<double> Boxes;
};

// Invalid input or an out-of-range component returns a default probe.
ProgramProbe ProbeProgram(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, std::size_t Component, bool bInscribed);

// The rigorous dual bound at the given duals (one per row); +inf for invalid input, a bad index or a wrong dual count.
double ProbeDualBound(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, std::size_t Component, bool bInscribed, const std::vector<double>& Duals);
}
