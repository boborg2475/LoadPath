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

// The rigorous dual bound from the given duals (one per row) after the same polish the solver applies; +inf for invalid input, a bad index or a wrong dual count.
double ProbeDualBound(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, std::size_t Component, bool bInscribed, const std::vector<double>& Duals);

// A is dense, column-major, Rows x Columns, built by Apply on unit vectors; B is RowBounds; Boxes is ColumnBoxes; maximise x[LoadFactorColumn] subject to A x = B, x >= 0.
struct ExportedProgram
{
	int Rows = 0;
	int Columns = 0;
	std::vector<double> A;
	std::vector<double> B;
	std::vector<double> Boxes;
	int LoadFactorColumn = -1;
};

// Invalid input or an out-of-range component returns a default export.
ExportedProgram ExportProgram(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, std::size_t Component, bool bInscribed);

// min Cost^T x s.t. A x <= B, x >= 0 (Cost >= 0), A column-major B.size() x Cost.size(); the production facet dual simplex with its pivot limit and Bland threshold as arguments.
bool FacetSimplexForTest(const std::vector<double>& Cost, const std::vector<double>& A, const std::vector<double>& B, int PivotLimit, int BlandAfter, std::vector<double>& X);

// ProbeDualBound with the facet polish's pivot limit replaced.
double ProbeDualBoundWithFacetPivotLimit(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, std::size_t Component, bool bInscribed, const std::vector<double>& Duals, int FacetPivotLimit);

// The production Decide over every component, with Margins replacing the stall-recovery margins for this call.
CertificateResult DecideWithRecoveryMargins(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, std::span<const double> Margins);
}
