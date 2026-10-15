#pragma once

#include "loadpath/PhaseTimings.h"

#if defined(LOADPATH_PROFILE)
#include <chrono>
#endif

namespace loadpath::profile
{
#if defined(LOADPATH_PROFILE)
// Adds the seconds between construction and destruction to the referenced field.
class ScopedPhase
{
public:
	explicit ScopedPhase(double& InSeconds)
		: Seconds(InSeconds)
		, Start(std::chrono::steady_clock::now())
	{
	}
	~ScopedPhase() { Seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - Start).count(); }
	ScopedPhase(const ScopedPhase&) = delete;
	ScopedPhase& operator=(const ScopedPhase&) = delete;

private:
	double& Seconds;
	std::chrono::steady_clock::time_point Start;
};
#else
// Without LOADPATH_PROFILE no clock is read and nothing is written.
class ScopedPhase
{
public:
	explicit ScopedPhase(double&) {}
	ScopedPhase(const ScopedPhase&) = delete;
	ScopedPhase& operator=(const ScopedPhase&) = delete;
};
#endif

template <class Body>
decltype(auto) Timed(double& Seconds, Body&& Run)
{
	const ScopedPhase Scope(Seconds);
	return Run();
}

// The normal build calls compute(); the profiled build splits it to time the analysis apart, with the same ordering and the same factor.
// The split builds the permuted copy of K twice, the second booked to FactorizationSeconds, so profiled sparse factor time overstates the normal build by one sparse-matrix copy.
template <class Factor, class Matrix>
void AnalyseAndFactorise(Factor& Into, const Matrix& K, PhaseTimings& Timings)
{
#if defined(LOADPATH_PROFILE)
	Timed(Timings.AnalysisSeconds, [&] { Into.analyzePattern(K); });
	Timed(Timings.FactorizationSeconds, [&] { Into.factorize(K); });
#else
	static_cast<void>(Timings);
	Into.compute(K);
#endif
}

inline void Accumulate(PhaseTimings& Sum, const PhaseTimings& Add)
{
	Sum.AssemblySeconds += Add.AssemblySeconds;
	Sum.AnalysisSeconds += Add.AnalysisSeconds;
	Sum.FactorizationSeconds += Add.FactorizationSeconds;
	Sum.SolveSeconds += Add.SolveSeconds;
	Sum.CheckSeconds += Add.CheckSeconds;
	Sum.ReadoutSeconds += Add.ReadoutSeconds;
	Sum.TotalSeconds += Add.TotalSeconds;
}
}
