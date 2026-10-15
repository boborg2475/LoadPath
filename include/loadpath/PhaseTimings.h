#pragma once

namespace loadpath
{
// Seconds spent in each phase of an elastic solve; every field stays 0.0 unless built with LOADPATH_PROFILE.
struct PhaseTimings
{
	double AssemblySeconds = 0.0;
	double AnalysisSeconds = 0.0;
	double FactorizationSeconds = 0.0;
	double SolveSeconds = 0.0;
	double CheckSeconds = 0.0;
	double ReadoutSeconds = 0.0;
	// The phases cover only part of this: the pivot rank scan, finiteness checks, active-set updates, release restoration, cohesive dissipation and the mechanism power test count only here.
	double TotalSeconds = 0.0;
};
}
