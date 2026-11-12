#pragma once

#include "loadpath/Cascade.h"
#include "loadpath/StandingAuthority.h"
#include "loadpath/StructureFile.h"
#include "loadpath/Vec3.h"

#include <string_view>

namespace loadpath
{
// The engine's tunables; data/settings.json carries the same keys in snake_case with the same defaults.
struct SolverSettings
{
	int MaxActiveSetIterations = 50;
	double PivotToleranceRelative = 1e-10;
	int DenseDofLimit = 600;
	double StickMarginRelative = 1e-9;
	double HeldForceToleranceRelative = 1e-12;
	double DryNoiseFloorRelative = 1e-12;
	double NeutralLoadToleranceRelative = 1e-12;
	double AdmissibilityToleranceRelative = 1e-9;
	double UnconstrainedComponent = 1e-6;
	double DryRecloseFactor = 2.0;
	double SparseAcceptanceMargin = 1.0;
	double OpenTensionToleranceRelative = 1e-12;
	double CloseCompressionToleranceRelative = 1e-12;
	double SoftSpringRelative = 0.0;
	double SoftSpringMechanismRatio = 3.1622776601683795;
};

struct CascadeSettings
{
	double CrackBandFraction = 0.5;
	double CrackBandEdgeRelative = 1e-9;
	double CrushEdgeRelative = 1e-9;
	double MotionAxisThresholdRelative = 1e-3;
	int MaxPasses = 32;
	UndecidedResolution OnUndecided = UndecidedResolution::Collapse;
};

// Every debug option's real-life default is off.
struct DebugSettings
{
	bool bRecordPasses = false;
};

struct Settings
{
	SolverSettings Solver;
	CascadeSettings Cascade;
	DebugSettings Debug;
};

struct SettingsResult
{
	Settings Value;
	FileError Error;

	bool Ok() const { return Error.Message.empty(); }
};

// Every key present and of the right type, no unknown keys, or Error names the key and line.
SettingsResult LoadSettings(std::string_view DefaultsJson);

// Overrides may hold any subset of the keys; an unknown key, a wrong type or a non-finite number is an error naming it.
SettingsResult ApplyOverrides(const Settings& Defaults, std::string_view OverridesJson);

// Debug points into the given Settings, which must outlive the options.
StandingOptions ToStandingOptions(const Settings& Value, Vec3 Gravity);
CascadeOptions ToCascadeOptions(const Settings& Value, Vec3 Gravity);
}
