#pragma once

#include "loadpath/Cascade.h"
#include "loadpath/Catalogue.h"
#include "loadpath/Handles.h"
#include "loadpath/Settings.h"
#include "loadpath/StandingAuthority.h"
#include "loadpath/StructuralAuthority.h"
#include "loadpath/Structure.h"
#include "loadpath/StructureFile.h"
#include "loadpath/Units.h"
#include "loadpath/Vec3.h"
#include "loadpath/Viewer.h"
#include "loadpath/geometry/BoxContacts.h"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace loadpath
{
enum class ScenarioTier : std::uint8_t { Default, SlowInDebug, Slow };

// Met when the highest utilisation lies in [Low, High) and an At joint attains it within 1e-9 relative, governed by Axis when given.
struct ScenarioUtilisation
{
	double Low = 0.0, High = 0.0;
	std::vector<std::string> At;
	std::optional<GoverningAxis> Axis;
};

// Body names are already expanded from patterns; Cracked keeps its "<a>/<b>" patterns, matched when compared.
struct ScenarioExpect
{
	std::string State = "Stands";
	// False: Released is one set, read as the union of its lists.
	bool bPartition = false;
	std::vector<std::vector<std::string>> Released;
	std::vector<std::string> Either;
	std::optional<int> Passes;
	std::optional<std::vector<std::string>> Cracked;
	// Compared as a set; absent means nothing crushes.
	std::vector<std::string> Crushed;
	std::optional<ScenarioUtilisation> BeforeUtilisation, AfterUtilisation;
	// False: no released cluster is forced; true: at least one is.
	std::optional<bool> Forced;
	// Every body here must be live at the end: not released, crushed or removed.
	std::vector<std::string> Stands;
};

enum class ScenarioStepKind : std::uint8_t { Cascade, Judge, Remove, Load, Lateral };

struct ScenarioStep
{
	ScenarioStepKind Kind = ScenarioStepKind::Cascade;
	// Remove: the expanded names. Load: the one body.
	std::vector<std::string> Names;
	Vec3 ForceN;
	// Fractions of g along x and y.
	double LateralX = 0.0, LateralY = 0.0;
};

// Per-body vectors are indexed by slot: BodyNames[Id.Index] names body Id, and ProfileNames likewise for profiles.
struct Scenario
{
	std::string Title, Summary;
	std::optional<std::string> Deferred;
	ScenarioTier Tier = ScenarioTier::Default;
	Structure Model;
	std::vector<BodyId> BodyIds;
	std::vector<std::string> BodyNames, BodyMaterials;
	// An arch voussoir's box is the largest one inscribed in its sector, turned by its BodyRotations quaternion (w, x, y, z).
	std::vector<geometry::Box> BodyBoxes;
	std::vector<std::array<double, 4>> BodyRotations;
	std::vector<double> BodyMassesKg;
	std::vector<bool> BodyGrounded;
	std::vector<std::string> ProfileNames;
	std::vector<ScenarioStep> Steps;
	std::optional<ScenarioExpect> Expect;
	// Invalid when no body has the name.
	BodyId Find(std::string_view Name) const;
};

struct ScenarioLoadResult
{
	Scenario Value;
	FileError Error;
	bool Ok() const { return Error.Message.empty(); }
};

// A refused document loads nothing: the value is empty.
ScenarioLoadResult LoadScenario(std::string_view Json, const Catalogue& Rows);

struct ScenarioStepRecord
{
	// "cascade", "judge", "remove", "load" or "lateral_g".
	std::string Kind;
	std::vector<std::string> Removed;
	std::vector<std::vector<std::string>> Released;
	std::string State;
	int Passes = 0;
};

struct ScenarioJointReading
{
	// "<a>/<b>" in the interface's body order.
	std::string Joint;
	InterfaceReading Reading;
};

// States are names: a cascade state, or a judge's component verdict ("Collapses").
struct ScenarioOutcome
{
	std::vector<std::string> Names, Removed;
	std::vector<std::vector<std::string>> Clusters;
	std::string State = "Stands";
	int Passes = 0;
	// "<a>/<b>" for every joint a cascade cracked.
	std::vector<std::string> Cracked;
	// Every body a cascade crushed, in order.
	std::vector<std::string> Crushed;
	std::vector<ScenarioStepRecord> Steps;
	std::vector<CascadeVerdict> Verdicts;
	// The judge of the structure as the before view draws it: every remove step applied, every load and lateral_g applied, no cascade run.
	std::optional<ComponentVerdict> Before;
	// Every live joint as that before judge read it, and as the last cascade or judge step read it.
	std::vector<ScenarioJointReading> BeforeReadings, AfterReadings;
	// The bodies of every released cluster that was forced.
	std::vector<std::vector<std::string>> ForcedClusters;
	Vec3 Gravity{0.0, 0.0, -GravityMPerS2};
	std::vector<BodyLoad> Loads;
	// Set when a step cannot run; the run stops there.
	std::string RunError;
	// Wall time of the steps; never written by the scenario export.
	double ElapsedMs = 0.0;
};

// Mutates Value.Model step by step; the overloads without engines make the standard combined authority and cascade, and those without Settings use the compiled defaults.
ScenarioOutcome RunScenario(Scenario& Value, const ICascade& Cascade, const IStructuralAuthority& Authority, const Settings& Tunables);
ScenarioOutcome RunScenario(Scenario& Value, const ICascade& Cascade, const IStructuralAuthority& Authority);
ScenarioOutcome RunScenario(Scenario& Value, const Settings& Tunables);
ScenarioOutcome RunScenario(Scenario& Value);

struct ExpectationReport
{
	bool bMet = false;
	std::vector<std::string> Lines;
};

ExpectationReport CompareExpectation(const ScenarioExpect& Expect, const ScenarioOutcome& Outcome);

struct PatternExpansion
{
	std::vector<std::string> Names;
	std::string Error;
	bool Ok() const { return Error.empty(); }
};

// '*' matches any run of characters, dots included; "[a-b]" one integer from a to b. Matching nothing is an error.
PatternExpansion ExpandPattern(std::string_view Pattern, std::span<const std::string> Names);

// Expected is set only when the file's expectation is not met.
ViewerScene ViewerSceneOf(const Scenario& Value, const ScenarioOutcome& Outcome, std::string_view Slug);
std::string WriteViewerJson(const Scenario& Value, const ScenarioOutcome& Outcome, std::string_view Slug);
}
