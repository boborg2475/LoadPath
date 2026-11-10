#include "CatalogueTestSupport.h"
#include "StaticsTestSupport.h"

#include "loadpath/Settings.h"

#include "doctest.h"

#include <cstddef>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
std::string ReadSettingsFile()
{
	std::ifstream File(std::string(LOADPATH_DATA_PATH) + "/settings.json", std::ios::binary);
	std::ostringstream Text;
	Text << File.rdbuf();
	return Text.str();
}

std::string Replace(std::string Text, std::string_view From, std::string_view To)
{
	const std::size_t At = Text.find(From);
	INFO("fixture text must contain " << std::string(From));
	REQUIRE(At != std::string::npos);
	if (At != std::string::npos)
	{
		Text.replace(At, From.size(), To);
	}
	return Text;
}

int LineOf(std::string_view Text, std::string_view Needle)
{
	const std::size_t At = Text.find(Needle);
	REQUIRE(At != std::string_view::npos);
	int Line = 1;
	for (std::size_t I = 0; I < At && I < Text.size(); ++I)
	{
		Line += Text[I] == '\n' ? 1 : 0;
	}
	return Line;
}

bool SameSettings(const Settings& A, const Settings& B)
{
	return A.Solver.MaxActiveSetIterations == B.Solver.MaxActiveSetIterations && A.Solver.PivotToleranceRelative == B.Solver.PivotToleranceRelative &&
		A.Solver.DenseDofLimit == B.Solver.DenseDofLimit && A.Solver.StickMarginRelative == B.Solver.StickMarginRelative &&
		A.Solver.HeldForceToleranceRelative == B.Solver.HeldForceToleranceRelative && A.Solver.DryNoiseFloorRelative == B.Solver.DryNoiseFloorRelative &&
		A.Solver.NeutralLoadToleranceRelative == B.Solver.NeutralLoadToleranceRelative &&
		A.Solver.AdmissibilityToleranceRelative == B.Solver.AdmissibilityToleranceRelative && A.Solver.UnconstrainedComponent == B.Solver.UnconstrainedComponent &&
		A.Solver.DryRecloseFactor == B.Solver.DryRecloseFactor && A.Solver.SparseAcceptanceMargin == B.Solver.SparseAcceptanceMargin &&
		A.Solver.OpenTensionToleranceRelative == B.Solver.OpenTensionToleranceRelative && A.Solver.CloseCompressionToleranceRelative == B.Solver.CloseCompressionToleranceRelative &&
		A.Cascade.CrackBandEdgeRelative == B.Cascade.CrackBandEdgeRelative && A.Cascade.CrushEdgeRelative == B.Cascade.CrushEdgeRelative &&
		A.Cascade.MotionAxisThresholdRelative == B.Cascade.MotionAxisThresholdRelative &&
		A.Cascade.CrackBandFraction == B.Cascade.CrackBandFraction && A.Cascade.MaxPasses == B.Cascade.MaxPasses && A.Cascade.OnUndecided == B.Cascade.OnUndecided && A.Debug.bRecordPasses == B.Debug.bRecordPasses;
}

Settings LoadOk(const std::string& Json)
{
	const SettingsResult Loaded = LoadSettings(Json);
	INFO("settings error line " << Loaded.Error.Line << ": " << Loaded.Error.Message);
	REQUIRE(Loaded.Ok());
	return Loaded.Value;
}

void CheckNames(const SettingsResult& Result, std::string_view Key, int Line)
{
	INFO("error line " << Result.Error.Line << ": " << Result.Error.Message);
	CHECK_FALSE(Result.Ok());
	CHECK(Result.Error.Message.find(Key) != std::string::npos);
	CHECK(Result.Error.Line == Line);
}

Settings DistinctSettings()
{
	Settings S;
	S.Solver.MaxActiveSetIterations = 7;
	S.Solver.PivotToleranceRelative = 2e-3;
	S.Solver.DenseDofLimit = 11;
	S.Solver.StickMarginRelative = 3e-3;
	S.Solver.HeldForceToleranceRelative = 4e-3;
	S.Solver.DryNoiseFloorRelative = 5e-3;
	S.Solver.NeutralLoadToleranceRelative = 6e-3;
	S.Solver.AdmissibilityToleranceRelative = 7e-3;
	S.Solver.UnconstrainedComponent = 8e-3;
	S.Solver.DryRecloseFactor = 9e-3;
	S.Solver.SparseAcceptanceMargin = 1.1e-2;
	S.Solver.OpenTensionToleranceRelative = 1.5e-2;
	S.Solver.CloseCompressionToleranceRelative = 1.6e-2;
	S.Cascade.CrackBandFraction = 0.25;
	S.Cascade.CrackBandEdgeRelative = 1.2e-2;
	S.Cascade.CrushEdgeRelative = 1.3e-2;
	S.Cascade.MotionAxisThresholdRelative = 1.4e-2;
	S.Cascade.MaxPasses = 3;
	S.Debug.bRecordPasses = true;
	return S;
}

struct DryStack
{
	Structure Model;
	InterfaceId Bed;
};

DryStack BuildDryTwoBrickStack()
{
	DryStack S;
	const ProfileId Dry = AddProfileOk(S.Model, DryJoint(0.7));
	const BodyId Ground = AddGroundPlane(S.Model);
	const Box Lower = BoxOfDensity(Vec3{0.0, 0.0, 0.0325}, Vec3{0.1, 0.05, 0.0325}, 1800.0);
	const Box Upper = BoxOfDensity(Vec3{0.0, 0.0, 0.0975}, Vec3{0.1, 0.05, 0.0325}, 1800.0);
	const BodyId LowerId = AddBox(S.Model, Lower);
	const BodyId UpperId = AddBox(S.Model, Upper);
	AddInterfaceOk(S.Model, OnGroundPlane(Ground, LowerId, Lower, Dry));
	S.Bed = AddInterfaceOk(S.Model, *StackedFaces(LowerId, Lower, UpperId, Upper, Dry));
	return S;
}

std::string WithoutKey(std::string Text, std::string_view Key)
{
	const std::string Quoted = "\"" + std::string(Key) + "\"";
	const std::size_t At = Text.find(Quoted);
	INFO("fixture text must contain " << Quoted);
	REQUIRE(At != std::string::npos);
	if (At == std::string::npos)
	{
		return Text;
	}
	const std::size_t Start = Text.rfind('\n', At) + 1;
	const std::size_t End = Text.find('\n', At) + 1;
	const bool bHadComma = Text.find(',', At) < End;
	Text.erase(Start, End - Start);
	if (!bHadComma)
	{
		const std::size_t Comma = Text.rfind(',', Start);
		REQUIRE(Comma != std::string::npos);
		Text.erase(Comma, 1);
	}
	return Text;
}

std::string WithValue(std::string Text, std::string_view Key, std::string_view Value)
{
	const std::string Quoted = "\"" + std::string(Key) + "\":";
	const std::size_t At = Text.find(Quoted);
	INFO("fixture text must contain " << Quoted);
	REQUIRE(At != std::string::npos);
	if (At == std::string::npos)
	{
		return Text;
	}
	const std::size_t ValueAt = Text.find_first_not_of(' ', At + Quoted.size());
	const std::size_t ValueEnd = Text.find_first_of(",\r\n", ValueAt);
	Text.replace(ValueAt, ValueEnd - ValueAt, Value);
	return Text;
}

struct ToleranceKey
{
	const char* Section;
	const char* Key;
	double Default;
	double (*Read)(const Settings&);
};

const ToleranceKey EngineTolerances[] = {
	{"solver", "dry_reclose_factor", 2.0, [](const Settings& S) { return S.Solver.DryRecloseFactor; }},
	{"solver", "sparse_acceptance_margin", 1.0, [](const Settings& S) { return S.Solver.SparseAcceptanceMargin; }},
	{"solver", "open_tension_tolerance_relative", 1e-12, [](const Settings& S) { return S.Solver.OpenTensionToleranceRelative; }},
	{"solver", "close_compression_tolerance_relative", 1e-12, [](const Settings& S) { return S.Solver.CloseCompressionToleranceRelative; }},
	{"cascade", "crack_band_edge_relative", 1e-9, [](const Settings& S) { return S.Cascade.CrackBandEdgeRelative; }},
	{"cascade", "crush_edge_relative", 1e-9, [](const Settings& S) { return S.Cascade.CrushEdgeRelative; }},
	{"cascade", "motion_axis_threshold_relative", 1e-3, [](const Settings& S) { return S.Cascade.MotionAxisThresholdRelative; }},
};

int OpenCorners(const StandingResult& Result)
{
	int Count = 0;
	for (const InterfaceReading& R : Result.Readings)
	{
		for (const CornerReading& C : R.Corners)
		{
			Count += C.bOpen ? 1 : 0;
		}
	}
	return Count;
}
}

TEST_CASE("Settings.DefaultsFileLoadsAndMatchesTheCompiledDefaults")
{
	const std::string Json = ReadSettingsFile();
	REQUIRE_FALSE(Json.empty());
	const Settings Loaded = LoadOk(Json);
	CHECK(SameSettings(Loaded, Settings{}));
	CHECK(Loaded.Solver.MaxActiveSetIterations == 50);
	CHECK(Loaded.Solver.PivotToleranceRelative == 1e-10);
	CHECK(Loaded.Solver.DenseDofLimit == 600);
	CHECK(Loaded.Solver.StickMarginRelative == 1e-9);
	CHECK(Loaded.Solver.HeldForceToleranceRelative == 1e-12);
	CHECK(Loaded.Solver.DryNoiseFloorRelative == 1e-12);
	CHECK(Loaded.Solver.NeutralLoadToleranceRelative == 1e-12);
	CHECK(Loaded.Solver.AdmissibilityToleranceRelative == 1e-9);
	CHECK(Loaded.Solver.UnconstrainedComponent == 1e-6);
	CHECK(Loaded.Solver.DryRecloseFactor == 2.0);
	CHECK(Loaded.Solver.SparseAcceptanceMargin == 1.0);
	CHECK(Loaded.Cascade.CrackBandFraction == 0.5);
	CHECK(Loaded.Cascade.CrackBandEdgeRelative == 1e-9);
	CHECK(Loaded.Cascade.CrushEdgeRelative == 1e-9);
	CHECK(Loaded.Cascade.MotionAxisThresholdRelative == 1e-3);
	CHECK(Loaded.Cascade.MaxPasses == 32);
	CHECK(Loaded.Cascade.OnUndecided == UndecidedResolution::Collapse);
	CHECK(Loaded.Debug.bRecordPasses == false);
}

TEST_CASE("Settings.EngineTolerances.EachIsAKeyInTheDefaultsFileWithTodaysValue")
{
	const std::string Json = ReadSettingsFile();
	REQUIRE_FALSE(Json.empty());
	const Settings Compiled{};
	for (const ToleranceKey& Each : EngineTolerances)
	{
		INFO(std::string(Each.Section) << "." << std::string(Each.Key));
		CHECK(Json.find("\"" + std::string(Each.Key) + "\"") != std::string::npos);
		CHECK(Each.Read(Compiled) == Each.Default);
		CHECK(Each.Read(LoadOk(Json)) == Each.Default);
		CHECK(Each.Read(LoadOk(WithValue(Json, Each.Key, "0.375"))) == 0.375);
	}
}

TEST_CASE("Settings.EngineTolerances.AMissingOneIsAnErrorNamingIt")
{
	const std::string Json = ReadSettingsFile();
	for (const ToleranceKey& Each : EngineTolerances)
	{
		const std::string Name = std::string(Each.Section) + "." + Each.Key;
		INFO(Name);
		const std::string Missing = WithoutKey(Json, Each.Key);
		CheckNames(LoadSettings(Missing), Name, LineOf(Missing, "\"" + std::string(Each.Section) + "\""));
	}
}

TEST_CASE("Settings.EngineTolerances.AnOverrideSetsOnlyItsKey")
{
	const Settings Defaults{};
	for (const ToleranceKey& Each : EngineTolerances)
	{
		const std::string Name = std::string(Each.Section) + "." + Each.Key;
		INFO(Name);
		const std::string Overrides = "{\n  \"" + std::string(Each.Section) + "\": {\n    \"" + Each.Key + "\": 0.625\n  }\n}\n";
		const SettingsResult Applied = ApplyOverrides(Defaults, Overrides);
		INFO("error line " << Applied.Error.Line << ": " << Applied.Error.Message);
		CHECK(Applied.Ok());
		CHECK(Each.Read(Applied.Value) == 0.625);
		for (const ToleranceKey& Other : EngineTolerances)
		{
			if (&Other != &Each)
			{
				INFO("untouched " << std::string(Other.Key));
				CHECK(Other.Read(Applied.Value) == Other.Default);
			}
		}
		const std::string Wrong = "{\n  \"" + std::string(Each.Section) + "\": {\n    \"" + Each.Key + "\": \"0.625\"\n  }\n}\n";
		const SettingsResult Typed = ApplyOverrides(Defaults, Wrong);
		CheckNames(Typed, Name, 3);
		CHECK(Typed.Error.Message.find("wrong type") != std::string::npos);
	}
}

TEST_CASE("Settings.UndecidedResolutionReadsItsTwoNamesAndRejectsAnyOther")
{
	const std::string Json = ReadSettingsFile();
	{
		const std::string Stands = Replace(Json, "\"undecided_resolution\": \"collapse\"", "\"undecided_resolution\": \"stands\"");
		CHECK(LoadOk(Stands).Cascade.OnUndecided == UndecidedResolution::Stands);
	}
	{
		const std::string Bad = Replace(Json, "\"undecided_resolution\": \"collapse\"", "\"undecided_resolution\": \"maybe\"");
		CheckNames(LoadSettings(Bad), "cascade.undecided_resolution", LineOf(Bad, "\"maybe\""));
	}
	{
		const std::string Wrong = Replace(Json, "\"undecided_resolution\": \"collapse\"", "\"undecided_resolution\": 1");
		CheckNames(LoadSettings(Wrong), "cascade.undecided_resolution", LineOf(Wrong, "\"undecided_resolution\""));
	}
	{
		const SettingsResult Applied = ApplyOverrides(Settings{}, "{\n  \"cascade\": {\n    \"undecided_resolution\": \"stands\"\n  }\n}\n");
		REQUIRE(Applied.Ok());
		CHECK(Applied.Value.Cascade.OnUndecided == UndecidedResolution::Stands);
		CheckNames(ApplyOverrides(Settings{}, "{\n  \"cascade\": {\n    \"undecided_resolution\": \"Collapse\"\n  }\n}\n"), "cascade.undecided_resolution", 3);
	}
}

TEST_CASE("Settings.AMissingKeyIsAnErrorNamingIt")
{
	const std::string Json = ReadSettingsFile();
	{
		const std::string Missing = Replace(Json, "    \"dense_dof_limit\": 600,\n", "");
		CheckNames(LoadSettings(Missing), "solver.dense_dof_limit", LineOf(Missing, "\"solver\""));
	}
	{
		const std::string Missing = Replace(Json, ",\n  \"debug\": {\n    \"record_passes\": false\n  }", "");
		CheckNames(LoadSettings(Missing), "debug", 1);
	}
	{
		const std::string Missing = Replace(Json, "    \"max_passes\": 32\n", "");
		const std::string Fixed = Replace(Missing, "\"crack_band_fraction\": 0.5,", "\"crack_band_fraction\": 0.5");
		CheckNames(LoadSettings(Fixed), "cascade.max_passes", LineOf(Fixed, "\"cascade\""));
	}
}

TEST_CASE("Settings.AnUnknownKeyIsAnErrorNamingIt")
{
	const std::string Json = ReadSettingsFile();
	{
		const std::string Unknown = Replace(Json, "    \"dense_dof_limit\": 600,\n", "    \"dense_dof_limit\": 600,\n    \"bogus\": 1,\n");
		CheckNames(LoadSettings(Unknown), "solver.bogus", LineOf(Unknown, "\"bogus\""));
	}
	{
		const std::string Unknown = Replace(Json, "  \"debug\": {", "  \"extra\": {},\n  \"debug\": {");
		CheckNames(LoadSettings(Unknown), "extra", LineOf(Unknown, "\"extra\""));
	}
	{
		const std::string Overrides = "{\n  \"cascade\": {\n    \"max_pases\": 1\n  }\n}\n";
		CheckNames(ApplyOverrides(Settings{}, Overrides), "cascade.max_pases", 3);
	}
}

TEST_CASE("Settings.AWrongTypeIsAnErrorNamingIt")
{
	const std::string Json = ReadSettingsFile();
	{
		const std::string Wrong = Replace(Json, "\"max_passes\": 32", "\"max_passes\": \"32\"");
		CheckNames(LoadSettings(Wrong), "cascade.max_passes", LineOf(Wrong, "\"max_passes\""));
	}
	{
		const std::string Wrong = Replace(Json, "\"record_passes\": false", "\"record_passes\": 1");
		CheckNames(LoadSettings(Wrong), "debug.record_passes", LineOf(Wrong, "\"record_passes\""));
	}
	{
		const std::string Wrong = Replace(Json, "\"dense_dof_limit\": 600", "\"dense_dof_limit\": 600.5");
		CheckNames(LoadSettings(Wrong), "solver.dense_dof_limit", LineOf(Wrong, "\"dense_dof_limit\""));
	}
	{
		const std::string Wrong = "{\n  \"solver\": 5,\n  \"cascade\": { \"crack_band_fraction\": 0.5, \"max_passes\": 32 },\n  \"debug\": { \"record_passes\": false }\n}\n";
		CheckNames(LoadSettings(Wrong), "solver", 2);
	}
	{
		const std::string Overrides = "{\n  \"debug\": {\n    \"record_passes\": \"yes\"\n  }\n}\n";
		CheckNames(ApplyOverrides(Settings{}, Overrides), "debug.record_passes", 3);
	}
}

TEST_CASE("Settings.ANonFiniteNumberIsAnError")
{
	const std::string Json = ReadSettingsFile();
	const std::string Huge = Replace(Json, "\"crack_band_fraction\": 0.5", "\"crack_band_fraction\": 1e999");
	const SettingsResult Loaded = LoadSettings(Huge);
	INFO("error line " << Loaded.Error.Line << ": " << Loaded.Error.Message);
	CHECK_FALSE(Loaded.Ok());
	CHECK(Loaded.Error.Line == LineOf(Huge, "1e999"));
	const SettingsResult Overridden = ApplyOverrides(Settings{}, "{\n  \"solver\": {\n    \"unconstrained_component\": -1e999\n  }\n}\n");
	INFO("override error line " << Overridden.Error.Line << ": " << Overridden.Error.Message);
	CHECK_FALSE(Overridden.Ok());
	CHECK(Overridden.Error.Line == 3);
}

TEST_CASE("Settings.OverridesChangeOnlyTheKeysTheyName")
{
	const Settings Defaults = LoadOk(ReadSettingsFile());
	const SettingsResult Applied = ApplyOverrides(Defaults, "{ \"cascade\": { \"crack_band_fraction\": 0.1 }, \"debug\": { \"record_passes\": true } }");
	INFO("error line " << Applied.Error.Line << ": " << Applied.Error.Message);
	REQUIRE(Applied.Ok());
	CHECK(Applied.Value.Cascade.CrackBandFraction == 0.1);
	CHECK(Applied.Value.Debug.bRecordPasses == true);
	Settings Expected = Defaults;
	Expected.Cascade.CrackBandFraction = 0.1;
	Expected.Debug.bRecordPasses = true;
	CHECK(SameSettings(Applied.Value, Expected));
	const SettingsResult Empty = ApplyOverrides(Defaults, "{}");
	REQUIRE(Empty.Ok());
	CHECK(SameSettings(Empty.Value, Defaults));
}

TEST_CASE("Settings.ResetIsTheDefaultsWithoutTheOverrides")
{
	const std::string Json = ReadSettingsFile();
	const Settings Defaults = LoadOk(Json);
	const SettingsResult Applied = ApplyOverrides(Defaults, "{ \"cascade\": { \"max_passes\": 1 }, \"solver\": { \"dry_noise_floor_relative\": 0.5 } }");
	REQUIRE(Applied.Ok());
	CHECK_FALSE(SameSettings(Applied.Value, Defaults));
	const Settings Reset = LoadOk(Json);
	CHECK(SameSettings(Reset, Defaults));
	CHECK(SameSettings(Reset, Settings{}));
}

TEST_CASE("Settings.ToOptionsCarriesEveryField")
{
	const Settings S = DistinctSettings();
	const Vec3 Gravity{1.0, 2.0, -3.0};
	const StandingOptions Standing = ToStandingOptions(S, Gravity);
	CHECK(Standing.Gravity.X == 1.0);
	CHECK(Standing.Gravity.Y == 2.0);
	CHECK(Standing.Gravity.Z == -3.0);
	CHECK(Standing.MaxActiveSetIterations == 7);
	CHECK(Standing.PivotToleranceRelative == 2e-3);
	CHECK(Standing.DenseDofLimit == 11);
	CHECK(Standing.StickMarginRelative == 3e-3);
	CHECK(Standing.HeldForceToleranceRelative == 4e-3);
	CHECK(Standing.DryNoiseFloorRelative == 5e-3);
	CHECK(Standing.NeutralLoadToleranceRelative == 6e-3);
	CHECK(Standing.AdmissibilityToleranceRelative == 7e-3);
	CHECK(Standing.UnconstrainedComponent == 8e-3);
	CHECK(Standing.DryRecloseFactor == 9e-3);
	CHECK(Standing.SparseAcceptanceMargin == 1.1e-2);
	CHECK(Standing.OpenTensionToleranceRelative == 1.5e-2);
	CHECK(Standing.CloseCompressionToleranceRelative == 1.6e-2);
	CHECK(Standing.Debug == &S.Debug);
	const CascadeOptions Cascade = ToCascadeOptions(S, Gravity);
	CHECK(Cascade.CrackBandFraction == 0.25);
	CHECK(Cascade.CrackBandEdgeRelative == 1.2e-2);
	CHECK(Cascade.CrushEdgeRelative == 1.3e-2);
	CHECK(Cascade.MotionAxisThresholdRelative == 1.4e-2);
	CHECK(Cascade.Authority.Standing.DryRecloseFactor == 9e-3);
	CHECK(Cascade.Authority.Standing.SparseAcceptanceMargin == 1.1e-2);
	CHECK(Cascade.Authority.Standing.OpenTensionToleranceRelative == 1.5e-2);
	CHECK(Cascade.Authority.Standing.CloseCompressionToleranceRelative == 1.6e-2);
	CHECK(Cascade.MaxPasses == 3);
	CHECK(Cascade.Authority.Standing.Gravity.Z == -3.0);
	CHECK(Cascade.Authority.Standing.DryNoiseFloorRelative == 5e-3);
	CHECK(Cascade.Authority.Standing.MaxActiveSetIterations == 7);
	CHECK(Cascade.Authority.Standing.Debug == &S.Debug);
	const StandingOptions Plain = ToStandingOptions(Settings{}, Vec3{0.0, 0.0, -GravityMPerS2});
	const StandingOptions Compiled{};
	CHECK(Plain.MaxActiveSetIterations == Compiled.MaxActiveSetIterations);
	CHECK(Plain.PivotToleranceRelative == Compiled.PivotToleranceRelative);
	CHECK(Plain.DenseDofLimit == Compiled.DenseDofLimit);
	CHECK(Plain.StickMarginRelative == Compiled.StickMarginRelative);
	CHECK(Plain.HeldForceToleranceRelative == Compiled.HeldForceToleranceRelative);
	CHECK(Plain.DryNoiseFloorRelative == Compiled.DryNoiseFloorRelative);
	CHECK(Plain.NeutralLoadToleranceRelative == Compiled.NeutralLoadToleranceRelative);
	CHECK(Plain.AdmissibilityToleranceRelative == Compiled.AdmissibilityToleranceRelative);
	CHECK(Plain.UnconstrainedComponent == Compiled.UnconstrainedComponent);
	CHECK(Plain.DryRecloseFactor == Compiled.DryRecloseFactor);
	CHECK(Plain.SparseAcceptanceMargin == Compiled.SparseAcceptanceMargin);
	CHECK(Plain.OpenTensionToleranceRelative == Compiled.OpenTensionToleranceRelative);
	CHECK(Plain.CloseCompressionToleranceRelative == Compiled.CloseCompressionToleranceRelative);
	const CascadeOptions PlainCascade = ToCascadeOptions(Settings{}, Vec3{0.0, 0.0, -GravityMPerS2});
	const CascadeOptions CompiledCascade{};
	CHECK(PlainCascade.CrackBandEdgeRelative == CompiledCascade.CrackBandEdgeRelative);
	CHECK(PlainCascade.CrushEdgeRelative == CompiledCascade.CrushEdgeRelative);
	CHECK(PlainCascade.MotionAxisThresholdRelative == CompiledCascade.MotionAxisThresholdRelative);
}

TEST_CASE("Statics.Settings.TheSolverReadsItsTolerancesFromTheOptions")
{
	const DryStack S = BuildDryTwoBrickStack();
	const StandingResult Default = SolveWith(S.Model, StandingOptions{});
	INFO("default " << Default.State << " after " << Default.ActiveSetIterations << " iterations, open corners " << OpenCorners(Default));
	REQUIRE(Default.State == StandingState::Stands);
	REQUIRE(OpenCorners(Default) == 0);
	StandingOptions Absurd;
	Absurd.DryNoiseFloorRelative = 0.5;
	const StandingResult Noisy = SolveWith(S.Model, Absurd);
	INFO("absurd floor " << Noisy.State << " after " << Noisy.ActiveSetIterations << " iterations, open corners " << OpenCorners(Noisy));
	CHECK(OpenCorners(Noisy) > 0);
	CHECK(Noisy.State != StandingState::Stands);
}
