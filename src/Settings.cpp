#include "loadpath/Settings.h"

#include "JsonReader.h"

#include <cmath>
#include <initializer_list>
#include <limits>
#include <string>

namespace loadpath
{
namespace
{
using namespace detail;

class SettingsReader
{
public:
	SettingsReader(bool bRequireAll, FileError& Error)
		: bRequireAll(bRequireAll)
		, Error(Error)
	{
	}

	bool Read(const JsonValue& Root, Settings& Out)
	{
		const JsonValue* Solver = nullptr;
		const JsonValue* Cascade = nullptr;
		const JsonValue* Debug = nullptr;
		return OnlyKeys(Root, "", {"solver", "cascade", "debug"}) && Section(Root, "solver", Solver) && Section(Root, "cascade", Cascade) &&
			Section(Root, "debug", Debug) && ReadSolver(Solver, Out.Solver) && ReadCascade(Cascade, Out.Cascade) && ReadDebug(Debug, Out.Debug);
	}

private:
	bool Fail(int Line, const std::string& Message)
	{
		Error = {Line, Message};
		return false;
	}

	bool OnlyKeys(const JsonValue& Object, std::string_view Path, std::initializer_list<std::string_view> Allowed)
	{
		for (std::size_t Index = 0; Index < Object.Keys.size(); ++Index)
		{
			const std::string& Key = Object.Keys[Index];
			if (std::find(Allowed.begin(), Allowed.end(), Key) == Allowed.end())
			{
				return Fail(Object.Items[Index].Line, "unknown key " + Qualified(Path, Key));
			}
		}
		return true;
	}

	static std::string Qualified(std::string_view Path, std::string_view Key)
	{
		return Path.empty() ? std::string(Key) : std::string(Path) + "." + std::string(Key);
	}

	bool Get(const JsonValue& Object, std::string_view Path, std::string_view Key, JsonType Type, const JsonValue*& Out)
	{
		Out = Find(Object, Key);
		if (Out == nullptr)
		{
			return !bRequireAll || Fail(Object.Line, "missing key " + Qualified(Path, Key));
		}
		if (Out->Type != Type)
		{
			return Fail(Out->Line, Qualified(Path, Key) + " has the wrong type");
		}
		return true;
	}

	bool Section(const JsonValue& Root, std::string_view Key, const JsonValue*& Out) { return Get(Root, "", Key, JsonType::Object, Out); }

	bool Double(const JsonValue* Object, std::string_view Path, std::string_view Key, double& Out)
	{
		const JsonValue* Value = nullptr;
		if (Object == nullptr || !Get(*Object, Path, Key, JsonType::Number, Value))
		{
			return Object == nullptr;
		}
		if (Value == nullptr)
		{
			return true;
		}
		if (!std::isfinite(Value->Number))
		{
			return Fail(Value->Line, Qualified(Path, Key) + " is not finite");
		}
		Out = Value->Number;
		return true;
	}

	bool Int(const JsonValue* Object, std::string_view Path, std::string_view Key, int& Out)
	{
		double Number = static_cast<double>(Out);
		if (!Double(Object, Path, Key, Number))
		{
			return false;
		}
		const double Rounded = std::nearbyint(Number);
		if (Rounded != Number || std::abs(Number) > static_cast<double>(std::numeric_limits<int>::max()))
		{
			return Fail(Find(*Object, Key)->Line, Qualified(Path, Key) + " must be an integer");
		}
		Out = static_cast<int>(Rounded);
		return true;
	}

	bool Bool(const JsonValue* Object, std::string_view Path, std::string_view Key, bool& Out)
	{
		const JsonValue* Value = nullptr;
		if (Object == nullptr || !Get(*Object, Path, Key, JsonType::Bool, Value))
		{
			return Object == nullptr;
		}
		if (Value != nullptr)
		{
			Out = Value->Bool;
		}
		return true;
	}

	bool Resolution(const JsonValue* Object, std::string_view Path, std::string_view Key, UndecidedResolution& Out)
	{
		const JsonValue* Value = nullptr;
		if (Object == nullptr || !Get(*Object, Path, Key, JsonType::String, Value))
		{
			return Object == nullptr;
		}
		if (Value == nullptr)
		{
			return true;
		}
		if (Value->Text == "collapse")
		{
			Out = UndecidedResolution::Collapse;
			return true;
		}
		if (Value->Text == "stands")
		{
			Out = UndecidedResolution::Stands;
			return true;
		}
		return Fail(Value->Line, Qualified(Path, Key) + " must be \"collapse\" or \"stands\"");
	}

	bool ReadSolver(const JsonValue* Object, SolverSettings& Out)
	{
		const std::string_view Path = "solver";
		return (Object == nullptr ||
				OnlyKeys(*Object, Path,
					{"max_active_set_iterations", "pivot_tolerance_relative", "dense_dof_limit", "stick_margin_relative", "held_force_tolerance_relative",
						"dry_noise_floor_relative", "neutral_load_tolerance_relative", "admissibility_tolerance_relative", "unconstrained_component", "dry_reclose_factor",
							"sparse_acceptance_margin", "open_tension_tolerance_relative", "close_compression_tolerance_relative", "restore_closing_rate_threshold"})) &&
			Int(Object, Path, "max_active_set_iterations", Out.MaxActiveSetIterations) && Double(Object, Path, "pivot_tolerance_relative", Out.PivotToleranceRelative) &&
			Int(Object, Path, "dense_dof_limit", Out.DenseDofLimit) && Double(Object, Path, "stick_margin_relative", Out.StickMarginRelative) &&
			Double(Object, Path, "held_force_tolerance_relative", Out.HeldForceToleranceRelative) && Double(Object, Path, "dry_noise_floor_relative", Out.DryNoiseFloorRelative) &&
			Double(Object, Path, "neutral_load_tolerance_relative", Out.NeutralLoadToleranceRelative) &&
			Double(Object, Path, "admissibility_tolerance_relative", Out.AdmissibilityToleranceRelative) && Double(Object, Path, "unconstrained_component", Out.UnconstrainedComponent) &&
			Double(Object, Path, "dry_reclose_factor", Out.DryRecloseFactor) && Double(Object, Path, "sparse_acceptance_margin", Out.SparseAcceptanceMargin) &&
			Double(Object, Path, "open_tension_tolerance_relative", Out.OpenTensionToleranceRelative) &&
			Double(Object, Path, "close_compression_tolerance_relative", Out.CloseCompressionToleranceRelative) &&
			Double(Object, Path, "restore_closing_rate_threshold", Out.RestoreClosingRateThreshold);
	}

	bool ReadCascade(const JsonValue* Object, CascadeSettings& Out)
	{
		const std::string_view Path = "cascade";
		return (Object == nullptr || OnlyKeys(*Object, Path, {"undecided_resolution", "crack_band_fraction", "crack_band_edge_relative", "crush_edge_relative",
										  "motion_axis_threshold_relative", "max_passes"})) &&
			Resolution(Object, Path, "undecided_resolution", Out.OnUndecided) && Double(Object, Path, "crack_band_fraction", Out.CrackBandFraction) &&
			Double(Object, Path, "crack_band_edge_relative", Out.CrackBandEdgeRelative) && Double(Object, Path, "crush_edge_relative", Out.CrushEdgeRelative) &&
			Double(Object, Path, "motion_axis_threshold_relative", Out.MotionAxisThresholdRelative) && Int(Object, Path, "max_passes", Out.MaxPasses);
	}

	bool ReadDebug(const JsonValue* Object, DebugSettings& Out)
	{
		const std::string_view Path = "debug";
		return (Object == nullptr || OnlyKeys(*Object, Path, {"record_passes"})) && Bool(Object, Path, "record_passes", Out.bRecordPasses);
	}

	bool bRequireAll;
	FileError& Error;
};

SettingsResult ReadSettings(const Settings& Start, std::string_view Json, bool bRequireAll)
{
	SettingsResult Result;
	Result.Value = Start;
	JsonParser Parser(Json);
	JsonValue Root;
	if (!Parser.ParseDocument(Root))
	{
		Result.Error = Parser.Error;
		Result.Value = Settings{};
		return Result;
	}
	if (!SettingsReader(bRequireAll, Result.Error).Read(Root, Result.Value))
	{
		Result.Value = Settings{};
	}
	return Result;
}
}

SettingsResult LoadSettings(std::string_view DefaultsJson) { return ReadSettings(Settings{}, DefaultsJson, true); }

SettingsResult ApplyOverrides(const Settings& Defaults, std::string_view OverridesJson) { return ReadSettings(Defaults, OverridesJson, false); }

StandingOptions ToStandingOptions(const Settings& Value, Vec3 Gravity)
{
	StandingOptions Options;
	Options.Gravity = Gravity;
	Options.MaxActiveSetIterations = Value.Solver.MaxActiveSetIterations;
	Options.PivotToleranceRelative = Value.Solver.PivotToleranceRelative;
	Options.DenseDofLimit = Value.Solver.DenseDofLimit;
	Options.StickMarginRelative = Value.Solver.StickMarginRelative;
	Options.HeldForceToleranceRelative = Value.Solver.HeldForceToleranceRelative;
	Options.DryNoiseFloorRelative = Value.Solver.DryNoiseFloorRelative;
	Options.NeutralLoadToleranceRelative = Value.Solver.NeutralLoadToleranceRelative;
	Options.AdmissibilityToleranceRelative = Value.Solver.AdmissibilityToleranceRelative;
	Options.UnconstrainedComponent = Value.Solver.UnconstrainedComponent;
	Options.DryRecloseFactor = Value.Solver.DryRecloseFactor;
	Options.SparseAcceptanceMargin = Value.Solver.SparseAcceptanceMargin;
	Options.OpenTensionToleranceRelative = Value.Solver.OpenTensionToleranceRelative;
	Options.CloseCompressionToleranceRelative = Value.Solver.CloseCompressionToleranceRelative;
	Options.RestoreClosingRateThreshold = Value.Solver.RestoreClosingRateThreshold;
	Options.Debug = &Value.Debug;
	return Options;
}

CascadeOptions ToCascadeOptions(const Settings& Value, Vec3 Gravity)
{
	CascadeOptions Options;
	Options.Authority.Standing = ToStandingOptions(Value, Gravity);
	Options.CrackBandFraction = Value.Cascade.CrackBandFraction;
	Options.CrackBandEdgeRelative = Value.Cascade.CrackBandEdgeRelative;
	Options.CrushEdgeRelative = Value.Cascade.CrushEdgeRelative;
	Options.MotionAxisThresholdRelative = Value.Cascade.MotionAxisThresholdRelative;
	Options.MaxPasses = Value.Cascade.MaxPasses;
	Options.OnUndecided = Value.Cascade.OnUndecided;
	return Options;
}
}
