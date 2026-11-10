#include "BitwiseTestSupport.h"
#include "CatalogueTestSupport.h"
#include "StaticsTestSupport.h"

#include "loadpath/Catalogue.h"
#include "loadpath/Settings.h"
#include "loadpath/geometry/RunningBondWall.h"

#include "doctest.h"

#include <cmath>
#include <cstddef>
#include <optional>
#include <span>
#include <utility>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
const DebugSettings RecordingOn{true};

StrengthProfile LimeProfile()
{
	const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
	INFO("line " << Loaded.Error.Line << ": " << Loaded.Error.Message);
	REQUIRE(Loaded.Ok());
	const StrengthProfile Lime = JointOrFail(Loaded.Value, "lime_mortar_handmade_brick_upc_mean").Profile;
	REQUIRE(Lime.TensileStrengthPa == 0.0);
	REQUIRE(Lime.ShearCohesionPa > 0.0);
	return Lime;
}

struct PulledBlock
{
	Structure Model;
	BodyId Block;
	InterfaceId Bed;
	double WeightN = 0.0;
	double PullN = 0.0;
	double ForceScaleN = 0.0;
	double CornerTensionN = 0.0;
	std::vector<BodyLoad> Loads;
};

PulledBlock BuildPulledBlock(double Excess)
{
	PulledBlock F;
	const ProfileId Profile = AddProfileOk(F.Model, LimeProfile());
	const BodyId Ground = AddGroundPlane(F.Model);
	const Box Shape = BoxOfDensity(Vec3{0.0, 0.0, 0.05}, Vec3{0.1, 0.05, 0.05}, 2000.0);
	F.Block = AddBox(F.Model, Shape);
	F.Bed = AddInterfaceOk(F.Model, OnGroundPlane(Ground, F.Block, Shape, Profile));
	F.WeightN = Shape.MassKg * GravityMPerS2;
	F.PullN = (1.0 + Excess) * F.WeightN;
	F.ForceScaleN = F.WeightN + F.PullN;
	F.CornerTensionN = (F.PullN - F.WeightN) / 4.0;
	F.Loads = {BodyLoad{F.Block, Vec3{0.0, 0.0, F.PullN}, Shape.CentreM}};
	return F;
}

constexpr double PullExcess = 8e-5;

StandingOptions WithOpenTolerance(double Relative)
{
	StandingOptions Options;
	Options.OpenTensionToleranceRelative = Relative;
	return Options;
}

StandingOptions LimeLateral(double CloseRelative)
{
	StandingOptions Options = LateralLoad(0.1);
	Options.Debug = &RecordingOn;
	Options.CloseCompressionToleranceRelative = CloseRelative;
	return Options;
}

struct LimeWall
{
	Structure Model;
	geometry::Wall Value;
};

LimeWall BuildLimeWall(int Courses, int BricksPerCourse)
{
	LimeWall W;
	const StrengthProfile Lime = LimeProfile();
	geometry::WallSpec Spec;
	Spec.Bed = AddProfileOk(W.Model, Lime);
	Spec.Perpend = AddProfileOk(W.Model, Lime);
	Spec.Courses = Courses;
	Spec.BricksPerCourse = BricksPerCourse;
	W.Value = geometry::BuildRunningBondWall(W.Model, Spec);
	return W;
}

const std::vector<ElasticPassRecord>& RecordedPasses(const StandingResult& Result)
{
	REQUIRE(Result.Components.size() == 1u);
	return Result.Components.front().Passes;
}

struct Reclosure
{
	std::size_t Pass = 0;
	std::vector<std::pair<std::size_t, std::size_t>> Corners;
};

std::optional<Reclosure> FirstReclosure(const std::vector<ElasticPassRecord>& Passes)
{
	for (std::size_t K = 1; K < Passes.size(); ++K)
	{
		Reclosure Found;
		Found.Pass = K;
		for (std::size_t I = 0; I < Passes[K].Joints.size(); ++I)
		{
			for (std::size_t C = 0; C < 4; ++C)
			{
				if (Passes[K - 1].Joints[I].Corners[C] == CornerState::Open && Passes[K].Joints[I].Corners[C] != CornerState::Open)
				{
					Found.Corners.emplace_back(I, C);
				}
			}
		}
		if (!Found.Corners.empty())
		{
			return Found;
		}
	}
	return std::nullopt;
}

bool IsReclosed(const Reclosure& R, std::size_t Joint, std::size_t Corner)
{
	for (const auto& [I, C] : R.Corners)
	{
		if (I == Joint && C == Corner)
		{
			return true;
		}
	}
	return false;
}

void CheckSameStatesBefore(const std::vector<ElasticPassRecord>& Expected, const std::vector<ElasticPassRecord>& Got, std::size_t Pass)
{
	REQUIRE(Got.size() >= Pass);
	REQUIRE(Expected.size() >= Pass);
	for (std::size_t K = 0; K < Pass; ++K)
	{
		REQUIRE(Got[K].Joints.size() == Expected[K].Joints.size());
		for (std::size_t I = 0; I < Expected[K].Joints.size(); ++I)
		{
			for (std::size_t C = 0; C < 4; ++C)
			{
				INFO("pass " << Expected[K].Iteration << " joint " << I << " corner " << C);
				CHECK(Got[K].Joints[I].Corners[C] == Expected[K].Joints[I].Corners[C]);
			}
		}
	}
}

StandingResult SolveDry(const Structure& Model, StandingOptions Options, double OpenRelative, double CloseRelative, std::span<const BodyLoad> Loads = {})
{
	Options.OpenTensionToleranceRelative = OpenRelative;
	Options.CloseCompressionToleranceRelative = CloseRelative;
	return SolveWith(Model, Options, Loads);
}
}

TEST_CASE("Statics.NonDry.AClosedCornerHoldsTensionBelowTheOpenToleranceAndOpensPastIt")
{
	const PulledBlock F = BuildPulledBlock(PullExcess);
	const double TensionRelative = F.CornerTensionN / F.ForceScaleN;
	INFO("corner tension " << F.CornerTensionN << " N, " << TensionRelative << " of the force scale " << F.ForceScaleN << " N");
	REQUIRE(TensionRelative > 1e-6);
	REQUIRE(TensionRelative < 1e-4);
	const StandingResult Held = SolveWith(F.Model, WithOpenTolerance(1e-4), F.Loads);
	const InterfaceReading* HeldBed = FindReading(Held, F.Bed);
	REQUIRE(HeldBed != nullptr);
	INFO("open tolerance 1e-4: " << Held.State);
	for (const CornerReading& C : HeldBed->Corners)
	{
		CHECK_FALSE(C.bOpen);
	}
	const StandingResult Opened = SolveWith(F.Model, WithOpenTolerance(1e-6), F.Loads);
	const InterfaceReading* OpenedBed = FindReading(Opened, F.Bed);
	REQUIRE(OpenedBed != nullptr);
	INFO("open tolerance 1e-6: " << Opened.State);
	for (const CornerReading& C : OpenedBed->Corners)
	{
		CHECK(C.bOpen);
	}
}

TEST_CASE("Statics.NonDry.TensionInsideTheOpenToleranceReadsZeroOnAClosedCorner")
{
	const PulledBlock F = BuildPulledBlock(PullExcess);
	const StandingResult Held = SolveWith(F.Model, WithOpenTolerance(1e-4), F.Loads);
	const InterfaceReading* Bed = FindReading(Held, F.Bed);
	REQUIRE(Bed != nullptr);
	INFO("open tolerance 1e-4: " << Held.State << "; corner tension " << F.CornerTensionN << " N; bed normal " << Bed->NormalForceN << " N, tension utilisation " << Bed->TensionUtilisation);
	CHECK(Held.State == StandingState::Stands);
	CHECK(std::isfinite(Bed->TensionUtilisation));
	CHECK(Bed->TensionUtilisation == 0.0);
	for (const CornerReading& C : Bed->Corners)
	{
		CAPTURE(C.NormalForceN);
		CHECK_FALSE(C.bOpen);
		CHECK(C.NormalForceN == 0.0);
	}
}

TEST_CASE("Statics.NonDry.AnOpenCornerStaysOpenBelowTheCloseTolerance")
{
	const LimeWall W = BuildLimeWall(1, 5);
	REQUIRE(W.Value.bComplete);
	const StandingResult Default = SolveWith(W.Model, LimeLateral(1e-12));
	const std::optional<Reclosure> First = FirstReclosure(RecordedPasses(Default));
	REQUIRE(First.has_value());
	const StandingResult Raised = SolveWith(W.Model, LimeLateral(1e-4));
	const std::vector<ElasticPassRecord>& Expected = RecordedPasses(Default);
	const std::vector<ElasticPassRecord>& Got = RecordedPasses(Raised);
	CheckSameStatesBefore(Expected, Got, First->Pass);
	REQUIRE(Got.size() > First->Pass);
	const ElasticPassRecord& At = Got[First->Pass];
	for (std::size_t I = 0; I < At.Joints.size(); ++I)
	{
		for (std::size_t C = 0; C < 4; ++C)
		{
			INFO("pass " << At.Iteration << " joint " << I << " corner " << C << " re-closes at the default close tolerance: " << IsReclosed(*First, I, C));
			if (IsReclosed(*First, I, C))
			{
				CHECK(At.Joints[I].Corners[C] == CornerState::Open);
			}
			else
			{
				CHECK(At.Joints[I].Corners[C] == Expected[First->Pass].Joints[I].Corners[C]);
			}
		}
	}
}

TEST_CASE("Statics.NonDry.AnOpenCornerReclosesAboveTheCloseTolerance")
{
	const LimeWall W = BuildLimeWall(1, 5);
	REQUIRE(W.Value.bComplete);
	const StandingResult Default = SolveWith(W.Model, LimeLateral(1e-12));
	const std::optional<Reclosure> First = FirstReclosure(RecordedPasses(Default));
	REQUIRE(First.has_value());
	const StandingResult Raised = SolveWith(W.Model, LimeLateral(1e-8));
	const std::vector<ElasticPassRecord>& Expected = RecordedPasses(Default);
	const std::vector<ElasticPassRecord>& Got = RecordedPasses(Raised);
	CheckSameStatesBefore(Expected, Got, First->Pass + 1);
	for (const auto& [I, C] : First->Corners)
	{
		INFO("joint " << I << " corner " << C);
		CHECK(Got[First->Pass].Joints[I].Corners[C] != CornerState::Open);
	}
}

TEST_CASE("Statics.Dry.CornersIgnoreTheNonDryOpenAndCloseTolerances")
{
	const StrengthProfile Dry = DryJoint(0.7);
	struct Case
	{
		int Courses, BricksPerCourse;
		double LambdaX, EndPushOfWeight;
	};
	for (const Case Each : {Case{1, 3, 0.0, 0.2}, Case{1, 3, 0.0, 1e-9}, Case{3, 4, 0.1, 0.0}})
	{
		CAPTURE(Each.Courses);
		CAPTURE(Each.BricksPerCourse);
		CAPTURE(Each.LambdaX);
		CAPTURE(Each.EndPushOfWeight);
		Structure Model;
		geometry::WallSpec Spec;
		Spec.Bed = AddProfileOk(Model, Dry);
		Spec.Perpend = AddProfileOk(Model, Dry);
		Spec.Courses = Each.Courses;
		Spec.BricksPerCourse = Each.BricksPerCourse;
		const geometry::Wall W = geometry::BuildRunningBondWall(Model, Spec);
		REQUIRE(W.bComplete);
		const BodyId End = W.Courses[0].front();
		const double WeightN = Spec.DensityKgPerM3 * Spec.BrickM.X * Spec.BrickM.Y * Spec.BrickM.Z * GravityMPerS2;
		std::vector<BodyLoad> Push;
		if (Each.EndPushOfWeight > 0.0)
		{
			Push.push_back(BodyLoad{End, Vec3{Each.EndPushOfWeight * WeightN, 0.0, 0.0}, Model.Body(End).CentroidM});
		}
		const StandingOptions Base = LateralLoad(Each.LambdaX);
		const StandingResult Plain = SolveDry(Model, Base, 1e-12, 1e-12, Push);
		const StandingResult Raised = SolveDry(Model, Base, 1e-2, 1e-2, Push);
		INFO("defaults: " << Plain.State << "; both tolerances 1e-2: " << Raised.State);
		CHECK(bitwise::Same(Raised, Plain));
	}
}

TEST_CASE("Statics.NonDry.PassRecordsReadTheSameOpenToleranceAsTheReadings")
{
	const PulledBlock F = BuildPulledBlock(PullExcess);
	StandingOptions Options = WithOpenTolerance(1e-4);
	Options.Debug = &RecordingOn;
	const StandingResult Held = SolveWith(F.Model, Options, F.Loads);
	const InterfaceReading* Bed = FindReading(Held, F.Bed);
	REQUIRE(Bed != nullptr);
	const std::vector<ElasticPassRecord>& Passes = RecordedPasses(Held);
	REQUIRE_FALSE(Passes.empty());
	for (const ElasticPassRecord& Pass : Passes)
	{
		REQUIRE(Pass.Joints.size() == 1u);
		const JointPassRecord& Joint = Pass.Joints.front();
		INFO("pass " << Pass.Iteration << ": recorded normal " << Joint.NormalForceN << " N, final reading " << Bed->NormalForceN << " N");
		for (std::size_t C = 0; C < 4; ++C)
		{
			CAPTURE(C);
			CHECK(Joint.CornerNormalN[C] == 0.0);
		}
		CHECK(Joint.NormalForceN == Bed->NormalForceN);
	}
}
