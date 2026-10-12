#include "CatalogueTestSupport.h"
#include "OwnFieldTestSupport.h"
#include "SlowInDebug.h"
#include "StaticAdmissibilityOracle.h"
#include "StaticsTestSupport.h"

#include "loadpath/Catalogue.h"
#include "loadpath/geometry/RunningBondWall.h"

#include "doctest.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double DryFriction = 0.7;

struct DryWall
{
	Structure Model;
	geometry::Wall Value;
};

DryWall BuildDryWall(int Courses, int BricksPerCourse)
{
	DryWall W;
	geometry::WallSpec Spec;
	Spec.Bed = AddProfileOk(W.Model, DryJoint(DryFriction));
	Spec.Perpend = AddProfileOk(W.Model, DryJoint(DryFriction));
	Spec.Courses = Courses;
	Spec.BricksPerCourse = BricksPerCourse;
	W.Value = geometry::BuildRunningBondWall(W.Model, Spec);
	return W;
}

StandingOptions Biaxial(double LambdaX, double LambdaY)
{
	StandingOptions Options;
	Options.Gravity = oracle::BiaxialGravity(LambdaX, LambdaY);
	return Options;
}

struct LateralPoint
{
	int Courses, BricksPerCourse;
	double LambdaX, LambdaY;
	int IterationBar;
};
}

TEST_CASE("Statics.Dry.AReclosingCornerWithBuiltUpSlipEntersSlipping")
{
	const DryWall W = BuildDryWall(3, 4);
	REQUIRE(W.Value.bComplete);
	const StandingResult Result = SolveWith(W.Model, Biaxial(0.1, 0.0));
	INFO("3 x 4 dry wall at lambda x 0.1: " << Result.State << " after " << Result.ActiveSetIterations << " iterations");
	REQUIRE(Result.State == StandingState::Stands);
	int SlippingCorners = 0;
	for (const InterfaceId& Id : W.Value.Perpends)
	{
		const InterfaceReading* Reading = FindReading(Result, Id);
		REQUIRE(Reading != nullptr);
		for (const CornerReading& C : Reading->Corners)
		{
			if (!C.bSlipping)
			{
				continue;
			}
			++SlippingCorners;
			CAPTURE(C.NormalForceN);
			CHECK_FALSE(C.bOpen);
			CHECK(C.NormalForceN > 0.0);
			CHECK(Length(C.ShearForceN) == doctest::Approx(DryFriction * C.NormalForceN).epsilon(1e-9));
		}
	}
	CHECK(SlippingCorners > 0);
}

TEST_CASE("Statics.Lime.AReclosingCohesiveCornerStaysOutOfSlipping")
{
	const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
	INFO("line " << Loaded.Error.Line << ": " << Loaded.Error.Message);
	REQUIRE(Loaded.Ok());
	const StrengthProfile Lime = JointOrFail(Loaded.Value, "lime_mortar_handmade_brick_upc_mean").Profile;
	REQUIRE(Lime.TensileStrengthPa == 0.0);
	REQUIRE(Lime.ShearCohesionPa > 0.0);
	Structure Model;
	geometry::WallSpec Spec;
	Spec.Bed = AddProfileOk(Model, Lime);
	Spec.Perpend = AddProfileOk(Model, Lime);
	Spec.Courses = 4;
	Spec.BricksPerCourse = 5;
	const geometry::Wall W = geometry::BuildRunningBondWall(Model, Spec);
	REQUIRE(W.bComplete);
	const StandingResult Result = SolveWith(Model, Biaxial(0.1, 0.0));
	INFO("4 x 5 lime wall at lambda x 0.1: " << Result.State << " after " << Result.ActiveSetIterations << " iterations");
	REQUIRE(Result.State == StandingState::Stands);
	int Slipping = 0, OpenCorners = 0, AboveFriction = 0, AboveCohesive = 0;
	double WorstExcessN = 0.0, WorstOverCohesive = 0.0;
	for (const InterfaceReading& R : Result.Readings)
	{
		const double CohesionN = Lime.ShearCohesionPa * Model.Interface(R.Id).AreaM2 / 4.0;
		for (const CornerReading& C : R.Corners)
		{
			Slipping += C.bSlipping ? 1 : 0;
			OpenCorners += C.bOpen ? 1 : 0;
			const double ShearN = Length(C.ShearForceN);
			const double FrictionN = Lime.FrictionCoefficient * std::max(C.NormalForceN, 0.0);
			if (C.bOpen || !(ShearN > FrictionN))
			{
				continue;
			}
			++AboveFriction;
			WorstExcessN = std::max(WorstExcessN, ShearN - FrictionN);
			WorstOverCohesive = std::max(WorstOverCohesive, ShearN / (CohesionN + FrictionN));
			AboveCohesive += ShearN > CohesionN + FrictionN ? 1 : 0;
		}
	}
	MESSAGE("4 x 5 lime wall: " << Result.ActiveSetIterations << " iterations, " << OpenCorners << " open, " << Slipping << " slipping, " << AboveFriction << " closed corners above mu N, worst excess " << WorstExcessN << " N, worst shear over c A + mu N " << WorstOverCohesive);
	CHECK(Slipping == 0);
	CHECK(OpenCorners > 0);
	CHECK(AboveFriction > 0);
	CHECK(AboveCohesive == 0);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Statics.Dry.LateralWallsStand")
{
	for (const LateralPoint P : {LateralPoint{3, 4, 0.1, 0.0, 20}, LateralPoint{5, 5, 0.05, 0.0, 20}, LateralPoint{4, 2, 0.30, 0.15, 20}, LateralPoint{10, 8, 0.2, 0.0, 30}})
	{
		CAPTURE(P.Courses);
		CAPTURE(P.BricksPerCourse);
		CAPTURE(P.LambdaX);
		CAPTURE(P.LambdaY);
		const DryWall W = BuildDryWall(P.Courses, P.BricksPerCourse);
		REQUIRE(W.Value.bComplete);
		const StandingResult Result = SolveWith(W.Model, Biaxial(P.LambdaX, P.LambdaY));
		INFO(P.Courses << " x " << P.BricksPerCourse << " at (" << P.LambdaX << ", " << P.LambdaY << "): " << Result.State << " after " << Result.ActiveSetIterations << " iterations");
		CHECK(Result.State == StandingState::Stands);
		CHECK(Result.ActiveSetIterations <= P.IterationBar);
	}
}

TEST_CASE("Statics.Dry.NamedBridgesStandOnlyWithTheHeldForceAlongTheTrialShear")
{
	struct NamedDraw
	{
		int Draw;
		int IterationBar;
	};
	for (const NamedDraw N : {NamedDraw{3329, 30}, NamedDraw{3253, 20}})
	{
		const oracle::SeededDraw D = oracle::MakeSeededDraw(N.Draw);
		const StandingResult Result = SolveWith(D.Model, Biaxial(D.LambdaX, D.LambdaY));
		INFO("draw " << N.Draw << " (seed " << D.Seed << "): " << D.Fixture << ", mu " << D.Friction << ", lambda (" << D.LambdaX << ", " << D.LambdaY << "): " << Result.State << " after " << Result.ActiveSetIterations << " iterations");
		CHECK(D.Fixture.rfind("bridge", 0) == 0);
		CHECK(Result.State == StandingState::Stands);
		CHECK(Result.ActiveSetIterations <= N.IterationBar);
	}
}

namespace
{
struct GridSize
{
	int Courses, BricksPerCourse;
};

constexpr GridSize GridSizes[] = {{1, 5}, {2, 5}, {3, 4}, {3, 5}, {4, 2}, {5, 4}, {5, 5}, {10, 8}};
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Statics.Dry.TheLateralGridStandsInItsOwnField")
{
	const auto Start = std::chrono::steady_clock::now();
	int Standing = 0, Points = 0, FieldFailures = 0, WorstIterations = 0;
	double WorstResidualOverW = 0.0, WorstConeExcessOverW = 0.0;
	for (const GridSize S : GridSizes)
	{
		const DryWall W = BuildDryWall(S.Courses, S.BricksPerCourse);
		REQUIRE(W.Value.bComplete);
		for (const double LambdaX : {0.0, 0.05, 0.1, 0.2, 0.3})
		{
			for (const double LambdaY : {0.0, 0.05, 0.1, 0.15})
			{
				++Points;
				const StandingOptions Options = Biaxial(LambdaX, LambdaY);
				const StandingResult Result = SolveWith(W.Model, Options);
				INFO(S.Courses << " x " << S.BricksPerCourse << " at (" << LambdaX << ", " << LambdaY << "): " << Result.State << " after " << Result.ActiveSetIterations << " iterations");
				CHECK(Result.State == StandingState::Stands);
				CHECK(Result.ActiveSetIterations <= 50);
				if (LambdaX == 0.0 && LambdaY == 0.0)
				{
					CHECK(Result.ActiveSetIterations <= 2);
				}
				if (Result.State != StandingState::Stands)
				{
					continue;
				}
				++Standing;
				WorstIterations = std::max(WorstIterations, Result.ActiveSetIterations);
				const OwnFieldCheck Field = CheckOwnField(W.Model, Result, Options.Gravity);
				INFO("own field: residual " << Field.WorstResidualOverW << " W, tension " << Field.WorstTensionOverW << " W, cone excess " << Field.WorstConeExcessOverW << " W");
				CHECK(Field.Violations == 0);
				FieldFailures += Field.Violations == 0 ? 0 : 1;
				WorstResidualOverW = std::max(WorstResidualOverW, Field.WorstResidualOverW);
				WorstConeExcessOverW = std::max(WorstConeExcessOverW, Field.WorstConeExcessOverW);
			}
		}
	}
	const double Seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - Start).count();
	MESSAGE("lateral dry grid: " << Points << " points, " << Standing << " Stand, worst " << WorstIterations << " iterations, " << FieldFailures << " fail their own field, worst residual " << WorstResidualOverW << " W, worst cone excess " << WorstConeExcessOverW << " W, " << Seconds << " s");
	CHECK(Points == 160);
}

TEST_CASE("Slow.Statics.Dry.TheLateralGridSubsetTheOracleAgrees")
{
	struct Lambda
	{
		double X, Y;
	};
	const auto Start = std::chrono::steady_clock::now();
	int Points = 0, Refuted = 0;
	for (const GridSize S : GridSizes)
	{
		const DryWall W = BuildDryWall(S.Courses, S.BricksPerCourse);
		REQUIRE(W.Value.bComplete);
		for (const Lambda L : {Lambda{0.1, 0.0}, Lambda{0.3, 0.15}, Lambda{0.2, 0.05}})
		{
			++Points;
			const StandingResult Result = SolveWith(W.Model, Biaxial(L.X, L.Y));
			INFO(S.Courses << " x " << S.BricksPerCourse << " at (" << L.X << ", " << L.Y << "): " << Result.State << " after " << Result.ActiveSetIterations << " iterations");
			REQUIRE(Result.State == StandingState::Stands);
			const oracle::OracleVerdict Oracle = oracle::StaticAdmissibility(W.Model, oracle::BiaxialGravity(L.X, L.Y));
			INFO("oracle " << Oracle.Verdict << ", inner residual / demand " << Oracle.InnerResidualOverDemand);
			CHECK(Oracle.Verdict == oracle::Admissibility::CanStand);
			Refuted += Oracle.Verdict == oracle::Admissibility::CanStand ? 0 : 1;
		}
	}
	const double Seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - Start).count();
	MESSAGE("lateral dry grid subset: " << Points << " points, " << Refuted << " not CanStand by the oracle, " << Seconds << " s");
	CHECK(Points == 24);
}

TEST_CASE("Slow.Statics.Dry.TheSparsePathWallStandsUnderLateralLoad")
{
	const DryWall W = BuildDryWall(14, 10);
	REQUIRE(W.Value.bComplete);
	REQUIRE(6 * static_cast<int>(W.Model.LiveBodies().size()) > StandingOptions{}.DenseDofLimit);
	const StandingResult Result = SolveWith(W.Model, Biaxial(0.2, 0.1));
	INFO("14 x 10 dry wall at (0.2, 0.1): " << Result.State << " after " << Result.ActiveSetIterations << " iterations");
	CHECK(Result.State == StandingState::Stands);
	CHECK(Result.ActiveSetIterations <= 50);
}
