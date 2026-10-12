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
StrengthProfile LoadLime()
{
	const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
	REQUIRE(Loaded.Ok());
	return JointOrFail(Loaded.Value, "lime_mortar_handmade_brick_upc_mean").Profile;
}

struct LimeWall
{
	Structure Model;
	geometry::Wall Value;
};

LimeWall BuildWall(const StrengthProfile& Profile, int Courses, int BricksPerCourse)
{
	LimeWall W;
	geometry::WallSpec Spec;
	Spec.Bed = AddProfileOk(W.Model, Profile);
	Spec.Perpend = AddProfileOk(W.Model, Profile);
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

struct Extremes
{
	double MaxNormalN = 0.0;
	double MaxShearN = 0.0;
};

Extremes ReadExtremes(const StandingResult& Result)
{
	Extremes E;
	for (const InterfaceReading& R : Result.Readings)
	{
		for (const CornerReading& C : R.Corners)
		{
			E.MaxNormalN = std::max(E.MaxNormalN, std::abs(C.NormalForceN));
			E.MaxShearN = std::max(E.MaxShearN, Length(C.ShearForceN));
		}
	}
	return E;
}
}

struct LatePoint
{
	int Courses, BricksPerCourse;
	double LambdaX, LambdaY;
	int IterationBar;
};

struct PinnedPoint
{
	int Courses, BricksPerCourse;
	double LambdaX, LambdaY;
	int Iterations;
	double MaxNormalN, MaxShearN;
};

void CheckCohesiveField(const LimeWall& W, const StandingResult& Result, const StandingOptions& Options)
{
	const OwnFieldCheck Field = CheckOwnField(W.Model, Result, Options.Gravity, {}, true);
	INFO("cohesive own field: residual " << Field.WorstResidualOverW << " W, tension " << Field.WorstTensionOverW << " W, cone excess " << Field.WorstConeExcessOverW << " W, cohesive slipping " << Field.CohesiveSlipping);
	CHECK(Field.CohesiveSlipping == 0);
	CHECK(Field.Violations == 0);
}

TEST_CASE("Statics.Lime.LateralWallsStand")
{
	const StrengthProfile Lime = LoadLime();
	const LatePoint Points[] = {{4, 5, 0.8, 0.0, 12}, {4, 5, 0.8, 0.1, 12}, {5, 5, 0.8, 0.0, 12}, {10, 8, 0.2, 0.1, 12}, {10, 8, 0.5, 0.1, 12}, {10, 8, 0.8, 0.0, 20}, {10, 8, 0.8, 0.1, 12}};
	for (const LatePoint& P : Points)
	{
		const LimeWall W = BuildWall(Lime, P.Courses, P.BricksPerCourse);
		REQUIRE(W.Value.bComplete);
		const StandingOptions Options = Biaxial(P.LambdaX, P.LambdaY);
		const StandingResult Result = SolveWith(W.Model, Options);
		INFO(P.Courses << " x " << P.BricksPerCourse << " lime wall at (" << P.LambdaX << ", " << P.LambdaY << "): " << Result.State << " after " << Result.ActiveSetIterations << " iterations");
		CHECK(Result.State == StandingState::Stands);
		CHECK(Result.ActiveSetIterations <= P.IterationBar);
		if (Result.State == StandingState::Stands)
		{
			CheckCohesiveField(W, Result, Options);
		}
	}
}

TEST_CASE("Statics.Lime.ConvergedWallsAreUnchanged")
{
	const StrengthProfile Lime = LoadLime();
	const PinnedPoint Points[] = {
		{3, 4, 0.1, 0.0, 4, 22.83163006793832, 2.1484524713948074},
		{4, 5, 0.5, 0.0, 5, 42.725107133428025, 14.496238496799792},
		{5, 5, 0.5, 0.1, 3, 67.964814712879132, 18.580489263348561},
		{10, 8, 0.05, 0.0, 5, 76.890086341346773, 3.7303970138714044},
		{10, 8, 0.3, 0.0, 5, 110.15518781370804, 22.382382083228187},
		{5, 5, 0.8, 0.1, 5, 82.184191744052953, 31.623716336487416},
		{10, 8, 0.5, 0.0, 5, 137.10177394943292, 37.325653612931731}};
	for (const PinnedPoint& P : Points)
	{
		const LimeWall W = BuildWall(Lime, P.Courses, P.BricksPerCourse);
		REQUIRE(W.Value.bComplete);
		const StandingResult Result = SolveWith(W.Model, Biaxial(P.LambdaX, P.LambdaY));
		const Extremes E = ReadExtremes(Result);
		INFO(P.Courses << " x " << P.BricksPerCourse << " lime wall at (" << P.LambdaX << ", " << P.LambdaY << "): " << Result.State << " after " << Result.ActiveSetIterations << " iterations, max |N| " << E.MaxNormalN << " N, max |S| " << E.MaxShearN << " N");
		CHECK(Result.State == StandingState::Stands);
		CHECK(Result.ActiveSetIterations == P.Iterations);
		CHECK(E.MaxNormalN == doctest::Approx(P.MaxNormalN).epsilon(1e-12));
		CHECK(E.MaxShearN == doctest::Approx(P.MaxShearN).epsilon(1e-12));
	}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Statics.Lime.TheLateralGridStandsInItsOwnField")
{
	struct Size
	{
		int Courses, BricksPerCourse;
	};
	const auto Start = std::chrono::steady_clock::now();
	const StrengthProfile Lime = LoadLime();
	int Points = 0, Standing = 0, Undecided = 0, UnexpectedUndecided = 0, FieldFailures = 0;
	for (const Size S : {Size{3, 4}, Size{4, 5}, Size{5, 5}, Size{6, 8}, Size{10, 8}})
	{
		const LimeWall W = BuildWall(Lime, S.Courses, S.BricksPerCourse);
		REQUIRE(W.Value.bComplete);
		for (const double LambdaX : {0.05, 0.3, 0.6, 0.7, 0.9, 1.0, 1.2})
		{
			for (const double LambdaY : {0.0, 0.05, 0.1, 0.15})
			{
				++Points;
				const StandingOptions Options = Biaxial(LambdaX, LambdaY);
				const StandingResult Result = SolveWith(W.Model, Options);
				INFO(S.Courses << " x " << S.BricksPerCourse << " lime wall at (" << LambdaX << ", " << LambdaY << "): " << Result.State << " after " << Result.ActiveSetIterations << " iterations");
				if (Result.State == StandingState::Undecided)
				{
					++Undecided;
					const bool bKnownException = S.Courses == 10 && S.BricksPerCourse == 8 && LambdaX >= 1.0;
					CHECK_MESSAGE(bKnownException, "Undecided outside the known exceptions (10 x 8 at lambda x >= 1.0)");
					UnexpectedUndecided += bKnownException ? 0 : 1;
					continue;
				}
				if (Result.State != StandingState::Stands)
				{
					continue;
				}
				++Standing;
				const OwnFieldCheck Field = CheckOwnField(W.Model, Result, Options.Gravity, {}, true);
				INFO("cohesive own field: residual " << Field.WorstResidualOverW << " W, tension " << Field.WorstTensionOverW << " W, cone excess " << Field.WorstConeExcessOverW << " W, cohesive slipping " << Field.CohesiveSlipping);
				CHECK(Field.Violations == 0);
				FieldFailures += Field.Violations == 0 ? 0 : 1;
			}
		}
	}
	const double Seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - Start).count();
	MESSAGE("lateral lime grid: " << Points << " points, " << Standing << " Stand, " << Undecided << " Undecided (" << UnexpectedUndecided << " outside the known exceptions), " << FieldFailures << " fail their own field, " << Seconds << " s");
	CHECK(Points == 140);
	CHECK(Undecided <= 5);
	CHECK(UnexpectedUndecided == 0);
}

TEST_CASE("Statics.Lime.AForgottenShearAboveTheCohesiveCapacityIsKept")
{
	StrengthProfile Weak = LoadLime();
	Weak.ShearCohesionPa = 1000.0;
	const LimeWall W = BuildWall(Weak, 2, 5);
	REQUIRE(W.Value.bComplete);
	for (const double LambdaX : {0.3, 0.5, 0.7})
	{
		const StandingOptions Options = Biaxial(LambdaX, 0.0);
		const StandingResult Result = SolveWith(W.Model, Options);
		INFO("2 x 5 wall, c 1 kPa, at lambda x " << LambdaX << ": " << Result.State << " after " << Result.ActiveSetIterations << " iterations");
		CHECK(Result.State != StandingState::Stands);
	}
}
