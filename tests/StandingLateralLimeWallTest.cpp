#include "BrickFixtures.h"
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
#include <iomanip>
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
	int Points = 0, Standing = 0, Undecided = 0, Mechanisms = 0, UnexpectedUndecided = 0, FieldFailures = 0;
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
					Mechanisms += Result.State == StandingState::Mechanism ? 1 : 0;
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
	MESSAGE("lateral lime grid: " << Points << " points, " << Standing << " Stand, " << Mechanisms << " Mechanism, " << Undecided << " Undecided (" << UnexpectedUndecided << " outside the known exceptions), " << FieldFailures << " fail their own field, " << Seconds << " s");
	CHECK(Points == 140);
	CHECK(Undecided <= 5);
	CHECK(Standing == 128);
	CHECK(Mechanisms == 7);
	CHECK(UnexpectedUndecided == 0);
}

TEST_CASE("Statics.Lime.AWallWhoseForgottenShearWouldExceedTheCohesiveCapacityReadsOverstressed")
{
	const StrengthProfile Lime = LoadLime();
	StrengthProfile WeakPerpend = Lime;
	WeakPerpend.ShearCohesionPa = 100.0;
	LimeWall W;
	geometry::WallSpec Spec;
	Spec.Bed = AddProfileOk(W.Model, Lime);
	Spec.Perpend = AddProfileOk(W.Model, WeakPerpend);
	Spec.Courses = 2;
	Spec.BricksPerCourse = 5;
	W.Value = geometry::BuildRunningBondWall(W.Model, Spec);
	REQUIRE(W.Value.bComplete);
	for (const double LambdaX : {0.3, 0.5, 0.7})
	{
		const StandingOptions Options = Biaxial(LambdaX, 0.0);
		const StandingResult Result = SolveWith(W.Model, Options);
		INFO("2 x 5 wall, lime beds, perpend c 100 Pa, at lambda x " << LambdaX << ": " << Result.State << " after " << Result.ActiveSetIterations << " iterations");
		CHECK(Result.State == StandingState::Overstressed);
		CHECK(Result.ActiveSetIterations <= 8);
	}
}

TEST_CASE("Statics.Lime.WallsTheGuardKeptFromStandingStand")
{
	struct GuardWall
	{
		int Courses, BricksPerCourse;
		double BedCohesionPa, PerpendCohesionPa;
		double LambdaX, LambdaY;
		int IterationBar;
	};
	const StrengthProfile Lime = LoadLime();
	const GuardWall Walls[] = {{4, 3, 3000.0, Lime.ShearCohesionPa, 0.7, 0.1, 10}, {6, 3, 10000.0, 10000.0, 0.8, 0.1, 15}};
	for (const GuardWall& G : Walls)
	{
		StrengthProfile Bed = Lime;
		Bed.ShearCohesionPa = G.BedCohesionPa;
		StrengthProfile Perpend = Lime;
		Perpend.ShearCohesionPa = G.PerpendCohesionPa;
		LimeWall W;
		geometry::WallSpec Spec;
		Spec.Bed = AddProfileOk(W.Model, Bed);
		Spec.Perpend = AddProfileOk(W.Model, Perpend);
		Spec.Courses = G.Courses;
		Spec.BricksPerCourse = G.BricksPerCourse;
		W.Value = geometry::BuildRunningBondWall(W.Model, Spec);
		REQUIRE(W.Value.bComplete);
		const StandingOptions Options = Biaxial(G.LambdaX, G.LambdaY);
		const StandingResult Result = SolveWith(W.Model, Options);
		INFO(G.Courses << " x " << G.BricksPerCourse << " wall, bed c " << G.BedCohesionPa << " Pa, perpend c " << G.PerpendCohesionPa << " Pa, at (" << G.LambdaX << ", " << G.LambdaY << "): " << Result.State << " after " << Result.ActiveSetIterations << " iterations");
		CHECK(Result.State == StandingState::Stands);
		CHECK(Result.ActiveSetIterations <= G.IterationBar);
		if (Result.State == StandingState::Stands)
		{
			CheckCohesiveField(W, Result, Options);
		}
	}
}

TEST_CASE("Statics.Lime.ACornerThatNeverResetsKeepsItsSignedZeroShear")
{
	const BrickOnGround F = MakeBrickOnGround(LoadLime());
	const StandingResult Result = SolveWith(F.Model, StandingOptions{});
	REQUIRE(Result.State == StandingState::Stands);
	const auto Bed = std::find_if(Result.Readings.begin(), Result.Readings.end(), [&](const InterfaceReading& R) { return R.Id == F.Bed; });
	REQUIRE(Bed != Result.Readings.end());
	const CornerReading& Corner = Bed->Corners[0];
	INFO("bed corner 0: shear " << Corner.ShearForceN << ", sign bits " << std::signbit(Corner.ShearForceN.X) << std::signbit(Corner.ShearForceN.Y) << std::signbit(Corner.ShearForceN.Z) << ", open " << Corner.bOpen);
	REQUIRE_FALSE(Corner.bOpen);
	CHECK(Corner.ShearForceN.Z == 0.0);
	CHECK(std::signbit(Corner.ShearForceN.Z));
}

TEST_CASE("Statics.Lime.FormerlyConvergedPointsReadTheirNewFields")
{
	const StrengthProfile Lime = LoadLime();
	const PinnedPoint Points[] = {
		{3, 4, 1.2, 0.15, 6, 49.816631363081264, 29.556976095207304},
		{5, 5, 0.9, 0.1, 9, 87.497747722552006, 34.819311436076617},
		{5, 5, 0.9, 0.15, 8, 94.248653300064646, 40.088168321761252},
		{6, 8, 0.6, 0.1, 6, 93.213701071544406, 27.790229597248331},
		{6, 8, 0.6, 0.15, 6, 101.32347046905915, 27.561923758931908},
		{6, 8, 0.7, 0.05, 8, 91.018702837705661, 32.592493302271812},
		{6, 8, 0.9, 0.1, 8, 111.78183582127048, 40.55068787161828},
		{6, 8, 1.0, 0.1, 14, 121.22622444645363, 60.059450615264957},
		{10, 8, 0.6, 0.05, 5, 173.78145638393102, 45.935027704957143},
		{10, 8, 0.6, 0.15, 10, 271.91934861802361, 94.691299028027757},
		{10, 8, 0.7, 0.05, 9, 189.1186667858608, 65.611734306910918},
		{10, 8, 0.9, 0.1, 14, 278.83572634465452, 100.73764135296223}};
	for (const PinnedPoint& P : Points)
	{
		const LimeWall W = BuildWall(Lime, P.Courses, P.BricksPerCourse);
		REQUIRE(W.Value.bComplete);
		const StandingResult Result = SolveWith(W.Model, Biaxial(P.LambdaX, P.LambdaY));
		const Extremes E = ReadExtremes(Result);
		INFO(P.Courses << " x " << P.BricksPerCourse << " lime wall at (" << P.LambdaX << ", " << P.LambdaY << "): " << Result.State << " after " << Result.ActiveSetIterations << " iterations, max |N| " << std::setprecision(17) << E.MaxNormalN << " N, max |S| " << E.MaxShearN << " N");
		CHECK(Result.State == StandingState::Stands);
		CHECK(Result.ActiveSetIterations == P.Iterations);
		CHECK(E.MaxNormalN == doctest::Approx(P.MaxNormalN).epsilon(1e-12));
		CHECK(E.MaxShearN == doctest::Approx(P.MaxShearN).epsilon(1e-12));
	}
}

TEST_CASE("Statics.Lime.SolvingTwiceReadsTheSameField")
{
	const StrengthProfile Lime = LoadLime();
	const LimeWall Tall = BuildWall(Lime, 10, 8);
	const LimeWall Other = BuildWall(Lime, 4, 5);
	REQUIRE(Tall.Value.bComplete);
	REQUIRE(Other.Value.bComplete);
	const StandingOptions Options = Biaxial(0.8, 0.0);
	const StandingResult First = SolveWith(Tall.Model, Options);
	const StandingResult Between = SolveWith(Other.Model, Options);
	const StandingResult Second = SolveWith(Tall.Model, Options);
	INFO("10 x 8 at (0.8, 0): " << First.State << " after " << First.ActiveSetIterations << ", then " << Second.State << " after " << Second.ActiveSetIterations << "; 4 x 5 between: " << Between.State << " after " << Between.ActiveSetIterations);
	CHECK(First.ActiveSetIterations == 16);
	CHECK(Second.ActiveSetIterations == 16);
	CHECK(First.State == Second.State);
	CHECK(First.ActiveSetIterations == Second.ActiveSetIterations);
	REQUIRE(First.Readings.size() == Second.Readings.size());
	int Differing = 0;
	for (std::size_t I = 0; I < First.Readings.size(); ++I)
	{
		for (std::size_t C = 0; C < First.Readings[I].Corners.size(); ++C)
		{
			const CornerReading& A = First.Readings[I].Corners[C];
			const CornerReading& B = Second.Readings[I].Corners[C];
			const bool bSameStatus = A.bOpen == B.bOpen && A.bSlipping == B.bSlipping;
			const bool bSameNormal = A.NormalForceN == B.NormalForceN && std::signbit(A.NormalForceN) == std::signbit(B.NormalForceN);
			const bool bSameShear = A.ShearForceN.X == B.ShearForceN.X && A.ShearForceN.Y == B.ShearForceN.Y && A.ShearForceN.Z == B.ShearForceN.Z && std::signbit(A.ShearForceN.X) == std::signbit(B.ShearForceN.X) && std::signbit(A.ShearForceN.Y) == std::signbit(B.ShearForceN.Y) && std::signbit(A.ShearForceN.Z) == std::signbit(B.ShearForceN.Z);
			Differing += bSameStatus && bSameNormal && bSameShear ? 0 : 1;
		}
	}
	CHECK(Differing == 0);
}

namespace
{
struct HuntCase
{
	int Courses, BricksPerCourse;
	double BedCohesionPa, BedFriction, PerpendCohesionPa, PerpendFriction;
	Vec3 GravityMPerS2;
	struct Push
	{
		int Course, Brick;
		Vec3 ForceN, PointM;
	};
	std::vector<Push> Pushes;
};

StandingResult SolveHunt(const HuntCase& H)
{
	StrengthProfile Bed = LoadLime();
	StrengthProfile Perpend = Bed;
	Bed.ShearCohesionPa = H.BedCohesionPa;
	Bed.FrictionCoefficient = H.BedFriction;
	Perpend.ShearCohesionPa = H.PerpendCohesionPa;
	Perpend.FrictionCoefficient = H.PerpendFriction;
	LimeWall W;
	geometry::WallSpec Spec;
	Spec.Bed = AddProfileOk(W.Model, Bed);
	Spec.Perpend = AddProfileOk(W.Model, Perpend);
	Spec.Courses = H.Courses;
	Spec.BricksPerCourse = H.BricksPerCourse;
	W.Value = geometry::BuildRunningBondWall(W.Model, Spec);
	REQUIRE(W.Value.bComplete);
	std::vector<BodyLoad> Loads;
	for (const HuntCase::Push& P : H.Pushes)
	{
		Loads.push_back(BodyLoad{W.Value.Courses[P.Course][P.Brick], P.ForceN, P.PointM});
	}
	StandingOptions Options;
	Options.Gravity = H.GravityMPerS2;
	return SolveWith(W.Model, Options, Loads);
}
}

TEST_CASE("Statics.Lime.ARestoreStepReclosureNeitherCountsNorResets")
{
	const HuntCase CountsMatter{1, 6, 30.0, 0.75972470052418806, 30.0, 0.69412002901115355, Vec3{0.92095371388797675, 0.0, -9.80665},
		{{0, 3, Vec3{-1.0523595483503445, -34.055520837471484, 49.168054889091884}, Vec3{0.75328582369418118, -0.027436240015488902, 0.017198466186857199}},
		 {0, 4, Vec3{6.6582138033156735, 7.1438837004842251, 0.53305392142438968}, Vec3{0.91799851924929288, 0.022086205388845939, 0.024546325909667158}}}};
	const StandingResult Counted = SolveHunt(CountsMatter);
	INFO("1 x 6 weak lime wall with two held loads: " << Counted.State << " after " << Counted.ActiveSetIterations);
	CHECK(Counted.State == StandingState::Overstressed);
	CHECK(Counted.ActiveSetIterations == 15);

	const HuntCase KeepsReference{5, 2, 160000.0, 0.0, 300.0, 0.75422437877084136, Vec3{9.3178669623399966, 1.7860632068278335, -9.80665},
		{{1, 2, Vec3{27.485129299353883, -22.353013601915244, 28.920075248566114}, Vec3{0.30517405110484042, 0.027483145320517918, 0.10532130730518167}},
		 {1, 1, Vec3{2.7416680132160907, -3.8504440158373101, -2.6771397979574099}, Vec3{0.21949657397784628, -0.0077664640969666299, 0.12663048397739793}}}};
	const StandingResult Kept = SolveHunt(KeepsReference);
	INFO("5 x 2 lime wall with two held loads: " << Kept.State << " after " << Kept.ActiveSetIterations);
	CHECK(Kept.State == StandingState::Mechanism);
	CHECK(Kept.ActiveSetIterations == 13);
}
