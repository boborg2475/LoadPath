#include "AnchorFixtures.h"
#include "CatalogueTestSupport.h"
#include "FactorTestSupport.h"
#include "SlowInDebug.h"
#include "StaticAdmissibilityOracle.h"
#include "StaticsTestSupport.h"

#include "loadpath/Catalogue.h"
#include "loadpath/geometry/RunningBondWall.h"

#include "doctest.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;
using namespace loadpath::test::oracle;

namespace
{
StandingOptions WithDenseDofLimit(StandingOptions Options, int Limit)
{
	Options.DenseDofLimit = Limit;
	return Options;
}

Comparison CompareSparseWithDense(const FactorCase& C, bool bAllowTrade)
{
	return CompareOptions(C, WithDenseDofLimit(C.Options, INT_MAX), WithDenseDofLimit(C.Options, 0), "dense", "sparse", bAllowTrade);
}

struct Tally
{
	int Draws = 0;
	int Disagreeing = 0;
	int Traded = 0;
	int IterationsDiffer = 0;
	int Certified = 0;
	int MotionMismatches = 0;
	double WorstMotionGap = 0.0;
	Gaps WorstCertified;
	int ByDenseState[5] = {0, 0, 0, 0, 0};

	void Add(const Comparison& R)
	{
		++Draws;
		++ByDenseState[static_cast<int>(R.StateA)];
		Disagreeing += R.Mismatches > 0 ? 1 : 0;
		Traded += R.bStateClassTraded ? 1 : 0;
		IterationsDiffer += R.bIterationsDiffer ? 1 : 0;
		Certified += R.bCertified ? 1 : 0;
		MotionMismatches += R.MotionMismatches;
		WorstMotionGap = std::max(WorstMotionGap, R.WorstMotionGap);
		if (R.bCertified)
		{
			WorstCertified.WorstRelative = std::max(WorstCertified.WorstRelative, R.G.WorstRelative);
			WorstCertified.WorstForceAbsoluteOverW = std::max(WorstCertified.WorstForceAbsoluteOverW, R.G.WorstForceAbsoluteOverW);
			WorstCertified.WorstUtilisationAbsolute = std::max(WorstCertified.WorstUtilisationAbsolute, R.G.WorstUtilisationAbsolute);
		}
	}

	void Report(const std::string& What) const
	{
		const int TradeBudget = std::max(1, Draws / 1000);
		MESSAGE("over " << Draws << " " << What << " (dense Stands " << ByDenseState[0] << ", Overstressed " << ByDenseState[1] << ", Mechanism " << ByDenseState[2] << ", Undecided " << ByDenseState[3] << ", InvalidInput " << ByDenseState[4] << "): " << Traded << " traded mechanism and undecided (budget " << TradeBudget << "), " << IterationsDiffer << " differ in iterations, " << Disagreeing << " disagree, " << Certified << " certified with worst gaps " << WorstCertified.WorstRelative << " relative, " << WorstCertified.WorstForceAbsoluteOverW << " W, " << WorstCertified.WorstUtilisationAbsolute << " utilisation; " << MotionMismatches << " mechanism bodies move differently, worst motion gap " << WorstMotionGap);
		CHECK(Disagreeing == 0);
		CHECK(Traded <= TradeBudget);
	}
};

int FreeBodies(int Courses, int Bricks) { return Courses * Bricks + Courses / 2; }

constexpr int WallDrawCount = bOptimisedBuild ? 12 : 0;
constexpr int StackDrawCount = bOptimisedBuild ? 1000 : 40;
constexpr std::uint32_t Ec6WallSeedOffset = 2000000u;
constexpr std::uint32_t DryWallSeedOffset = 2100000u;
constexpr int DofPerFreeBody = 6;

void DrawWallSize(std::mt19937& Rng, int& Courses, int& Bricks)
{
	do
	{
		Courses = 5 + static_cast<int>(Rng() % 8u);
		Bricks = 8 + static_cast<int>(Rng() % 9u);
	} while (FreeBodies(Courses, Bricks) < 60 || FreeBodies(Courses, Bricks) > 200);
}

FactorCase SeededEc6Wall(const Catalogue& Rows, int Draw)
{
	const std::uint32_t Seed = MasterSeed + Ec6WallSeedOffset + static_cast<std::uint32_t>(Draw);
	std::mt19937 Rng(Seed);
	int Courses = 0;
	int Bricks = 0;
	DrawWallSize(Rng, Courses, Bricks);
	const double LambdaX = Uniform(Rng, 0.0, 0.5);
	const double LambdaY = Uniform(Rng, 0.0, 0.5);
	const Structure Model = RunningBondWall(JointOrFail(Rows, "clay_gp_mortar_bed_ec6").Profile, JointOrFail(Rows, "clay_gp_mortar_perpend_ec6").Profile, Courses, Bricks);
	const std::string Name = "master seed " + std::to_string(MasterSeed) + ", ec6 wall draw " + std::to_string(Draw) + " (seed " + std::to_string(Seed) + "): " + std::to_string(Courses) + " x " + std::to_string(Bricks) + ", lambda (" + std::to_string(LambdaX) + ", " + std::to_string(LambdaY) + ")";
	return FactorCase{Name, Model, Biaxial(LambdaX, LambdaY), {}};
}

FactorCase SeededDryWallAtRest(int Draw)
{
	const std::uint32_t Seed = MasterSeed + DryWallSeedOffset + static_cast<std::uint32_t>(Draw);
	std::mt19937 Rng(Seed);
	int Courses = 0;
	int Bricks = 0;
	DrawWallSize(Rng, Courses, Bricks);
	const double Friction = Uniform(Rng, 0.4, 1.0);
	const std::string Name = "master seed " + std::to_string(MasterSeed) + ", dry wall draw " + std::to_string(Draw) + " (seed " + std::to_string(Seed) + "): " + std::to_string(Courses) + " x " + std::to_string(Bricks) + ", mu " + std::to_string(Friction) + ", at rest";
	return FactorCase{Name, RunningBondWall(DryJoint(Friction), DryJoint(Friction), Courses, Bricks), StandingOptions{}, {}};
}

bool SameVec(const Vec3& A, const Vec3& B) { return A.X == B.X && A.Y == B.Y && A.Z == B.Z; }

int CountDifferences(const StandingResult& A, const StandingResult& B)
{
	if (A.Readings.size() != B.Readings.size() || A.Mechanism.size() != B.Mechanism.size())
	{
		return INT_MAX;
	}
	int Differing = A.State == B.State && A.ActiveSetIterations == B.ActiveSetIterations && SameIdList(A.UnconstrainedBodies, B.UnconstrainedBodies) ? 0 : 1;
	for (std::size_t I = 0; I < A.Readings.size(); ++I)
	{
		const InterfaceReading& X = A.Readings[I];
		const InterfaceReading& Y = B.Readings[I];
		bool bSame = SameHandle(X.Id, Y.Id) && X.Utilisation == Y.Utilisation && X.Axis == Y.Axis && X.CompressionUtilisation == Y.CompressionUtilisation && X.TensionUtilisation == Y.TensionUtilisation && X.ShearUtilisation == Y.ShearUtilisation && X.NormalForceN == Y.NormalForceN && SameVec(X.ShearForceN, Y.ShearForceN) && SameVec(X.MomentNm, Y.MomentNm);
		for (std::size_t K = 0; K < 4; ++K)
		{
			const CornerReading& P = X.Corners[K];
			const CornerReading& Q = Y.Corners[K];
			bSame = bSame && P.NormalForceN == Q.NormalForceN && SameVec(P.ShearForceN, Q.ShearForceN) && P.bOpen == Q.bOpen && P.bSlipping == Q.bSlipping;
		}
		Differing += bSame ? 0 : 1;
	}
	for (std::size_t I = 0; I < A.Mechanism.size(); ++I)
	{
		const MechanismMotion& P = A.Mechanism[I];
		const MechanismMotion& Q = B.Mechanism[I];
		Differing += SameHandle(P.Body, Q.Body) && SameVec(P.TranslationM, Q.TranslationM) && SameVec(P.RotationRad, Q.RotationRad) ? 0 : 1;
	}
	return Differing;
}

void CheckBitIdentical(const StandingResult& A, const StandingResult& B)
{
	CHECK(A.State == B.State);
	CHECK(A.ActiveSetIterations == B.ActiveSetIterations);
	CHECK(SameIdList(A.UnconstrainedBodies, B.UnconstrainedBodies));
	REQUIRE(A.Readings.size() == B.Readings.size());
	REQUIRE(A.Mechanism.size() == B.Mechanism.size());
	CHECK(CountDifferences(A, B) == 0);
}

double Sign(double X, double Floor) { return X > Floor ? 1.0 : (X < -Floor ? -1.0 : 0.0); }

double SignFloor(const std::vector<MechanismMotion>& Motion)
{
	double Largest = 0.0;
	for (const MechanismMotion& M : Motion)
	{
		Largest = std::max({Largest, std::abs(M.TranslationM.X), std::abs(M.TranslationM.Y), std::abs(M.TranslationM.Z), std::abs(M.RotationRad.X), std::abs(M.RotationRad.Y), std::abs(M.RotationRad.Z)});
	}
	return 1e-9 * Largest;
}

struct HiddenPivotDraw
{
	int Draw;
	const char* Fixture;
	StandingState DenseState;
	int DenseIterations;
};
}

TEST_CASE("Statics.Sparse.AgreesWithTheDensePath")
{
	SUBCASE("seeded ec6 walls under lateral load and dry walls at rest, 60 to 200 bodies")
	{
		const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
		REQUIRE(Loaded.Ok());
		if (!Loaded.Ok())
		{
			return;
		}
		Tally T;
		const Clock::time_point Start = Clock::now();
		for (int Draw = 0; Draw < WallDrawCount; ++Draw)
		{
			T.Add(CompareSparseWithDense(SeededEc6Wall(Loaded.Value, Draw), true));
			T.Add(CompareSparseWithDense(SeededDryWallAtRest(Draw), true));
		}
		T.Report("wall draws");
		MESSAGE("walls took " << SecondsSince(Start) << " s");
	}
	SUBCASE("seeded stacks and bridges")
	{
		Tally T;
		const Clock::time_point Start = Clock::now();
		for (int Draw = 0; Draw < StackDrawCount; ++Draw)
		{
			const SeededDraw Each = MakeSeededDraw(Draw);
			const std::string Name = "master seed " + std::to_string(MasterSeed) + ", draw " + std::to_string(Draw) + " (generator seed " + std::to_string(Each.Seed) + "): " + Each.Fixture + ", lambda (" + std::to_string(Each.LambdaX) + ", " + std::to_string(Each.LambdaY) + ")";
			T.Add(CompareSparseWithDense(FactorCase{Name, Each.Model, Biaxial(Each.LambdaX, Each.LambdaY), {}}, true));
		}
		T.Report("stack and bridge draws");
		MESSAGE("stacks and bridges took " << SecondsSince(Start) << " s");
	}
}

TEST_CASE("Statics.Sparse.HiddenPivotsAreCaughtByTheEstimate")
{
	const HiddenPivotDraw Draws[] = {
		{423, "bridge half span 0.128871, top offset -0.00122245", StandingState::Mechanism, 4},
		{556, "bridge half span 0.100377, top offset 0.000650907", StandingState::Mechanism, 3},
		{708, "bridge half span 0.107515, top offset 0.0131901", StandingState::Undecided, 50},
		{715, "bridge half span 0.124973, top offset -0.0118885", StandingState::Mechanism, 41},
	};
	for (const HiddenPivotDraw& D : Draws)
	{
		const SeededDraw Each = MakeSeededDraw(D.Draw);
		const std::string Name = "draw " + std::to_string(D.Draw) + ": " + Each.Fixture + ", lambda (" + std::to_string(Each.LambdaX) + ", " + std::to_string(Each.LambdaY) + ")";
		CHECK_MESSAGE(Each.Fixture == std::string(D.Fixture), Name);
		const FactorCase C{Name, Each.Model, Biaxial(Each.LambdaX, Each.LambdaY), {}};
		const StandingResult Dense = SolveWith(C.Model, WithDenseDofLimit(C.Options, INT_MAX));
		CHECK_MESSAGE(Dense.State == D.DenseState, Name);
		CHECK_MESSAGE(Dense.ActiveSetIterations == D.DenseIterations, Name);
		const Comparison R = CompareSparseWithDense(C, false);
		CHECK_MESSAGE(R.Mismatches == 0, Name << ": dense " << R.StateA << ", sparse " << R.StateB);
	}
}

TEST_CASE("Statics.Sparse.TheLimitBoundaryIsInclusive")
{
	const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
	REQUIRE(Loaded.Ok());
	if (!Loaded.Ok())
	{
		return;
	}
	struct BoundaryCase
	{
		std::string Name;
		Structure Model;
		StandingOptions Options;
	};
	const BoundaryCase Cases[] = {
		{"8 x 12 ec6 wall under lambda (0.2, 0.1)", RunningBondWall(JointOrFail(Loaded.Value, "clay_gp_mortar_bed_ec6").Profile, JointOrFail(Loaded.Value, "clay_gp_mortar_perpend_ec6").Profile, 8, 12), Biaxial(0.2, 0.1)},
		{"8 x 12 dry wall at rest", RunningBondWall(DryJoint(0.7), DryJoint(0.7), 8, 12), StandingOptions{}},
	};
	for (const BoundaryCase& C : Cases)
	{
		INFO(C.Name);
		REQUIRE(DofPerFreeBody * static_cast<int>(C.Model.LiveBodyCount() - 1) == 600);
		const StandingResult Default = SolveWith(C.Model, C.Options);
		const StandingResult Dense = SolveWith(C.Model, WithDenseDofLimit(C.Options, INT_MAX));
		const StandingResult Sparse = SolveWith(C.Model, WithDenseDofLimit(C.Options, 599));
		const int DefaultDiffers = CountDifferences(Default, Dense);
		const int SparseDiffers = CountDifferences(Sparse, Dense);
		MESSAGE(C.Name << ": " << Dense.Readings.size() << " readings; default against INT_MAX differs in " << DefaultDiffers << ", limit 599 against INT_MAX in " << SparseDiffers);
		CHECK(SparseDiffers > 0);
		CheckBitIdentical(Default, Dense);
	}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Statics.Sparse.AMechanismAboveTheLimitReadsTheSameAsBelowIt")
{
	const Structure Model = RunningBondWall(DryJoint(0.7), DryJoint(0.7), 10, 10);
	const int Dof = DofPerFreeBody * static_cast<int>(Model.LiveBodyCount() - 1);
	REQUIRE(Dof > 600);
	const StandingOptions Options = Biaxial(0.0, 0.3);
	const StandingResult Dense = SolveWith(Model, WithDenseDofLimit(Options, INT_MAX));
	const StandingResult Sparse = SolveWith(Model, WithDenseDofLimit(Options, 600));
	MESSAGE("10 x 10 dry wall (" << Dof << " DoF) at lambda (0, 0.3): dense " << Dense.State << " after " << Dense.ActiveSetIterations << " with " << Dense.Mechanism.size() << " moving bodies, sparse " << Sparse.State << " after " << Sparse.ActiveSetIterations << " with " << Sparse.Mechanism.size());
	REQUIRE(Dense.State == StandingState::Mechanism);
	CHECK(Sparse.State == StandingState::Mechanism);
	REQUIRE(Dense.Mechanism.size() == Sparse.Mechanism.size());
	REQUIRE(Dense.Readings.size() == Sparse.Readings.size());
	if (Dense.Mechanism.size() != Sparse.Mechanism.size() || Dense.Readings.size() != Sparse.Readings.size())
	{
		return;
	}
	int BodyMismatches = 0;
	int SignMismatches = 0;
	const double FloorA = SignFloor(Dense.Mechanism);
	const double FloorB = SignFloor(Sparse.Mechanism);
	for (std::size_t I = 0; I < Dense.Mechanism.size(); ++I)
	{
		const MechanismMotion& A = Dense.Mechanism[I];
		const MechanismMotion& B = Sparse.Mechanism[I];
		BodyMismatches += SameHandle(A.Body, B.Body) ? 0 : 1;
		SignMismatches += Sign(A.TranslationM.X, FloorA) == Sign(B.TranslationM.X, FloorB) && Sign(A.TranslationM.Y, FloorA) == Sign(B.TranslationM.Y, FloorB) && Sign(A.TranslationM.Z, FloorA) == Sign(B.TranslationM.Z, FloorB) ? 0 : 1;
	}
	CHECK(BodyMismatches == 0);
	CHECK(SignMismatches == 0);
	int FlagMismatches = 0;
	for (std::size_t I = 0; I < Dense.Readings.size(); ++I)
	{
		for (std::size_t K = 0; K < 4; ++K)
		{
			FlagMismatches += Dense.Readings[I].Corners[K].bOpen == Sparse.Readings[I].Corners[K].bOpen ? 0 : 1;
		}
	}
	CHECK(FlagMismatches == 0);
}

TEST_CASE("Statics.Sparse.ADenseFixtureKeepsItsPath")
{
	CHECK(StandingOptions{}.DenseDofLimit == 600);
	SUBCASE("the coursed free wall just below t / H")
	{
		const CoursedWall W = MakeCoursedWall(10, 0.1025, 1.0, 0.65, 0.7);
		const StandingOptions Options = LateralLoad(0.1025 / 0.65 - 1e-6);
		CheckBitIdentical(SolveWith(W.Model, Options), SolveWith(W.Model, WithDenseDofLimit(Options, INT_MAX)));
	}
	SUBCASE("the 8 x 10 dry wall under lambda (0, 0.3)")
	{
		const Structure Model = RunningBondWall(DryJoint(0.7), DryJoint(0.7), 8, 10);
		const StandingOptions Options = Biaxial(0.0, 0.3);
		CheckBitIdentical(SolveWith(Model, Options), SolveWith(Model, WithDenseDofLimit(Options, INT_MAX)));
	}
}

TEST_CASE("Statics.Sparse.AnUnreachableAcceptanceMarginDeclinesToTheDensePath")
{
	const Structure Model = RunningBondWall(DryJoint(0.7), DryJoint(0.7), 8, 12);
	const StandingOptions Sparse = WithDenseDofLimit(StandingOptions{}, 599);
	const StandingResult Dense = SolveWith(Model, WithDenseDofLimit(StandingOptions{}, INT_MAX));
	const StandingResult Accepted = SolveWith(Model, Sparse);
	MESSAGE("8 x 12 dry wall at rest: dense " << Dense.State << ", sparse at the default margin " << Accepted.State << " differing in " << CountDifferences(Accepted, Dense) << " readings");
	REQUIRE(Dense.State == StandingState::Stands);
	REQUIRE(CountDifferences(Accepted, Dense) > 0);
	StandingOptions Unreachable = Sparse;
	Unreachable.SparseAcceptanceMargin = 1.0e30;
	const StandingResult Declined = SolveWith(Model, Unreachable);
	CHECK(Declined.State == Dense.State);
	CheckBitIdentical(Declined, Dense);
}

#if defined(NDEBUG)
TEST_CASE("Slow.Statics.Sparse.TheLargeWallSolvesInsideItsBudget")
{
	const Structure Model = Ec6Wall(32, 40);
	REQUIRE(Model.LiveBodyCount() == 32u * 40u + 16u + 1u);
	const Clock::time_point Start = Clock::now();
	const StandingResult Result = SolveWith(Model, StandingOptions{});
	const double Seconds = SecondsSince(Start);
	MESSAGE("ec6 wall 40 x 32 (" << Model.LiveBodyCount() - 1 << " bricks): " << Result.State << " after " << Result.ActiveSetIterations << " in " << Seconds << " s");
	CHECK(Result.State == StandingState::Stands);
	CHECK(Seconds < 1.0);
}
#endif
