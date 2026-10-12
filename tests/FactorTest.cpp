#include "AnchorFixtures.h"
#include "BrickFixtures.h"
#include "CascadeTestSupport.h"
#include "CatalogueTestSupport.h"
#include "FactorTestSupport.h"
#include "StaticAdmissibilityOracle.h"
#include "SlowInDebug.h"
#include "StaticsTestSupport.h"

#include "loadpath/Catalogue.h"
#include "loadpath/geometry/RunningBondWall.h"

#include "doctest.h"

#include <algorithm>
#include <chrono>
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
using FactorKind = StandingOptions::FactorKind;

StandingOptions WithFactor(StandingOptions Options, FactorKind Kind)
{
	Options.Factor = Kind;
	return Options;
}

StandingOptions AllDense(StandingOptions Options)
{
	Options.DenseDofLimit = INT_MAX;
	return Options;
}

Comparison CompareKinds(const FactorCase& C, bool bAllowTrade)
{
	return CompareOptions(C, WithFactor(C.Options, FactorKind::Reference), WithFactor(C.Options, FactorKind::Blocked), "reference", "blocked", bAllowTrade);
}

std::vector<FactorCase> NamedCases()
{
	std::vector<FactorCase> Cases;
	const double TanAlpha = 0.25 / 1.0;
	const BlockOnGround Housner = MakeBlockOnGround(0.25, 0.25, 1.0, 0.7);
	Cases.push_back({"Housner block at its tipping ratio", Housner.Model, LateralLoad(TanAlpha), {}});
	Cases.push_back({"Housner block at 0.99 of its tipping ratio", Housner.Model, LateralLoad(0.99 * TanAlpha), {}});
	Cases.push_back({"three-brick bridge at rest", MakeThreeBrickBridge(DryJoint(0.7)).Model, StandingOptions{}, {}});
	Cases.push_back({"4 x 5 dry wall at rest", RunningBondWall(DryJoint(0.7), DryJoint(0.7), 4, 5), StandingOptions{}, {}});
	Cases.push_back({"4 x 5 ec6 wall at rest", Ec6Wall(4, 5), StandingOptions{}, {}});
	geometry::Wall EndPushWall;
	FactorCase EndPush{"1 x 3 dry wall under a 0.2 W end push", RunningBondWall(DryJoint(0.7), DryJoint(0.7), 1, 3, &EndPushWall), StandingOptions{}, {}};
	REQUIRE(EndPushWall.Courses.size() == 1u);
	const BodyId End = EndPushWall.Courses[0].front();
	const double BrickWeightN = EndPush.Model.Body(End).MassKg * GravityMPerS2;
	EndPush.Loads = {BodyLoad{End, Vec3{0.2 * BrickWeightN, 0.0, 0.0}, EndPush.Model.Body(End).CentroidM}};
	Cases.push_back(EndPush);
	return Cases;
}

std::vector<FactorCase> MultiPanelCases()
{
	std::vector<FactorCase> Cases;
	Cases.push_back({"8 x 10 dry wall under lambda (0, 0.3)", RunningBondWall(DryJoint(0.7), DryJoint(0.7), 8, 10), Biaxial(0.0, 0.3), {}});
	Cases.push_back({"8 x 10 ec6 wall under lambda (0, 0.5)", Ec6Wall(8, 10), Biaxial(0.0, 0.5), {}});
	return Cases;
}

constexpr int FactorDrawCount = bOptimisedBuild ? 6000 : 40;
constexpr int WallDrawCount = bOptimisedBuild ? 120 : 3;
constexpr std::uint32_t WallSeedOffset = 1000000u;
}

TEST_CASE("Statics.Factor.BlockedAgreesWithTheReference")
{
	SUBCASE("named fixtures")
	{
		for (const FactorCase& C : NamedCases())
		{
			CompareKinds(C, false);
		}
	}
	SUBCASE("seeded stacks and bridges")
	{
		int Disagreeing = 0;
		int Traded = 0;
		int IterationsDiffer = 0;
		Gaps Worst;
		Gaps WorstCertified;
		int CertifiedDisagreeing = 0;
		int MotionMismatches = 0;
		double WorstMotionGap = 0.0;
		for (int Draw = 0; Draw < FactorDrawCount; ++Draw)
		{
			const SeededDraw Each = MakeSeededDraw(Draw);
			const std::string Name = "master seed " + std::to_string(MasterSeed) + ", draw " + std::to_string(Draw) + " (generator seed " + std::to_string(Each.Seed) + "): " + Each.Fixture + ", lambda (" + std::to_string(Each.LambdaX) + ", " + std::to_string(Each.LambdaY) + ")";
			const Comparison R = CompareKinds(FactorCase{Name, Each.Model, Biaxial(Each.LambdaX, Each.LambdaY), {}}, true);
			Disagreeing += R.Mismatches > 0 ? 1 : 0;
			Gaps& Into = R.bCertified ? WorstCertified : Worst;
			CertifiedDisagreeing += R.bCertified && R.Mismatches > 0 ? 1 : 0;
			Traded += R.bStateClassTraded ? 1 : 0;
			IterationsDiffer += R.bIterationsDiffer ? 1 : 0;
			MotionMismatches += R.MotionMismatches;
			WorstMotionGap = std::max(WorstMotionGap, R.WorstMotionGap);
			Into.WorstRelative = std::max(Into.WorstRelative, R.G.WorstRelative);
			Into.WorstForceAbsoluteOverW = std::max(Into.WorstForceAbsoluteOverW, R.G.WorstForceAbsoluteOverW);
			Into.WorstUtilisationAbsolute = std::max(Into.WorstUtilisationAbsolute, R.G.WorstUtilisationAbsolute);
		}
		const int TradeBudget = std::max(1, FactorDrawCount / 1000);
		MESSAGE("over " << FactorDrawCount << " draws: " << Traded << " traded mechanism and undecided (budget " << TradeBudget << "), " << IterationsDiffer << " differ in iterations, worst relative reading gap above the absolute floor " << Worst.WorstRelative << ", worst absolute force gap " << Worst.WorstForceAbsoluteOverW << " W, worst absolute utilisation gap " << Worst.WorstUtilisationAbsolute << " on mechanism draws (not asserted); " << Disagreeing << " draws disagree, " << CertifiedDisagreeing << " of them stands or overstressed, whose worst gaps are " << WorstCertified.WorstRelative << " relative, " << WorstCertified.WorstForceAbsoluteOverW << " W, " << WorstCertified.WorstUtilisationAbsolute << " utilisation; " << MotionMismatches << " mechanism bodies move differently, worst motion gap " << WorstMotionGap);
		CHECK(Disagreeing == 0);
		CHECK(Traded <= TradeBudget);
	}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Statics.Factor.BlockedAgreesWithTheReferenceAcrossPanels")
{
	SUBCASE("named walls")
	{
		for (const FactorCase& C : MultiPanelCases())
		{
			const Clock::time_point Start = Clock::now();
			const Comparison R = CompareKinds(C, false);
			MESSAGE(C.Name << ": " << R.Mismatches << " mismatches, worst motion gap " << R.WorstMotionGap << ", " << SecondsSince(Start) << " s");
		}
	}
	SUBCASE("seeded walls")
	{
		int Disagreeing = 0;
		int Traded = 0;
		int MultiPanel = 0;
		int Mechanisms = 0;
		double WorstMotionGap = 0.0;
		for (int Draw = 0; Draw < WallDrawCount; ++Draw)
		{
			const std::uint32_t Seed = MasterSeed + WallSeedOffset + static_cast<std::uint32_t>(Draw);
			std::mt19937 Rng(Seed);
			const int Courses = 3 + static_cast<int>(Rng() % 4u);
			const int Bricks = 3 + static_cast<int>(Rng() % 6u);
			const double Friction = Uniform(Rng, 0.4, 1.0);
			const double LambdaX = Uniform(Rng, 0.0, 0.5);
			const double LambdaY = Uniform(Rng, 0.0, 0.5);
			const std::string Name = "master seed " + std::to_string(MasterSeed) + ", wall draw " + std::to_string(Draw) + " (seed " + std::to_string(Seed) + "): " + std::to_string(Courses) + " x " + std::to_string(Bricks) + " dry wall, mu " + std::to_string(Friction) + ", lambda (" + std::to_string(LambdaX) + ", " + std::to_string(LambdaY) + ")";
			const FactorCase C{Name, RunningBondWall(DryJoint(Friction), DryJoint(Friction), Courses, Bricks), Biaxial(LambdaX, LambdaY), {}};
			MultiPanel += 6 * (C.Model.LiveBodyCount() - 1) > 64 ? 1 : 0;
			const Comparison R = CompareKinds(C, true);
			Disagreeing += R.Mismatches > 0 ? 1 : 0;
			Traded += R.bStateClassTraded ? 1 : 0;
			Mechanisms += R.WorstMotionGap > 0.0 || R.MotionMismatches > 0 ? 1 : 0;
			WorstMotionGap = std::max(WorstMotionGap, R.WorstMotionGap);
		}
		const int TradeBudget = std::max(1, WallDrawCount / 1000);
		MESSAGE("over " << WallDrawCount << " wall draws (" << MultiPanel << " above one panel): " << Traded << " traded mechanism and undecided (budget " << TradeBudget << "), " << Disagreeing << " disagree, worst motion gap " << WorstMotionGap);
		CHECK(Disagreeing == 0);
		CHECK(Traded <= TradeBudget);
	}
}

TEST_CASE("Slow.Statics.Factor.AMidSizeWallSolvesInsideItsBudget")
{
	const Structure Model = Ec6Wall(16, 20);
	REQUIRE(Model.LiveBodyCount() == 16u * 20u + 8u + 1u);
	Clock::time_point Start = Clock::now();
	const StandingResult Blocked = SolveWith(Model, AllDense(StandingOptions{}));
	const double BlockedS = SecondsSince(Start);
	Start = Clock::now();
	const StandingResult Reference = SolveWith(Model, AllDense(WithFactor(StandingOptions{}, FactorKind::Reference)));
	const double ReferenceS = SecondsSince(Start);
	MESSAGE("ec6 wall 20 x 16 (" << Model.LiveBodyCount() - 1 << " bricks): blocked " << BlockedS << " s (" << Blocked.State << " after " << Blocked.ActiveSetIterations << "), reference " << ReferenceS << " s (" << Reference.State << " after " << Reference.ActiveSetIterations << ")");
	CHECK(Blocked.State == StandingState::Stands);
	CHECK(BlockedS < 1.0);
}

#if defined(NDEBUG)
TEST_CASE("Measure.Factor.FortyByThirtyTwoWallElastic")
{
	const Structure Model = Ec6Wall(32, 40);
	const Clock::time_point Start = Clock::now();
	const StandingResult Blocked = SolveWith(Model, AllDense(StandingOptions{}));
	MESSAGE("ec6 wall 40 x 32 (" << Model.LiveBodyCount() - 1 << " bricks): blocked elastic " << SecondsSince(Start) << " s (" << Blocked.State << " after " << Blocked.ActiveSetIterations << ")");
}
#endif
