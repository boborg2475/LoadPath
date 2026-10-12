#include "AnchorFixtures.h"
#include "BrickFixtures.h"
#include "CascadeTestSupport.h"
#include "CatalogueTestSupport.h"
#include "FactorPins.h"
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
#include <iomanip>
#include <iterator>
#include <random>
#include <span>
#include <sstream>
#include <string>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;
using namespace loadpath::test::oracle;

namespace
{
constexpr double PinRelative = 1.0e-10;
constexpr double PinAbsolute = 1.0e-12;

StandingOptions AllDense(StandingOptions Options)
{
	Options.DenseDofLimit = INT_MAX;
	return Options;
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

FactorCase SeededCase(int Draw)
{
	SeededDraw Each = MakeSeededDraw(Draw);
	const std::string Name = "master seed " + std::to_string(MasterSeed) + ", draw " + std::to_string(Draw) + " (generator seed " + std::to_string(Each.Seed) + "): " + Each.Fixture + ", lambda (" + std::to_string(Each.LambdaX) + ", " + std::to_string(Each.LambdaY) + ")";
	return FactorCase{Name, std::move(Each.Model), Biaxial(Each.LambdaX, Each.LambdaY), {}};
}

FactorCase WallCase(int Draw)
{
	const std::uint32_t Seed = MasterSeed + WallSeedOffset + static_cast<std::uint32_t>(Draw);
	std::mt19937 Rng(Seed);
	const int Courses = 3 + static_cast<int>(Rng() % 4u);
	const int Bricks = 3 + static_cast<int>(Rng() % 6u);
	const double Friction = Uniform(Rng, 0.4, 1.0);
	const double LambdaX = Uniform(Rng, 0.0, 0.5);
	const double LambdaY = Uniform(Rng, 0.0, 0.5);
	const std::string Name = "master seed " + std::to_string(MasterSeed) + ", wall draw " + std::to_string(Draw) + " (seed " + std::to_string(Seed) + "): " + std::to_string(Courses) + " x " + std::to_string(Bricks) + " dry wall, mu " + std::to_string(Friction) + ", lambda (" + std::to_string(LambdaX) + ", " + std::to_string(LambdaY) + ")";
	return FactorCase{Name, RunningBondWall(DryJoint(Friction), DryJoint(Friction), Courses, Bricks), Biaxial(LambdaX, LambdaY), {}};
}

double Tangential(const InterfaceReading& R) { return std::sqrt(R.ShearForceN.X * R.ShearForceN.X + R.ShearForceN.Y * R.ShearForceN.Y + R.ShearForceN.Z * R.ShearForceN.Z); }

bool PinNear(double Got, double Want)
{
	if (std::isnan(Got) || std::isnan(Want))
	{
		return std::isnan(Got) && std::isnan(Want);
	}
	if (Got == Want)
	{
		return true;
	}
	return std::abs(Got - Want) <= PinRelative * std::max(std::abs(Got), std::abs(Want)) + PinAbsolute;
}

struct PinCheck
{
	int Failed = 0;
	int Readings = 0;
};

void CheckPin(const FactorCase& C, const FactorPin& Pin, std::span<const ReadingPin> Readings, PinCheck& Out)
{
	const StandingResult Got = SolveWith(C.Model, C.Options, C.Loads);
	bool bOk = static_cast<int>(Got.State) == Pin.State && Got.ActiveSetIterations == Pin.Iterations && static_cast<int>(Got.UnconstrainedBodies.size()) == Pin.Unconstrained;
	std::string Detail;
	for (const ReadingPin& R : Readings.subspan(static_cast<std::size_t>(Pin.FirstReading), static_cast<std::size_t>(Pin.ReadingCount)))
	{
		++Out.Readings;
		const InterfaceReading* Found = FindReading(Got, InterfaceId{R.Index, R.Generation});
		const double Value = Found == nullptr ? NaN : (R.bTangential ? Tangential(*Found) : Found->NormalForceN);
		if (Found == nullptr || !PinNear(Value, R.Value))
		{
			bOk = false;
			std::ostringstream Line;
			Line << std::setprecision(17) << " interface " << R.Index << (R.bTangential ? " tangential " : " normal ");
			if (Found == nullptr)
			{
				Line << "missing";
			}
			else
			{
				Line << Value;
			}
			Line << " pinned " << R.Value << ";";
			Detail += Line.str();
		}
	}
	if (!bOk)
	{
		++Out.Failed;
		FAIL_CHECK(C.Name << ": got " << Got.State << " after " << Got.ActiveSetIterations << " with " << Got.UnconstrainedBodies.size() << " unconstrained, pinned " << static_cast<StandingState>(Pin.State) << " after " << Pin.Iterations << " with " << Pin.Unconstrained << ";" << Detail);
	}
}
}

TEST_CASE("Statics.Factor.TheBlockedFactorReadsItsPins")
{
	SUBCASE("named fixtures")
	{
		const std::vector<FactorCase> Cases = NamedCases();
		REQUIRE(Cases.size() == std::size(NamedPins));
		PinCheck Out;
		for (std::size_t I = 0; I < Cases.size(); ++I)
		{
			CheckPin(Cases[I], NamedPins[I], NamedReadingPins, Out);
		}
		CHECK(Out.Failed == 0);
	}
	SUBCASE("seeded stacks and bridges")
	{
		REQUIRE(static_cast<std::size_t>(FactorDrawCount) <= std::size(SeededPins));
		PinCheck Out;
		for (int Draw = 0; Draw < FactorDrawCount; ++Draw)
		{
			CheckPin(SeededCase(Draw), SeededPins[Draw], SeededReadingPins, Out);
		}
		MESSAGE("over " << FactorDrawCount << " draws: " << Out.Failed << " differ from their pins, " << Out.Readings << " readings compared");
		CHECK(Out.Failed == 0);
	}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Statics.Factor.TheBlockedFactorReadsItsPinsAcrossPanels")
{
	SUBCASE("named walls")
	{
		const std::vector<FactorCase> Cases = MultiPanelCases();
		REQUIRE(Cases.size() == std::size(PanelPins));
		PinCheck Out;
		for (std::size_t I = 0; I < Cases.size(); ++I)
		{
			const Clock::time_point Start = Clock::now();
			CheckPin(Cases[I], PanelPins[I], PanelReadingPins, Out);
			MESSAGE(Cases[I].Name << ": " << SecondsSince(Start) << " s");
		}
		CHECK(Out.Failed == 0);
	}
	SUBCASE("seeded walls")
	{
		REQUIRE(static_cast<std::size_t>(WallDrawCount) <= std::size(WallPins));
		PinCheck Out;
		int MultiPanel = 0;
		for (int Draw = 0; Draw < WallDrawCount; ++Draw)
		{
			const FactorCase C = WallCase(Draw);
			MultiPanel += 6 * (C.Model.LiveBodyCount() - 1) > 64 ? 1 : 0;
			CheckPin(C, WallPins[Draw], WallReadingPins, Out);
		}
		MESSAGE("over " << WallDrawCount << " wall draws (" << MultiPanel << " above one panel): " << Out.Failed << " differ from their pins, " << Out.Readings << " readings compared");
		CHECK(Out.Failed == 0);
	}
}

TEST_CASE("Slow.Statics.Factor.AMidSizeWallSolvesInsideItsBudget")
{
	const Structure Model = Ec6Wall(16, 20);
	REQUIRE(Model.LiveBodyCount() == 16u * 20u + 8u + 1u);
	const Clock::time_point Start = Clock::now();
	const StandingResult Blocked = SolveWith(Model, AllDense(StandingOptions{}));
	const double BlockedS = SecondsSince(Start);
	MESSAGE("ec6 wall 20 x 16 (" << Model.LiveBodyCount() - 1 << " bricks): blocked " << BlockedS << " s (" << Blocked.State << " after " << Blocked.ActiveSetIterations << ")");
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
