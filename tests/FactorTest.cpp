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
#include <fstream>
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
	int Traded = 0;
	int Readings = 0;
};

int TradeBudget(int Draws) { return std::max(1, Draws / 1000); }

bool PinsCertified(int State) { return State == static_cast<int>(StandingState::Stands) || State == static_cast<int>(StandingState::Overstressed); }

bool PinsUncertified(int State) { return State == static_cast<int>(StandingState::Mechanism) || State == static_cast<int>(StandingState::Undecided); }

void CheckPin(const FactorCase& C, const FactorPin& Pin, std::span<const ReadingPin> Readings, PinCheck& Out)
{
	const StandingResult Got = SolveWith(C.Model, C.Options, C.Loads);
	const int State = static_cast<int>(Got.State);
	const int Unconstrained = static_cast<int>(Got.UnconstrainedBodies.size());
	if (State != Pin.State && PinsUncertified(State) && PinsUncertified(Pin.State))
	{
		++Out.Traded;
		MESSAGE(C.Name << ": traded, got " << Got.State << ", pinned " << static_cast<StandingState>(Pin.State));
		return;
	}
	bool bOk = State == Pin.State;
	if (PinsCertified(Pin.State))
	{
		bOk = bOk && Got.ActiveSetIterations == Pin.Iterations && Unconstrained == Pin.Unconstrained;
	}
	else if (Pin.State == static_cast<int>(StandingState::Mechanism))
	{
		bOk = bOk && Unconstrained == Pin.Unconstrained;
	}
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
		FAIL_CHECK(C.Name << ": got " << Got.State << " after " << Got.ActiveSetIterations << " with " << Unconstrained << " unconstrained, pinned " << static_cast<StandingState>(Pin.State) << " after " << Pin.Iterations << " with " << Pin.Unconstrained << ";" << Detail);
	}
}

void CheckTable(const PinCheck& Out, int Draws)
{
	MESSAGE("over " << Draws << " draws: " << Out.Failed << " differ from their pins, " << Out.Traded << " traded mechanism and undecided (budget " << TradeBudget(Draws) << "), " << Out.Readings << " readings compared");
	CHECK(Out.Failed == 0);
	CHECK(Out.Traded <= TradeBudget(Draws));
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
		CheckTable(Out, static_cast<int>(Cases.size()));
	}
	SUBCASE("seeded stacks and bridges")
	{
		REQUIRE(static_cast<std::size_t>(FactorDrawCount) <= std::size(SeededPins));
		PinCheck Out;
		for (int Draw = 0; Draw < FactorDrawCount; ++Draw)
		{
			CheckPin(SeededCase(Draw), SeededPins[Draw], SeededReadingPins, Out);
		}
		CheckTable(Out, FactorDrawCount);
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
		CheckTable(Out, static_cast<int>(Cases.size()));
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
		MESSAGE(MultiPanel << " of " << WallDrawCount << " wall draws above one panel");
		CheckTable(Out, WallDrawCount);
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

namespace
{
void EmitPinValue(std::ostream& Out, double V)
{
	if (std::isnan(V))
	{
		Out << "std::numeric_limits<double>::quiet_NaN()";
	}
	else if (std::isinf(V))
	{
		Out << (V > 0 ? "" : "-") << "std::numeric_limits<double>::infinity()";
	}
	else
	{
		Out << std::setprecision(17) << V;
	}
}

void EmitPinTable(std::ostream& Out, const char* Name, const std::vector<FactorCase>& Cases)
{
	std::ostringstream Rows;
	std::ostringstream Reads;
	int Next = 0;
	for (const FactorCase& C : Cases)
	{
		const StandingResult B = SolveWith(C.Model, C.Options, C.Loads);
		const int State = static_cast<int>(B.State);
		const bool bCertified = PinsCertified(State);
		const int Iterations = bCertified ? B.ActiveSetIterations : -1;
		const int Unconstrained = B.State == StandingState::Undecided ? -1 : static_cast<int>(B.UnconstrainedBodies.size());
		int Count = 0;
		for (const StandingComponent& Comp : bCertified ? B.Components : std::vector<StandingComponent>{})
		{
			for (int Kind = 0; Kind < 2; ++Kind)
			{
				const InterfaceReading* Best = nullptr;
				double BestV = -1.0;
				for (const std::size_t Slot : Comp.ReadingSlots)
				{
					const InterfaceReading& Rd = B.Readings[Slot];
					const double V = Kind == 0 ? std::abs(Rd.NormalForceN) : Tangential(Rd);
					if (std::isfinite(V) && V > BestV)
					{
						BestV = V;
						Best = &Rd;
					}
				}
				if (Best == nullptr)
				{
					continue;
				}
				Reads << "\t{" << Best->Id.Index << "u, " << Best->Id.Generation << "u, " << (Kind == 1 ? "true" : "false") << ", ";
				EmitPinValue(Reads, Kind == 0 ? Best->NormalForceN : Tangential(*Best));
				Reads << "},\n";
				++Count;
			}
		}
		Rows << "\t{" << State << ", " << Iterations << ", " << Unconstrained << ", " << Next << ", " << Count << "},\n";
		Next += Count;
	}
	Out << "inline constexpr FactorPin " << Name << "Pins[] = {\n" << Rows.str() << "};\n";
	Out << "inline constexpr ReadingPin " << Name << "ReadingPins[] = {\n" << Reads.str() << "};\n";
	MESSAGE(std::string(Name) << ": " << Cases.size() << " cases, " << Next << " readings");
}

std::string PinsHeaderPath()
{
	const std::string Source = __FILE__;
	const std::size_t Slash = Source.find_last_of("/\\");
	return (Slash == std::string::npos ? std::string() : Source.substr(0, Slash + 1)) + "FactorPins.h";
}
}

TEST_CASE("Measure.Factor.GeneratePins")
{
	std::ostringstream Out;
	Out << "#pragma once\n\n";
	Out << "// Generated by Measure.Factor.GeneratePins (tests/FactorTest.cpp, Release) from the code under test; regenerate only with a reason recorded in ANCHORS.\n";
	Out << "// Iterations are -1 unless the state is Stands or Overstressed, and Unconstrained is -1 on Undecided: those fields are not pinned.\n\n";
	Out << "#include <cstdint>\n\nnamespace loadpath::test\n{\n";
	Out << "struct FactorPin\n{\n\tint State;\n\tint Iterations;\n\tint Unconstrained;\n\tint FirstReading;\n\tint ReadingCount;\n};\n\n";
	Out << "struct ReadingPin\n{\n\tstd::uint32_t Index;\n\tstd::uint32_t Generation;\n\tbool bTangential;\n\tdouble Value;\n};\n\n";
	EmitPinTable(Out, "Named", NamedCases());
	EmitPinTable(Out, "Panel", MultiPanelCases());
	std::vector<FactorCase> Seeded;
	for (int Draw = 0; Draw < FactorDrawCount; ++Draw)
	{
		Seeded.push_back(SeededCase(Draw));
	}
	EmitPinTable(Out, "Seeded", Seeded);
	std::vector<FactorCase> Walls;
	for (int Draw = 0; Draw < WallDrawCount; ++Draw)
	{
		Walls.push_back(WallCase(Draw));
	}
	EmitPinTable(Out, "Wall", Walls);
	Out << "}\n";
	const std::string Path = PinsHeaderPath();
	std::ofstream File(Path, std::ios::binary | std::ios::trunc);
	REQUIRE(File.good());
	File << Out.str();
	File.close();
	CHECK(File.good());
	MESSAGE("wrote " << Path);
}
#endif
