#include "AnchorFixtures.h"
#include "BrickFixtures.h"
#include "CascadeTestSupport.h"
#include "CatalogueTestSupport.h"
#include "StaticAdmissibilityOracle.h"
#include "StaticsTestSupport.h"

#include "loadpath/Catalogue.h"
#include "loadpath/geometry/RunningBondWall.h"

#include "doctest.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;
using namespace loadpath::test::oracle;

namespace
{
using Clock = std::chrono::steady_clock;
using FactorKind = StandingOptions::FactorKind;

double SecondsSince(Clock::time_point Start) { return std::chrono::duration<double>(Clock::now() - Start).count(); }

struct FactorCase
{
	std::string Name;
	Structure Model;
	StandingOptions Options;
	std::vector<BodyLoad> Loads;
};

double WeightN(const Structure& Model)
{
	double Sum = 0.0;
	for (const BodyId& Id : Model.LiveBodies())
	{
		if (!Model.Body(Id).bGrounded)
		{
			Sum += Model.Body(Id).MassKg * GravityMPerS2;
		}
	}
	return Sum;
}

StandingOptions WithFactor(StandingOptions Options, FactorKind Kind)
{
	Options.Factor = Kind;
	return Options;
}

struct Gaps
{
	double WorstRelative = 0.0;
	double WorstForceAbsoluteOverW = 0.0;
	double WorstUtilisationAbsolute = 0.0;
};

bool Near(double Got, double Want, double AbsoluteFloor, Gaps& G, bool bForce, double WeightScale)
{
	if (std::isnan(Got) || std::isnan(Want))
	{
		return std::isnan(Got) && std::isnan(Want);
	}
	if (Got == Want)
	{
		return true;
	}
	const double Gap = std::abs(Got - Want);
	const double Scale = std::max(std::abs(Got), std::abs(Want));
	if (std::isfinite(Gap))
	{
		if (Gap > AbsoluteFloor)
		{
			G.WorstRelative = std::max(G.WorstRelative, Gap / Scale);
		}
		if (bForce)
		{
			G.WorstForceAbsoluteOverW = std::max(G.WorstForceAbsoluteOverW, Gap / WeightScale);
		}
		else
		{
			G.WorstUtilisationAbsolute = std::max(G.WorstUtilisationAbsolute, Gap);
		}
	}
	return Gap <= 1e-7 * Scale + AbsoluteFloor;
}

std::vector<BodyId> MechanismBodies(const StandingResult& R)
{
	std::vector<BodyId> Out;
	for (const MechanismMotion& M : R.Mechanism)
	{
		Out.push_back(M.Body);
	}
	return Out;
}

bool Certified(StandingState S) { return S == StandingState::Stands || S == StandingState::Overstressed || S == StandingState::InvalidInput; }

struct Comparison
{
	int Mismatches = 0;
	bool bStateClassTraded = false;
	bool bIterationsDiffer = false;
	bool bCertified = false;
	Gaps G;
};

Comparison CompareKinds(const FactorCase& C, bool bAllowTrade)
{
	const StandingResult Ref = SolveWith(C.Model, WithFactor(C.Options, FactorKind::Reference), C.Loads);
	const StandingResult Blk = SolveWith(C.Model, WithFactor(C.Options, FactorKind::Blocked), C.Loads);
	const double W = WeightN(C.Model);
	const double ForceFloorN = 1e-9 * W;
	const double WeightScale = W > 0.0 ? W : 1.0;
	INFO(C.Name << ": reference " << Ref.State << " after " << Ref.ActiveSetIterations << ", blocked " << Blk.State << " after " << Blk.ActiveSetIterations);
	Comparison Out;
	auto Expect = [&](bool bOk, const std::string& What) {
		if (!bOk)
		{
			++Out.Mismatches;
			FAIL_CHECK(C.Name << ": " << What);
		}
	};
	const bool bSameState = Ref.State == Blk.State;
	Out.bCertified = bSameState && (Ref.State == StandingState::Stands || Ref.State == StandingState::Overstressed);
	if (!bSameState && !Certified(Ref.State) && !Certified(Blk.State))
	{
		Out.bStateClassTraded = true;
		Expect(bAllowTrade, "mechanism and undecided traded on a fixture that allows no trade");
	}
	else
	{
		Expect(bSameState, "state");
	}
	Out.bIterationsDiffer = Ref.ActiveSetIterations != Blk.ActiveSetIterations;
	Expect(!Out.bCertified || std::abs(Ref.ActiveSetIterations - Blk.ActiveSetIterations) <= 2, "active-set iterations differ by more than 2");
	if (!bSameState || Ref.State == StandingState::Undecided)
	{
		return Out;
	}
	Expect(SameIdList(Ref.UnconstrainedBodies, Blk.UnconstrainedBodies), "unconstrained bodies");
	Expect(SameIdList(MechanismBodies(Ref), MechanismBodies(Blk)), "mechanism bodies");
	Expect(Ref.Readings.size() == Blk.Readings.size(), "reading count");
	auto Force = [&](double X, double Y) { return Near(X, Y, ForceFloorN, Out.G, true, WeightScale); };
	auto Util = [&](double X, double Y) { return Near(X, Y, 1e-9, Out.G, false, WeightScale); };
	for (std::size_t I = 0; I < Ref.Readings.size() && I < Blk.Readings.size(); ++I)
	{
		const InterfaceReading& A = Ref.Readings[I];
		const InterfaceReading& B = Blk.Readings[I];
		const std::string At = "reading " + std::to_string(I) + " ";
		Expect(SameHandle(A.Id, B.Id), At + "id");
		for (std::size_t K = 0; K < 4; ++K)
		{
			Expect(!Out.bCertified || A.Corners[K].bOpen == B.Corners[K].bOpen, At + "corner " + std::to_string(K) + " open flag");
		}
		if (!Out.bCertified)
		{
			Force(A.NormalForceN, B.NormalForceN);
			Util(A.Utilisation, B.Utilisation);
			continue;
		}
		Expect(Force(A.NormalForceN, B.NormalForceN), At + "normal force " + std::to_string(A.NormalForceN) + " vs " + std::to_string(B.NormalForceN));
		const bool bShearX = Force(A.ShearForceN.X, B.ShearForceN.X);
		const bool bShearY = Force(A.ShearForceN.Y, B.ShearForceN.Y);
		const bool bShearZ = Force(A.ShearForceN.Z, B.ShearForceN.Z);
		Expect(bShearX && bShearY && bShearZ, At + "shear force");
		Expect(Util(A.Utilisation, B.Utilisation), At + "utilisation");
		Expect(Util(A.CompressionUtilisation, B.CompressionUtilisation), At + "compression utilisation");
		Expect(Util(A.TensionUtilisation, B.TensionUtilisation), At + "tension utilisation");
		Expect(Util(A.ShearUtilisation, B.ShearUtilisation), At + "shear utilisation");
	}
	return Out;
}

StandingOptions Biaxial(double LambdaX, double LambdaY)
{
	StandingOptions Options;
	Options.Gravity = BiaxialGravity(LambdaX, LambdaY);
	return Options;
}

Structure RunningBondWall(const StrengthProfile& Bed, const StrengthProfile& Perpend, int Courses, int BricksPerCourse, geometry::Wall* Out = nullptr)
{
	Structure Model;
	geometry::WallSpec Spec;
	Spec.Bed = AddProfileOk(Model, Bed);
	Spec.Perpend = AddProfileOk(Model, Perpend);
	Spec.Courses = Courses;
	Spec.BricksPerCourse = BricksPerCourse;
	const geometry::Wall W = geometry::BuildRunningBondWall(Model, Spec);
	if (Out != nullptr)
	{
		*Out = W;
	}
	return Model;
}

Structure Ec6Wall(int Courses, int BricksPerCourse)
{
	const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
	INFO("line " << Loaded.Error.Line << ": " << Loaded.Error.Message);
	REQUIRE(Loaded.Ok());
	return RunningBondWall(JointOrFail(Loaded.Value, "clay_gp_mortar_bed_ec6").Profile, JointOrFail(Loaded.Value, "clay_gp_mortar_perpend_ec6").Profile, Courses, BricksPerCourse);
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

constexpr int FactorDrawCount = bOptimisedBuild ? 6000 : 40;
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
			Into.WorstRelative = std::max(Into.WorstRelative, R.G.WorstRelative);
			Into.WorstForceAbsoluteOverW = std::max(Into.WorstForceAbsoluteOverW, R.G.WorstForceAbsoluteOverW);
			Into.WorstUtilisationAbsolute = std::max(Into.WorstUtilisationAbsolute, R.G.WorstUtilisationAbsolute);
		}
		const int TradeBudget = std::max(1, FactorDrawCount / 1000);
		MESSAGE("over " << FactorDrawCount << " draws: " << Traded << " traded mechanism and undecided (budget " << TradeBudget << "), " << IterationsDiffer << " differ in iterations, worst relative reading gap above the absolute floor " << Worst.WorstRelative << ", worst absolute force gap " << Worst.WorstForceAbsoluteOverW << " W, worst absolute utilisation gap " << Worst.WorstUtilisationAbsolute << " on mechanism draws (not asserted); " << Disagreeing << " draws disagree, " << CertifiedDisagreeing << " of them stands or overstressed, whose worst gaps are " << WorstCertified.WorstRelative << " relative, " << WorstCertified.WorstForceAbsoluteOverW << " W, " << WorstCertified.WorstUtilisationAbsolute << " utilisation");
		CHECK(Disagreeing == 0);
		CHECK(Traded <= TradeBudget);
	}
}

TEST_CASE("Slow.Statics.Factor.AMidSizeWallSolvesInsideItsBudget")
{
	const Structure Model = Ec6Wall(16, 20);
	REQUIRE(Model.LiveBodyCount() == 16u * 20u + 8u + 1u);
	Clock::time_point Start = Clock::now();
	const StandingResult Blocked = SolveWith(Model, StandingOptions{});
	const double BlockedS = SecondsSince(Start);
	Start = Clock::now();
	const StandingResult Reference = SolveWith(Model, WithFactor(StandingOptions{}, FactorKind::Reference));
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
	const StandingResult Blocked = SolveWith(Model, StandingOptions{});
	MESSAGE("ec6 wall 40 x 32 (" << Model.LiveBodyCount() - 1 << " bricks): blocked elastic " << SecondsSince(Start) << " s (" << Blocked.State << " after " << Blocked.ActiveSetIterations << ")");
}
#endif
