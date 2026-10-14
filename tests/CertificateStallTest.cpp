#include "AnchorFixtures.h"
#include "ArchFixtures.h"
#include "BrickFixtures.h"
#include "CatalogueTestSupport.h"
#include "CertificateTestSupport.h"
#include "FactorTestSupport.h"
#include "SlowInDebug.h"
#include "StaticAdmissibilityOracle.h"
#include "StaticsTestSupport.h"
#include "WedgeFixture.h"

#include "loadpath/Catalogue.h"
#include "loadpath/CollapseCertificate.h"
#include "loadpath/StructuralAuthority.h"
#include "loadpath/geometry/RunningBondWall.h"
#include "loadpath/geometry/Warehouse.h"

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
template <class O>
constexpr bool HasStallRule = requires(O X) {
	X.StallWindow = 0;
	X.StallImprovement = 0.0;
};

template <class O>
O WithStall(O Options, int Window, double Improvement)
{
	if constexpr (HasStallRule<O>)
	{
		Options.StallWindow = Window;
		Options.StallImprovement = Improvement;
	}
	else
	{
		(void)Window;
		(void)Improvement;
	}
	return Options;
}

template <class O>
O WithStallWindow(O Options, int Window)
{
	if constexpr (HasStallRule<O>)
	{
		Options.StallWindow = Window;
	}
	else
	{
		(void)Window;
	}
	return Options;
}

template <class O>
void CheckStallDefaults()
{
	if constexpr (HasStallRule<O>)
	{
		CHECK(O{}.StallWindow == 20);
		CHECK(O{}.StallImprovement == 0.5);
	}
}

void RequireTheStallRuleExists()
{
	CHECK_MESSAGE(HasStallRule<CertificateOptions>, "CertificateOptions has no StallWindow / StallImprovement, so the rule was neither set nor disabled");
}

constexpr double PinnedRelative = 1.0e-10;
constexpr double PinnedAbsolute = 1.0e-10;
constexpr int DefaultStallWindow = 20;
constexpr double DefaultStallImprovement = 0.5;

bool Near(double Got, double Want, double Relative, double Absolute)
{
	return std::abs(Got - Want) <= Relative * std::max(std::abs(Got), std::abs(Want)) + Absolute;
}

struct NamedCase
{
	std::string Name;
	Structure Model;
	CertificateOptions Options;
};

struct PinnedCase
{
	std::string Name;
	Structure Model;
	CertificateOptions Options;
	CertificateState State;
	int Iterations;
	double LoadFactorLower;
	double LoadFactorUpper;
};

StrengthProfile CappedDryJoint()
{
	StrengthProfile Profile = DryJoint(5.0);
	Profile.MaxShearStrengthPa = 0.3 * BrickMassKg * GravityMPerS2 / (BrickLengthM * BrickWidthM);
	return Profile;
}

std::vector<oracle::SeededDraw> FirstSeededBridges(std::size_t Count)
{
	std::vector<oracle::SeededDraw> Bridges;
	for (int Draw = 0; Bridges.size() < Count; ++Draw)
	{
		oracle::SeededDraw D = oracle::MakeSeededDraw(Draw);
		if (D.Fixture.rfind("bridge", 0) == 0)
		{
			Bridges.push_back(std::move(D));
		}
	}
	return Bridges;
}

void Add(std::vector<PinnedCase>& Cases, std::string Name, Structure Model, CertificateOptions Options, CertificateState State, int Iterations, double Lower, double Upper)
{
	Cases.push_back({std::move(Name), std::move(Model), Options, State, Iterations, Lower, Upper});
}

std::vector<PinnedCase> PinnedCases()
{
	using S = CertificateState;
	std::vector<PinnedCase> Cases;
	Add(Cases, "arch eta 0.1095", MakeSemicircularArch(0.1095, DryJoint(0.7)).Model, CertificateOptions{}, S::Stands, 11, 999.99999999997044, 1000.0);
	Add(Cases, "hollow pile at lateral 0.5015", MakeHollowPile(0.7).Model, CertificateLateral(0.5015), S::Collapses, 21, 6.9218768195853732e-13, 3.8196297499201228e-11);
	CertificateOptions Wedge;
	Wedge.ConeGenerators = 64;
	Add(Cases, "itasca wedge 0.05 deg above critical", MakeItascaWedge(AssociativeWedgeFrictionAngleRad(ItascaWedgeGeometry()) + 0.05 * RadPerDegree).Model, Wedge, S::Stands, 13, 999.99999999775275, 1000.0);
	Add(Cases, "dry 6 x 5 wall at lateral 0.3", RunningBondWall(DryJoint(0.6), DryJoint(0.6), 6, 5), CertificateLateral(0.3), S::Stands, 11, 999.99999999978434, 1000.0);
	Add(Cases, "ec6 4 x 4 wall at lateral 0.4", Ec6Wall(4, 4), CertificateLateral(0.4), S::Stands, 21, 625.07665742313804, 637.32263311103361);
	Add(Cases, "ec6 4 x 5 wall at rest", Ec6Wall(4, 5), CertificateOptions{}, S::Stands, 12, 999.99999999999238, 1000.0);
	Add(Cases, "housner block at 0.99 tan alpha", MakeBlockOnGround(0.25, 0.25, 1.0, 0.7).Model, CertificateLateral(0.99 * 0.25), S::Stands, 9, 999.99999999439137, 1000.0);
	Add(Cases, "shear-capped brick at lateral 1", MakeBrickOnGround(CappedDryJoint()).Model, CertificateLateral(1.0), S::Collapses, 8, 0.29423558410632117, 0.29999999998514099);
	Add(Cases, "shear-capped brick at lateral 0.29", MakeBrickOnGround(CappedDryJoint()).Model, CertificateLateral(0.29), S::Stands, 8, 1.0146054624826155, 1.034482758617457);
	std::vector<oracle::SeededDraw> Bridges = FirstSeededBridges(2);
	const int BridgeIterations[] = {11, 10};
	const double BridgeLower[] = {999.99999999884676, 999.99999999374359};
	for (std::size_t Index = 0; Index < Bridges.size(); ++Index)
	{
		oracle::SeededDraw& D = Bridges[Index];
		Add(Cases, "seed " + std::to_string(D.Seed) + " " + D.Fixture, std::move(D.Model), CertificateUnderGravity(oracle::BiaxialGravity(D.LambdaX, D.LambdaY)), S::Stands, BridgeIterations[Index], BridgeLower[Index], 1000.0);
	}
	Add(Cases, "dry 8 x 12 wall at lateral 0.8", RunningBondWall(DryJoint(0.6), DryJoint(0.6), 8, 12), CertificateLateral(0.8), S::Collapses, 11, 1.2203697574971972e-12, 1.7974968171468829e-12);
	return Cases;
}

std::vector<NamedCase> AgreementCases()
{
	std::vector<NamedCase> Cases;
	const int Draws = oracle::bOptimisedBuild ? 60 : 12;
	for (int Draw = 0; Draw < Draws; ++Draw)
	{
		oracle::SeededDraw D = oracle::MakeSeededDraw(Draw);
		Cases.push_back({"seed " + std::to_string(D.Seed) + " " + D.Fixture, std::move(D.Model), CertificateUnderGravity(oracle::BiaxialGravity(D.LambdaX, D.LambdaY))});
	}
	const std::vector<double> Lateral = oracle::bOptimisedBuild ? std::vector<double>{0.05, 0.3, 0.8} : std::vector<double>{0.05, 0.3};
	for (const double Lambda : Lateral)
	{
		Cases.push_back({"dry 6 x 5 wall at lateral " + std::to_string(Lambda), RunningBondWall(DryJoint(0.6), DryJoint(0.6), 6, 5), CertificateLateral(Lambda)});
	}
	if (oracle::bOptimisedBuild)
	{
		Cases.push_back({"ec6 4 x 4 wall at lateral 0.4", Ec6Wall(4, 4), CertificateLateral(0.4)});
	}
	return Cases;
}

Structure HalfWarehouse()
{
	const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
	REQUIRE(Loaded.Ok());
	geometry::WarehouseSpec Spec;
	Spec.LengthBricks = 30;
	Spec.WidthBricks = 15;
	Spec.Courses = 20;
	Spec.Bays = 3;
	Spec.PlanksPerBay = 4;
	Spec.Openings = {geometry::Opening{0, 10, 3, 0, 12}, geometry::Opening{2, 8, 3, 8, 14}};
	Structure Model;
	Spec.Bed = AddProfileOk(Model, JointOrFail(Loaded.Value, "clay_gp_mortar_bed_ec6").Profile);
	Spec.Perpend = AddProfileOk(Model, JointOrFail(Loaded.Value, "clay_gp_mortar_perpend_ec6").Profile);
	Spec.Bearing = AddProfileOk(Model, JointOrFail(Loaded.Value, "c24_bearing_perp_to_grain").Profile);
	Spec.Plank = AddProfileOk(Model, FastenerProfile(FastenerOrFail(Loaded.Value, "ec5_screw_8x60_steel_to_glulam"), 2, Spec.PlankWidthM * Spec.BeamWidthM));
	geometry::BuildWarehouse(Model, Spec);
	return Model;
}

constexpr std::size_t HalfWarehouseWallBodies = 595;
constexpr double HalfWarehouseWallLower = 350.2085;
constexpr double HalfWarehouseWallUpper = 350.4239;
constexpr double HalfWarehouseWallRelative = 1.0e-6;
constexpr int HalfWarehouseWallIterationCeiling = 69;
constexpr double UnreachableTolerance = 1.0e-15;
constexpr int StalledWallFiringIteration = 69;
constexpr int FailedResolveIterations = 23;
constexpr int SecondRungIterations = 29;
constexpr double SecondRungLower = 624.4515807676147;
}

TEST_CASE("Slow.Certificate.Stall.TheHalfWarehouseWallConvergesUnderTheRule")
{
	if (!oracle::bOptimisedBuild)
	{
		return;
	}
	const Structure Model = HalfWarehouse();
	const std::vector<std::vector<BodyId>>& Parts = Model.Components().FreeBodies;
	const auto Wall = std::find_if(Parts.begin(), Parts.end(), [](const std::vector<BodyId>& Part) { return Part.size() == HalfWarehouseWallBodies; });
	REQUIRE(Wall != Parts.end());
	const Clock::time_point Start = Clock::now();
	const CertificateResult Got = MakeLimitAnalysisCertificate()->Decide(Model, {}, CertificateOptions{}, *Wall);
	const double Seconds = SecondsSince(Start);
	MESSAGE(std::setprecision(12) << "595-body wall: " << Got.State << " [" << Got.LoadFactorLower << ", " << Got.LoadFactorUpper << "] in " << Got.Iterations << " iterations, " << Seconds << " s");
	CHECK(Got.State == CertificateState::Stands);
	CHECK(Near(Got.LoadFactorLower, HalfWarehouseWallLower, HalfWarehouseWallRelative, 0.0));
	CHECK(Near(Got.LoadFactorUpper, HalfWarehouseWallUpper, HalfWarehouseWallRelative, 0.0));
	CHECK(Got.Iterations <= HalfWarehouseWallIterationCeiling);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.Stall.ANeverConvergingSolveStopsBeforeTheCap")
{
	RequireTheStallRuleExists();
	CertificateOptions Options = CertificateLateral(0.4);
	Options.FeasibilityTolerance = UnreachableTolerance;
	Options.bStallRecovery = false;
	const Structure Model = Ec6Wall(4, 4);
	const CertificateResult Off = DecideWith(Model, WithStallWindow(Options, 0));
	const CertificateResult On = DecideWith(Model, Options);
	INFO(std::setprecision(17) << "ec6 4 x 4 wall at lateral 0.4, tolerance 1e-15: rule off " << Off.State << " in " << Off.Iterations << " [" << Off.LoadFactorLower << ", " << Off.LoadFactorUpper << "], rule on " << On.State << " in " << On.Iterations << " [" << On.LoadFactorLower << ", " << On.LoadFactorUpper << "]");
	CHECK(Off.State == CertificateState::Undecided);
	CHECK(Off.Iterations == Options.MaxIterations);
	CHECK(On.State == Off.State);
	CHECK(On.Iterations < Options.MaxIterations);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.Stall.AConvergingSolveIsNeverCutShort")
{
	RequireTheStallRuleExists();
	int Mismatches = 0;
	for (const PinnedCase& C : PinnedCases())
	{
		for (const int Window : {DefaultStallWindow, 0})
		{
			const CertificateResult Got = DecideWith(C.Model, WithStallWindow(C.Options, Window));
			const bool bSame = Got.State == C.State && Got.Iterations == C.Iterations && Near(Got.LoadFactorLower, C.LoadFactorLower, PinnedRelative, PinnedAbsolute) && Near(Got.LoadFactorUpper, C.LoadFactorUpper, PinnedRelative, PinnedAbsolute);
			INFO(std::setprecision(17) << C.Name << " window " << Window << ": got " << Got.State << " in " << Got.Iterations << " [" << Got.LoadFactorLower << ", " << Got.LoadFactorUpper << "], pinned " << C.State << " in " << C.Iterations << " [" << C.LoadFactorLower << ", " << C.LoadFactorUpper << "]");
			CHECK(bSame);
			Mismatches += bSame ? 0 : 1;
		}
	}
	int Agreement = 0;
	for (const NamedCase& C : AgreementCases())
	{
		const CertificateResult On = DecideWith(C.Model, C.Options);
		const CertificateResult Off = DecideWith(C.Model, WithStallWindow(C.Options, 0));
		const bool bSame = On.State == Off.State && On.Iterations == Off.Iterations && On.LoadFactorLower == Off.LoadFactorLower && On.LoadFactorUpper == Off.LoadFactorUpper;
		INFO(std::setprecision(17) << C.Name << ": on " << On.State << " in " << On.Iterations << " [" << On.LoadFactorLower << ", " << On.LoadFactorUpper << "], off " << Off.State << " in " << Off.Iterations << " [" << Off.LoadFactorLower << ", " << Off.LoadFactorUpper << "]");
		CHECK(bSame);
		Mismatches += bSame ? 0 : 1;
		++Agreement;
	}
	MESSAGE(Agreement << " agreement cases, " << Mismatches << " mismatches");
	CHECK(Mismatches == 0);
}

TEST_CASE("Certificate.Stall.OptionsAreValidated")
{
	RequireTheStallRuleExists();
	CheckStallDefaults<CertificateOptions>();
	const BrickOnGround F = MakeBrickOnGround(DryJoint(0.7));
	const double Nan = std::numeric_limits<double>::quiet_NaN();
	const double Inf = std::numeric_limits<double>::infinity();
	struct Bad
	{
		const char* Name;
		int Window;
		double Improvement;
	};
	const Bad Cases[] = {{"window -1", -1, DefaultStallImprovement}, {"improvement 0", DefaultStallWindow, 0.0}, {"improvement 1", DefaultStallWindow, 1.0}, {"improvement -0.5", DefaultStallWindow, -0.5},
		{"improvement 1.5", DefaultStallWindow, 1.5}, {"improvement NaN", DefaultStallWindow, Nan}, {"improvement +inf", DefaultStallWindow, Inf}, {"improvement -inf", DefaultStallWindow, -Inf}};
	for (const Bad& B : Cases)
	{
		const CertificateOptions Options = WithStall(CertificateOptions{}, B.Window, B.Improvement);
		const CertificateResult Direct = DecideWith(F.Model, Options);
		AuthorityOptions Combined;
		Combined.Certificate = Options;
		const AuthorityResult Judged = MakeCombinedAuthority(MakeRbsmStandingAuthority(), MakeLimitAnalysisCertificate())->Judge(F.Model, {}, Combined, nullptr);
		INFO(std::string(B.Name) << ": Decide " << Direct.State);
		CHECK(Direct.State == CertificateState::InvalidInput);
		CHECK(Judged.Worst == ComponentVerdict::InvalidInput);
		CHECK(Judged.Components.empty());
	}
	const CertificateResult Disabled = DecideWith(F.Model, WithStall(CertificateOptions{}, 0, DefaultStallImprovement));
	CHECK(Disabled.State == CertificateState::Stands);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.Stall.TheDefaultWindowLeavesConvergingSolvesRoom")
{
	RequireTheStallRuleExists();
	CHECK(CertificateOptions{}.StallWindow == 20);
	CertificateOptions Options = CertificateLateral(0.4);
	Options.FeasibilityTolerance = UnreachableTolerance;
	Options.bStallRecovery = false;
	const CertificateResult Got = DecideWith(Ec6Wall(4, 4), Options);
	INFO("ec6 4 x 4 wall at lateral 0.4, tolerance 1e-15, default options: " << Got.State << " in " << Got.Iterations);
	CHECK(Got.State == CertificateState::Undecided);
	CHECK(Got.Iterations == StalledWallFiringIteration);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.Stall.TheWindowFiresExactlyItsLengthAfterItsStart")
{
	RequireTheStallRuleExists();
	CertificateOptions Options = CertificateLateral(0.4);
	Options.FeasibilityTolerance = UnreachableTolerance;
	Options.bStallRecovery = false;
	const Structure Model = Ec6Wall(4, 4);
	const CertificateResult Twenty = DecideWith(Model, WithStallWindow(Options, 20));
	const CertificateResult TwentyOne = DecideWith(Model, WithStallWindow(Options, 21));
	INFO("ec6 4 x 4 wall at lateral 0.4, tolerance 1e-15: window 20 " << Twenty.State << " in " << Twenty.Iterations << ", window 21 " << TwentyOne.State << " in " << TwentyOne.Iterations);
	CHECK(Twenty.State == CertificateState::Undecided);
	CHECK(TwentyOne.State == CertificateState::Undecided);
	CHECK(Twenty.Iterations == StalledWallFiringIteration);
	CHECK(TwentyOne.Iterations == StalledWallFiringIteration + 1);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.Stall.ImprovementMustBeByTheFactor")
{
	RequireTheStallRuleExists();
	CertificateOptions Options = CertificateLateral(0.4);
	Options.FeasibilityTolerance = UnreachableTolerance;
	Options.bStallRecovery = false;
	const Structure Model = Ec6Wall(4, 4);
	const CertificateResult Default = DecideWith(Model, WithStallWindow(Options, 3));
	const CertificateResult Strict = DecideWith(Model, WithStall(Options, 3, 1.0e-6));
	INFO("ec6 4 x 4 wall at lateral 0.4, tolerance 1e-15: window 3 factor 0.5 " << Default.State << " in " << Default.Iterations << ", window 3 factor 1e-6 " << Strict.State << " in " << Strict.Iterations);
	CHECK(Default.State == CertificateState::Undecided);
	CHECK(Default.Iterations == 7);
	CHECK(Strict.State == CertificateState::Undecided);
	CHECK(Strict.Iterations == 6);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.Stall.WithRecoveryTheStalledWallStands")
{
	CertificateOptions Options = CertificateLateral(0.4);
	Options.FeasibilityTolerance = UnreachableTolerance;
	CHECK(Options.bStallRecovery);
	const CertificateResult Got = DecideWith(Ec6Wall(4, 4), Options);
	INFO(std::setprecision(17) << "ec6 4 x 4 wall at lateral 0.4, tolerance 1e-15, recovery on: " << Got.State << " in " << Got.Iterations << " [" << Got.LoadFactorLower << ", " << Got.LoadFactorUpper << "]");
	CHECK(Got.State == CertificateState::Stands);
	CHECK(Near(Got.LoadFactorLower, 625.07665742313804, 1.0e-3, 0.0));
	CHECK(Got.LoadFactorLower <= 625.07665742313804 * (1.0 + 1.0e-9));
	CHECK(Near(Got.LoadFactorUpper, 637.32263311103361, 1.0e-3, 0.0));
	CHECK(Got.LoadFactorUpper >= 637.32263311103361 * (1.0 - 1.0e-9));
	CHECK(Got.Iterations <= Options.MaxIterations);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.Stall.AFailedCappedResolveStaysUndecided")
{
	CertificateOptions Options = CertificateLateral(0.4);
	Options.FeasibilityTolerance = UnreachableTolerance;
	Options.MaxIterations = FailedResolveIterations;
	CHECK(Options.bStallRecovery);
	const CertificateResult Got = DecideWith(Ec6Wall(4, 4), Options);
	INFO(std::setprecision(17) << "ec6 4 x 4 wall at lateral 0.4, tolerance 1e-15, MaxIterations " << Options.MaxIterations << ", recovery on: " << Got.State << " in " << Got.Iterations << " [" << Got.LoadFactorLower << ", " << Got.LoadFactorUpper << "]");
	MESSAGE(std::setprecision(17) << "failed capped re-solve fixture: " << Got.State << " in " << Got.Iterations << " [" << Got.LoadFactorLower << ", " << Got.LoadFactorUpper << "]");
	CHECK(Got.State == CertificateState::Undecided);
	CHECK(Got.Iterations <= Options.MaxIterations);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.Stall.TheSecondRungDecidesWhenTheFirstFails")
{
	CertificateOptions Options = CertificateLateral(0.4);
	Options.FeasibilityTolerance = UnreachableTolerance;
	Options.MaxIterations = SecondRungIterations;
	CHECK(Options.bStallRecovery);
	const CertificateResult Got = DecideWith(Ec6Wall(4, 4), Options);
	INFO(std::setprecision(17) << "ec6 4 x 4 wall at lateral 0.4, tolerance 1e-15, MaxIterations " << Options.MaxIterations << ", recovery on: " << Got.State << " in " << Got.Iterations << " [" << Got.LoadFactorLower << ", " << Got.LoadFactorUpper << "]");
	CHECK(Got.State == CertificateState::Stands);
	CHECK(Near(Got.LoadFactorLower, SecondRungLower, PinnedRelative, 0.0));
	CHECK(Got.LoadFactorLower <= 625.07665742313804 * (1.0 + 1.0e-9));
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.Solver.RecoveryLeavesConvergedComponentsUnchanged")
{
	int Components = 0;
	int Mismatches = 0;
	for (const NamedCase& C : AgreementCases())
	{
		CertificateOptions Off = C.Options;
		Off.bStallRecovery = false;
		const CertificateResult With = DecideWith(C.Model, C.Options);
		const CertificateResult Without = DecideWith(C.Model, Off);
		REQUIRE(With.Components.size() == Without.Components.size());
		for (std::size_t Index = 0; Index < With.Components.size(); ++Index)
		{
			const CertificateComponent& A = With.Components[Index];
			const CertificateComponent& B = Without.Components[Index];
			const bool bSame = A.State == B.State && A.Iterations == B.Iterations && A.LoadFactorLower == B.LoadFactorLower && A.LoadFactorUpper == B.LoadFactorUpper;
			INFO(std::setprecision(17) << C.Name << " component " << Index << ": recovery on " << A.State << " in " << A.Iterations << " [" << A.LoadFactorLower << ", " << A.LoadFactorUpper << "], off " << B.State << " in " << B.Iterations << " [" << B.LoadFactorLower << ", " << B.LoadFactorUpper << "]");
			CHECK(bSame);
			Mismatches += bSame ? 0 : 1;
			++Components;
		}
	}
	MESSAGE(Components << " components, " << Mismatches << " mismatches");
	CHECK(Components > 0);
	CHECK(Mismatches == 0);
}
