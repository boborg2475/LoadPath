#include "AnchorFixtures.h"
#include "ArchFixtures.h"
#include "BrickFixtures.h"
#include "CertificateTestSupport.h"
#include "FactorTestSupport.h"
#include "SlowInDebug.h"
#include "StaticAdmissibilityOracle.h"
#include "StaticsTestSupport.h"
#include "WedgeFixture.h"

#include "loadpath/CollapseCertificate.h"
#include "loadpath/geometry/RunningBondWall.h"

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <string>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
template <class O>
constexpr bool HasCoupling = requires(O X) {
	X.Coupling = O::CouplingKind::InFactor;
	X.Coupling = O::CouplingKind::Bordered;
};

enum class Coupling
{
	InFactor,
	Bordered
};

template <class O>
O WithCoupling(O Options, Coupling Kind)
{
	if constexpr (HasCoupling<O>)
	{
		Options.Coupling = Kind == Coupling::Bordered ? O::CouplingKind::Bordered : O::CouplingKind::InFactor;
	}
	else
	{
		(void)Kind;
	}
	return Options;
}

template <class O>
void CheckCouplingDefault()
{
	if constexpr (HasCoupling<O>)
	{
		CHECK(O{}.Coupling == O::CouplingKind::Bordered);
	}
}

void RequireTheCouplingExists()
{
	CHECK_MESSAGE(HasCoupling<CertificateOptions>, "CertificateOptions has no Coupling, so both decisions factored the coupling in the normal matrix");
	CheckCouplingDefault<CertificateOptions>();
}

CertificateOptions Sparse(CertificateOptions Options, Coupling Kind)
{
	Options.DenseRowLimit = 0;
	return WithCoupling(Options, Kind);
}

constexpr double AgreementRelative = 1.0e-9;
constexpr double AgreementAbsolute = 1.0e-12;
constexpr double MotionAbsolute = 1.0e-9;
constexpr double PinnedRelative = 1.0e-10;
constexpr double PinnedAbsolute = 1.0e-10;
constexpr int SingularIterations = 9;
constexpr int SingularIterationSlack = 2;
constexpr double SingularLoadFactor = 1000.0;
constexpr int LargeWallCourses = 20;
constexpr int LargeWallBricks = 15;
constexpr std::size_t LargeWallBodies = 310;
constexpr double LargeWallRatioCeiling = 1.0;

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

struct CouplingGaps
{
	double WorstLoadFactorRelative = 0.0;
	double WorstMotion = 0.0;
	int WorstIterations = 0;
	int Mismatches = 0;
};

bool Near(double Got, double Want, double Relative, double Absolute)
{
	return std::abs(Got - Want) <= Relative * std::max(std::abs(Got), std::abs(Want)) + Absolute;
}

bool NearTracked(double Got, double Want, CouplingGaps& G)
{
	const double Scale = std::max(std::abs(Got), std::abs(Want));
	if (Scale > 0.0)
	{
		G.WorstLoadFactorRelative = std::max(G.WorstLoadFactorRelative, std::abs(Got - Want) / Scale);
	}
	return Near(Got, Want, AgreementRelative, AgreementAbsolute);
}

bool SameInterfaces(const std::vector<InterfaceId>& A, const std::vector<InterfaceId>& B)
{
	if (A.size() != B.size())
	{
		return false;
	}
	for (const InterfaceId& Id : A)
	{
		if (!HasInterface(B, Id))
		{
			return false;
		}
	}
	return true;
}

double MotionGap(const MechanismMotion& A, const MechanismMotion& B)
{
	const Vec3& Ta = A.TranslationM;
	const Vec3& Tb = B.TranslationM;
	const Vec3& Ra = A.RotationRad;
	const Vec3& Rb = B.RotationRad;
	return std::max({std::abs(Ta.X - Tb.X), std::abs(Ta.Y - Tb.Y), std::abs(Ta.Z - Tb.Z), std::abs(Ra.X - Rb.X), std::abs(Ra.Y - Rb.Y), std::abs(Ra.Z - Rb.Z)});
}

void CompareCouplings(const NamedCase& C, CouplingGaps& G)
{
	const CertificateResult InFactor = DecideWith(C.Model, Sparse(C.Options, Coupling::InFactor));
	const CertificateResult Bordered = DecideWith(C.Model, Sparse(C.Options, Coupling::Bordered));
	INFO(std::setprecision(17) << C.Name << ": in factor " << InFactor.State << " [" << InFactor.LoadFactorLower << ", " << InFactor.LoadFactorUpper << "] in " << InFactor.Iterations << ", bordered " << Bordered.State << " [" << Bordered.LoadFactorLower << ", " << Bordered.LoadFactorUpper << "] in " << Bordered.Iterations);
	bool bAgree = InFactor.State == Bordered.State;
	bAgree = NearTracked(Bordered.LoadFactorLower, InFactor.LoadFactorLower, G) && bAgree;
	bAgree = NearTracked(Bordered.LoadFactorUpper, InFactor.LoadFactorUpper, G) && bAgree;
	G.WorstIterations = std::max(G.WorstIterations, std::abs(Bordered.Iterations - InFactor.Iterations));
	bAgree = Bordered.Iterations == InFactor.Iterations && bAgree;
	bAgree = SameInterfaces(InFactor.HingingInterfaces, Bordered.HingingInterfaces) && bAgree;
	bAgree = InFactor.Mechanism.size() == Bordered.Mechanism.size() && bAgree;
	for (const MechanismMotion& Want : InFactor.Mechanism)
	{
		const MechanismMotion* Got = FindMotion(Bordered, Want.Body);
		if (Got == nullptr)
		{
			bAgree = false;
			continue;
		}
		const double Gap = MotionGap(*Got, Want);
		G.WorstMotion = std::max(G.WorstMotion, Gap);
		bAgree = Gap <= MotionAbsolute && bAgree;
	}
	CHECK(bAgree);
	G.Mismatches += bAgree ? 0 : 1;
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
	Cases.push_back({"dry 8 x 12 boundary wall at lateral 0.8", RunningBondWall(DryJoint(0.6), DryJoint(0.6), 8, 12), CertificateLateral(0.8)});
	if (oracle::bOptimisedBuild)
	{
		Cases.push_back({"ec6 4 x 4 wall at lateral 0.4", Ec6Wall(4, 4), CertificateLateral(0.4)});
		Cases.push_back({"ec6 20 x 15 wall at rest", Ec6Wall(LargeWallCourses, LargeWallBricks), CertificateOptions{}});
	}
	return Cases;
}

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
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.Coupling.BorderedAgreesWithTheCouplingInTheFactor")
{
	RequireTheCouplingExists();
	const Clock::time_point Start = Clock::now();
	CouplingGaps G;
	const std::vector<NamedCase> Cases = AgreementCases();
	for (const NamedCase& C : Cases)
	{
		CompareCouplings(C, G);
	}
	MESSAGE(Cases.size() << " cases, " << G.Mismatches << " disagree; worst load factor relative " << G.WorstLoadFactorRelative << ", worst motion " << G.WorstMotion << ", worst iteration gap " << G.WorstIterations << ", " << SecondsSince(Start) << " s");
	CHECK(G.Mismatches == 0);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.Coupling.ThePinnedFixturesReadTheSame")
{
	RequireTheCouplingExists();
	int Mismatches = 0;
	for (const PinnedCase& C : PinnedCases())
	{
		const CertificateResult Got = DecideWith(C.Model, Sparse(C.Options, Coupling::Bordered));
		const bool bSame = Got.State == C.State && Got.Iterations == C.Iterations && Near(Got.LoadFactorLower, C.LoadFactorLower, PinnedRelative, PinnedAbsolute) && Near(Got.LoadFactorUpper, C.LoadFactorUpper, PinnedRelative, PinnedAbsolute);
		INFO(std::setprecision(17) << C.Name << ": got " << Got.State << " in " << Got.Iterations << " [" << Got.LoadFactorLower << ", " << Got.LoadFactorUpper << "], pinned " << C.State << " in " << C.Iterations << " [" << C.LoadFactorLower << ", " << C.LoadFactorUpper << "]");
		CHECK(bSame);
		Mismatches += bSame ? 0 : 1;
	}
	CHECK(Mismatches == 0);
}

TEST_CASE("Certificate.Coupling.ASingularSiteBlockIsRegularisedOnTheBorderedPath")
{
	RequireTheCouplingExists();
	const Structure Model = RunningBondWall(DryJoint(0.0), DryJoint(0.0), 9, 12);
	REQUIRE(Model.LiveBodyCount() == 113);
	const CertificateResult Got = DecideWith(Model, Sparse(CertificateLateral(0.0), Coupling::Bordered));
	INFO(std::setprecision(17) << "frictionless 9 x 12 wall, bordered: " << Got.State << " [" << Got.LoadFactorLower << ", " << Got.LoadFactorUpper << "] in " << Got.Iterations);
	CHECK(Got.State == CertificateState::Stands);
	CHECK(Near(Got.LoadFactorLower, SingularLoadFactor, AgreementRelative, AgreementAbsolute));
	CHECK(Near(Got.LoadFactorUpper, SingularLoadFactor, AgreementRelative, AgreementAbsolute));
	CHECK(std::abs(Got.Iterations - SingularIterations) <= SingularIterationSlack);
}

#if defined(NDEBUG)
TEST_CASE("Slow.Certificate.Coupling.IsFasterThanTheCouplingInTheFactorOnALargeComponent")
{
	RequireTheCouplingExists();
	const Structure Model = Ec6Wall(LargeWallCourses, LargeWallBricks);
	REQUIRE(Model.Components().FreeBodies.size() == 1u);
	REQUIRE(Model.Components().FreeBodies[0].size() == LargeWallBodies);
	const CertificateOptions InFactorOptions = Sparse(CertificateOptions{}, Coupling::InFactor);
	const CertificateOptions BorderedOptions = Sparse(CertificateOptions{}, Coupling::Bordered);
	double InFactorMin = 1.0e300;
	double BorderedMin = 1.0e300;
	CertificateResult InFactor;
	CertificateResult Bordered;
	for (int Round = 0; Round < 2; ++Round)
	{
		Clock::time_point Start = Clock::now();
		InFactor = DecideWith(Model, InFactorOptions);
		InFactorMin = std::min(InFactorMin, SecondsSince(Start));
		Start = Clock::now();
		Bordered = DecideWith(Model, BorderedOptions);
		BorderedMin = std::min(BorderedMin, SecondsSince(Start));
	}
	const double Ratio = BorderedMin / InFactorMin;
	MESSAGE(std::setprecision(10) << "ec6 20 x 15 wall: in factor " << InFactor.State << " [" << InFactor.LoadFactorLower << ", " << InFactor.LoadFactorUpper << "] in " << InFactor.Iterations << " iterations, min " << InFactorMin << " s; bordered " << Bordered.State << " [" << Bordered.LoadFactorLower << ", " << Bordered.LoadFactorUpper << "] in " << Bordered.Iterations << " iterations, min " << BorderedMin << " s; ratio " << Ratio);
	CHECK(InFactor.State == CertificateState::Stands);
	CHECK(Bordered.State == InFactor.State);
	CHECK(Near(Bordered.LoadFactorLower, InFactor.LoadFactorLower, AgreementRelative, AgreementAbsolute));
	CHECK(Near(Bordered.LoadFactorUpper, InFactor.LoadFactorUpper, AgreementRelative, AgreementAbsolute));
	CHECK(Ratio < LargeWallRatioCeiling);
}
#endif
