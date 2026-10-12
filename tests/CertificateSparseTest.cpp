#include "ArchFixtures.h"
#include "CatalogueTestSupport.h"
#include "CertificateTestSupport.h"
#include "FactorTestSupport.h"
#include "SlowInDebug.h"
#include "StaticAdmissibilityOracle.h"
#include "StaticsTestSupport.h"
#include "WedgeFixture.h"

#include "loadpath/Catalogue.h"
#include "loadpath/CollapseCertificate.h"

#include "doctest.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <iomanip>
#include <cstdlib>
#include <string>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
template <class O>
constexpr bool HasDenseRowLimit = requires(O X) { X.DenseRowLimit; };

template <class O>
O WithDenseRowLimit(O Options, int Limit)
{
	if constexpr (HasDenseRowLimit<O>)
	{
		Options.DenseRowLimit = Limit;
	}
	else
	{
		(void)Limit;
	}
	return Options;
}

void RequireTheRowLimitExists()
{
	CHECK_MESSAGE(HasDenseRowLimit<CertificateOptions>, "CertificateOptions has no DenseRowLimit, so both decisions took the dense path");
}

constexpr double LoadFactorRelative = 1.0e-7;
constexpr double LoadFactorAbsolute = 1.0e-9;
constexpr double MotionAbsolute = 1.0e-6;
constexpr int IterationSlack = 2;

struct SparseCase
{
	std::string Name;
	Structure Model;
	CertificateOptions Options;
};

struct SparseGaps
{
	double WorstLoadFactorRelative = 0.0;
	double WorstMotion = 0.0;
	int WorstIterations = 0;
	int Mismatches = 0;
	int Collapsing = 0;
	int Standing = 0;
	double SlowestSeconds = 0.0;
	std::string Slowest;
};

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

bool NearLoadFactor(double Got, double Want, SparseGaps& G)
{
	const double Gap = std::abs(Got - Want);
	const double Scale = std::max(std::abs(Got), std::abs(Want));
	if (Scale > 0.0)
	{
		G.WorstLoadFactorRelative = std::max(G.WorstLoadFactorRelative, Gap / Scale);
	}
	return Gap <= LoadFactorRelative * Scale + LoadFactorAbsolute;
}

double MotionGap(const MechanismMotion& A, const MechanismMotion& B)
{
	const Vec3& Ta = A.TranslationM;
	const Vec3& Tb = B.TranslationM;
	const Vec3& Ra = A.RotationRad;
	const Vec3& Rb = B.RotationRad;
	return std::max({std::abs(Ta.X - Tb.X), std::abs(Ta.Y - Tb.Y), std::abs(Ta.Z - Tb.Z), std::abs(Ra.X - Rb.X), std::abs(Ra.Y - Rb.Y), std::abs(Ra.Z - Rb.Z)});
}

void CompareSparseWithDense(const SparseCase& C, SparseGaps& G)
{
	const Clock::time_point Start = Clock::now();
	const CertificateResult Dense = DecideWith(C.Model, WithDenseRowLimit(C.Options, INT_MAX));
	const CertificateResult Sparse = DecideWith(C.Model, WithDenseRowLimit(C.Options, 0));
	INFO(C.Name << ": dense " << Dense.State << " [" << Dense.LoadFactorLower << ", " << Dense.LoadFactorUpper << "] in " << Dense.Iterations << ", sparse " << Sparse.State << " [" << Sparse.LoadFactorLower << ", " << Sparse.LoadFactorUpper << "] in " << Sparse.Iterations);
	if (Dense.State == CertificateState::Collapses)
	{
		++G.Collapsing;
	}
	if (Dense.State == CertificateState::Stands)
	{
		++G.Standing;
	}
	bool bAgree = Dense.State == Sparse.State;
	bAgree = NearLoadFactor(Sparse.LoadFactorLower, Dense.LoadFactorLower, G) && bAgree;
	bAgree = NearLoadFactor(Sparse.LoadFactorUpper, Dense.LoadFactorUpper, G) && bAgree;
	const int IterationGap = std::abs(Sparse.Iterations - Dense.Iterations);
	G.WorstIterations = std::max(G.WorstIterations, IterationGap);
	bAgree = IterationGap <= IterationSlack && bAgree;
	bAgree = SameInterfaces(Dense.HingingInterfaces, Sparse.HingingInterfaces) && bAgree;
	bAgree = Dense.Mechanism.size() == Sparse.Mechanism.size() && bAgree;
	for (const MechanismMotion& Want : Dense.Mechanism)
	{
		const MechanismMotion* Got = FindMotion(Sparse, Want.Body);
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
	const double Seconds = SecondsSince(Start);
	if (Seconds > G.SlowestSeconds)
	{
		G.SlowestSeconds = Seconds;
		G.Slowest = C.Name;
	}
}

std::vector<SparseCase> SparseCases()
{
	std::vector<SparseCase> Cases;
	const int Draws = oracle::bOptimisedBuild ? 60 : 12;
	for (int Draw = 0; Draw < Draws; ++Draw)
	{
		oracle::SeededDraw D = oracle::MakeSeededDraw(Draw);
		SparseCase C;
		C.Name = "seed " + std::to_string(D.Seed) + " " + D.Fixture;
		C.Model = std::move(D.Model);
		C.Options = CertificateUnderGravity(oracle::BiaxialGravity(D.LambdaX, D.LambdaY));
		Cases.push_back(std::move(C));
	}
	const std::vector<double> Lateral = oracle::bOptimisedBuild ? std::vector<double>{0.05, 0.3, 0.8} : std::vector<double>{0.05, 0.3};
	for (const double Lambda : Lateral)
	{
		SparseCase C;
		C.Name = "dry 6 x 5 wall at lateral " + std::to_string(Lambda);
		C.Model = RunningBondWall(DryJoint(0.6), DryJoint(0.6), 6, 5);
		C.Options = CertificateLateral(Lambda);
		Cases.push_back(std::move(C));
	}
	if (!oracle::bOptimisedBuild)
	{
		return Cases;
	}
	SparseCase Boundary;
	Boundary.Name = "dry 8 x 12 boundary wall at lateral 0.8";
	Boundary.Model = RunningBondWall(DryJoint(0.6), DryJoint(0.6), 8, 12);
	Boundary.Options = CertificateLateral(0.8);
	Cases.push_back(std::move(Boundary));
	SparseCase Ec6;
	Ec6.Name = "ec6 4 x 4 wall at lateral 0.4";
	Ec6.Model = Ec6Wall(4, 4);
	Ec6.Options = CertificateLateral(0.4);
	Cases.push_back(std::move(Ec6));
	SparseCase Large;
	Large.Name = "ec6 20 x 15 wall at rest";
	Large.Model = Ec6Wall(20, 15);
	Large.Options = CertificateOptions{};
	Cases.push_back(std::move(Large));
	return Cases;
}

constexpr double ArchFriction = 0.7;
constexpr double ArchLeastEta = 0.107478;
constexpr double ArchEtaTolerance = 1.0e-5;
constexpr double PileLambda = 0.5005;
constexpr double PileLambdaTolerance = 1.0e-3;
constexpr double WedgeToleranceDeg = 0.05;
constexpr double BoundaryWallLateral = 0.8;
constexpr int LargeWallCourses = 20;
constexpr int LargeWallBricks = 15;
constexpr int LargeWallRowLimit = 600;
constexpr double LargeWallRatioCeiling = 0.67;
}

TEST_CASE("Certificate.Sparse.AgreesWithTheDensePath")
{
	RequireTheRowLimitExists();
	const Clock::time_point Start = Clock::now();
	SparseGaps G;
	const std::vector<SparseCase> Cases = SparseCases();
	for (const SparseCase& C : Cases)
	{
		CompareSparseWithDense(C, G);
	}
	MESSAGE(Cases.size() << " cases (" << G.Collapsing << " collapse, " << G.Standing << " stand), " << G.Mismatches << " disagree; worst load factor relative " << G.WorstLoadFactorRelative << ", worst motion " << G.WorstMotion << ", worst iteration gap " << G.WorstIterations << ", slowest " << G.Slowest << " at " << G.SlowestSeconds << " s, " << SecondsSince(Start) << " s");
	CHECK(G.Collapsing > 0);
	CHECK(G.Standing > 0);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.Sparse.TheAnchorsReadTheSameOnTheSparsePath")
{
	RequireTheRowLimitExists();
	const Clock::time_point Start = Clock::now();
	const CertificateOptions Sparse = WithDenseRowLimit(CertificateOptions{}, 0);

	const CertificateResult ArchThick = DecideWith(MakeSemicircularArch(ArchLeastEta + ArchEtaTolerance, DryJoint(ArchFriction)).Model, Sparse);
	const CertificateResult ArchThin = DecideWith(MakeSemicircularArch(ArchLeastEta - ArchEtaTolerance, DryJoint(ArchFriction)).Model, Sparse);
	INFO("arch: eta + tol " << ArchThick.State << ", eta - tol " << ArchThin.State);
	CHECK(ArchThick.State == CertificateState::Stands);
	CHECK(ArchThin.State == CertificateState::Collapses);

	const HollowPile Pile = MakeHollowPile(0.7);
	const CertificateResult PileHolds = DecideWith(Pile.Model, WithDenseRowLimit(CertificateLateral(PileLambda - PileLambdaTolerance), 0));
	const CertificateResult PileFalls = DecideWith(Pile.Model, WithDenseRowLimit(CertificateLateral(PileLambda + PileLambdaTolerance), 0));
	INFO("pile: lambda - tol " << PileHolds.State << ", lambda + tol " << PileFalls.State);
	CHECK(PileHolds.State == CertificateState::Stands);
	CHECK(PileFalls.State == CertificateState::Collapses);

	CertificateOptions WedgeOptions = Sparse;
	WedgeOptions.ConeGenerators = 64;
	const double CriticalRad = AssociativeWedgeFrictionAngleRad(ItascaWedgeGeometry());
	const CertificateResult WedgeHolds = DecideWith(MakeItascaWedge(CriticalRad + WedgeToleranceDeg * RadPerDegree).Model, WedgeOptions);
	const CertificateResult WedgeSlides = DecideWith(MakeItascaWedge(CriticalRad - WedgeToleranceDeg * RadPerDegree).Model, WedgeOptions);
	INFO("wedge at " << CriticalRad / RadPerDegree << " deg: + tol " << WedgeHolds.State << ", - tol " << WedgeSlides.State);
	CHECK(WedgeHolds.State == CertificateState::Stands);
	CHECK(WedgeSlides.State == CertificateState::Collapses);
	MESSAGE("anchors on the sparse path in " << SecondsSince(Start) << " s");
}

TEST_CASE("Certificate.Sparse.ADenseFixtureKeepsItsPath")
{
	RequireTheRowLimitExists();
	const HollowPile Pile = MakeHollowPile(0.7);
	const CertificateOptions Options = CertificateLateral(PileLambda + PileLambdaTolerance);
	const CertificateResult Default = DecideWith(Pile.Model, Options);
	const CertificateResult Dense = DecideWith(Pile.Model, WithDenseRowLimit(Options, INT_MAX));
	INFO("default " << Default.State << " [" << Default.LoadFactorLower << ", " << Default.LoadFactorUpper << "] in " << Default.Iterations << ", dense " << Dense.State << " [" << Dense.LoadFactorLower << ", " << Dense.LoadFactorUpper << "] in " << Dense.Iterations);
	CHECK(Default.State == CertificateState::Collapses);
	CHECK(Default.State == Dense.State);
	CHECK(Default.LoadFactorLower == Dense.LoadFactorLower);
	CHECK(Default.LoadFactorUpper == Dense.LoadFactorUpper);
	CHECK(Default.Iterations == Dense.Iterations);
	CHECK(SameInterfaces(Default.HingingInterfaces, Dense.HingingInterfaces));
	CHECK(Default.Mechanism.size() == Dense.Mechanism.size());
	for (const MechanismMotion& Want : Dense.Mechanism)
	{
		const MechanismMotion* Got = FindMotion(Default, Want.Body);
		CHECK(Got != nullptr);
		if (Got == nullptr)
		{
			return;
		}
		CHECK(MotionGap(*Got, Want) == 0.0);
	}
}

#if defined(NDEBUG)
TEST_CASE("Slow.Certificate.Sparse.IsFasterThanDenseOnALargeComponent")
{
	RequireTheRowLimitExists();
	const Structure Model = Ec6Wall(LargeWallCourses, LargeWallBricks);
	const CertificateOptions Sparse = WithDenseRowLimit(CertificateOptions{}, LargeWallRowLimit);
	const CertificateOptions Dense = WithDenseRowLimit(CertificateOptions{}, INT_MAX);
	double DenseMin = 1.0e300;
	double SparseMin = 1.0e300;
	CertificateResult DenseResult;
	CertificateResult SparseResult;
	for (int Round = 0; Round < 2; ++Round)
	{
		Clock::time_point Start = Clock::now();
		DenseResult = DecideWith(Model, Dense);
		DenseMin = std::min(DenseMin, SecondsSince(Start));
		Start = Clock::now();
		SparseResult = DecideWith(Model, Sparse);
		SparseMin = std::min(SparseMin, SecondsSince(Start));
	}
	const double Ratio = SparseMin / DenseMin;
	MESSAGE(std::setprecision(10) << "ec6 " << LargeWallCourses << " x " << LargeWallBricks << " wall, " << Model.LiveBodyCount() << " live bodies: dense " << DenseResult.State << " [" << DenseResult.LoadFactorLower << ", " << DenseResult.LoadFactorUpper << "] in " << DenseResult.Iterations << " iterations, min " << DenseMin << " s; sparse " << SparseResult.State << " [" << SparseResult.LoadFactorLower << ", " << SparseResult.LoadFactorUpper << "] in " << SparseResult.Iterations << " iterations, min " << SparseMin << " s; ratio " << Ratio);
	SparseGaps G;
	CHECK(DenseResult.State == CertificateState::Stands);
	CHECK(SparseResult.State == DenseResult.State);
	CHECK(NearLoadFactor(SparseResult.LoadFactorLower, DenseResult.LoadFactorLower, G));
	CHECK(NearLoadFactor(SparseResult.LoadFactorUpper, DenseResult.LoadFactorUpper, G));
	CHECK(Ratio < LargeWallRatioCeiling);
}
#endif

TEST_CASE("Certificate.Sparse.TheRowLimitBoundaryIsInclusive")
{
	RequireTheRowLimitExists();
	const Structure Model = RunningBondWall(DryJoint(0.6), DryJoint(0.6), 8, 12);
	REQUIRE(Model.LiveBodyCount() == 101);
	const CertificateOptions Options = CertificateLateral(BoundaryWallLateral);
	const CertificateResult Default = DecideWith(Model, Options);
	const CertificateResult Dense = DecideWith(Model, WithDenseRowLimit(Options, INT_MAX));
	const CertificateResult Sparse = DecideWith(Model, WithDenseRowLimit(Options, 599));
	INFO(std::setprecision(17) << "default " << Default.State << " [" << Default.LoadFactorLower << ", " << Default.LoadFactorUpper << "] in " << Default.Iterations << ", dense " << Dense.State << " [" << Dense.LoadFactorLower << ", " << Dense.LoadFactorUpper << "] in " << Dense.Iterations << ", limit 599 " << Sparse.State << " [" << Sparse.LoadFactorLower << ", " << Sparse.LoadFactorUpper << "] in " << Sparse.Iterations);
	CHECK(Dense.State == CertificateState::Collapses);
	CHECK(Default.State == Dense.State);
	CHECK(Default.LoadFactorLower == Dense.LoadFactorLower);
	CHECK(Default.LoadFactorUpper == Dense.LoadFactorUpper);
	CHECK(Default.Iterations == Dense.Iterations);
	CHECK(SameInterfaces(Default.HingingInterfaces, Dense.HingingInterfaces));
	CHECK(Default.Mechanism.size() == Dense.Mechanism.size());
	for (const MechanismMotion& Want : Dense.Mechanism)
	{
		const MechanismMotion* Got = FindMotion(Default, Want.Body);
		REQUIRE(Got != nullptr);
		CHECK(MotionGap(*Got, Want) == 0.0);
	}
	SparseGaps G;
	CHECK(Sparse.State == Dense.State);
	CHECK(NearLoadFactor(Sparse.LoadFactorLower, Dense.LoadFactorLower, G));
	CHECK(NearLoadFactor(Sparse.LoadFactorUpper, Dense.LoadFactorUpper, G));
	const bool bPathDiffers = Sparse.LoadFactorLower != Dense.LoadFactorLower || Sparse.LoadFactorUpper != Dense.LoadFactorUpper || Sparse.Iterations != Dense.Iterations;
	CHECK_MESSAGE(bPathDiffers, "limit 599 read bit-identical to the dense path, so it cannot show which path the default took");
}

TEST_CASE("Certificate.Sparse.ARankDeficientNormalMatrixIsRegularisedOnTheSparsePath")
{
	RequireTheRowLimitExists();
	const Structure Model = RunningBondWall(DryJoint(0.0), DryJoint(0.0), 9, 12);
	REQUIRE(Model.LiveBodyCount() == 113);
	const CertificateOptions Options = CertificateLateral(0.0);
	const CertificateResult Sparse = DecideWith(Model, Options);
	const CertificateResult Dense = DecideWith(Model, WithDenseRowLimit(Options, INT_MAX));
	INFO("sparse " << Sparse.State << " [" << Sparse.LoadFactorLower << ", " << Sparse.LoadFactorUpper << "] in " << Sparse.Iterations << ", dense " << Dense.State << " [" << Dense.LoadFactorLower << ", " << Dense.LoadFactorUpper << "] in " << Dense.Iterations);
	SparseGaps G;
	CHECK(Dense.State == CertificateState::Stands);
	CHECK(Sparse.State == Dense.State);
	CHECK(NearLoadFactor(Sparse.LoadFactorLower, Dense.LoadFactorLower, G));
	CHECK(NearLoadFactor(Sparse.LoadFactorUpper, Dense.LoadFactorUpper, G));
	CHECK(std::abs(Sparse.Iterations - Dense.Iterations) <= IterationSlack);
}
