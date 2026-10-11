#include "AnchorFixtures.h"
#include "BrickFixtures.h"
#include "CertificateTestSupport.h"
#include "StaticAdmissibilityOracle.h"
#include "StaticsTestSupport.h"

#include "doctest.h"

#include <cmath>
#include <cstddef>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double ThresholdMargin = 1.0e-6;
constexpr double LoadFactorRelative = 1.0e-7;
constexpr double LoadFactorAbsolute = 1.0e-6;
constexpr double ShapeTolerance = 1.0e-6;
const Vec3 PlusX{1.0, 0.0, 0.0};
const Vec3 PlusY{0.0, 1.0, 0.0};
const Vec3 PlusZ{0.0, 0.0, 1.0};

double BrickWeightN()
{
	return BrickMassKg * GravityMPerS2;
}

double BedAreaM2()
{
	return BrickLengthM * BrickWidthM;
}

StrengthProfile Bonded(double TensileStrengthPa, double ShearCohesionPa, double Friction)
{
	StrengthProfile Profile = DryJoint(Friction);
	Profile.TensileStrengthPa = TensileStrengthPa;
	Profile.ShearCohesionPa = ShearCohesionPa;
	return Profile;
}

void CheckLoadFactorsAt(const CertificateResult& Result, double Expected)
{
	INFO("load factors [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "], expected " << Expected);
	CHECK(std::abs(Result.LoadFactorLower - Expected) <= LoadFactorRelative * Expected + LoadFactorAbsolute);
	CHECK(std::abs(Result.LoadFactorUpper - Expected) <= LoadFactorRelative * Expected + LoadFactorAbsolute);
}

void CheckLoadFactorsBracket(const CertificateResult& Result, double Expected)
{
	INFO("load factors [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "], expected inside " << Expected);
	CHECK(Result.LoadFactorLower <= Expected * (1.0 + LoadFactorRelative) + LoadFactorAbsolute);
	CHECK(Result.LoadFactorUpper >= Expected * (1.0 - LoadFactorRelative) - LoadFactorAbsolute);
}

void CheckOnlyBodyAndInterface(const CertificateResult& Result, BodyId Body, InterfaceId Bed)
{
	INFO("moving " << Describe(Result.UnconstrainedBodies) << ", hinging " << Describe(Result.HingingInterfaces));
	CHECK(Result.UnconstrainedBodies.size() == 1u);
	CHECK(HasBody(Result.UnconstrainedBodies, Body));
	CHECK(Result.HingingInterfaces.size() == 1u);
	CHECK(HasInterface(Result.HingingInterfaces, Bed));
}

void CheckBracketsWithin(const ThresholdBracket& Bracket, double Expected, double Tolerance)
{
	INFO("last standing " << Bracket.LastStanding << ", first failing " << Bracket.FirstFailing << ", expected " << Expected);
	CHECK(Bracket.bBracketed);
	CHECK(std::abs(Bracket.LastStanding - Expected) <= Tolerance);
	CHECK(std::abs(Bracket.FirstFailing - Expected) <= Tolerance);
}

ThresholdBracket CertificateTiltThreshold(const Structure& Model)
{
	return Threshold([&](double TiltRad) { return CertificateStands(Model, CertificateTilt(TiltRad)); }, 0.0, Pi / 4.0, 1.0e-8);
}

bool RectangleHoldsTheResultant(double LambdaX, double LambdaY)
{
	return std::abs(LambdaX) * BrickHeightM <= BrickLengthM && std::abs(LambdaY) * BrickHeightM <= BrickWidthM;
}
}

TEST_CASE("Certificate.Anchor.BrickOnFlatGroundStandsWithTheCappedLoadFactor")
{
	CHECK(DryJoint(0.6).CompressiveStrengthPa * BedAreaM2() / BrickWeightN() > CertificateOptions{}.LoadFactorCap);
	for (const double Friction : {0.6, 5.0})
	{
		for (const double Cap : {CertificateOptions{}.LoadFactorCap, 50.0})
		{
			INFO("friction " << Friction << ", cap " << Cap);
			const BrickOnGround F = MakeBrickOnGround(DryJoint(Friction));
			CertificateOptions Options;
			Options.LoadFactorCap = Cap;
			const CertificateResult Result = DecideWith(F.Model, Options);
			INFO("state " << Result.State << ", load factors [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "]");
			CHECK(Result.State == CertificateState::Stands);
			CHECK(Result.LoadFactorLower >= Cap * (1.0 - Options.FeasibilityTolerance));
			CHECK(Result.LoadFactorUpper <= Cap * (1.0 + Options.FeasibilityTolerance));
			CHECK(Result.LoadFactorLower <= Result.LoadFactorUpper);
		}
	}
}

TEST_CASE("Certificate.Anchor.SingleBrickLoadFactorsMatchTheRigidBlockClosedForm")
{
	struct Case
	{
		double Friction;
		double LambdaX;
		double LambdaY;
	};
	const Case Cases[] = {
		{0.6, 0.3, 0.0}, {0.6, 0.55, 0.0}, {0.6, 0.65, 0.0}, {0.6, 1.0, 0.0}, {0.6, 0.3, 0.4}, {0.6, 0.5, 0.5},
		{5.0, 1.0, 0.0}, {5.0, 3.0, 0.0}, {5.0, 3.5, 0.0}, {5.0, 4.5, 0.0}, {5.0, 2.5, 0.25}, {5.0, 3.0, 0.875}, {5.0, 2.0, 1.75}, {5.0, 3.375, 0.375},
	};
	const double Ratio = CircumscribedOverInscribed(16);
	const double Cap = CertificateOptions{}.LoadFactorCap;
	int StandingRows = 0;
	int CollapsingRows = 0;
	for (const Case& Each : Cases)
	{
		INFO("friction " << Each.Friction << ", lambda (" << Each.LambdaX << ", " << Each.LambdaY << ")");
		const double Magnitude = std::hypot(Each.LambdaX, Each.LambdaY);
		const bool bInscribedStands = RectangleHoldsTheResultant(Each.LambdaX, Each.LambdaY) && Magnitude <= Each.Friction / Ratio;
		const bool bCircumscribedFails = !RectangleHoldsTheResultant(Each.LambdaX, Each.LambdaY) || Magnitude > Each.Friction * Ratio;
		CHECK(bInscribedStands != bCircumscribedFails);
		const double ExpectedLoadFactor = bInscribedStands ? Cap : 0.0;
		StandingRows += bInscribedStands ? 1 : 0;
		CollapsingRows += bCircumscribedFails ? 1 : 0;
		const BrickOnGround F = MakeBrickOnGround(DryJoint(Each.Friction));
		const CertificateResult Result = DecideWith(F.Model, CertificateLateral(Each.LambdaX, Each.LambdaY));
		INFO("state " << Result.State);
		CHECK(Result.State == (bInscribedStands ? CertificateState::Stands : CertificateState::Collapses));
		CheckLoadFactorsAt(Result, ExpectedLoadFactor);
		CHECK(Result.LoadFactorUpper - Result.LoadFactorLower <= ExpectedLoadFactor * (Ratio - 1.0) + LoadFactorAbsolute);
	}
	CHECK(StandingRows > 0);
	CHECK(CollapsingRows > 0);
}

TEST_CASE("Certificate.Anchor.SingleBrickTipsAtItsLengthAndWidthOverHeight")
{
	const double Friction = 5.0;
	struct Case
	{
		const char* Name;
		Vec3 Direction;
		double ThresholdLambda;
	};
	const Case Cases[] = {{"across the length", PlusX, BrickLengthM / BrickHeightM}, {"across the width", PlusY, BrickWidthM / BrickHeightM}};
	const BrickOnGround F = MakeBrickOnGround(DryJoint(Friction));
	for (const Case& Each : Cases)
	{
		INFO("case: " << doctest::String(Each.Name) << ", threshold " << Each.ThresholdLambda);
		CHECK(Each.ThresholdLambda * (1.0 + ThresholdMargin) < Friction / CircumscribedOverInscribed(16));
		const double Below = Each.ThresholdLambda * (1.0 - ThresholdMargin);
		const double Above = Each.ThresholdLambda * (1.0 + ThresholdMargin);
		const CertificateResult Standing = DecideWith(F.Model, CertificateLateral(Below * Each.Direction.X, Below * Each.Direction.Y));
		const CertificateResult Tipping = DecideWith(F.Model, CertificateLateral(Above * Each.Direction.X, Above * Each.Direction.Y));
		INFO("below reads " << Standing.State << ", above reads " << Tipping.State);
		CHECK(Standing.State == CertificateState::Stands);
		CHECK(Tipping.State == CertificateState::Collapses);
		CheckOnlyBodyAndInterface(Tipping, F.Brick, F.Bed);
	}
}

TEST_CASE("Certificate.Anchor.SingleBrickSlidesAtFrictionWithinThePolygonBand")
{
	const double Friction = 0.6;
	struct Case
	{
		int Generators;
		double AngleRad;
		double Magnitude;
		CertificateState Expected;
	};
	const double Rho16 = CircumscribedOverInscribed(16);
	const double Rho32 = CircumscribedOverInscribed(32);
	const double Below = 1.0 - ThresholdMargin;
	const double Above = 1.0 + ThresholdMargin;
	const double MidEdge16 = Pi / 16.0;
	const Case Cases[] = {
		{16, 0.0, Friction * Below, CertificateState::Stands},
		{16, 0.0, Friction * Above, CertificateState::Undecided},
		{16, 0.0, Friction * Rho16 * Below, CertificateState::Undecided},
		{16, 0.0, Friction * Rho16 * Above, CertificateState::Collapses},
		{16, MidEdge16, Friction / Rho16 * Below, CertificateState::Stands},
		{16, MidEdge16, Friction / Rho16 * Above, CertificateState::Undecided},
		{16, MidEdge16, Friction * Below, CertificateState::Undecided},
		{16, MidEdge16, Friction * Above, CertificateState::Collapses},
		{32, 0.0, Friction * Below, CertificateState::Stands},
		{32, 0.0, Friction * Above, CertificateState::Undecided},
		{32, 0.0, Friction * Rho32 * Below, CertificateState::Undecided},
		{32, 0.0, Friction * Rho32 * Above, CertificateState::Collapses},
	};
	const BrickOnGround F = MakeBrickOnGround(DryJoint(Friction));
	for (const Case& Each : Cases)
	{
		const double LambdaX = Each.Magnitude * std::cos(Each.AngleRad);
		const double LambdaY = Each.Magnitude * std::sin(Each.AngleRad);
		INFO(Each.Generators << " generators, load angle " << Each.AngleRad << " rad, lambda " << Each.Magnitude << " = (" << LambdaX << ", " << LambdaY << ")");
		CHECK(RectangleHoldsTheResultant(LambdaX, LambdaY));
		CertificateOptions Options = CertificateLateral(LambdaX, LambdaY);
		Options.ConeGenerators = Each.Generators;
		const CertificateResult Result = DecideWith(F.Model, Options);
		INFO("state " << Result.State << ", load factors [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "]");
		CHECK(Result.State == Each.Expected);
		if (Each.Expected == CertificateState::Collapses)
		{
			CheckOnlyBodyAndInterface(Result, F.Brick, F.Bed);
		}
	}
}

TEST_CASE("Certificate.Anchor.TensileBondCarriesTheBrickPastTheTippingTilt")
{
	const double TiltRad = 80.0 * RadPerDegree;
	const double Friction = 0.7;
	const double CohesionPa = 1.0e5;
	const double WeightN = BrickWeightN();
	const double UpliftShareOfWeight = (BrickHeightM / BrickLengthM) * std::sin(TiltRad) - std::cos(TiltRad);
	CHECK(std::tan(TiltRad) > BrickLengthM / BrickHeightM);
	CHECK(UpliftShareOfWeight > 0.0);
	CHECK(CohesionPa * BedAreaM2() / CircumscribedOverInscribed(16) > 4.0 * WeightN * std::sin(TiltRad));
	CHECK(DryJoint(Friction).CompressiveStrengthPa * BedAreaM2() / 4.0 > 4.0 * WeightN);
	const Box Shape = BrickShape();
	const Vec3 DownhillEdgeM{BrickLengthM / 2.0, 0.0, 0.0};
	for (const double LoadFactor : {2.0, 0.5})
	{
		INFO("load factor " << LoadFactor);
		const double TensileStrengthPa = LoadFactor * WeightN * UpliftShareOfWeight / BedAreaM2();
		const BrickOnGround F = MakeBrickOnGround(Bonded(TensileStrengthPa, CohesionPa, Friction));
		const CertificateResult Result = DecideWith(F.Model, CertificateTilt(TiltRad));
		INFO("ft " << TensileStrengthPa << " Pa: state " << Result.State);
		CHECK(Result.State == (LoadFactor >= 1.0 ? CertificateState::Stands : CertificateState::Collapses));
		CheckLoadFactorsAt(Result, LoadFactor);
		if (LoadFactor >= 1.0)
		{
			continue;
		}
		CheckOnlyBodyAndInterface(Result, F.Brick, F.Bed);
		const MechanismMotion* Motion = FindMotion(Result, F.Brick);
		CHECK(Motion != nullptr);
		if (Motion != nullptr)
		{
			CheckRotationAboutEdge(*Motion, Shape.CentreM, DownhillEdgeM, PlusY, PlusX, ShapeTolerance);
		}
	}

	const BrickOnGround Unbonded = MakeBrickOnGround(Bonded(0.0, CohesionPa, Friction));
	const CertificateResult Result = DecideWith(Unbonded.Model, CertificateTilt(TiltRad));
	INFO("ft 0: state " << Result.State);
	CHECK(Result.State == CertificateState::Collapses);
	CheckLoadFactorsAt(Result, 0.0);
}

TEST_CASE("Certificate.Anchor.CohesionCarriesTheBedPastTheFrictionTilt")
{
	const double TiltRad = 40.0 * RadPerDegree;
	const double Friction = 0.3;
	const double Rho = CircumscribedOverInscribed(16);
	const double S = std::sin(TiltRad);
	const double C = std::cos(TiltRad);
	CHECK(std::tan(TiltRad) > Friction * Rho);
	CHECK(C - (BrickHeightM / BrickLengthM) * S > 0.0);
	const double StandingFactor = 2.0;
	const double CollapsingFactor = 0.5;
	CHECK(StandingFactor / Rho * (S - Friction * C) / (S - Friction / Rho * C) > 1.0);
	CHECK(CollapsingFactor * Rho * (S - Friction * C) / (S - Friction * Rho * C) < 1.0);
	for (const double LoadFactor : {StandingFactor, CollapsingFactor})
	{
		INFO("load factor " << LoadFactor);
		const double CohesionPa = LoadFactor * BrickWeightN() * (S - Friction * C) / BedAreaM2();
		const BrickOnGround F = MakeBrickOnGround(Bonded(0.0, CohesionPa, Friction));
		const CertificateResult Result = DecideWith(F.Model, CertificateTilt(TiltRad));
		INFO("c " << CohesionPa << " Pa: state " << Result.State);
		CHECK(Result.State == (LoadFactor >= 1.0 ? CertificateState::Stands : CertificateState::Collapses));
		CheckLoadFactorsBracket(Result, LoadFactor);
		if (LoadFactor >= 1.0)
		{
			continue;
		}
		CheckOnlyBodyAndInterface(Result, F.Brick, F.Bed);
		const MechanismMotion* Motion = FindMotion(Result, F.Brick);
		CHECK(Motion != nullptr);
		if (Motion != nullptr)
		{
			CheckAssociativeSlideAlong(*Motion, PlusX, PlusZ, ShapeTolerance);
		}
	}

	const BrickOnGround Dry = MakeBrickOnGround(Bonded(0.0, 0.0, Friction));
	const CertificateResult Result = DecideWith(Dry.Model, CertificateTilt(TiltRad));
	INFO("c 0: state " << Result.State);
	CHECK(Result.State == CertificateState::Collapses);
	CheckLoadFactorsAt(Result, 0.0);
}

TEST_CASE("Certificate.Anchor.CompressiveStrengthBoundsTheLoadFactorOnFlatGround")
{
	for (const double LoadFactor : {0.5, 2.0})
	{
		INFO("load factor " << LoadFactor);
		StrengthProfile Profile = DryJoint(0.7);
		Profile.CompressiveStrengthPa = LoadFactor * BrickWeightN() / BedAreaM2();
		const BrickOnGround F = MakeBrickOnGround(Profile);
		const CertificateResult Result = DecideWith(F.Model, CertificateOptions{});
		INFO("fc " << Profile.CompressiveStrengthPa << " Pa: state " << Result.State);
		CHECK(Result.State == (LoadFactor >= 1.0 ? CertificateState::Stands : CertificateState::Collapses));
		CheckLoadFactorsAt(Result, LoadFactor);
	}
}

TEST_CASE("Certificate.Anchor.InclineTipsAtAtanWidthOverHeight")
{
	const double WidthM = 0.10;
	const double HeightM = 0.15;
	const double DepthM = 0.10;
	const double Friction = 0.7;
	CHECK(Friction / CircumscribedOverInscribed(16) > WidthM / HeightM);
	const BlockOnGround F = MakeBlockOnGround(WidthM, DepthM, HeightM, Friction);
	const double ExpectedRad = std::atan(WidthM / HeightM);
	CheckBracketsWithin(CertificateTiltThreshold(F.Model), ExpectedRad, 1.0e-6);
	CHECK(DecideWith(F.Model, CertificateTilt(ExpectedRad - ThresholdMargin)).State == CertificateState::Stands);
	CHECK(DecideWith(F.Model, CertificateTilt(ExpectedRad + ThresholdMargin)).State == CertificateState::Collapses);
}

TEST_CASE("Certificate.Anchor.InclineSlidesAtAtanFrictionAndCollapsesPastThePolygonBand")
{
	const double WidthM = 0.10;
	const double HeightM = 0.15;
	const double DepthM = 0.10;
	const double Friction = 0.6;
	const double Rho = CircumscribedOverInscribed(16);
	CHECK(Friction * Rho < WidthM / HeightM);
	const BlockOnGround F = MakeBlockOnGround(WidthM, DepthM, HeightM, Friction);
	const double InscribedRad = std::atan(Friction);
	const double CircumscribedRad = std::atan(Friction * Rho);
	CheckBracketsWithin(CertificateTiltThreshold(F.Model), InscribedRad, 1.0e-6);
	CHECK(DecideWith(F.Model, CertificateTilt(InscribedRad - ThresholdMargin)).State == CertificateState::Stands);
	CHECK(DecideWith(F.Model, CertificateTilt(InscribedRad + ThresholdMargin)).State == CertificateState::Undecided);
	CHECK(DecideWith(F.Model, CertificateTilt(CircumscribedRad - ThresholdMargin)).State == CertificateState::Undecided);
	const CertificateResult Sliding = DecideWith(F.Model, CertificateTilt(CircumscribedRad + ThresholdMargin));
	CHECK(Sliding.State == CertificateState::Collapses);
	CheckOnlyBodyAndInterface(Sliding, F.Block, F.Bed);
}

TEST_CASE("Certificate.Anchor.HousnerBlockStandsBelowHalfWidthOverHalfHeightAndCollapsesAbove")
{
	struct HousnerCase
	{
		double WidthM;
		double HeightM;
	};
	const HousnerCase Cases[] = {{0.25, 1.0}, {0.17, 1.0}, {0.12, 1.0}};
	const double DepthM = 0.25;
	const double Friction = 0.7;
	for (const HousnerCase& Case : Cases)
	{
		const double TanAlpha = (Case.WidthM / 2.0) / (Case.HeightM / 2.0);
		INFO("2b = " << Case.WidthM << ", 2h = " << Case.HeightM << ", tan alpha = " << TanAlpha);
		CHECK(Friction / CircumscribedOverInscribed(16) > TanAlpha);
		const BlockOnGround F = MakeBlockOnGround(Case.WidthM, DepthM, Case.HeightM, Friction);
		CHECK(DecideWith(F.Model, CertificateLateral(TanAlpha * (1.0 - ThresholdMargin))).State == CertificateState::Stands);
		CHECK(DecideWith(F.Model, CertificateLateral(TanAlpha * (1.0 + ThresholdMargin))).State == CertificateState::Collapses);
	}
}

TEST_CASE("Certificate.Anchor.FreeWallCollapsesAtThicknessOverHeight")
{
	const double ThicknessM = BrickWidthM;
	const double HeightM = 10.0 * BrickHeightM;
	const double LengthM = 1.0;
	const double Friction = 0.7;
	const double Lambda = ThicknessM / HeightM;
	CHECK(Friction / CircumscribedOverInscribed(16) > Lambda);
	const BlockOnGround F = MakeBlockOnGround(ThicknessM, LengthM, HeightM, Friction, BrickDensityKgPerM3());
	CHECK(DecideWith(F.Model, CertificateLateral(Lambda * (1.0 - ThresholdMargin))).State == CertificateState::Stands);
	const CertificateResult Failing = DecideWith(F.Model, CertificateLateral(Lambda * (1.0 + ThresholdMargin)));
	CHECK(Failing.State == CertificateState::Collapses);
	CheckOnlyBodyAndInterface(Failing, F.Block, F.Bed);
}

TEST_CASE("Certificate.Anchor.CoursedFreeWallHingesAtTheBase")
{
	const int CourseCount = 10;
	const double ThicknessM = BrickWidthM;
	const double HeightM = CourseCount * BrickHeightM;
	const double LengthM = 1.0;
	const double Friction = 0.7;
	const double Lambda = ThicknessM / HeightM;
	const CoursedWall W = MakeCoursedWall(CourseCount, ThicknessM, LengthM, HeightM, Friction);
	CHECK(DecideWith(W.Model, CertificateLateral(Lambda - ThresholdMargin)).State == CertificateState::Stands);
	const CertificateResult Failing = DecideWith(W.Model, CertificateLateral(Lambda + ThresholdMargin));
	INFO("state " << Failing.State << ", hinging " << Describe(Failing.HingingInterfaces));
	CHECK(Failing.State == CertificateState::Collapses);
	CHECK(HasInterface(Failing.HingingInterfaces, W.JointsBottomUp.front()));
}

TEST_CASE("Certificate.Anchor.HarmonicStackStandsInsideItsOverhangAndCollapsesBeyond")
{
	for (int Count = 2; Count <= 4; ++Count)
	{
		INFO("n = " << Count);
		const Stack Inside = BuildOnTable(HarmonicStack(Count, 1.0 - ThresholdMargin), 0.0);
		const CertificateResult InsideResult = DecideWith(Inside.Model, CertificateOptions{});
		INFO("inside reads " << InsideResult.State);
		CHECK(InsideResult.State == CertificateState::Stands);
		const Stack Beyond = BuildOnTable(HarmonicStack(Count, 1.0 + ThresholdMargin), 0.0);
		const CertificateResult BeyondResult = DecideWith(Beyond.Model, CertificateOptions{});
		INFO("beyond reads " << BeyondResult.State);
		CHECK(BeyondResult.State == CertificateState::Collapses);
	}
}

TEST_CASE("Certificate.Anchor.InvertedThreeTriangleCollapsesAndFourDiamondStands")
{
	const Stack Triangle = BuildOnTable(BrickRows({1, 2, 3}), 5.0);
	const CertificateResult TriangleResult = DecideWith(Triangle.Model, CertificateOptions{});
	INFO("inverted 3-triangle reads " << TriangleResult.State);
	CHECK(TriangleResult.State == CertificateState::Collapses);

	const Stack Diamond = BuildOnTable(BrickRows({1, 2, 3, 4, 3, 2, 1}), 5.0);
	const CertificateResult DiamondResult = DecideWith(Diamond.Model, CertificateOptions{});
	INFO("4-diamond reads " << DiamondResult.State);
	CHECK(DiamondResult.State == CertificateState::Stands);
}

TEST_CASE("Certificate.Anchor.TwoBrickStackPastFrictionCollapses")
{
	const double Friction = 0.6;
	const double LambdaX = 0.625;
	const double LambdaY = 0.875;
	CHECK(std::hypot(LambdaX, LambdaY) > Friction * CircumscribedOverInscribed(16));
	const TwoBrickStack S = MakeTwoBrickStack(DryJoint(Friction), 0.03);
	const CertificateResult Result = DecideWith(S.Model, CertificateLateral(LambdaX, LambdaY));
	INFO("state " << Result.State << ", load factors [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "]");
	CHECK(Result.State == CertificateState::Collapses);
}

TEST_CASE("Certificate.Anchor.ThreeBrickBridgeStandsAtEveryProvableBiaxialPoint")
{
	const double Friction = 5.0;
	const double HalfHeightM = BrickHeightM / 2.0;
	const double OverlapLowM = BridgeHalfSpanM - BrickLengthM / 2.0;
	const double OverlapHighM = BrickLengthM / 2.0;
	const double RightBedHighXM = BridgeHalfSpanM + BrickLengthM / 2.0;
	struct Case
	{
		double LambdaX;
		double LambdaY;
	};
	const Case Cases[] = {{2.5, 0.75}, {2.25, 0.5}, {2.25, 0.625}, {2.125, 0.625}, {2.0, 0.0}, {2.125, 0.0}};
	const ThreeBrickBridge B = MakeThreeBrickBridge(DryJoint(Friction));
	for (const Case& Each : Cases)
	{
		INFO("lambda x " << Each.LambdaX << ", lambda y " << Each.LambdaY);
		const double JointResultantXM = Each.LambdaX * HalfHeightM;
		CHECK(JointResultantXM > OverlapLowM);
		CHECK(JointResultantXM < OverlapHighM);
		CHECK(Each.LambdaY * HalfHeightM < BrickWidthM / 2.0);
		CHECK(BridgeHalfSpanM / 2.0 + Each.LambdaX * BrickHeightM < RightBedHighXM);
		CHECK(Each.LambdaY * BrickHeightM < BrickWidthM / 2.0);
		CHECK(std::hypot(Each.LambdaX, Each.LambdaY) < Friction / CircumscribedOverInscribed(16));
		const CertificateResult Result = DecideWith(B.Model, CertificateLateral(Each.LambdaX, Each.LambdaY));
		INFO("state " << Result.State << ", load factors [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "]");
		CHECK(Result.State == CertificateState::Stands);
		CHECK(Result.Mechanism.empty());
	}
}

TEST_CASE("Certificate.Anchor.ThreeBrickBridgeStandsWhereTheOracleFindsAnAdmissibleField")
{
	struct Case
	{
		double LambdaX;
		double LambdaY;
	};
	const Case Cases[] = {{2.5, 0.875}, {2.375, 0.875}, {2.25, 0.875}};
	const ThreeBrickBridge B = MakeThreeBrickBridge(DryJoint(5.0));
	for (const Case& Each : Cases)
	{
		INFO("lambda x " << Each.LambdaX << ", lambda y " << Each.LambdaY);
		const oracle::OracleVerdict Oracle = oracle::StaticAdmissibility(B.Model, oracle::BiaxialGravity(Each.LambdaX, Each.LambdaY));
		INFO("oracle " << Oracle.Verdict << ", inner residual / demand " << Oracle.InnerResidualOverDemand);
		CHECK(Oracle.Verdict == oracle::Admissibility::CanStand);
		const CertificateResult Result = DecideWith(B.Model, CertificateLateral(Each.LambdaX, Each.LambdaY));
		INFO("state " << Result.State << ", load factors [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "]");
		CHECK(Result.State == CertificateState::Stands);
		CHECK(Result.Mechanism.empty());
	}
}

TEST_CASE("Certificate.Oracle.VerdictsMatchTheStaticAdmissibilityOracle")
{
	SUBCASE("the certificate reproduces the rigid single-brick closed form")
	{
		for (const double Friction : {0.6, 5.0})
		{
			const Structure Model = oracle::MakeStack(Friction, {0.0});
			int Disagreements = 0;
			int Standing = 0;
			for (int IX = 0; IX <= oracle::GridStepsX; ++IX)
			{
				for (int IY = 0; IY <= oracle::GridStepsY; ++IY)
				{
					const double LambdaX = oracle::GridStep * IX;
					const double LambdaY = oracle::GridStep * IY;
					const bool bClosedForm = oracle::RigidBrickCanStand(LambdaX, LambdaY, Friction);
					const CertificateState State = DecideWith(Model, CertificateLateral(LambdaX, LambdaY)).State;
					const CertificateState Expected = bClosedForm ? CertificateState::Stands : CertificateState::Collapses;
					Standing += State == CertificateState::Stands ? 1 : 0;
					Disagreements += State == Expected ? 0 : 1;
				}
			}
			INFO("friction " << Friction << ": certificate stands at " << Standing << " grid points, disagreements " << Disagreements);
			CHECK(Disagreements == 0);
			CHECK(Standing > 0);
		}
	}
	SUBCASE("seeded stacks and bridges")
	{
		int Stands = 0;
		int Collapses = 0;
		int Undecided = 0;
		for (int Draw = 0; Draw < oracle::DrawCount; ++Draw)
		{
			const oracle::SeededDraw Each = oracle::MakeSeededDraw(Draw);
			const CertificateResult Result = DecideWith(Each.Model, CertificateLateral(Each.LambdaX, Each.LambdaY));
			const oracle::OracleVerdict Oracle = oracle::StaticAdmissibility(Each.Model, oracle::BiaxialGravity(Each.LambdaX, Each.LambdaY));
			INFO("master seed " << oracle::MasterSeed << ", draw " << Draw << " (generator seed " << Each.Seed << "): " << Each.Fixture << ", friction " << Each.Friction << ", lambda (" << Each.LambdaX << ", " << Each.LambdaY << ")");
			INFO("certificate " << Result.State << " [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "] after " << Result.Iterations << " iterations; oracle " << Oracle.Verdict << ", inner residual / demand " << Oracle.InnerResidualOverDemand << ", outer " << Oracle.OuterResidualOverDemand);
			CHECK(Oracle.bConverged);
			if (Oracle.Verdict == oracle::Admissibility::CanStand)
			{
				CHECK(Result.State == CertificateState::Stands);
			}
			if (Oracle.Verdict == oracle::Admissibility::CannotStand)
			{
				CHECK(Result.State == CertificateState::Collapses);
			}
			if (Result.State == CertificateState::Undecided)
			{
				CHECK(Oracle.Verdict == oracle::Admissibility::Ambiguous);
			}
			Stands += Result.State == CertificateState::Stands ? 1 : 0;
			Collapses += Result.State == CertificateState::Collapses ? 1 : 0;
			Undecided += Result.State == CertificateState::Undecided ? 1 : 0;
		}
		INFO("over " << oracle::DrawCount << " draws: certificate stands " << Stands << ", collapses " << Collapses << ", undecided " << Undecided);
		CHECK(Stands > 0);
		CHECK(Collapses > 0);
	}
}
