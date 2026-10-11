#include "AnchorFixtures.h"
#include "BrickFixtures.h"
#include "CertificateTestSupport.h"
#include "StaticAdmissibilityOracle.h"
#include "StaticsTestSupport.h"

#include "doctest.h"

#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>
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
	CHECK(Failing.HingingInterfaces.size() == 1u);
	CHECK(Failing.UnconstrainedBodies.size() == W.Courses.size());
	for (const WallCourse& Course : W.Courses)
	{
		CHECK(HasBody(Failing.UnconstrainedBodies, Course.Body));
	}
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

namespace
{
bool GridPointNearClosedFormBoundary(int IX, int IY, double Friction)
{
	const bool bHere = oracle::RigidBrickCanStand(oracle::GridStep * IX, oracle::GridStep * IY, Friction);
	for (int DX = -1; DX <= 1; ++DX)
	{
		for (int DY = -1; DY <= 1; ++DY)
		{
			if (oracle::RigidBrickCanStand(oracle::GridStep * (IX + DX), oracle::GridStep * (IY + DY), Friction) != bHere)
			{
				return true;
			}
		}
	}
	return false;
}

constexpr int GridStride = oracle::bOptimisedBuild ? 1 : 4;

struct BrickPlacement
{
	BodyId Body;
	Box Shape;
};

struct RunningBondWall
{
	Structure Model;
	std::vector<std::vector<BrickPlacement>> Courses;
	std::vector<InterfaceId> Interfaces;
	std::vector<std::pair<BodyId, BodyId>> InterfaceBodies;
	BodyId Unsupported;
};

RunningBondWall MakeRunningBondWallWithoutLastFirstCourseBrick(double Friction)
{
	const int CourseCount = 4;
	const int BricksPerCourse = 4;
	RunningBondWall W;
	const ProfileId P = AddProfileOk(W.Model, DryJoint(Friction));
	const BodyId Ground = AddGroundPlane(W.Model);
	W.Courses.resize(CourseCount);
	for (int Course = 0; Course < CourseCount; ++Course)
	{
		const double OffsetM = (Course % 2 == 1) ? BrickLengthM / 2.0 : 0.0;
		for (int Index = 0; Index < BricksPerCourse; ++Index)
		{
			if (Course == 0 && Index == BricksPerCourse - 1)
			{
				continue;
			}
			const Box Shape = BrickShape(OffsetM + Index * BrickLengthM, Course);
			W.Courses[static_cast<std::size_t>(Course)].push_back(BrickPlacement{AddBox(W.Model, Shape), Shape});
		}
	}
	for (const BrickPlacement& Brick : W.Courses[0])
	{
		W.Interfaces.push_back(AddInterfaceOk(W.Model, OnGroundPlane(Ground, Brick.Body, Brick.Shape, P)));
		W.InterfaceBodies.push_back({Ground, Brick.Body});
	}
	for (std::size_t Course = 1; Course < W.Courses.size(); ++Course)
	{
		for (const BrickPlacement& Upper : W.Courses[Course])
		{
			for (const BrickPlacement& Lower : W.Courses[Course - 1])
			{
				const std::optional<InterfaceDesc> Desc = StackedFaces(Lower.Body, Lower.Shape, Upper.Body, Upper.Shape, P);
				if (Desc)
				{
					W.Interfaces.push_back(AddInterfaceOk(W.Model, *Desc));
					W.InterfaceBodies.push_back({Lower.Body, Upper.Body});
				}
			}
		}
	}
	W.Unsupported = W.Courses[1].back().Body;
	return W;
}
}

TEST_CASE("Certificate.Anchor.TensileStrengthBuysNoFrictionUnderCompression")
{
	const double TiltRad = 40.0 * RadPerDegree;
	const double Friction = 0.3;
	const double TrueLoadFactor = 0.9;
	const double S = std::sin(TiltRad);
	const double C = std::cos(TiltRad);
	CHECK(std::tan(TiltRad) > Friction * CircumscribedOverInscribed(16));
	CHECK(C - (BrickHeightM / BrickLengthM) * S > 0.0);
	const double CohesionPa = TrueLoadFactor * BrickWeightN() * (S - Friction * C) / BedAreaM2();
	for (const double TensileShare : {0.0, 0.5, 1.0})
	{
		const double TensileStrengthPa = TensileShare * CohesionPa;
		INFO("ft " << TensileStrengthPa << " Pa, c " << CohesionPa << " Pa");
		const BrickOnGround F = MakeBrickOnGround(Bonded(TensileStrengthPa, CohesionPa, Friction));
		const CertificateResult Result = DecideWith(F.Model, CertificateTilt(TiltRad));
		INFO("state " << Result.State << ", load factors [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "]");
		CHECK(Result.State != CertificateState::Stands);
		CHECK(Result.LoadFactorLower <= TrueLoadFactor * (1.0 + LoadFactorRelative) + LoadFactorAbsolute);
		CHECK(std::abs(Result.LoadFactorLower - TrueLoadFactor) <= LoadFactorRelative * TrueLoadFactor);
		if (TensileShare == 0.0)
		{
			CHECK(Result.State == CertificateState::Collapses);
		}
	}
}

TEST_CASE("Certificate.Anchor.OnlyTheSlidingBrickIsNamed")
{
	const double JointFriction = 0.3;
	const double LambdaX = 0.5;
	CHECK(LambdaX > JointFriction * CircumscribedOverInscribed(16));
	CHECK(LambdaX * BrickHeightM < BrickLengthM / 2.0);
	Structure Model;
	const ProfileId BedProfile = AddProfileOk(Model, DryJoint(5.0));
	const ProfileId JointProfile = AddProfileOk(Model, DryJoint(JointFriction));
	const BodyId Ground = AddGroundPlane(Model);
	const BodyId Lower = AddBox(Model, BrickShape(0.0, 0));
	const BodyId Upper = AddBox(Model, BrickShape(0.0, 1));
	AddInterfaceOk(Model, OnGroundPlane(Ground, Lower, BrickShape(0.0, 0), BedProfile));
	const InterfaceId Joint = AddInterfaceOk(Model, *StackedFaces(Lower, BrickShape(0.0, 0), Upper, BrickShape(0.0, 1), JointProfile));
	const CertificateResult Result = DecideWith(Model, CertificateLateral(LambdaX));
	INFO("state " << Result.State);
	CHECK(Result.State == CertificateState::Collapses);
	CheckOnlyBodyAndInterface(Result, Upper, Joint);
}

TEST_CASE("Certificate.Anchor.OnlyTheOverhangingCapTips")
{
	const double CapShiftXM = 0.17;
	CHECK(CapShiftXM > BrickLengthM / 2.0);
	CHECK(CapShiftXM < BrickLengthM);
	CHECK(CapShiftXM / 2.0 < BrickLengthM / 2.0);
	Structure Model;
	const ProfileId P = AddProfileOk(Model, DryJoint(0.6));
	const BodyId Ground = AddGroundPlane(Model);
	const BodyId Pillar = AddBox(Model, BrickShape(0.0, 0));
	const BodyId Cap = AddBox(Model, BrickShape(CapShiftXM, 1));
	AddInterfaceOk(Model, OnGroundPlane(Ground, Pillar, BrickShape(0.0, 0), P));
	const InterfaceId Joint = AddInterfaceOk(Model, *StackedFaces(Pillar, BrickShape(0.0, 0), Cap, BrickShape(CapShiftXM, 1), P));
	const CertificateResult Result = DecideWith(Model, CertificateOptions{});
	INFO("state " << Result.State);
	CHECK(Result.State == CertificateState::Collapses);
	CheckOnlyBodyAndInterface(Result, Cap, Joint);
}

TEST_CASE("Certificate.Anchor.OnlyTheOverhangingPairTipsAsOneBody")
{
	const double B2XM = 0.10;
	const double B3XM = 0.20;
	const Vec3 EdgePointM{BrickLengthM / 2.0, 0.0, BrickHeightM};
	CHECK(B2XM < EdgePointM.X);
	CHECK((B2XM + B3XM) / 2.0 > EdgePointM.X);
	CHECK(std::abs(B3XM - B2XM) < BrickLengthM / 2.0);
	CHECK((B2XM + B3XM) / 3.0 < BrickLengthM / 2.0);
	Structure Model;
	const ProfileId P = AddProfileOk(Model, DryJoint(0.6));
	const BodyId Ground = AddGroundPlane(Model);
	const BodyId Pillar = AddBox(Model, BrickShape(0.0, 0));
	const BodyId B2 = AddBox(Model, BrickShape(B2XM, 1));
	const BodyId B3 = AddBox(Model, BrickShape(B3XM, 2));
	AddInterfaceOk(Model, OnGroundPlane(Ground, Pillar, BrickShape(0.0, 0), P));
	const InterfaceId Joint = AddInterfaceOk(Model, *StackedFaces(Pillar, BrickShape(0.0, 0), B2, BrickShape(B2XM, 1), P));
	AddInterfaceOk(Model, *StackedFaces(B2, BrickShape(B2XM, 1), B3, BrickShape(B3XM, 2), P));
	const CertificateResult Result = DecideWith(Model, CertificateOptions{});
	INFO("state " << Result.State << ", moving " << Describe(Result.UnconstrainedBodies) << ", hinging " << Describe(Result.HingingInterfaces));
	CHECK(Result.State == CertificateState::Collapses);
	CHECK(Result.UnconstrainedBodies.size() == 2u);
	CHECK(HasBody(Result.UnconstrainedBodies, B2));
	CHECK(HasBody(Result.UnconstrainedBodies, B3));
	CHECK(Result.HingingInterfaces.size() == 1u);
	CHECK(HasInterface(Result.HingingInterfaces, Joint));
	const MechanismMotion* M2 = FindMotion(Result, B2);
	const MechanismMotion* M3 = FindMotion(Result, B3);
	REQUIRE(M2 != nullptr);
	REQUIRE(M3 != nullptr);
	INFO("B2 " << M2->TranslationM << " / " << M2->RotationRad << ", B3 " << M3->TranslationM << " / " << M3->RotationRad);
	const double RotationSize = Length(M2->RotationRad);
	CHECK(RotationSize > 0.0);
	CHECK(Length(Minus(M2->RotationRad, M3->RotationRad)) <= 1.0e-9 * RotationSize);
	CHECK(Length(Minus(M2->RotationRad, Times(Dot(M2->RotationRad, PlusY), PlusY))) <= ShapeTolerance * RotationSize);
	const Vec3 AboutEdge2 = Cross(M2->RotationRad, Minus(BrickShape(B2XM, 1).CentreM, EdgePointM));
	const Vec3 AboutEdge3 = Cross(M3->RotationRad, Minus(BrickShape(B3XM, 2).CentreM, EdgePointM));
	CHECK(Length(Minus(M2->TranslationM, AboutEdge2)) <= ShapeTolerance * Length(AboutEdge2));
	CHECK(Length(Minus(M3->TranslationM, AboutEdge3)) <= ShapeTolerance * Length(AboutEdge3));
}

TEST_CASE("Certificate.Anchor.OnlyTheUnsupportedEndBrickFalls")
{
	const RunningBondWall W = MakeRunningBondWallWithoutLastFirstCourseBrick(0.6);
	bool bHasSupportBelow = false;
	for (const std::pair<BodyId, BodyId>& Bodies : W.InterfaceBodies)
	{
		bHasSupportBelow = bHasSupportBelow || SameHandle(Bodies.second, W.Unsupported);
	}
	CHECK_FALSE(bHasSupportBelow);
	const CertificateResult Result = DecideWith(W.Model, CertificateOptions{});
	INFO("state " << Result.State << ", moving " << Describe(Result.UnconstrainedBodies) << ", hinging " << Describe(Result.HingingInterfaces));
	CHECK(Result.State == CertificateState::Collapses);
	CHECK(Result.UnconstrainedBodies.size() == 1u);
	CHECK(HasBody(Result.UnconstrainedBodies, W.Unsupported));
	CHECK_FALSE(Result.HingingInterfaces.empty());
	int NotTouching = 0;
	for (const InterfaceId Hinging : Result.HingingInterfaces)
	{
		for (std::size_t I = 0; I < W.Interfaces.size(); ++I)
		{
			const bool bTouches = SameHandle(W.InterfaceBodies[I].first, W.Unsupported) || SameHandle(W.InterfaceBodies[I].second, W.Unsupported);
			NotTouching += SameHandle(W.Interfaces[I], Hinging) && !bTouches ? 1 : 0;
		}
	}
	CHECK(NotTouching == 0);
}

TEST_CASE("Certificate.Anchor.PointLoadArmSetsTheTippingThreshold")
{
	const double Friction = 5.0;
	const double WeightN = BrickWeightN();
	const double ArmAboveBedM = BrickHeightM;
	const double ThresholdN = WeightN * (BrickLengthM / 2.0) / ArmAboveBedM;
	CHECK(std::abs(ThresholdN * ArmAboveBedM - WeightN * BrickLengthM / 2.0) <= 1.0e-12 * WeightN);
	CHECK(ThresholdN * 1.01 / WeightN < Friction / CircumscribedOverInscribed(16));
	const BrickOnGround F = MakeBrickOnGround(DryJoint(Friction));
	const Vec3 TopCentreM = Plus(F.Model.Body(F.Brick).CentroidM, Vec3{0.0, 0.0, BrickHeightM / 2.0});
	CHECK(std::abs(TopCentreM.Z - ArmAboveBedM) <= 1.0e-12);
	for (const double Share : {0.99, 1.01})
	{
		INFO("P = " << Share << " of " << ThresholdN << " N");
		const BodyLoad Loads[] = {BodyLoad{F.Brick, Vec3{Share * ThresholdN, 0.0, 0.0}, TopCentreM}};
		const CertificateResult Result = DecideWith(F.Model, CertificateOptions{}, Loads);
		INFO("state " << Result.State);
		CHECK(Result.State == (Share < 1.0 ? CertificateState::Stands : CertificateState::Collapses));
	}
}

TEST_CASE("Certificate.Anchor.EccentricPointLoadCrushesTheLoadedCorners")
{
	const double WeightN = BrickWeightN();
	StrengthProfile Profile = DryJoint(0.6);
	Profile.CompressiveStrengthPa = 2.0 * WeightN / BedAreaM2();
	const BrickOnGround F = MakeBrickOnGround(Profile);
	const Vec3 OffsetM{0.05, 0.02, 0.5};
	const double PointLoadN = 3.0 * WeightN;
	const double TotalN = WeightN + PointLoadN;
	const double EccentricityXM = PointLoadN * OffsetM.X / TotalN;
	const double EccentricityYM = PointLoadN * OffsetM.Y / TotalN;
	const double Expected = Profile.CompressiveStrengthPa * BedAreaM2() / (TotalN * (1.0 + EccentricityXM / (BrickLengthM / 2.0)));
	CHECK(EccentricityYM / (BrickWidthM / 2.0) < EccentricityXM / (BrickLengthM / 2.0));
	CHECK(std::abs(Expected - 0.370690) <= 1.0e-6 * 0.370690);
	const BodyLoad Loads[] = {BodyLoad{F.Brick, Vec3{0.0, 0.0, -PointLoadN}, Plus(F.Model.Body(F.Brick).CentroidM, OffsetM)}};
	const CertificateResult Result = DecideWith(F.Model, CertificateOptions{}, Loads);
	INFO("state " << Result.State << ", load factors [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "], expected " << Expected);
	CHECK(Result.State == CertificateState::Collapses);
	CHECK(std::abs(Result.LoadFactorLower - Expected) <= 1.0e-6 * Expected);
}

TEST_CASE("Certificate.Anchor.CrushingBoundIncludesTheTensionOffset")
{
	const double BondPa = 1.0e5;
	for (const double Ratio : {0.5, 2.0})
	{
		INFO("fc = " << Ratio << " W/A");
		StrengthProfile Profile = Bonded(BondPa, BondPa, 0.6);
		Profile.CompressiveStrengthPa = Ratio * BrickWeightN() / BedAreaM2();
		const BrickOnGround F = MakeBrickOnGround(Profile);
		const CertificateResult Result = DecideWith(F.Model, CertificateOptions{});
		INFO("state " << Result.State << ", load factors [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "]");
		CHECK(std::abs(Result.LoadFactorLower - Ratio) <= LoadFactorRelative * Ratio);
		CHECK(std::abs(Result.LoadFactorUpper - Ratio) <= LoadFactorRelative * Ratio);
	}
}

TEST_CASE("Certificate.Anchor.CohesionIsScaledOnTheCircumscribedCone")
{
	const double CohesionPa = 1.0e3;
	const double Lambda = 0.1;
	const double AngleRad = Pi / 16.0;
	const BrickOnGround F = MakeBrickOnGround(Bonded(0.0, CohesionPa, 0.0));
	const double Inscribed = CohesionPa * BedAreaM2() * std::cos(AngleRad) / (Lambda * BrickWeightN());
	const double Circumscribed = CohesionPa * BedAreaM2() / (Lambda * BrickWeightN());
	CHECK(Circumscribed < CertificateOptions{}.LoadFactorCap);
	CHECK(Lambda * BrickHeightM < BrickWidthM);
	const CertificateResult Result = DecideWith(F.Model, CertificateLateral(Lambda * std::cos(AngleRad), Lambda * std::sin(AngleRad)));
	INFO("state " << Result.State << ", load factors [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "], expected [" << Inscribed << ", " << Circumscribed << "]");
	CHECK(std::abs(Result.LoadFactorLower - Inscribed) <= LoadFactorRelative * Inscribed);
	CHECK(std::abs(Result.LoadFactorUpper - Circumscribed) <= LoadFactorRelative * Circumscribed);
}

TEST_CASE("Certificate.Oracle.SingleBrickGridMatchesTheClosedForm")
{
	for (const double Friction : {0.6, 5.0})
	{
		const Structure Model = oracle::MakeStack(Friction, {0.0});
		int Disagreements = 0;
		int Standing = 0;
		int Visited = 0;
		for (int IX = 0; IX <= oracle::GridStepsX; ++IX)
		{
			for (int IY = 0; IY <= oracle::GridStepsY; ++IY)
			{
				const bool bOnStride = IX % GridStride == 0 && IY % GridStride == 0;
				if (!bOnStride && !GridPointNearClosedFormBoundary(IX, IY, Friction))
				{
					continue;
				}
				++Visited;
				const double LambdaX = oracle::GridStep * IX;
				const double LambdaY = oracle::GridStep * IY;
				const bool bClosedForm = oracle::RigidBrickCanStand(LambdaX, LambdaY, Friction);
				const CertificateState State = DecideWith(Model, CertificateLateral(LambdaX, LambdaY)).State;
				const CertificateState Expected = bClosedForm ? CertificateState::Stands : CertificateState::Collapses;
				Standing += State == CertificateState::Stands ? 1 : 0;
				Disagreements += State == Expected ? 0 : 1;
			}
		}
		INFO("friction " << Friction << ", stride " << GridStride << ": visited " << Visited << ", certificate stands at " << Standing << ", disagreements " << Disagreements);
		CHECK(Disagreements == 0);
		CHECK(Standing > 0);
	}
}

TEST_CASE("Certificate.Oracle.SeededDrawsMatchTheOracle")
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

TEST_CASE("Certificate.Anchor.InnerRegionCapsTensionAtCohesionOverFriction")
{
	const double Friction = 0.8;
	const double CohesionPa = 1000.0;
	const double TensilePa = 3000.0;
	const double TrueLoadFactor = 0.9;
	const double Ratio = CircumscribedOverInscribed(16);
	CHECK(CohesionPa / Friction < TensilePa);
	const BrickOnGround F = MakeBrickOnGround(Bonded(TensilePa, CohesionPa, Friction));
	const CertificateOptions Weightless = CertificateUnderGravity(Vec3{0.0, 0.0, 0.0});
	const Vec3 CentroidM = F.Model.Body(F.Brick).CentroidM;
	struct Case
	{
		const char* Name;
		Vec3 ForceN;
		double ExpectedUpper;
	};
	const double PullN = (CohesionPa / Friction) * BedAreaM2() / TrueLoadFactor;
	const double ShearN = CohesionPa * BedAreaM2() / TrueLoadFactor;
	const Case Cases[] = {
		{"pull", Vec3{0.0, 0.0, PullN}, TensilePa * BedAreaM2() / PullN},
		{"shear", Vec3{ShearN, 0.0, 0.0}, Ratio * (CohesionPa + Friction * TensilePa) * BedAreaM2() / ShearN},
	};
	CHECK(std::abs(Cases[0].ExpectedUpper - 2.16) <= 1.0e-12);
	CHECK(std::abs(Cases[1].ExpectedUpper - 3.11995) <= 1.0e-5);
	for (const Case& Each : Cases)
	{
		INFO(doctest::String(Each.Name) << ", force " << Each.ForceN << " N, expected [" << TrueLoadFactor << ", " << Each.ExpectedUpper << "]");
		const BodyLoad Loads[] = {BodyLoad{F.Brick, Each.ForceN, CentroidM}};
		const CertificateResult Result = DecideWith(F.Model, Weightless, Loads);
		INFO("state " << Result.State << ", load factors [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "]");
		CHECK(Result.State == CertificateState::Undecided);
		CHECK(std::abs(Result.LoadFactorLower - TrueLoadFactor) <= LoadFactorRelative * TrueLoadFactor);
		CHECK(std::abs(Result.LoadFactorUpper - Each.ExpectedUpper) <= LoadFactorRelative * Each.ExpectedUpper);
	}
}

TEST_CASE("Certificate.Anchor.OnlyTheCapSlidesOffABondedPier")
{
	const double LambdaX = 0.5;
	const double CapFriction = 0.3;
	const Vec3 PierHalfM{0.1, 0.05, 0.5};
	const Box PierShape{Vec3{0.0, 0.0, PierHalfM.Z}, PierHalfM, 272.0};
	const Box CapShape{Vec3{0.0, 0.0, 2.0 * PierHalfM.Z + BrickHeightM / 2.0}, Vec3{BrickLengthM / 2.0, BrickWidthM / 2.0, BrickHeightM / 2.0}, BrickMassKg};
	CHECK(LambdaX > CapFriction * CircumscribedOverInscribed(16));
	CHECK(LambdaX * BrickHeightM / 2.0 < PierHalfM.X);
	Structure Model;
	const ProfileId BedProfile = AddProfileOk(Model, Bonded(1.0e6, 1.0e6, 0.6));
	const ProfileId CapProfile = AddProfileOk(Model, DryJoint(CapFriction));
	const BodyId Ground = AddGroundPlane(Model);
	const BodyId Pier = AddBox(Model, PierShape);
	const BodyId Cap = AddBox(Model, CapShape);
	AddInterfaceOk(Model, OnGroundPlane(Ground, Pier, PierShape, BedProfile));
	const InterfaceId Joint = AddInterfaceOk(Model, *StackedFaces(Pier, PierShape, Cap, CapShape, CapProfile));
	const CertificateResult Result = DecideWith(Model, CertificateLateral(LambdaX));
	INFO("state " << Result.State << ", load factors [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "]");
	CHECK(Result.State == CertificateState::Collapses);
	CheckOnlyBodyAndInterface(Result, Cap, Joint);
}

TEST_CASE("Certificate.Anchor.FailedSelectionReadsUndecidedWithEmptyLists")
{
	const RunningBondWall W = MakeRunningBondWallWithoutLastFirstCourseBrick(0.6);
	const CertificateResult Full = DecideWith(W.Model, CertificateOptions{});
	INFO("full run " << Full.State << " [" << Full.LoadFactorLower << ", " << Full.LoadFactorUpper << "] after " << Full.Iterations << " iterations");
	REQUIRE(Full.State == CertificateState::Collapses);
	REQUIRE(Full.LoadFactorUpper < 1.0);
	CertificateOptions Limited;
	Limited.MaxIterations = Full.Iterations;
	const CertificateResult Result = DecideWith(W.Model, Limited);
	INFO("limited to " << Limited.MaxIterations << ": " << Result.State << " [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "], moving " << Describe(Result.UnconstrainedBodies) << ", hinging " << Describe(Result.HingingInterfaces) << ", mechanism bodies " << Result.Mechanism.size());
	CHECK(Result.State == CertificateState::Undecided);
	CHECK(Result.UnconstrainedBodies.empty());
	CHECK(Result.HingingInterfaces.empty());
	CHECK(Result.Mechanism.empty());
	CHECK(Result.LoadFactorUpper < 1.0);
}

namespace
{
struct BystanderFixture
{
	Structure Model;
	BodyId Lower;
	BodyId Top;
	BodyId Bystander;
	InterfaceId TopJoint;
	InterfaceId BystanderJoint;
};

BystanderFixture MakeTippingTopWithBystander(double OverhangScale, double BystanderMassKg, const StrengthProfile& BystanderProfile)
{
	BystanderFixture F;
	const ProfileId Dry = AddProfileOk(F.Model, DryJoint(TableStackFriction));
	const ProfileId BystanderP = AddProfileOk(F.Model, BystanderProfile);
	const Box Table{Vec3{-5.0, 0.0, -0.5}, Vec3{5.0, 5.0, 0.5}, 0.0};
	const Box LowerShape{Vec3{-0.3, 0.0, 0.1}, Vec3{0.5, 0.25, 0.1}, 1.0};
	const Box TopShape{Vec3{0.2 + 0.5 * OverhangScale, 0.0, 0.3}, Vec3{0.5, 0.25, 0.1}, 1.0};
	const Box BystanderShape{Vec3{-0.65, 0.0, 0.3}, Vec3{0.1, 0.25, 0.1}, BystanderMassKg};
	const BodyId TableId = AddBox(F.Model, Table, true);
	F.Lower = AddBox(F.Model, LowerShape);
	F.Top = AddBox(F.Model, TopShape);
	F.Bystander = AddBox(F.Model, BystanderShape);
	AddInterfaceOk(F.Model, *StackedFaces(TableId, Table, F.Lower, LowerShape, Dry));
	F.TopJoint = AddInterfaceOk(F.Model, *StackedFaces(F.Lower, LowerShape, F.Top, TopShape, Dry));
	F.BystanderJoint = AddInterfaceOk(F.Model, *StackedFaces(F.Lower, LowerShape, F.Bystander, BystanderShape, BystanderP));
	return F;
}
}

TEST_CASE("Certificate.Anchor.AStableBystanderIsNotListed")
{
	struct Case
	{
		const char* Name;
		double OverhangScale;
		double BystanderMassKg;
		StrengthProfile Profile;
	};
	const Case Cases[] = {
		{"dry, 1e-4", 1.0e-4, 1.0e-5, DryJoint(TableStackFriction)},
		{"dry, 1e-6", 1.0e-6, 1.0e-5, DryJoint(TableStackFriction)},
		{"bonded, 1e-4", 1.0e-4, 1.0e-4, Bonded(1.0e-3, 1.0e-3, TableStackFriction)},
	};
	for (const Case& Each : Cases)
	{
		const BystanderFixture F = MakeTippingTopWithBystander(Each.OverhangScale, Each.BystanderMassKg, Each.Profile);
		const CertificateResult Result = DecideWith(F.Model, CertificateOptions{});
		const MechanismMotion* Bystander = FindMotion(Result, F.Bystander);
		INFO(doctest::String(Each.Name) << ": state " << Result.State << ", moving " << Describe(Result.UnconstrainedBodies) << ", hinging " << Describe(Result.HingingInterfaces));
		INFO("bystander motion " << (Bystander ? Bystander->TranslationM : Vec3{}) << " / " << (Bystander ? Bystander->RotationRad : Vec3{}));
		CHECK(Result.State == CertificateState::Collapses);
		CHECK_FALSE(HasBody(Result.UnconstrainedBodies, F.Bystander));
		CHECK_FALSE(HasInterface(Result.HingingInterfaces, F.BystanderJoint));
		CheckOnlyBodyAndInterface(Result, F.Top, F.TopJoint);
	}
}

TEST_CASE("Certificate.Property.HingingInterfacesTouchAMovingBody")
{
	const auto CheckEveryHingeTouches = [](const Structure& Model, const CertificateResult& Result) {
		if (Result.State != CertificateState::Collapses)
		{
			return 0;
		}
		INFO("moving " << Describe(Result.UnconstrainedBodies) << ", hinging " << Describe(Result.HingingInterfaces));
		int Orphans = 0;
		for (const InterfaceId Hinging : Result.HingingInterfaces)
		{
			const InterfaceRecord& Record = Model.Interface(Hinging);
			const bool bTouches = HasBody(Result.UnconstrainedBodies, Record.BodyA) || HasBody(Result.UnconstrainedBodies, Record.BodyB);
			INFO("hinging interface " << Describe(std::vector<InterfaceId>{Hinging}) << " touches a moving body: " << bTouches);
			CHECK(bTouches);
			Orphans += bTouches ? 0 : 1;
		}
		return Orphans;
	};
	int Collapses = 0;
	for (int Draw = 0; Draw < oracle::DrawCount; ++Draw)
	{
		const oracle::SeededDraw Each = oracle::MakeSeededDraw(Draw);
		INFO("master seed " << oracle::MasterSeed << ", draw " << Draw << " (generator seed " << Each.Seed << "): " << Each.Fixture << ", friction " << Each.Friction << ", lambda (" << Each.LambdaX << ", " << Each.LambdaY << ")");
		const CertificateResult Result = DecideWith(Each.Model, CertificateLateral(Each.LambdaX, Each.LambdaY));
		CheckEveryHingeTouches(Each.Model, Result);
		Collapses += Result.State == CertificateState::Collapses ? 1 : 0;
	}
	for (int Count = 2; Count <= 8; ++Count)
	{
		INFO("harmonic n = " << Count << ", overhang scale 1 + 1e-6");
		const Stack S = BuildOnTable(HarmonicStack(Count, 1.0 + 1.0e-6), 0.0);
		const CertificateResult Result = DecideWith(S.Model, CertificateOptions{});
		CheckEveryHingeTouches(S.Model, Result);
		Collapses += Result.State == CertificateState::Collapses ? 1 : 0;
	}
	INFO("collapsing cases " << Collapses);
	CHECK(Collapses > 0);
}
