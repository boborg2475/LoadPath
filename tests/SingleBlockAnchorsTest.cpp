#include "AnchorFixtures.h"
#include "StaticsTestSupport.h"

#include "doctest.h"

#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double ShapeTolerance = 1.0e-6;
const Vec3 SlopeDirection{1.0, 0.0, 0.0};
const Vec3 EdgeAxis{0.0, 1.0, 0.0};

void CheckTipsAboutTheDownhillEdge(const StandingResult& Result, BodyId Block, const Box& Shape)
{
	CHECK(Result.State == StandingState::Mechanism);
	const MechanismMotion* Motion = FindMotion(Result, Block);
	CHECK(Motion != nullptr);
	if (Motion == nullptr)
	{
		return;
	}
	CheckRotationAboutEdge(*Motion, Shape.CentreM, DownhillEdgePointM(Shape), EdgeAxis, SlopeDirection, ShapeTolerance);
}

void CheckSlidesDownhill(const StandingResult& Result, BodyId Block)
{
	CHECK(Result.State == StandingState::Mechanism);
	const MechanismMotion* Motion = FindMotion(Result, Block);
	CHECK(Motion != nullptr);
	if (Motion == nullptr)
	{
		return;
	}
	CheckTranslationAlong(*Motion, SlopeDirection, ShapeTolerance);
}

void CheckBracketsWithin(const ThresholdBracket& Bracket, double Expected, double Tolerance)
{
	INFO("last standing " << Bracket.LastStanding << ", first failing " << Bracket.FirstFailing << ", expected " << Expected);
	CHECK(Bracket.bBracketed);
	CHECK(std::abs(Bracket.LastStanding - Expected) <= Tolerance);
	CHECK(std::abs(Bracket.FirstFailing - Expected) <= Tolerance);
}

ThresholdBracket TiltThreshold(const Structure& Model)
{
	return Threshold([&](double TiltRad) { return SolveWith(Model, TiltedPlane(TiltRad)).State == StandingState::Stands; }, 0.0, Pi / 4.0, 1.0e-8);
}

void CheckTiltMatchesLateralLoad(const Structure& Model, double TiltRad)
{
	INFO("tilt " << TiltRad);
	CHECK(SolveWith(Model, TiltedPlane(TiltRad)).State == SolveWith(Model, LateralLoad(std::tan(TiltRad))).State);
}

enum class SlopeRegime
{
	Stable,
	Slides,
	Topples,
	SlidesAndTopples
};

const char* SlopeRegimeName(SlopeRegime Regime)
{
	switch (Regime)
	{
	case SlopeRegime::Stable: return "Stable";
	case SlopeRegime::Slides: return "Slides";
	case SlopeRegime::Topples: return "Topples";
	case SlopeRegime::SlidesAndTopples: return "SlidesAndTopples";
	}
	return "SlopeRegime(?)";
}

SlopeRegime ClosedFormRegime(double WidthM, double HeightM, double FrictionAngleRad, double SlopeRad)
{
	const bool bSlides = FrictionAngleRad < SlopeRad;
	const bool bTopples = WidthM / HeightM < std::tan(SlopeRad);
	if (bSlides && bTopples)
	{
		return SlopeRegime::SlidesAndTopples;
	}
	if (bSlides)
	{
		return SlopeRegime::Slides;
	}
	return bTopples ? SlopeRegime::Topples : SlopeRegime::Stable;
}

struct SlopeCase
{
	const char* Name;
	double HeightM;
	double FrictionAngleDeg;
	SlopeRegime Published;
};

const SlopeCase ItascaSlopeCases[] = {
	{"stable", 10.0, 40.1, SlopeRegime::Stable},
	{"slides", 5.0, 20.0, SlopeRegime::Slides},
	{"topples", 25.0, 50.0, SlopeRegime::Topples},
	{"slides and topples", 20.0, 30.0, SlopeRegime::SlidesAndTopples},
};

Vec3 PlaneUpwardNormal(double DipDeg, double DipDirectionDeg)
{
	const double Dip = DipDeg * RadPerDegree;
	const double Direction = DipDirectionDeg * RadPerDegree;
	return Vec3{std::sin(Dip) * std::sin(Direction), std::sin(Dip) * std::cos(Direction), std::cos(Dip)};
}

struct WedgeGeometry
{
	Vec3 NormalA;
	Vec3 NormalB;
	Vec3 LineDown;
	Vec3 SideA;
	Vec3 SideB;
};

WedgeGeometry ItascaWedgeGeometry()
{
	WedgeGeometry G;
	G.NormalA = PlaneUpwardNormal(40.0, 130.0);
	G.NormalB = PlaneUpwardNormal(60.0, 220.0);
	G.LineDown = Unit(Cross(G.NormalA, G.NormalB));
	if (G.LineDown.Z > 0.0)
	{
		G.LineDown = Times(-1.0, G.LineDown);
	}
	G.SideA = Unit(Cross(G.NormalA, G.LineDown));
	if (Dot(G.NormalB, G.SideA) < 0.0)
	{
		G.SideA = Times(-1.0, G.SideA);
	}
	G.SideB = Unit(Cross(G.NormalB, G.LineDown));
	if (Dot(G.NormalA, G.SideB) < 0.0)
	{
		G.SideB = Times(-1.0, G.SideB);
	}
	return G;
}

double ClassicalWedgeFrictionAngleRad(const WedgeGeometry& G)
{
	const Vec3 Up{0.0, 0.0, 1.0};
	const Vec3 UpAcrossLine = Minus(Up, Times(Dot(Up, G.LineDown), G.LineDown));
	const double Cosine = Dot(G.NormalA, G.NormalB);
	const double OnA = Dot(G.NormalA, UpAcrossLine);
	const double OnB = Dot(G.NormalB, UpAcrossLine);
	const double ReactionA = (OnA - Cosine * OnB) / (1.0 - Cosine * Cosine);
	const double ReactionB = (OnB - Cosine * OnA) / (1.0 - Cosine * Cosine);
	CHECK(ReactionA > 0.0);
	CHECK(ReactionB > 0.0);
	return std::atan(std::abs(Dot(Up, G.LineDown)) / (ReactionA + ReactionB));
}

struct Wedge
{
	Structure Model;
	BodyId Body;
	Vec3 CentroidM;
};

InterfaceDesc WedgeFace(BodyId Ground, BodyId WedgeBody, const Vec3& Normal, const Vec3& LineDown, const Vec3& CentreM, double LengthM, double HeightM, ProfileId Profile)
{
	InterfaceDesc Desc;
	Desc.BodyA = Ground;
	Desc.BodyB = WedgeBody;
	Desc.NormalAToB = Normal;
	Desc.CentreM = CentreM;
	Desc.AxisU = LineDown;
	Desc.AxisV = Cross(Normal, LineDown);
	Desc.HalfExtentUM = LengthM / 2.0;
	Desc.HalfExtentVM = HeightM / 4.0;
	Desc.Profile = Profile;
	return Desc;
}

Wedge MakeItascaWedge(double FrictionAngleRad)
{
	const WedgeGeometry G = ItascaWedgeGeometry();
	const double LengthM = 10.0;
	const double HeightM = 5.0;
	const Vec3 Apex{};
	const Vec3 Toe = Times(LengthM, G.LineDown);
	const Vec3 TipA = Plus(Times(LengthM / 2.0, G.LineDown), Times(HeightM, G.SideA));
	const Vec3 TipB = Plus(Times(LengthM / 2.0, G.LineDown), Times(HeightM, G.SideB));
	const double VolumeM3 = std::abs(Dot(Toe, Cross(TipA, TipB))) / 6.0;

	Wedge W;
	W.CentroidM = Times(0.25, Plus(Plus(Apex, Toe), Plus(TipA, TipB)));
	const ProfileId P = AddProfileOk(W.Model, DryJoint(std::tan(FrictionAngleRad)));
	const Vec3 FaceCentreA = Times(1.0 / 3.0, Plus(Plus(Apex, Toe), TipA));
	const Vec3 FaceCentreB = Times(1.0 / 3.0, Plus(Plus(Apex, Toe), TipB));
	const BodyId GroundA = AddBox(W.Model, Box{Minus(FaceCentreA, G.NormalA), Vec3{1.0, 1.0, 1.0}, 0.0}, true);
	const BodyId GroundB = AddBox(W.Model, Box{Minus(FaceCentreB, G.NormalB), Vec3{1.0, 1.0, 1.0}, 0.0}, true);
	W.Body = AddBox(W.Model, Box{W.CentroidM, Vec3{1.0, 1.0, 1.0}, RockDensityKgPerM3 * VolumeM3});
	AddInterfaceOk(W.Model, WedgeFace(GroundA, W.Body, G.NormalA, G.LineDown, FaceCentreA, LengthM, HeightM, P));
	AddInterfaceOk(W.Model, WedgeFace(GroundB, W.Body, G.NormalB, G.LineDown, FaceCentreB, LengthM, HeightM, P));
	return W;
}
}

TEST_CASE("Statics.Anchor.InclineTipsAtAtanWidthOverHeightAboutTheDownhillEdge")
{
	const double WidthM = 0.10;
	const double HeightM = 0.15;
	const double DepthM = 0.10;
	const double Friction = 0.7;
	CHECK(Friction > WidthM / HeightM);
	const BlockOnGround F = MakeBlockOnGround(WidthM, DepthM, HeightM, Friction);
	const double ExpectedRad = std::atan(WidthM / HeightM);

	const ThresholdBracket Bracket = TiltThreshold(F.Model);
	CheckBracketsWithin(Bracket, ExpectedRad, 1.0e-6);
	if (!Bracket.bBracketed)
	{
		return;
	}
	CheckTipsAboutTheDownhillEdge(SolveWith(F.Model, TiltedPlane(Bracket.FirstFailing)), F.Block, F.Shape);
	for (const double OffsetRad : {-1.0e-6, 1.0e-6})
	{
		CheckTiltMatchesLateralLoad(F.Model, ExpectedRad + OffsetRad);
	}
}

TEST_CASE("Statics.Anchor.InclineSlidesAtAtanFrictionAsAPureTranslation")
{
	const double WidthM = 0.10;
	const double HeightM = 0.15;
	const double DepthM = 0.10;
	const double Friction = 0.6;
	CHECK(Friction < WidthM / HeightM);
	const BlockOnGround F = MakeBlockOnGround(WidthM, DepthM, HeightM, Friction);
	const double ExpectedRad = std::atan(Friction);

	const ThresholdBracket Bracket = TiltThreshold(F.Model);
	CheckBracketsWithin(Bracket, ExpectedRad, 1.0e-6);
	if (!Bracket.bBracketed)
	{
		return;
	}
	CheckSlidesDownhill(SolveWith(F.Model, TiltedPlane(Bracket.FirstFailing)), F.Block);
}

TEST_CASE("Statics.Anchor.HousnerBlockUpliftsAtHalfWidthOverHalfHeightAboutTheToe")
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
		CHECK(Friction > TanAlpha);
		const BlockOnGround F = MakeBlockOnGround(Case.WidthM, DepthM, Case.HeightM, Friction);
		CHECK(SolveWith(F.Model, LateralLoad(TanAlpha - 1.0e-6)).State == StandingState::Stands);
		CheckTipsAboutTheDownhillEdge(SolveWith(F.Model, LateralLoad(TanAlpha + 1.0e-6)), F.Block, F.Shape);
	}
}

TEST_CASE("Statics.Anchor.FreeWallOverturnsAtThicknessOverHeightAboutTheToe")
{
	const double ThicknessM = BrickWidthM;
	const double HeightM = 10.0 * BrickHeightM;
	const double LengthM = 1.0;
	const double Friction = 0.7;
	const double Lambda = ThicknessM / HeightM;
	CHECK(Friction > Lambda);
	const BlockOnGround F = MakeBlockOnGround(ThicknessM, LengthM, HeightM, Friction, BrickDensityKgPerM3());
	CHECK(SolveWith(F.Model, LateralLoad(Lambda - 1.0e-6)).State == StandingState::Stands);
	CheckTipsAboutTheDownhillEdge(SolveWith(F.Model, LateralLoad(Lambda + 1.0e-6)), F.Block, F.Shape);
}

TEST_CASE("Statics.Anchor.CoursedFreeWallHingesOnlyAtTheBase")
{
	const int CourseCount = 10;
	const double ThicknessM = BrickWidthM;
	const double HeightM = CourseCount * BrickHeightM;
	const double LengthM = 1.0;
	const double Friction = 0.7;
	const double Lambda = ThicknessM / HeightM;
	const CoursedWall W = MakeCoursedWall(CourseCount, ThicknessM, LengthM, HeightM, Friction);

	CHECK(SolveWith(W.Model, LateralLoad(Lambda - 1.0e-6)).State == StandingState::Stands);

	const StandingResult Failing = SolveWith(W.Model, LateralLoad(Lambda + 1.0e-6));
	CHECK(Failing.State == StandingState::Mechanism);
	INFO("unconstrained " << DescribeAscending(Failing.UnconstrainedBodies));
	CHECK(Failing.UnconstrainedBodies.size() == W.Courses.size());
	for (const WallCourse& Each : W.Courses)
	{
		CHECK(HasBody(Failing.UnconstrainedBodies, Each.Body));
	}
	const InterfaceReading* Base = FindReading(Failing, W.JointsBottomUp.front());
	const InterfaceReading* Top = FindReading(Failing, W.JointsBottomUp.back());
	CHECK(Base != nullptr);
	CHECK(Top != nullptr);
	if (Base == nullptr || Top == nullptr)
	{
		return;
	}
	CHECK(Base->Corners[1].bOpen);
	CHECK(Base->Corners[2].bOpen);
	for (std::size_t C = 0; C < 4; ++C)
	{
		INFO("top joint corner " << C);
		CHECK_FALSE(Top->Corners[C].bOpen);
	}
}

TEST_CASE("Statics.Anchor.ItascaSlopeBlockFallsIntoItsPublishedRegime")
{
	const double SlopeRad = 40.0 * RadPerDegree;
	const double WidthM = 10.0;
	const double DepthM = 1.0;
	for (const SlopeCase& Case : ItascaSlopeCases)
	{
		INFO("case: " << doctest::String(Case.Name));
		const double FrictionAngleRad = Case.FrictionAngleDeg * RadPerDegree;
		CHECK(SlopeRegimeName(ClosedFormRegime(WidthM, Case.HeightM, FrictionAngleRad, SlopeRad)) == doctest::String(SlopeRegimeName(Case.Published)));
		const BlockOnGround F = MakeBlockOnGround(WidthM, DepthM, Case.HeightM, std::tan(FrictionAngleRad));
		const StandingResult Result = SolveWith(F.Model, TiltedPlane(SlopeRad));
		switch (Case.Published)
		{
		case SlopeRegime::Stable:
			CHECK(Result.State == StandingState::Stands);
			break;
		case SlopeRegime::Slides:
			CheckSlidesDownhill(Result, F.Block);
			break;
		case SlopeRegime::Topples:
			CheckTipsAboutTheDownhillEdge(Result, F.Block, F.Shape);
			break;
		case SlopeRegime::SlidesAndTopples:
			CHECK(Result.State == StandingState::Mechanism);
			break;
		}
	}
}

TEST_CASE("Statics.Anchor.ItascaWedgeSlidesAlongTheIntersectionBelowItsCriticalFrictionAngle" * doctest::skip())
{
	const WedgeGeometry G = ItascaWedgeGeometry();
	const double CriticalRad = ClassicalWedgeFrictionAngleRad(G);
	CHECK(std::abs(CriticalRad - 33.36 * RadPerDegree) <= 0.005 * RadPerDegree);

	const ThresholdBracket Bracket = Threshold(
		[](double FrictionAngleRad) {
			const Wedge W = MakeItascaWedge(FrictionAngleRad);
			return SolveWith(W.Model, StandingOptions{}).State == StandingState::Stands;
		},
		60.0 * RadPerDegree, 20.0 * RadPerDegree, 1.0e-4 * RadPerDegree);
	INFO("critical " << CriticalRad / RadPerDegree << " deg");
	CheckBracketsWithin(Bracket, CriticalRad, 0.05 * RadPerDegree);
	if (!Bracket.bBracketed)
	{
		return;
	}
	const Wedge Failing = MakeItascaWedge(Bracket.FirstFailing);
	const StandingResult Result = SolveWith(Failing.Model, StandingOptions{});
	CHECK(Result.State == StandingState::Mechanism);
	const MechanismMotion* Motion = FindMotion(Result, Failing.Body);
	CHECK(Motion != nullptr);
	if (Motion == nullptr)
	{
		return;
	}
	CheckTranslationAlong(*Motion, G.LineDown, ShapeTolerance);
}
