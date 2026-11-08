#include "BrickFixtures.h"
#include "CatalogueTestSupport.h"
#include "StaticsTestSupport.h"
#include "WedgeFixture.h"

#include "loadpath/Catalogue.h"
#include "loadpath/StructuralAuthority.h"
#include "loadpath/geometry/RunningBondWall.h"

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <ostream>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace loadpath
{
inline const char* ComponentVerdictName(ComponentVerdict Verdict)
{
	switch (Verdict)
	{
	case ComponentVerdict::Stands: return "Stands";
	case ComponentVerdict::Overstressed: return "Overstressed";
	case ComponentVerdict::Collapses: return "Collapses";
	case ComponentVerdict::Undecided: return "Undecided";
	case ComponentVerdict::InvalidInput: return "InvalidInput";
	}
	return "ComponentVerdict(?)";
}

inline std::ostream& operator<<(std::ostream& Out, ComponentVerdict Verdict) { return Out << ComponentVerdictName(Verdict); }
}

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double ShapeTolerance = 1.0e-6;
const Vec3 PlusX{1.0, 0.0, 0.0};
const Vec3 PlusZ{0.0, 0.0, 1.0};

AuthorityResult JudgeWith(const Structure& Model, const AuthorityOptions& Options, std::span<const BodyLoad> Loads = {})
{
	return MakeElasticAuthority(MakeRbsmStandingAuthority())->Judge(Model, Loads, Options, nullptr);
}

StandingOptions BiaxialLateralLoad(double LambdaX, double LambdaY)
{
	StandingOptions Options;
	Options.Gravity = Vec3{LambdaX * GravityMPerS2, LambdaY * GravityMPerS2, -GravityMPerS2};
	return Options;
}

AuthorityOptions Biaxial(double LambdaX, double LambdaY)
{
	AuthorityOptions Options;
	Options.Standing = BiaxialLateralLoad(LambdaX, LambdaY);
	return Options;
}

AuthorityOptions Tilted(double TiltRad)
{
	AuthorityOptions Options;
	Options.Standing = TiltedPlane(TiltRad);
	return Options;
}

template <class Id>
bool SameIds(const std::vector<Id>& Got, const std::vector<Id>& Want)
{
	if (Got.size() != Want.size())
	{
		return false;
	}
	for (std::size_t I = 0; I < Got.size(); ++I)
	{
		if (!SameHandle(Got[I], Want[I]))
		{
			return false;
		}
	}
	return true;
}

bool HasReading(const AuthorityResult& Result, InterfaceId Id)
{
	for (const InterfaceReading& Reading : Result.Readings)
	{
		if (SameHandle(Reading.Id, Id))
		{
			return true;
		}
	}
	return false;
}

const MechanismMotion* FindMotion(const ComponentReport& Report, BodyId Body)
{
	for (const MechanismMotion& Motion : Report.Mechanism)
	{
		if (SameHandle(Motion.Body, Body))
		{
			return &Motion;
		}
	}
	return nullptr;
}

void CheckSlidesAlongWithoutClosing(const MechanismMotion& Motion, const Vec3& LoadDirection, const Vec3& JointNormal, double Relative)
{
	const Vec3 Normal = Unit(JointNormal);
	const double OpeningM = Dot(Motion.TranslationM, Normal);
	INFO("translation " << Motion.TranslationM << ", opening " << OpeningM);
	const MechanismMotion InPlane{Motion.Body, Minus(Motion.TranslationM, Times(OpeningM, Normal)), Motion.RotationRad};
	CheckTranslationAlong(InPlane, LoadDirection, Relative);
	CHECK(OpeningM >= -Relative * Length(Motion.TranslationM));
}

StrengthProfile CrushedUnderOwnWeight()
{
	StrengthProfile Profile = DryJoint(0.7);
	Profile.TensileStrengthPa = 1.0e4;
	Profile.ShearCohesionPa = 1.0e4;
	Profile.CompressiveStrengthPa = 0.5 * BrickMassKg * GravityMPerS2 / (BrickLengthM * BrickWidthM);
	return Profile;
}

StrengthProfile TornByAPull()
{
	StrengthProfile Profile = DryJoint(0.7);
	Profile.TensileStrengthPa = 1.0e4;
	Profile.ShearCohesionPa = 1.0e4;
	return Profile;
}

BodyLoad TearingPull(BodyId Brick, const Box& Shape)
{
	const double PullN = BrickMassKg * GravityMPerS2 + 2.0 * TornByAPull().TensileStrengthPa * BrickLengthM * BrickWidthM;
	return BodyLoad{Brick, Vec3{0.0, 0.0, PullN}, Shape.CentreM};
}

std::string DescribeReport(const ComponentReport& Report)
{
	std::ostringstream Out;
	Out << "bodies " << Describe(Report.Bodies) << ", verdict " << Report.Verdict << ", elastic " << Report.Elastic << ", release " << Describe(Report.Release);
	return Out.str();
}

std::string DescribeResult(const AuthorityResult& Result)
{
	std::ostringstream Out;
	Out << "worst " << Result.Worst << ", components " << Result.Components.size() << ", readings " << Result.Readings.size();
	for (const ComponentReport& Report : Result.Components)
	{
		Out << "; [" << DescribeReport(Report) << "]";
	}
	return Out.str();
}
}

TEST_CASE("Statics.Combined.PureElasticStandsPassesThrough")
{
	const BrickOnGround F = MakeBrickOnGround(DryJoint(0.7));
	const std::unique_ptr<IStructuralAuthority> Authority = MakeElasticAuthority(MakeRbsmStandingAuthority());
	const AuthorityResult Result = Authority->Judge(F.Model, {}, AuthorityOptions{}, nullptr);
	const StandingResult Elastic = SolveWith(F.Model, StandingOptions{});
	INFO(DescribeResult(Result) << "; elastic " << Elastic.State << " with " << Elastic.Readings.size() << " readings");
	CHECK(Elastic.State == StandingState::Stands);
	CHECK(Result.Worst == ComponentVerdict::Stands);
	CHECK(Result.Readings.size() == Elastic.Readings.size());
	CHECK(Result.Components.size() == 1u);
	if (Result.Components.size() != 1u)
	{
		return;
	}
	const ComponentReport& Only = Result.Components.front();
	CHECK(SameIds(Only.Bodies, {F.Brick}));
	CHECK(Only.Verdict == ComponentVerdict::Stands);
	CHECK(Only.Elastic == StandingState::Stands);
	CHECK(Only.Release.empty());
}

TEST_CASE("Parked.Statics.Combined.TwoBrickStackPastFrictionCollapses")
{
	const double Friction = 0.6;
	const double LambdaX = 0.625;
	const double LambdaY = 0.875;
	CHECK(std::hypot(LambdaX, LambdaY) > Friction / std::cos(Pi / 16.0));
	const TwoBrickStack S = MakeTwoBrickStack(DryJoint(Friction), 0.03);
	const AuthorityResult Result = JudgeWith(S.Model, Biaxial(LambdaX, LambdaY));
	INFO(DescribeResult(Result));
	CHECK(Result.Worst == ComponentVerdict::Collapses);
	CHECK(Result.Components.size() == 1u);
	if (Result.Components.size() != 1u)
	{
		return;
	}
	const ComponentReport& Stack = Result.Components.front();
	CHECK(SameIds(Stack.Bodies, {S.Lower, S.Upper}));
	CHECK(Stack.Verdict == ComponentVerdict::Collapses);
	CHECK_FALSE(Stack.Release.empty());
	for (const BodyId Released : Stack.Release)
	{
		CHECK((SameHandle(Released, S.Lower) || SameHandle(Released, S.Upper)));
	}
}

TEST_CASE("Parked.Statics.Combined.ThreeBrickBridgeStandsAtEveryProvableBiaxialPoint")
{
	struct Case
	{
		double LambdaX;
		double LambdaY;
	};
	const Case Cases[] = {{2.5, 0.75}, {2.25, 0.5}, {2.25, 0.625}, {2.125, 0.625}, {2.0, 0.0}, {2.125, 0.0}};
	const ThreeBrickBridge B = MakeThreeBrickBridge(DryJoint(5.0));
	for (const Case& Each : Cases)
	{
		const AuthorityResult Result = JudgeWith(B.Model, Biaxial(Each.LambdaX, Each.LambdaY));
		INFO("lambda x " << Each.LambdaX << ", lambda y " << Each.LambdaY << ": " << DescribeResult(Result));
		CHECK(Result.Worst == ComponentVerdict::Stands);
		CHECK(Result.Components.size() == 1u);
		for (const ComponentReport& Report : Result.Components)
		{
			CHECK(Report.Release.empty());
		}
	}
}

TEST_CASE("Parked.Statics.Combined.ThreeBrickBridgeTheElasticSeesFallReadsUndecided")
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
		const AuthorityResult Result = JudgeWith(B.Model, Biaxial(Each.LambdaX, Each.LambdaY));
		const StandingResult Elastic = SolveWith(B.Model, BiaxialLateralLoad(Each.LambdaX, Each.LambdaY));
		INFO("lambda x " << Each.LambdaX << ", lambda y " << Each.LambdaY << ": " << DescribeResult(Result) << "; elastic alone " << Elastic.State);
		CHECK(Elastic.State == StandingState::Mechanism);
		CHECK(Result.Worst == ComponentVerdict::Undecided);
		CHECK(Result.Components.size() == 1u);
		for (const ComponentReport& Report : Result.Components)
		{
			CHECK(Report.Elastic == StandingState::Mechanism);
			CHECK(Report.Release.empty());
			CHECK(Report.Mechanism.empty());
		}
	}
}

TEST_CASE("Parked.Statics.Combined.ThreeBrickBridgeStandsWhereTheElasticFindsNoMechanism")
{
	const ThreeBrickBridge B = MakeThreeBrickBridge(DryJoint(5.0));
	const AuthorityResult Result = JudgeWith(B.Model, Biaxial(2.5, 0.75));
	const StandingResult Elastic = SolveWith(B.Model, BiaxialLateralLoad(2.5, 0.75));
	INFO(DescribeResult(Result) << "; elastic alone " << Elastic.State);
	CHECK(Elastic.State == StandingState::Undecided);
	CHECK(Result.Worst == ComponentVerdict::Stands);
	CHECK(Result.Components.size() == 1u);
	for (const ComponentReport& Report : Result.Components)
	{
		CHECK(Report.Release.empty());
	}
}

TEST_CASE("Parked.Statics.Combined.DryTwoCourseWallTheElasticSeesFallReadsUndecided")
{
	const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
	REQUIRE(Loaded.Ok());
	StrengthProfile Bed = JointOrFail(Loaded.Value, "lime_mortar_handmade_brick_upc_mean").Profile;
	StrengthProfile Perpend = Bed;
	Bed.ShearCohesionPa = 0.0;
	Perpend.ShearCohesionPa = 0.0;
	Bed.FrictionCoefficient = 0.74706556370258748;
	Perpend.FrictionCoefficient = 0.67429357703282577;
	Structure Model;
	geometry::WallSpec Spec;
	Spec.Bed = AddProfileOk(Model, Bed);
	Spec.Perpend = AddProfileOk(Model, Perpend);
	Spec.Courses = 2;
	Spec.BricksPerCourse = 2;
	const geometry::Wall W = geometry::BuildRunningBondWall(Model, Spec);
	REQUIRE(W.bComplete);
	REQUIRE(W.Courses.size() == 2u);
	REQUIRE(W.Courses[0].size() == 2u);
	const std::vector<BodyLoad> Loads{BodyLoad{W.Courses[0][1], Vec3{-66.39608475530504, 51.879148710628506, -10.046133472861092}, Vec3{0.39583565996234071, 0.011794524107767088, 0.045273791494657996}}};
	const Vec3 Gravity{2.2757538861616866, 0.0, -GravityMPerS2};
	AuthorityOptions Options;
	Options.Standing.Gravity = Gravity;
	const AuthorityResult Result = JudgeWith(Model, Options, Loads);
	const StandingResult Elastic = SolveWith(Model, Options.Standing, Loads);
	INFO(DescribeResult(Result) << "; elastic alone " << Elastic.State);
	CHECK(Model.LiveBodyCount() == 6u);
	CHECK(Elastic.State == StandingState::Mechanism);
	CHECK(Result.Worst == ComponentVerdict::Undecided);
	CHECK(Result.Components.size() == 1u);
	for (const ComponentReport& Report : Result.Components)
	{
		CHECK(Report.Elastic == StandingState::Mechanism);
		CHECK(Report.Release.empty());
	}
}

TEST_CASE("Statics.Combined.ReportsEachComponentAndTheWorstVerdict")
{
	const double Friction = 0.7;
	const double PushOverWeight = 1.0;
	CHECK(PushOverWeight > Friction / std::cos(Pi / 16.0));
	CHECK(PushOverWeight < BrickLengthM / BrickHeightM);
	Structure Model;
	const BodyId Ground = AddGroundPlane(Model);
	const ProfileId Torn = AddProfileOk(Model, TornByAPull());
	const ProfileId Dry = AddProfileOk(Model, DryJoint(Friction));
	const BodyId TornBrick = AddBox(Model, BrickShape(0.0));
	const InterfaceId TornBed = AddInterfaceOk(Model, OnGroundPlane(Ground, TornBrick, BrickShape(0.0), Torn));
	const BodyId PushedBrick = AddBox(Model, BrickShape(1.0));
	const InterfaceId PushedBed = AddInterfaceOk(Model, OnGroundPlane(Ground, PushedBrick, BrickShape(1.0), Dry));
	const std::vector<BodyLoad> Loads{TearingPull(TornBrick, BrickShape(0.0)), BodyLoad{PushedBrick, Vec3{PushOverWeight * BrickMassKg * GravityMPerS2, 0.0, 0.0}, BrickShape(1.0).CentreM}};
	const AuthorityResult Result = JudgeWith(Model, AuthorityOptions{}, Loads);
	INFO(DescribeResult(Result));
	CHECK(Result.Worst == ComponentVerdict::Collapses);
	CHECK(HasReading(Result, TornBed));
	CHECK(HasReading(Result, PushedBed));
	CHECK(Result.Components.size() == 2u);
	if (Result.Components.size() != 2u)
	{
		return;
	}
	const ComponentReport& First = Result.Components[0];
	const ComponentReport& Second = Result.Components[1];
	CHECK(SameIds(First.Bodies, {TornBrick}));
	CHECK(First.Elastic == StandingState::Overstressed);
	CHECK(First.Verdict == ComponentVerdict::Overstressed);
	CHECK(First.Release.empty());
	CHECK(SameIds(Second.Bodies, {PushedBrick}));
	CHECK(Second.Verdict == ComponentVerdict::Collapses);
	CHECK(SameIds(Second.Release, {PushedBrick}));
}

TEST_CASE("Statics.Combined.CollapseReleaseIsTheElasticList")
{
	const double Friction = 0.6;
	const double TiltRad = 35.0 * RadPerDegree;
	CHECK(std::tan(TiltRad) > Friction / std::cos(Pi / 16.0));
	CHECK(std::tan(TiltRad) < BrickLengthM / BrickHeightM);
	const BrickOnGround F = MakeBrickOnGround(DryJoint(Friction));
	const AuthorityResult Result = JudgeWith(F.Model, Tilted(TiltRad));
	INFO(DescribeResult(Result));
	CHECK(Result.Worst == ComponentVerdict::Collapses);
	CHECK(Result.Components.size() == 1u);
	if (Result.Components.size() != 1u)
	{
		return;
	}
	const ComponentReport& Only = Result.Components.front();
	CHECK(Only.Verdict == ComponentVerdict::Collapses);
	CHECK(SameIds(Only.Release, {F.Brick}));
	CHECK(Only.Mechanism.size() == 1u);
	const MechanismMotion* Motion = FindMotion(Only, F.Brick);
	CHECK(Motion != nullptr);
	if (Motion == nullptr)
	{
		return;
	}
	CheckSlidesAlongWithoutClosing(*Motion, PlusX, PlusZ, ShapeTolerance);
}

TEST_CASE("Parked.Statics.Combined.UndecidedComponentIsRefereed")
{
	const BrickOnGround F = MakeBrickOnGround(DryJoint(0.6));
	const double TiltRad = 29.0 * RadPerDegree;
	const int Needed = SolveWith(F.Model, TiltedPlane(TiltRad)).ActiveSetIterations;
	CHECK(Needed >= 2);
	AuthorityOptions Options = Tilted(TiltRad);
	Options.Standing.MaxActiveSetIterations = Needed - 1;
	const StandingResult Elastic = SolveWith(F.Model, Options.Standing);
	const AuthorityResult Result = JudgeWith(F.Model, Options);
	INFO("passes needed " << Needed << ", capped elastic " << Elastic.State << "; " << DescribeResult(Result));
	CHECK(Elastic.State == StandingState::Undecided);
	CHECK(Result.Components.size() == 1u);
	if (Result.Components.size() != 1u)
	{
		return;
	}
	const ComponentReport& Only = Result.Components.front();
	CHECK(Only.Elastic == StandingState::Undecided);
	CHECK(Only.Verdict == ComponentVerdict::Stands);
}

TEST_CASE("Statics.Combined.ElasticInvalidInputPropagates")
{
	BrickOnGround F = MakeBrickOnGround(DryJoint(0.7));
	const BodyId Live = AddBox(F.Model, BrickShape(1.0));
	const std::vector<BodyLoad> Loads{BodyLoad{Live, Vec3{std::nan(""), 0.0, 0.0}, BrickShape(1.0).CentreM}};
	const AuthorityResult Result = JudgeWith(F.Model, AuthorityOptions{}, Loads);
	INFO(DescribeResult(Result));
	CHECK(SolveWith(F.Model, StandingOptions{}, Loads).State == StandingState::InvalidInput);
	CHECK(Result.Worst == ComponentVerdict::InvalidInput);
}

TEST_CASE("Statics.Combined.ComponentVerdictsAreLocal")
{
	const double TiltRad = 29.0 * RadPerDegree;
	Structure Model;
	const BodyId Ground = AddGroundPlane(Model);
	const ProfileId Torn = AddProfileOk(Model, TornByAPull());
	const ProfileId Dry = AddProfileOk(Model, DryJoint(0.6));
	const BodyId TornBrick = AddBox(Model, BrickShape(0.0));
	const InterfaceId TornBed = AddInterfaceOk(Model, OnGroundPlane(Ground, TornBrick, BrickShape(0.0), Torn));
	const BodyId CappedBrick = AddBox(Model, BrickShape(1.0));
	AddInterfaceOk(Model, OnGroundPlane(Ground, CappedBrick, BrickShape(1.0), Dry));
	AuthorityOptions Options = Tilted(TiltRad);
	Options.Standing.MaxActiveSetIterations = 1;
	const std::vector<BodyLoad> Loads{TearingPull(TornBrick, BrickShape(0.0))};
	const AuthorityResult Result = JudgeWith(Model, Options, Loads);
	INFO(DescribeResult(Result));
	CHECK(Result.Components.size() == 2u);
	if (Result.Components.size() != 2u)
	{
		return;
	}
	const ComponentReport& First = Result.Components[0];
	const ComponentReport& Second = Result.Components[1];
	CHECK(SameIds(First.Bodies, {TornBrick}));
	CHECK(First.Elastic == StandingState::Overstressed);
	for (const InterfaceReading& Reading : Result.Readings)
	{
		if (SameHandle(Reading.Id, TornBed))
		{
			CHECK(Reading.Axis == GoverningAxis::Tension);
			CHECK(Reading.TensionUtilisation > 1.0);
		}
	}
	CHECK(First.Verdict == ComponentVerdict::Overstressed);
	CHECK(First.Release.empty());
	CHECK(SameIds(Second.Bodies, {CappedBrick}));
	CHECK(Second.Elastic == StandingState::Undecided);
}

TEST_CASE("Statics.Combined.ACrushedComponentIsConsultedAndCollapses")
{
	const double TiltRad = 29.0 * RadPerDegree;
	const StrengthProfile Profile = CrushedUnderOwnWeight();
	const double StrengthOverWeight = Profile.CompressiveStrengthPa * BrickLengthM * BrickWidthM / (BrickMassKg * GravityMPerS2);
	const double CrushingLoadFactor = StrengthOverWeight / (std::cos(TiltRad) + BrickHeightM / BrickLengthM * std::sin(TiltRad));
	CHECK(CrushingLoadFactor < 1.0);
	const BrickOnGround F = MakeBrickOnGround(Profile);
	const AuthorityResult Result = JudgeWith(F.Model, Tilted(TiltRad));
	INFO(DescribeResult(Result));
	for (const InterfaceReading& Reading : Result.Readings)
	{
		CHECK(Reading.Axis == GoverningAxis::Compression);
		CHECK(Reading.TensionUtilisation <= 1.0);
		CHECK(Reading.ShearUtilisation <= 1.0);
	}
	CHECK(Result.Worst == ComponentVerdict::Collapses);
	CHECK(Result.Components.size() == 1u);
	if (Result.Components.size() != 1u)
	{
		return;
	}
	const ComponentReport& Only = Result.Components.front();
	CHECK(Only.Elastic == StandingState::Overstressed);
	CHECK(Only.Verdict == ComponentVerdict::Collapses);
	CHECK(SameIds(Only.Release, {F.Brick}));
}

TEST_CASE("Statics.Combined.NonFiniteReadingsNeverStand")
{
	Structure Model;
	const BodyId Ground = AddGroundPlane(Model);
	const ProfileId Crushed = AddProfileOk(Model, CrushedUnderOwnWeight());
	const ProfileId Dry = AddProfileOk(Model, DryJoint(0.7));
	const BodyId CrushedBrick = AddBox(Model, BrickShape(0.0));
	AddInterfaceOk(Model, OnGroundPlane(Ground, CrushedBrick, BrickShape(0.0), Crushed));
	const BodyId LoadedBrick = AddBox(Model, BrickShape(1.0));
	AddInterfaceOk(Model, OnGroundPlane(Ground, LoadedBrick, BrickShape(1.0), Dry));
	const Vec3 HugeForceN{0.0, 0.0, -1.0e308};
	const std::array<BodyLoad, 2> Loads{BodyLoad{LoadedBrick, HugeForceN, BrickShape(1.0).CentreM}, BodyLoad{LoadedBrick, HugeForceN, BrickShape(1.0).CentreM}};
	const AuthorityResult Result = JudgeWith(Model, AuthorityOptions{}, Loads);
	INFO(DescribeResult(Result));
	CHECK(Result.Worst != ComponentVerdict::Stands);
	CHECK(Result.Components.size() == 2u);
	if (Result.Components.size() != 2u)
	{
		return;
	}
	const ComponentReport& Loaded = Result.Components[1];
	CHECK(SameIds(Loaded.Bodies, {LoadedBrick}));
	CHECK(Loaded.Verdict != ComponentVerdict::Stands);
	CHECK(Loaded.Elastic != StandingState::Stands);
}

TEST_CASE("Statics.Combined.ElasticUndecidedReleasesNothing")
{
	const double Friction = 0.6;
	const double Lambda = 0.7;
	CHECK(Lambda > Friction);
	CHECK(Lambda < BrickLengthM / BrickHeightM);
	const BrickOnGround F = MakeBrickOnGround(DryJoint(Friction));
	AuthorityOptions Options = Biaxial(Lambda, 0.0);
	Options.Standing.MaxActiveSetIterations = 1;
	const StandingResult Elastic = SolveWith(F.Model, Options.Standing);
	const AuthorityResult Result = JudgeWith(F.Model, Options);
	INFO("elastic " << Elastic.State << "; " << DescribeResult(Result));
	CHECK(Elastic.State == StandingState::Undecided);
	CHECK(Result.Worst == ComponentVerdict::Undecided);
	CHECK(Result.Components.size() == 1u);
	if (Result.Components.size() != 1u)
	{
		return;
	}
	const ComponentReport& Only = Result.Components.front();
	CHECK(Only.Elastic == StandingState::Undecided);
	CHECK(Only.Verdict == ComponentVerdict::Undecided);
	CHECK(Only.Release.empty());
	CHECK(Only.Mechanism.empty());
}
