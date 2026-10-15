#include "BrickFixtures.h"
#include "CatalogueTestSupport.h"
#include "CertificateTestSupport.h"
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
constexpr double DryLoadFactorAbsolute = 1.0e-6;
constexpr double ShapeTolerance = 1.0e-6;
const Vec3 PlusX{1.0, 0.0, 0.0};
const Vec3 PlusZ{0.0, 0.0, 1.0};

class CountingCertificate final : public ICollapseCertificate
{
public:
	explicit CountingCertificate(int& InCalls)
		: Calls(InCalls)
		, Inner(MakeLimitAnalysisCertificate())
	{
	}

	CertificateResult Decide(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, std::span<const BodyId> Only) const override
	{
		++Calls;
		return Inner->Decide(Model, Loads, Options, Only);
	}

private:
	int& Calls;
	std::unique_ptr<ICollapseCertificate> Inner;
};

AuthorityResult JudgeWith(const Structure& Model, const AuthorityOptions& Options, std::span<const BodyLoad> Loads = {})
{
	return MakeCombinedAuthority(MakeRbsmStandingAuthority(), MakeLimitAnalysisCertificate())->Judge(Model, Loads, Options, nullptr);
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
	Options.Certificate = CertificateLateral(LambdaX, LambdaY);
	return Options;
}

AuthorityOptions Tilted(double TiltRad)
{
	AuthorityOptions Options;
	Options.Standing = TiltedPlane(TiltRad);
	Options.Certificate = CertificateTilt(TiltRad);
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

ComponentVerdict MappedKinematic(CertificateState State)
{
	switch (State)
	{
	case CertificateState::Stands: return ComponentVerdict::Stands;
	case CertificateState::Collapses: return ComponentVerdict::Collapses;
	case CertificateState::Undecided: return ComponentVerdict::Undecided;
	case CertificateState::InvalidInput: return ComponentVerdict::InvalidInput;
	}
	return ComponentVerdict::InvalidInput;
}

StrengthProfile CrushedUnderOwnWeight()
{
	StrengthProfile Profile = DryJoint(0.7);
	Profile.TensileStrengthPa = 1.0e4;
	Profile.ShearCohesionPa = 1.0e4;
	Profile.CompressiveStrengthPa = 0.5 * BrickMassKg * GravityMPerS2 / (BrickLengthM * BrickWidthM);
	return Profile;
}

std::string DescribeReport(const ComponentReport& Report)
{
	std::ostringstream Out;
	Out << "bodies " << Describe(Report.Bodies) << ", verdict " << Report.Verdict << ", elastic " << Report.Elastic << ", kinematic " << Report.Kinematic << ", consulted " << Report.bConsultedCertificate << ", disagreed " << Report.bDisagreed << ", load factors [" << Report.LoadFactorLower << ", " << Report.LoadFactorUpper << "], release " << Describe(Report.Release) << ", hinging " << Describe(Report.Hinging);
	return Out.str();
}

std::string DescribeResult(const AuthorityResult& Result)
{
	std::ostringstream Out;
	Out << "worst " << Result.Worst << ", components " << Result.Components.size() << ", disagreements " << Result.Disagreements << ", readings " << Result.Readings.size();
	for (const ComponentReport& Report : Result.Components)
	{
		Out << "; [" << DescribeReport(Report) << "]";
	}
	return Out.str();
}
}

TEST_CASE("Statics.Combined.PureElasticStandsNeverConsultsTheCertificate")
{
	const BrickOnGround F = MakeBrickOnGround(DryJoint(0.7));
	int Calls = 0;
	const std::unique_ptr<IStructuralAuthority> Authority = MakeCombinedAuthority(MakeRbsmStandingAuthority(), std::make_unique<CountingCertificate>(Calls));
	const AuthorityResult Result = Authority->Judge(F.Model, {}, AuthorityOptions{}, nullptr);
	const StandingResult Elastic = SolveWith(F.Model, StandingOptions{});
	INFO(DescribeResult(Result) << "; elastic " << Elastic.State << " with " << Elastic.Readings.size() << " readings, certificate calls " << Calls);
	CHECK(Elastic.State == StandingState::Stands);
	CHECK(Calls == 0);
	CHECK(Result.Worst == ComponentVerdict::Stands);
	CHECK(Result.Readings.size() == Elastic.Readings.size());
	CHECK(Result.Disagreements == 0);
	CHECK(Result.Components.size() == 1u);
	if (Result.Components.size() != 1u)
	{
		return;
	}
	const ComponentReport& Only = Result.Components.front();
	CHECK(SameIds(Only.Bodies, {F.Brick}));
	CHECK(Only.Verdict == ComponentVerdict::Stands);
	CHECK(Only.Elastic == StandingState::Stands);
	CHECK_FALSE(Only.bConsultedCertificate);
	CHECK(Only.Release.empty());
}

TEST_CASE("Statics.Combined.TwoBrickStackPastFrictionCollapses")
{
	const double Friction = 0.6;
	const double LambdaX = 0.625;
	const double LambdaY = 0.875;
	CHECK(std::hypot(LambdaX, LambdaY) > Friction * CircumscribedOverInscribed(16));
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
	CHECK(Stack.bConsultedCertificate);
	CHECK(Stack.Kinematic == CertificateState::Collapses);
	CHECK(Stack.Verdict == ComponentVerdict::Collapses);
	CHECK_FALSE(Stack.Release.empty());
	for (const BodyId Released : Stack.Release)
	{
		CHECK((SameHandle(Released, S.Lower) || SameHandle(Released, S.Upper)));
	}
	CHECK_FALSE(Stack.Hinging.empty());
	for (const InterfaceId Hinge : Stack.Hinging)
	{
		CHECK((SameHandle(Hinge, S.Bed) || SameHandle(Hinge, S.Joint)));
	}
	CHECK(std::abs(Stack.LoadFactorLower) <= DryLoadFactorAbsolute);
	CHECK(std::abs(Stack.LoadFactorUpper) <= DryLoadFactorAbsolute);
}

TEST_CASE("Statics.Combined.ThreeBrickBridgeStandsAtEveryProvableBiaxialPoint")
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

TEST_CASE("Statics.Combined.ThreeBrickBridgeTheElasticSeesFallReadsUndecided")
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
		CHECK(Result.Disagreements == 1);
		CHECK(Result.Components.size() == 1u);
		for (const ComponentReport& Report : Result.Components)
		{
			CHECK(Report.Elastic == StandingState::Mechanism);
			CHECK(Report.bConsultedCertificate);
			CHECK(Report.Kinematic == CertificateState::Stands);
			CHECK(Report.bDisagreed);
			CHECK(Report.Release.empty());
			CHECK(Report.Mechanism.empty());
			CHECK(Report.Hinging.empty());
		}
	}
}

TEST_CASE("Statics.Combined.ThreeBrickBridgeStandsWhereTheElasticFindsNoMechanism")
{
	const ThreeBrickBridge B = MakeThreeBrickBridge(DryJoint(5.0));
	const AuthorityResult Result = JudgeWith(B.Model, Biaxial(2.5, 0.75));
	const StandingResult Elastic = SolveWith(B.Model, BiaxialLateralLoad(2.5, 0.75));
	INFO(DescribeResult(Result) << "; elastic alone " << Elastic.State);
	CHECK(Elastic.State == StandingState::Undecided);
	CHECK(Result.Worst == ComponentVerdict::Stands);
	CHECK(Result.Disagreements == 0);
	CHECK(Result.Components.size() == 1u);
	for (const ComponentReport& Report : Result.Components)
	{
		CHECK(Report.Kinematic == CertificateState::Stands);
		CHECK_FALSE(Report.bDisagreed);
		CHECK(Report.Release.empty());
	}
}

TEST_CASE("Statics.Combined.DryTwoCourseWallTheElasticSeesFallReadsUndecided")
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
	Options.Certificate.Gravity = Gravity;
	const AuthorityResult Result = JudgeWith(Model, Options, Loads);
	const StandingResult Elastic = SolveWith(Model, Options.Standing, Loads);
	INFO(DescribeResult(Result) << "; elastic alone " << Elastic.State);
	CHECK(Model.LiveBodyCount() == 6u);
	CHECK(Elastic.State == StandingState::Mechanism);
	CHECK(Result.Worst == ComponentVerdict::Undecided);
	CHECK(Result.Disagreements == 1);
	CHECK(Result.Components.size() == 1u);
	for (const ComponentReport& Report : Result.Components)
	{
		CHECK(Report.Elastic == StandingState::Mechanism);
		CHECK(Report.Kinematic == CertificateState::Stands);
		CHECK(Report.bDisagreed);
		CHECK(Report.Release.empty());
	}
}

TEST_CASE("Statics.Combined.ReportsEachComponentAndTheWorstVerdict")
{
	const double Friction = 0.7;
	const double PushOverWeight = 1.0;
	CHECK(PushOverWeight > Friction * CircumscribedOverInscribed(16));
	CHECK(PushOverWeight < BrickLengthM / BrickHeightM);
	Structure Model;
	const BodyId Ground = AddGroundPlane(Model);
	const ProfileId Crushed = AddProfileOk(Model, CrushedUnderOwnWeight());
	const ProfileId Dry = AddProfileOk(Model, DryJoint(Friction));
	const BodyId CrushedBrick = AddBox(Model, BrickShape(0.0));
	const InterfaceId CrushedBed = AddInterfaceOk(Model, OnGroundPlane(Ground, CrushedBrick, BrickShape(0.0), Crushed));
	const BodyId PushedBrick = AddBox(Model, BrickShape(1.0));
	const InterfaceId PushedBed = AddInterfaceOk(Model, OnGroundPlane(Ground, PushedBrick, BrickShape(1.0), Dry));
	const std::vector<BodyLoad> Loads{BodyLoad{PushedBrick, Vec3{PushOverWeight * BrickMassKg * GravityMPerS2, 0.0, 0.0}, BrickShape(1.0).CentreM}};
	const AuthorityResult Result = JudgeWith(Model, AuthorityOptions{}, Loads);
	INFO(DescribeResult(Result));
	CHECK(Result.Worst == ComponentVerdict::Collapses);
	CHECK(Result.Disagreements == 0);
	CHECK(HasReading(Result, CrushedBed));
	CHECK(HasReading(Result, PushedBed));
	CHECK(Result.Components.size() == 2u);
	if (Result.Components.size() != 2u)
	{
		return;
	}
	const ComponentReport& First = Result.Components[0];
	const ComponentReport& Second = Result.Components[1];
	CHECK(SameIds(First.Bodies, {CrushedBrick}));
	CHECK(First.Elastic == StandingState::Overstressed);
	CHECK(First.Verdict == ComponentVerdict::Overstressed);
	CHECK_FALSE(First.bConsultedCertificate);
	CHECK(First.Release.empty());
	CHECK(SameIds(Second.Bodies, {PushedBrick}));
	CHECK(Second.bConsultedCertificate);
	CHECK(Second.Verdict == ComponentVerdict::Collapses);
	CHECK(SameIds(Second.Release, {PushedBrick}));
	CHECK(SameIds(Second.Hinging, {PushedBed}));
}

TEST_CASE("Statics.Combined.CollapseReleaseIsTheCertificateList")
{
	const double Friction = 0.6;
	const double TiltRad = 35.0 * RadPerDegree;
	CHECK(std::tan(TiltRad) > Friction * CircumscribedOverInscribed(16));
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
	CHECK(SameIds(Only.Hinging, {F.Bed}));
	CHECK(std::abs(Only.LoadFactorLower) <= DryLoadFactorAbsolute);
	CHECK(std::abs(Only.LoadFactorUpper) <= DryLoadFactorAbsolute);
	CHECK(Only.Mechanism.size() == 1u);
	const MechanismMotion* Motion = FindMotion(Only, F.Brick);
	CHECK(Motion != nullptr);
	if (Motion == nullptr)
	{
		return;
	}
	CheckAssociativeSlideAlong(*Motion, PlusX, PlusZ, ShapeTolerance);
}

TEST_CASE("Statics.Combined.UndecidedComponentIsRefereed")
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
	CHECK(Only.bConsultedCertificate);
	CHECK(Only.Kinematic == CertificateState::Stands);
	CHECK(Only.Verdict == MappedKinematic(Only.Kinematic));
	CHECK_FALSE(Only.bDisagreed);
	CHECK(Result.Disagreements == 0);
}

TEST_CASE("Statics.Combined.GravityMismatchIsInvalidInput")
{
	const BrickOnGround F = MakeBrickOnGround(DryJoint(0.7));
	AuthorityOptions Options;
	Options.Standing = TiltedPlane(0.1);
	const AuthorityResult Result = JudgeWith(F.Model, Options);
	INFO(DescribeResult(Result));
	CHECK(Result.Worst == ComponentVerdict::InvalidInput);
	CHECK(Result.Components.empty());
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
	const ProfileId Crushed = AddProfileOk(Model, CrushedUnderOwnWeight());
	const ProfileId Dry = AddProfileOk(Model, DryJoint(0.6));
	const BodyId CrushedBrick = AddBox(Model, BrickShape(0.0));
	AddInterfaceOk(Model, OnGroundPlane(Ground, CrushedBrick, BrickShape(0.0), Crushed));
	const BodyId CappedBrick = AddBox(Model, BrickShape(1.0));
	AddInterfaceOk(Model, OnGroundPlane(Ground, CappedBrick, BrickShape(1.0), Dry));
	AuthorityOptions Options = Tilted(TiltRad);
	Options.Standing.MaxActiveSetIterations = 1;
	const AuthorityResult Result = JudgeWith(Model, Options);
	INFO(DescribeResult(Result));
	CHECK(Result.Components.size() == 2u);
	if (Result.Components.size() != 2u)
	{
		return;
	}
	const ComponentReport& First = Result.Components[0];
	const ComponentReport& Second = Result.Components[1];
	CHECK(SameIds(First.Bodies, {CrushedBrick}));
	CHECK(First.Elastic == StandingState::Overstressed);
	CHECK(First.Verdict == ComponentVerdict::Overstressed);
	CHECK_FALSE(First.bConsultedCertificate);
	CHECK(First.Release.empty());
	CHECK(SameIds(Second.Bodies, {CappedBrick}));
	CHECK(Second.Elastic == StandingState::Undecided);
	CHECK(Second.bConsultedCertificate);
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

TEST_CASE("Statics.Combined.InvalidCertificateOptionsAreInvalidInput")
{
	const BrickOnGround F = MakeBrickOnGround(DryJoint(0.7));
	AuthorityOptions Options;
	Options.Certificate.ConeGenerators = 3;
	const AuthorityResult Result = JudgeWith(F.Model, Options);
	INFO(DescribeResult(Result));
	CHECK(Result.Worst == ComponentVerdict::InvalidInput);
	CHECK(Result.Components.empty());
	CHECK(Result.Readings.empty());
}

TEST_CASE("Statics.Combined.ElasticMechanismWithCertificateUndecidedReleasesNothing")
{
	const double Friction = 0.6;
	const double Lambda = 0.7;
	CHECK(Lambda > Friction);
	CHECK(Lambda < Friction * CircumscribedOverInscribed(4));
	CHECK(Lambda < BrickLengthM / BrickHeightM);
	const BrickOnGround F = MakeBrickOnGround(DryJoint(Friction));
	AuthorityOptions Options = Biaxial(Lambda, 0.0);
	Options.Certificate.ConeGenerators = 4;
	const StandingResult Elastic = SolveWith(F.Model, Options.Standing);
	const CertificateResult Kinematic = DecideWith(F.Model, Options.Certificate);
	const AuthorityResult Result = JudgeWith(F.Model, Options);
	INFO("elastic " << Elastic.State << ", certificate " << Kinematic.State << "; " << DescribeResult(Result));
	CHECK(Elastic.State == StandingState::Mechanism);
	CHECK(Kinematic.State == CertificateState::Undecided);
	CHECK(Result.Components.size() == 1u);
	if (Result.Components.size() != 1u)
	{
		return;
	}
	const ComponentReport& Only = Result.Components.front();
	CHECK(Only.Verdict == ComponentVerdict::Undecided);
	CHECK(Only.bConsultedCertificate);
	CHECK(Only.Kinematic == CertificateState::Undecided);
	CHECK(Only.Release.empty());
	CHECK(Only.Mechanism.empty());
	CHECK(Only.Hinging.empty());
	CHECK_FALSE(Only.bDisagreed);
	CHECK(Result.Disagreements == 0);
}

TEST_CASE("Statics.Combined.ElasticUndecidedWithCertificateUndecidedReleasesNothing")
{
	const double Friction = 0.6;
	const double Lambda = 0.7;
	CHECK(Lambda > Friction);
	CHECK(Lambda < Friction * CircumscribedOverInscribed(4));
	CHECK(Lambda < BrickLengthM / BrickHeightM);
	const BrickOnGround F = MakeBrickOnGround(DryJoint(Friction));
	AuthorityOptions Options = Biaxial(Lambda, 0.0);
	Options.Certificate.ConeGenerators = 4;
	Options.Standing.MaxActiveSetIterations = 1;
	const StandingResult Elastic = SolveWith(F.Model, Options.Standing);
	const CertificateResult Kinematic = DecideWith(F.Model, Options.Certificate);
	const AuthorityResult Result = JudgeWith(F.Model, Options);
	INFO("elastic " << Elastic.State << ", certificate " << Kinematic.State << "; " << DescribeResult(Result));
	CHECK(Elastic.State == StandingState::Undecided);
	CHECK(Kinematic.State == CertificateState::Undecided);
	CHECK(Result.Worst == ComponentVerdict::Undecided);
	CHECK(Result.Components.size() == 1u);
	if (Result.Components.size() != 1u)
	{
		return;
	}
	const ComponentReport& Only = Result.Components.front();
	CHECK(Only.Elastic == StandingState::Undecided);
	CHECK(Only.Verdict == ComponentVerdict::Undecided);
	CHECK(Only.bConsultedCertificate);
	CHECK(Only.Kinematic == CertificateState::Undecided);
	CHECK(Only.Release.empty());
	CHECK(Only.Mechanism.empty());
	CHECK(Only.Hinging.empty());
	CHECK_FALSE(Only.bDisagreed);
	CHECK(Result.Disagreements == 0);
}

TEST_CASE("Statics.Combined.LoadFactorsAreNaNWhenNotConsulted")
{
	const BrickOnGround F = MakeBrickOnGround(DryJoint(0.7));
	const AuthorityResult Result = JudgeWith(F.Model, AuthorityOptions{});
	INFO(DescribeResult(Result));
	CHECK(Result.Components.size() == 1u);
	if (Result.Components.size() != 1u)
	{
		return;
	}
	const ComponentReport& Only = Result.Components.front();
	CHECK_FALSE(Only.bConsultedCertificate);
	CHECK(std::isnan(Only.LoadFactorLower));
	CHECK(std::isnan(Only.LoadFactorUpper));
}

TEST_CASE("Deferred.Slice7.ItascaWedgeSlidesBelowThePhysicalCriticalFrictionAngle")
{
	const double FrictionAngleRad = 32.5 * RadPerDegree;
	CHECK(FrictionAngleRad < ClassicalWedgeFrictionAngleRad(ItascaWedgeGeometry()));
	CHECK(FrictionAngleRad > AssociativeWedgeFrictionAngleRad(ItascaWedgeGeometry()));
	const Wedge W = MakeItascaWedge(FrictionAngleRad);
	const AuthorityResult Result = JudgeWith(W.Model, AuthorityOptions{});
	INFO(DescribeResult(Result));
	CHECK(Result.Worst == ComponentVerdict::Collapses);
	CHECK(Result.Components.size() == 1u);
	if (Result.Components.size() != 1u)
	{
		return;
	}
	CHECK(SameIds(Result.Components.front().Release, {W.Body}));
}
