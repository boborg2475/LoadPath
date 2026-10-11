#include "BrickFixtures.h"
#include "CertificateTestSupport.h"
#include "StaticsTestSupport.h"

#include "loadpath/StructuralAuthority.h"

#include "doctest.h"

#include <cmath>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double CapFriction = 5.0;
constexpr double CapOverWeight = 0.3;
constexpr double LoadFactorRelative = 1.0e-6;

double BrickWeightN()
{
	return BrickMassKg * GravityMPerS2;
}

double BedAreaM2()
{
	return BrickLengthM * BrickWidthM;
}

StrengthProfile CappedDryJoint(double CapTimesAreaOverWeight)
{
	StrengthProfile Profile = DryJoint(CapFriction);
	Profile.MaxShearStrengthPa = CapTimesAreaOverWeight * BrickWeightN() / BedAreaM2();
	return Profile;
}

double InscribedFraction()
{
	return std::cos(Pi / 16.0);
}
}

TEST_CASE("Certificate.ShearCap.LateralLoadSlidesAtCapTimesAreaOverWeight")
{
	CHECK(CapOverWeight < CapFriction);
	CHECK(CapOverWeight < BrickLengthM / BrickHeightM);
	const BrickOnGround F = MakeBrickOnGround(CappedDryJoint(CapOverWeight));
	struct Case
	{
		double Lambda;
		CertificateState Expected;
	};
	const Case Cases[] = {{0.29, CertificateState::Stands}, {0.297, CertificateState::Undecided}, {0.31, CertificateState::Collapses}};
	CHECK(0.29 < CapOverWeight * InscribedFraction());
	CHECK(0.297 > CapOverWeight * InscribedFraction());
	for (const Case& Each : Cases)
	{
		const CertificateResult Result = DecideWith(F.Model, CertificateLateral(Each.Lambda));
		INFO("lambda " << Each.Lambda << ": state " << Result.State << ", load factors [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "], expected " << Each.Expected);
		CHECK(Result.State == Each.Expected);
	}
	const CertificateResult Unit = DecideWith(F.Model, CertificateLateral(1.0));
	INFO("lambda 1: state " << Unit.State);
	CheckRelative(Unit.LoadFactorLower, CapOverWeight * InscribedFraction(), LoadFactorRelative, "inscribed load factor at lambda 1");
	CheckRelative(Unit.LoadFactorUpper, CapOverWeight, LoadFactorRelative, "circumscribed load factor at lambda 1");
}

TEST_CASE("Certificate.ShearCap.TiltSlidesAtAsinCapTimesAreaOverWeight")
{
	const BrickOnGround F = MakeBrickOnGround(CappedDryJoint(CapOverWeight));
	const double StandingRad = std::asin(CapOverWeight * InscribedFraction()) - 1.0e-3;
	const double FailingRad = std::asin(CapOverWeight) + 1.0e-3;
	CHECK(std::tan(FailingRad) < CapFriction);
	const CertificateResult Standing = DecideWith(F.Model, CertificateTilt(StandingRad));
	const CertificateResult Failing = DecideWith(F.Model, CertificateTilt(FailingRad));
	INFO("tilt " << StandingRad / RadPerDegree << " deg: " << Standing.State << " [" << Standing.LoadFactorLower << ", " << Standing.LoadFactorUpper << "]; tilt " << FailingRad / RadPerDegree << " deg: " << Failing.State << " [" << Failing.LoadFactorLower << ", " << Failing.LoadFactorUpper << "]");
	CHECK(Standing.State == CertificateState::Stands);
	CHECK(Failing.State == CertificateState::Collapses);
}

TEST_CASE("Certificate.ShearCap.BoundsShearOnlySoABrickAtRestStands")
{
	const BrickOnGround F = MakeBrickOnGround(CappedDryJoint(0.01));
	const CertificateOptions Options;
	const CertificateResult Result = DecideWith(F.Model, Options);
	INFO("state " << Result.State << ", load factors [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "]");
	CHECK(Result.State == CertificateState::Stands);
	CheckRelative(Result.LoadFactorLower, Options.LoadFactorCap, LoadFactorRelative, "inscribed load factor at rest");
}

TEST_CASE("Certificate.ShearCap.ZeroCapCarriesNoShear")
{
	StrengthProfile Profile = DryJoint(0.7);
	Profile.MaxShearStrengthPa = 0.0;
	const BrickOnGround F = MakeBrickOnGround(Profile);
	const Vec3 PlusX{1.0, 0.0, 0.0};
	const Vec3 PlusZ{0.0, 0.0, 1.0};

	const CertificateResult Pushed = DecideWith(F.Model, CertificateLateral(0.1));
	INFO("lambda 0.1: state " << Pushed.State << ", load factors [" << Pushed.LoadFactorLower << ", " << Pushed.LoadFactorUpper << "], moving " << Describe(Pushed.UnconstrainedBodies) << ", hinging " << Describe(Pushed.HingingInterfaces));
	CHECK(Pushed.State == CertificateState::Collapses);
	CHECK(std::abs(Pushed.LoadFactorLower) <= 1.0e-6);
	CHECK(std::abs(Pushed.LoadFactorUpper) <= 1.0e-6);
	CHECK(Pushed.UnconstrainedBodies.size() == 1u);
	CHECK(HasBody(Pushed.UnconstrainedBodies, F.Brick));
	CHECK(Pushed.HingingInterfaces.size() == 1u);
	CHECK(HasInterface(Pushed.HingingInterfaces, F.Bed));
	const MechanismMotion* Motion = FindMotion(Pushed, F.Brick);
	CHECK(Motion != nullptr);
	if (Motion != nullptr)
	{
		CheckTranslationAlong(*Motion, PlusX, 1.0e-6);
		CHECK(Dot(Motion->TranslationM, PlusZ) <= 1.0e-9 * Length(Motion->TranslationM));
	}

	AuthorityOptions Options;
	Options.Standing = LateralLoad(0.1);
	Options.Certificate = CertificateLateral(0.1);
	const AuthorityResult Combined = MakeCombinedAuthority(MakeRbsmStandingAuthority(), MakeLimitAnalysisCertificate())->Judge(F.Model, {}, Options);
	INFO("combined worst " << static_cast<int>(Combined.Worst) << ", components " << Combined.Components.size());
	CHECK(Combined.Worst == ComponentVerdict::Collapses);
	CHECK(Combined.Components.size() == 1u);
	if (Combined.Components.size() == 1u)
	{
		CHECK(Combined.Components.front().Release.size() == 1u);
		CHECK(HasBody(Combined.Components.front().Release, F.Brick));
	}

	const CertificateOptions AtRestOptions;
	const CertificateResult AtRest = DecideWith(F.Model, AtRestOptions);
	INFO("at rest: state " << AtRest.State << ", load factors [" << AtRest.LoadFactorLower << ", " << AtRest.LoadFactorUpper << "]");
	CHECK(AtRest.State == CertificateState::Stands);
	CheckRelative(AtRest.LoadFactorLower, AtRestOptions.LoadFactorCap, LoadFactorRelative, "inscribed load factor at rest");

	StrengthProfile Tiny = DryJoint(0.7);
	Tiny.MaxShearStrengthPa = 1.0e-9;
	const BrickOnGround T = MakeBrickOnGround(Tiny);
	const CertificateResult TinyPushed = DecideWith(T.Model, CertificateLateral(0.1));
	INFO("cap 1e-9 Pa at lambda 0.1: state " << TinyPushed.State << ", load factors [" << TinyPushed.LoadFactorLower << ", " << TinyPushed.LoadFactorUpper << "]");
	CHECK(TinyPushed.State != CertificateState::Stands);
}

namespace
{
constexpr double LiftExcessOverWeight = 0.25;

StrengthProfile ZeroCapTensileJoint()
{
	StrengthProfile Profile = DryJoint(0.7);
	Profile.TensileStrengthPa = 2.0 * LiftExcessOverWeight * BrickWeightN() / BedAreaM2();
	Profile.ShearCohesionPa = Profile.TensileStrengthPa;
	Profile.MaxShearStrengthPa = 0.0;
	return Profile;
}

StrengthProfile ZeroCapCrushedUnderOwnWeight()
{
	StrengthProfile Profile = DryJoint(0.7);
	Profile.TensileStrengthPa = 1.0e4;
	Profile.ShearCohesionPa = 1.0e4;
	Profile.CompressiveStrengthPa = 0.5 * BrickWeightN() / BedAreaM2();
	Profile.MaxShearStrengthPa = 0.0;
	return Profile;
}
}

TEST_CASE("Certificate.ShearCap.ZeroCapKeepsTensionAndCrushing")
{
	SUBCASE("lifted brick holds by tension alone")
	{
		const StrengthProfile Profile = ZeroCapTensileJoint();
		CHECK(Profile.ShearCohesionPa >= Profile.FrictionCoefficient * Profile.TensileStrengthPa);
		const BrickOnGround F = MakeBrickOnGround(Profile);
		const std::vector<BodyLoad> Loads{BodyLoad{F.Brick, Vec3{0.0, 0.0, (1.0 + LiftExcessOverWeight) * BrickWeightN()}, BrickShape().CentreM}};
		const CertificateResult Result = DecideWith(F.Model, CertificateOptions{}, Loads);
		const double Expected = Profile.TensileStrengthPa * BedAreaM2() / (LiftExcessOverWeight * BrickWeightN());
		INFO("state " << Result.State << ", load factors [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "], expected " << Expected);
		CHECK(Result.State == CertificateState::Stands);
		CheckRelative(Result.LoadFactorLower, Expected, LoadFactorRelative, "inscribed load factor under lift");
		CheckRelative(Result.LoadFactorUpper, Expected, LoadFactorRelative, "circumscribed load factor under lift");
	}
	SUBCASE("brick crushed under its own weight")
	{
		const StrengthProfile Profile = ZeroCapCrushedUnderOwnWeight();
		const BrickOnGround F = MakeBrickOnGround(Profile);
		const CertificateResult Result = DecideWith(F.Model, CertificateOptions{});
		const double Expected = Profile.CompressiveStrengthPa * BedAreaM2() / BrickWeightN();
		INFO("state " << Result.State << ", load factors [" << Result.LoadFactorLower << ", " << Result.LoadFactorUpper << "], expected " << Expected);
		CHECK(Result.State == CertificateState::Collapses);
		CheckRelative(Result.LoadFactorLower, Expected, LoadFactorRelative, "inscribed load factor at rest");
		CheckRelative(Result.LoadFactorUpper, Expected, LoadFactorRelative, "circumscribed load factor at rest");
	}
}
