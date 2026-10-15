#include "CascadeTestSupport.h"

#include "doctest.h"

#include <cmath>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double UpliftTensilePa = 1.0e5;
constexpr double UpliftCohesionPa = 1.0e5;
constexpr double UpliftFriction = 0.7;
constexpr double UpliftForceN = 100.0;

std::vector<BodyLoad> UpwardAtCentroid(BodyId Brick)
{
	return {BodyLoad{Brick, Vec3{0.0, 0.0, UpliftForceN}, BrickShape().CentreM}};
}
}

TEST_CASE("Statics.Cracked.BondedBrickUnderUpliftCollapsesOnceCracked")
{
	const double BedAreaM2 = BrickLengthM * BrickWidthM;
	CHECK(UpliftForceN > BrickMassKg * GravityMPerS2);
	CHECK(UpliftForceN < UpliftTensilePa * BedAreaM2);
	CHECK(UpliftCohesionPa >= UpliftFriction * UpliftTensilePa);
	BrickOnGround F = MakeBrickOnGround(BondedJoint(UpliftTensilePa, UpliftCohesionPa, UpliftFriction));
	const std::vector<BodyLoad> Loads = UpwardAtCentroid(F.Brick);
	{
		const StandingResult Elastic = MakeRbsmStandingAuthority()->Solve(F.Model, Loads, StandingOptions{}, {});
		const CertificateResult Kinematic = MakeLimitAnalysisCertificate()->Decide(F.Model, Loads, CertificateOptions{}, {});
		INFO("bonded: elastic " << doctest::String(StandingStateName(Elastic.State)) << ", certificate " << doctest::String(CertificateStateName(Kinematic.State)));
		CHECK(Elastic.State == StandingState::Stands);
		CHECK(Kinematic.State == CertificateState::Stands);
	}
	CHECK(F.Model.Crack(F.Bed, 1));
	const StandingResult Elastic = MakeRbsmStandingAuthority()->Solve(F.Model, Loads, StandingOptions{}, {});
	const CertificateResult Kinematic = MakeLimitAnalysisCertificate()->Decide(F.Model, Loads, CertificateOptions{}, {});
	const AuthorityResult Combined = Judge(F.Model, AuthorityOptions{}, Loads);
	INFO("cracked: elastic " << doctest::String(StandingStateName(Elastic.State)) << ", certificate " << doctest::String(CertificateStateName(Kinematic.State)) << " t [" << Kinematic.LoadFactorLower << ", " << Kinematic.LoadFactorUpper << "], unconstrained " << Describe(Kinematic.UnconstrainedBodies) << "; combined " << DescribeAuthority(Combined));
	CHECK((Elastic.State == StandingState::Mechanism || Elastic.State == StandingState::Undecided));
	CHECK(Kinematic.State == CertificateState::Collapses);
	CHECK(std::abs(Kinematic.LoadFactorLower) <= 1.0e-6);
	CHECK(SameIdList(Kinematic.UnconstrainedBodies, {F.Brick}));
	CHECK(Combined.Worst == ComponentVerdict::Collapses);
	CHECK(Combined.Components.size() == 1u);
	if (!(Combined.Components.size() == 1u))
	{
		return;
	}
	CHECK(SameIdList(Combined.Components.front().Release, {F.Brick}));
}

TEST_CASE("Statics.Cracked.CrackedBedKeepsFrictionOnATwentyDegreeTilt")
{
	const double TiltRad = 20.0 * RadPerDegree;
	CHECK(std::tan(TiltRad) < UpliftFriction);
	BrickOnGround F = MakeBrickOnGround(BondedJoint(UpliftTensilePa, UpliftCohesionPa, UpliftFriction));
	CHECK(F.Model.Crack(F.Bed, 1));
	CHECK(F.Model.Interface(F.Bed).State == InterfaceState::Cracked);
	const AuthorityOptions Options = AuthorityTilted(TiltRad);
	const StandingResult Elastic = MakeRbsmStandingAuthority()->Solve(F.Model, {}, Options.Standing, {});
	const CertificateResult Kinematic = MakeLimitAnalysisCertificate()->Decide(F.Model, {}, Options.Certificate, {});
	const AuthorityResult Combined = Judge(F.Model, Options);
	INFO("elastic " << doctest::String(StandingStateName(Elastic.State)) << ", certificate " << doctest::String(CertificateStateName(Kinematic.State)) << "; combined " << DescribeAuthority(Combined));
	CHECK(Elastic.State == StandingState::Stands);
	CHECK(Kinematic.State == CertificateState::Stands);
	CHECK(Combined.Worst == ComponentVerdict::Stands);
}

TEST_CASE("Statics.Cracked.CrackedBedKeepsTheCrushingBound")
{
	BrickOnGround F = MakeBrickOnGround(CrushedByOwnWeight());
	CHECK(F.Model.Crack(F.Bed, 1));
	CHECK(F.Model.Interface(F.Bed).State == InterfaceState::Cracked);
	const StandingResult Elastic = MakeRbsmStandingAuthority()->Solve(F.Model, {}, StandingOptions{}, {});
	const CertificateResult Kinematic = MakeLimitAnalysisCertificate()->Decide(F.Model, {}, CertificateOptions{}, {});
	const AuthorityResult Combined = Judge(F.Model, AuthorityOptions{});
	INFO("elastic " << doctest::String(StandingStateName(Elastic.State)) << ", certificate " << doctest::String(CertificateStateName(Kinematic.State)) << "; combined " << DescribeAuthority(Combined));
	CHECK(Elastic.State == StandingState::Overstressed);
	CHECK(Elastic.Readings.size() == 1u);
	if (!(Elastic.Readings.size() == 1u))
	{
		return;
	}
	CHECK(Elastic.Readings.front().Axis == GoverningAxis::Compression);
	CHECK(Kinematic.State == CertificateState::Collapses);
	CHECK(Combined.Worst == ComponentVerdict::Collapses);
	CHECK(Combined.Components.size() == 1u);
	if (!(Combined.Components.size() == 1u))
	{
		return;
	}
	CHECK(Combined.Components.front().bConsultedCertificate);
	CHECK(SameIdList(Combined.Components.front().Release, {F.Brick}));
}
