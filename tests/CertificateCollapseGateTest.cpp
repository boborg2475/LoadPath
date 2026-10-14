#include "BrickFixtures.h"
#include "CertificateProbe.h"
#include "CertificateTestSupport.h"
#include "StaticsTestSupport.h"

#include "loadpath/CollapseCertificate.h"

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <iomanip>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double CohesionPa = 1.0e3;
constexpr double LooseTolerance = 1.0e-4;

struct SlidingBlock
{
	Structure Model;
	CertificateOptions Options;
	double OuterOptimum = 0.0;
};

SlidingBlock CohesiveBlockWithOuterOptimum(double OuterOptimum)
{
	SlidingBlock S;
	const Box Shape = BoxOfDensity(Vec3{0.0, 0.0, 0.05}, Vec3{0.5, 0.5, 0.05}, 2000.0);
	const double WeightN = Shape.MassKg * GravityMPerS2;
	const double AreaM2 = 1.0;
	StrengthProfile Profile = DryJoint(0.0);
	Profile.ShearCohesionPa = CohesionPa;
	const ProfileId P = AddProfileOk(S.Model, Profile);
	const BodyId Ground = AddGroundPlane(S.Model);
	const BodyId Block = AddBox(S.Model, Shape);
	AddInterfaceOk(S.Model, OnGroundPlane(Ground, Block, Shape, P));
	S.OuterOptimum = OuterOptimum;
	const double Lambda = CohesionPa * AreaM2 * CircumscribedOverInscribed(16) / (WeightN * S.OuterOptimum);
	S.Options = CertificateLateral(Lambda);
	S.Options.FeasibilityTolerance = LooseTolerance;
	return S;
}
}

TEST_CASE("Certificate.CollapseGate.ACapBelowOneIsInvalidInput")
{
	const Structure Model = MakeBrickOnGround(DryJoint(0.6)).Model;
	for (const double Cap : {0.5, 0.999, std::nextafter(1.0, 0.0)})
	{
		CertificateOptions Options;
		Options.LoadFactorCap = Cap;
		const CertificateResult R = DecideWith(Model, Options);
		INFO(std::setprecision(17) << "cap " << Cap << ": state " << R.State << " [" << R.LoadFactorLower << ", " << R.LoadFactorUpper << "]");
		CHECK(R.State == CertificateState::InvalidInput);
	}
	CertificateOptions AtOne;
	AtOne.LoadFactorCap = 1.0;
	const CertificateResult R = DecideWith(Model, AtOne);
	INFO("cap 1: state " << R.State << " [" << R.LoadFactorLower << ", " << R.LoadFactorUpper << "]");
	CHECK(R.State != CertificateState::InvalidInput);
	CHECK(R.State != CertificateState::Collapses);
}

TEST_CASE("Certificate.CollapseGate.ConvergedOuterBelowOneWithDualBoundAboveOneIsUndecided")
{
	const SlidingBlock AtOne = CohesiveBlockWithOuterOptimum(1.0);
	CertificateOptions TightAtOne = AtOne.Options;
	TightAtOne.FeasibilityTolerance = 1.0e-9;
	const detail::ProgramProbe TightProbe = detail::ProbeProgram(AtOne.Model, {}, TightAtOne, 0, false);
	const detail::ProgramProbe LooseProbe = detail::ProbeProgram(AtOne.Model, {}, AtOne.Options, 0, false);
	const double Gap = TightProbe.LoadFactor - LooseProbe.LoadFactor;
	INFO(std::setprecision(12) << "at t* = 1: tight t_out " << TightProbe.LoadFactor << ", loose t_out " << LooseProbe.LoadFactor << ", gap " << Gap);
	REQUIRE(TightProbe.bConverged);
	REQUIRE(LooseProbe.bConverged);
	REQUIRE(Gap > 0.0);
	const SlidingBlock S = CohesiveBlockWithOuterOptimum(1.0 + 0.5 * Gap);
	CertificateOptions Tight = S.Options;
	Tight.FeasibilityTolerance = 1.0e-9;
	const detail::ProgramProbe Reference = detail::ProbeProgram(S.Model, {}, Tight, 0, false);
	const detail::ProgramProbe Loose = detail::ProbeProgram(S.Model, {}, S.Options, 0, false);
	INFO(std::setprecision(12) << "analytic t*_out " << S.OuterOptimum << ", tight t_out " << Reference.LoadFactor << ", loose t_out " << Loose.LoadFactor << ", loose U_out " << Loose.DualBound << ", tolerance " << S.Options.FeasibilityTolerance);
	REQUIRE(Reference.bConverged);
	REQUIRE(std::abs(Reference.LoadFactor - S.OuterOptimum) <= 1.0e-7);
	REQUIRE(Loose.bConverged);
	REQUIRE(Loose.LoadFactor < 1.0);
	REQUIRE(Loose.DualBound >= 1.0);
	const CertificateResult R = DecideWith(S.Model, S.Options);
	INFO("state " << R.State << ", LoadFactorUpper " << R.LoadFactorUpper);
	CHECK(R.State != CertificateState::Collapses);
	CHECK(R.State == CertificateState::Undecided);
	CHECK(R.LoadFactorUpper >= 1.0);
	CHECK(R.LoadFactorUpper == doctest::Approx(std::min(S.Options.LoadFactorCap, Loose.DualBound)).epsilon(1.0e-12));
	CHECK(R.Mechanism.empty());
}
