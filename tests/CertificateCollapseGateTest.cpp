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
constexpr double OptimumAboveOne = 3.0e-5;

struct SlidingBlock
{
	Structure Model;
	CertificateOptions Options;
	double OuterOptimum = 0.0;
};

SlidingBlock CohesiveBlockJustAboveOne()
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
	S.OuterOptimum = 1.0 + OptimumAboveOne;
	const double Lambda = CohesionPa * AreaM2 * CircumscribedOverInscribed(16) / (WeightN * S.OuterOptimum);
	S.Options = CertificateLateral(Lambda);
	S.Options.FeasibilityTolerance = LooseTolerance;
	return S;
}
}

TEST_CASE("Certificate.CollapseGate.ConvergedOuterBelowOneWithDualBoundAboveOneIsUndecided")
{
	const SlidingBlock S = CohesiveBlockJustAboveOne();
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
