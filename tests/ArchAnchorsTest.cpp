#include "ArchFixtures.h"
#include "CertificateTestSupport.h"
#include "SlowInDebug.h"
#include "StaticsTestSupport.h"

#include "loadpath/StructuralAuthority.h"

#include "doctest.h"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double ArchFriction = 0.7;
constexpr double StandingEta = 0.1095;
constexpr double FailingEta = 0.1065;
constexpr double ContinuousLeastEta = 0.107478;

bool ArchStands(double Eta, const CertificateOptions& Options = CertificateOptions{})
{
	const SemicircularArch Arch = MakeSemicircularArch(Eta, DryJoint(ArchFriction));
	return CertificateStands(Arch.Model, Options);
}

std::vector<double> HingeAnglesDeg(const SemicircularArch& Arch, const std::vector<InterfaceId>& Hinging)
{
	std::vector<double> Angles;
	for (std::size_t Joint = 0; Joint < Arch.Joints.size(); ++Joint)
	{
		if (HasInterface(Hinging, Arch.Joints[Joint]))
		{
			Angles.push_back(Arch.JointAnglesDeg[Joint]);
		}
	}
	return Angles;
}

std::string DescribeAngles(const std::vector<double>& Angles)
{
	std::ostringstream Out;
	for (const double Angle : Angles)
	{
		Out << Angle << " ";
	}
	return Out.str();
}

double PileThresholdLambda(const Structure& Model, bool bCollapseThreshold)
{
	const auto Holds = [&](double Lambda) {
		const CertificateState State = DecideWith(Model, CertificateLateral(Lambda)).State;
		return bCollapseThreshold ? State != CertificateState::Collapses : State == CertificateState::Stands;
	};
	const ThresholdBracket Bracket = Threshold(Holds, 0.0, 2.0, 1.0e-3);
	CHECK(Bracket.bBracketed);
	return 0.5 * (Bracket.LastStanding + Bracket.FirstFailing);
}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.Anchor.CoupletHeymanArchStandsAboveLeastThicknessAndCollapsesBelow")
{
	const auto Start = std::chrono::steady_clock::now();
	const SemicircularArch Thick = MakeSemicircularArch(StandingEta, DryJoint(ArchFriction));
	const CertificateResult Standing = DecideWith(Thick.Model, CertificateOptions{});
	const SemicircularArch Thin = MakeSemicircularArch(FailingEta, DryJoint(ArchFriction));
	const CertificateResult Failing = DecideWith(Thin.Model, CertificateOptions{});
	INFO("eta " << StandingEta << ": " << Standing.State << " [" << Standing.LoadFactorLower << ", " << Standing.LoadFactorUpper << "]; eta " << FailingEta << ": " << Failing.State << " [" << Failing.LoadFactorLower << ", " << Failing.LoadFactorUpper << "]");
	CHECK(Standing.State == CertificateState::Stands);
	CHECK(Standing.LoadFactorLower >= 1.0);
	CHECK(Failing.State == CertificateState::Collapses);
	CHECK(Failing.LoadFactorUpper < 1.0);

	const ThresholdBracket Bracket = Threshold([](double Eta) { return ArchStands(Eta); }, StandingEta, FailingEta, 1.0e-6);
	const double Seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - Start).count();
	INFO("eta bracket: last standing " << Bracket.LastStanding << ", first failing " << Bracket.FirstFailing << ", " << Seconds << " s");
	MESSAGE("arch eta bracket [" << Bracket.FirstFailing << ", " << Bracket.LastStanding << "] in " << Seconds << " s");
	CHECK(Bracket.bBracketed);
	CHECK(Bracket.LastStanding <= ContinuousLeastEta + 1.0e-5);
	CHECK(Bracket.FirstFailing >= ContinuousLeastEta - 1.0e-5);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.Anchor.CoupletHeymanArchHingesAtTheCrownTheHaunchesAndTheSpringings")
{
	const SemicircularArch Arch = MakeSemicircularArch(FailingEta, DryJoint(ArchFriction));
	const CertificateResult Result = DecideWith(Arch.Model, CertificateOptions{});
	const std::vector<double> Angles = HingeAnglesDeg(Arch, Result.HingingInterfaces);
	INFO("state " << Result.State << ", hinges at " << DescribeAngles(Angles) << "deg, moving " << Describe(Result.UnconstrainedBodies));
	CHECK(Result.State == CertificateState::Collapses);
	CHECK(Angles.size() == Result.HingingInterfaces.size());
	bool bCrown = false;
	bool bHaunch = false;
	for (const double Angle : Angles)
	{
		const double Magnitude = std::abs(Angle);
		const bool bHaunchJoint = Magnitude == 51.0 || Magnitude == 54.0 || Magnitude == ArchHaunchJointDeg || Magnitude == 57.0;
		INFO("hinge at " << Angle << " deg");
		CHECK((Angle == 0 || bHaunchJoint || Magnitude == 90));
		bCrown = bCrown || Angle == 0;
		bHaunch = bHaunch || bHaunchJoint;
	}
	CHECK(bCrown);
	CHECK(bHaunch);
	CHECK_FALSE(Result.UnconstrainedBodies.empty());
	for (const BodyId Moving : Result.UnconstrainedBodies)
	{
		CHECK(HasBody(Arch.Voussoirs, Moving));
	}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.Anchor.CoupletHeymanArchSlidesAtTheSpringingBelowTheFrictionSwitch")
{
	CertificateOptions Options;
	Options.ConeGenerators = 32;
	const SemicircularArch Sliding = MakeSemicircularArch(StandingEta, DryJoint(std::tan(21.0 * RadPerDegree)));
	const CertificateResult Slides = DecideWith(Sliding.Model, Options);
	const SemicircularArch Holding = MakeSemicircularArch(StandingEta, DryJoint(std::tan(22.0 * RadPerDegree)));
	const CertificateResult Holds = DecideWith(Holding.Model, Options);
	INFO("phi 21 deg: " << Slides.State << " [" << Slides.LoadFactorLower << ", " << Slides.LoadFactorUpper << "], hinges at " << DescribeAngles(HingeAnglesDeg(Sliding, Slides.HingingInterfaces)) << "deg; phi 22 deg: " << Holds.State << " [" << Holds.LoadFactorLower << ", " << Holds.LoadFactorUpper << "]");
	CHECK(Slides.State == CertificateState::Collapses);
	CHECK((HasInterface(Slides.HingingInterfaces, Sliding.Joints.front()) || HasInterface(Slides.HingingInterfaces, Sliding.Joints.back())));
	CHECK(Holds.State == CertificateState::Stands);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.Anchor.OrdunaHollowPileCollapseFactorIsInsideThePublishedBracket")
{
	const auto Start = std::chrono::steady_clock::now();
	const HollowPile Pile = MakeHollowPile(0.7);
	CHECK(Pile.Blocks.size() == 30u);
	const double StandsUpTo = PileThresholdLambda(Pile.Model, false);
	const double CollapsesFrom = PileThresholdLambda(Pile.Model, true);
	const double Seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - Start).count();
	INFO("certificate stands up to lambda " << StandsUpTo << ", collapses from " << CollapsesFrom << ", " << Seconds << " s");
	MESSAGE("pile lambda [" << StandsUpTo << ", " << CollapsesFrom << "] in " << Seconds << " s");
	CHECK(StandsUpTo >= 0.427);
	CHECK(CollapsesFrom <= 0.553);

	AuthorityOptions Options;
	Options.Standing = LateralLoad(0.4);
	Options.Certificate = CertificateLateral(0.4);
	const AuthorityResult Combined = MakeCombinedAuthority(MakeRbsmStandingAuthority(), MakeLimitAnalysisCertificate())->Judge(Pile.Model, {}, Options, nullptr);
	const StandingResult Elastic = SolveWith(Pile.Model, LateralLoad(0.4));
	INFO("lambda 0.4: elastic " << Elastic.State << ", combined worst " << static_cast<int>(Combined.Worst) << ", components " << Combined.Components.size());
	CHECK(Combined.Components.size() == 1u);
	CHECK(Combined.Worst == ComponentVerdict::Stands);
	CHECK(Elastic.State != StandingState::Mechanism);
}
