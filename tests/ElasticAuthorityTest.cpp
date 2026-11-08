#include "BrickFixtures.h"
#include "CascadeTestSupport.h"
#include "StaticsTestSupport.h"

#include "loadpath/Cascade.h"
#include "loadpath/StandingAuthority.h"
#include "loadpath/StructuralAuthority.h"

#include "doctest.h"

#include <cmath>
#include <cstddef>
#include <memory>
#include <span>
#include <sstream>
#include <string>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double MotionAbsolute = 1.0e-12;

struct FloatingPair
{
	Structure Model;
	BodyId Lower, Upper;
};

FloatingPair MakeFloatingPair()
{
	FloatingPair P;
	const ProfileId Profile = AddProfileOk(P.Model, DryJoint(0.7));
	AddGroundPlane(P.Model);
	const Box LowerShape = BrickShape(0.0, 1);
	const Box UpperShape = BrickShape(0.0, 2);
	P.Lower = AddBox(P.Model, LowerShape);
	P.Upper = AddBox(P.Model, UpperShape);
	AddInterfaceOk(P.Model, *StackedFaces(P.Lower, LowerShape, P.Upper, UpperShape, Profile));
	return P;
}

AuthorityOptions ElasticOnly(const StandingOptions& Standing)
{
	AuthorityOptions Options;
	Options.Standing = Standing;
	return Options;
}

AuthorityResult JudgeElastic(const Structure& Model, const AuthorityOptions& Options, const AuthorityResult* Previous = nullptr)
{
	return MakeElasticAuthority(MakeRbsmStandingAuthority())->Judge(Model, {}, Options, Previous);
}

CascadeVerdict RunElastic(Structure& Model, const AuthorityOptions& Options)
{
	CascadeOptions Cascade;
	Cascade.Authority = Options;
	return MakeCascade(MakeElasticAuthority(MakeRbsmStandingAuthority()))->Run(Model, {}, Cascade, nullptr);
}

bool SameMotions(const std::vector<MechanismMotion>& Got, const std::vector<MechanismMotion>& Want)
{
	if (Got.size() != Want.size())
	{
		return false;
	}
	for (std::size_t I = 0; I < Got.size(); ++I)
	{
		const Vec3 DT = Minus(Got[I].TranslationM, Want[I].TranslationM);
		const Vec3 DR = Minus(Got[I].RotationRad, Want[I].RotationRad);
		if (!SameHandle(Got[I].Body, Want[I].Body) || Length(DT) > MotionAbsolute || Length(DR) > MotionAbsolute)
		{
			return false;
		}
	}
	return true;
}

std::vector<BodyId> MotionBodies(const std::vector<MechanismMotion>& Motions)
{
	std::vector<BodyId> Out;
	for (const MechanismMotion& Motion : Motions)
	{
		Out.push_back(Motion.Body);
	}
	return Out;
}

std::string DescribeReport(const ComponentReport& Report)
{
	std::ostringstream Out;
	Out << "bodies " << Describe(Report.Bodies) << ", verdict " << VerdictName(Report.Verdict) << ", elastic " << StandingStateName(Report.Elastic) << ", release " << Describe(Report.Release) << ", mechanism bodies " << Describe(MotionBodies(Report.Mechanism)) << ", reused " << Report.bReused;
	return Out.str();
}

std::string DescribeElastic(const AuthorityResult& Result)
{
	std::ostringstream Out;
	Out << "worst " << VerdictName(Result.Worst) << ", components " << Result.Components.size();
	for (const ComponentReport& Report : Result.Components)
	{
		Out << "; [" << DescribeReport(Report) << "]";
	}
	return Out.str();
}

void CheckReleasesTheElasticMechanism(Structure& Model, const AuthorityOptions& Options)
{
	const StandingResult Elastic = SolveWith(Model, Options.Standing);
	const AuthorityResult Result = JudgeElastic(Model, Options);
	INFO("elastic " << Elastic.State << ", unconstrained " << Describe(Elastic.UnconstrainedBodies) << "; " << DescribeElastic(Result));
	CHECK(Elastic.State == StandingState::Mechanism);
	if (!(Elastic.State == StandingState::Mechanism))
	{
		return;
	}
	CHECK(Elastic.Components.size() == 1u);
	if (!(Elastic.Components.size() == 1u))
	{
		return;
	}
	CHECK_FALSE(Elastic.Mechanism.empty());
	if (Elastic.Mechanism.empty())
	{
		return;
	}
	CHECK(Result.Worst == ComponentVerdict::Collapses);
	CHECK(Result.Components.size() == 1u);
	if (!(Result.Components.size() == 1u))
	{
		return;
	}
	const ComponentReport& Only = Result.Components.front();
	CHECK(Only.Elastic == StandingState::Mechanism);
	CHECK(Only.Verdict == ComponentVerdict::Collapses);
	CHECK(SameIdList(Only.Release, Elastic.UnconstrainedBodies));
	CHECK(SameMotions(Only.Mechanism, Elastic.Mechanism));

	const CascadeVerdict V = RunElastic(Model, Options);
	INFO(DescribeVerdict(V));
	CHECK(V.Clusters.size() == 1u);
	if (!(V.Clusters.size() == 1u))
	{
		return;
	}
	CHECK(SameIdList(V.Clusters.front().Bodies, Elastic.UnconstrainedBodies));
	CHECK(SameMotions({V.Clusters.front().Motion}, {Elastic.Mechanism.front()}));
}
}

TEST_CASE("Statics.Elastic.AGroundedMechanismCollapsesAndReleasesTheElasticBodies")
{
	BrickOnGround F = MakeBrickOnGround(DryJoint(0.6));
	CheckReleasesTheElasticMechanism(F.Model, ElasticOnly(LateralLoad(0.7)));
}

TEST_CASE("Statics.Elastic.AnUngroundedPairFallsAsOneClusterAlongGravity")
{
	const AuthorityOptions Options = ElasticOnly(StandingOptions{});
	const FloatingPair P = MakeFloatingPair();
	const AuthorityResult Result = JudgeElastic(P.Model, Options);
	INFO(DescribeElastic(Result));
	CHECK(Result.Components.size() == 1u);
	if (!(Result.Components.size() == 1u))
	{
		return;
	}
	CHECK(Result.Components.front().Verdict == ComponentVerdict::Collapses);
	CHECK(SameIdList(Result.Components.front().Release, {P.Lower, P.Upper}));
	CHECK(Result.Components.front().Mechanism.size() == 2u);
	for (const MechanismMotion& Motion : Result.Components.front().Mechanism)
	{
		CHECK(Length(Minus(Motion.TranslationM, Vec3{0.0, 0.0, -1.0})) <= MotionAbsolute);
		CHECK(Length(Motion.RotationRad) <= MotionAbsolute);
	}
	FloatingPair Cascaded = MakeFloatingPair();
	CheckReleasesTheElasticMechanism(Cascaded.Model, Options);
}

TEST_CASE("Statics.Elastic.AnUndecidedElasticSolveReleasesNothing")
{
	BrickOnGround F = MakeBrickOnGround(DryJoint(0.6));
	StandingOptions Standing = LateralLoad(0.7);
	Standing.MaxActiveSetIterations = 1;
	const AuthorityOptions Options = ElasticOnly(Standing);
	const StandingResult Elastic = SolveWith(F.Model, Standing);
	const AuthorityResult Result = JudgeElastic(F.Model, Options);
	INFO("elastic " << Elastic.State << "; " << DescribeElastic(Result));
	CHECK(Elastic.State == StandingState::Undecided);
	if (!(Elastic.State == StandingState::Undecided))
	{
		return;
	}
	CHECK(Result.Worst == ComponentVerdict::Undecided);
	CHECK(Result.Components.size() == 1u);
	if (!(Result.Components.size() == 1u))
	{
		return;
	}
	const ComponentReport& Only = Result.Components.front();
	CHECK(Only.Elastic == StandingState::Undecided);
	CHECK(Only.Verdict == ComponentVerdict::Undecided);
	CHECK(Only.Release.empty());
	CHECK(Only.Mechanism.empty());

	CascadeOptions Kept;
	Kept.Authority = Options;
	Kept.OnUndecided = UndecidedResolution::Stands;
	const CascadeVerdict V = MakeCascade(MakeElasticAuthority(MakeRbsmStandingAuthority()))->Run(F.Model, {}, Kept, nullptr);
	INFO(DescribeVerdict(V));
	CHECK(V.State == CascadeState::Stands);
	CHECK(V.Clusters.empty());
	CHECK(F.Model.IsLive(F.Brick));
}

TEST_CASE("Statics.Elastic.StandsAndOverstressedPassThrough")
{
	struct Row
	{
		const char* Name;
		StrengthProfile Profile;
		StandingOptions Standing;
		StandingState Elastic;
		ComponentVerdict Verdict;
	};
	const Row Rows[] = {
		{"dry brick at rest", DryJoint(0.7), StandingOptions{}, StandingState::Stands, ComponentVerdict::Stands},
		{"brick crushed on a 29 degree tilt", CrushedByOwnWeight(), TiltedPlane(29.0 * RadPerDegree), StandingState::Overstressed, ComponentVerdict::Overstressed},
	};
	for (const Row& R : Rows)
	{
		INFO("row " << std::string(R.Name));
		const BrickOnGround F = MakeBrickOnGround(R.Profile);
		const AuthorityResult Result = JudgeElastic(F.Model, ElasticOnly(R.Standing));
		INFO(DescribeElastic(Result));
		CHECK(Result.Worst == R.Verdict);
		CHECK(Result.Components.size() == 1u);
		if (!(Result.Components.size() == 1u))
		{
			continue;
		}
		const ComponentReport& Only = Result.Components.front();
		CHECK(Only.Elastic == R.Elastic);
		CHECK(Only.Verdict == R.Verdict);
		CHECK(Only.Release.empty());
	}
}

TEST_CASE("Statics.Elastic.AnUnchangedMechanismIsReusedWithItsRelease")
{
	const BrickOnGround F = MakeBrickOnGround(DryJoint(0.6));
	const AuthorityOptions Options = ElasticOnly(LateralLoad(0.7));
	const AuthorityResult First = JudgeElastic(F.Model, Options);
	const AuthorityResult Second = JudgeElastic(F.Model, Options, &First);
	INFO("first " << DescribeElastic(First) << "; second " << DescribeElastic(Second));
	CHECK(Second.Components.size() == 1u);
	if (!(Second.Components.size() == 1u))
	{
		return;
	}
	const ComponentReport& Only = Second.Components.front();
	CHECK(Only.bReused);
	CHECK(Only.Verdict == ComponentVerdict::Collapses);
	CHECK(SameIdList(Only.Release, {F.Brick}));
}
