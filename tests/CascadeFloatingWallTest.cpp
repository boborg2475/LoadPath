#include "CascadeTestSupport.h"
#include "CatalogueTestSupport.h"

#include "loadpath/Catalogue.h"
#include "loadpath/geometry/RunningBondWall.h"

#include "doctest.h"

#include <algorithm>
#include <chrono>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
std::vector<BodyId> LiveFreeBodies(const Structure& Model, const std::vector<std::vector<BodyId>>& Courses)
{
	std::vector<BodyId> Out;
	for (const std::vector<BodyId>& Course : Courses)
	{
		for (const BodyId& Id : Course)
		{
			if (Model.IsLive(Id))
			{
				Out.push_back(Id);
			}
		}
	}
	std::sort(Out.begin(), Out.end(), [](const BodyId& A, const BodyId& B) { return A.Index < B.Index; });
	return Out;
}

void ReportLayers(const Structure& Model)
{
	const StandingResult Elastic = SolveWith(Model, StandingOptions{});
	MESSAGE("elastic: " << doctest::String(StandingStateName(Elastic.State)) << ", unconstrained " << Describe(Elastic.UnconstrainedBodies) << ", mechanism bodies " << Elastic.Mechanism.size());
	const CertificateResult Cert = DecideWith(Model, CertificateOptions{});
	MESSAGE("certificate: " << doctest::String(CertificateStateName(Cert.State)) << ", t [" << Cert.LoadFactorLower << ", " << Cert.LoadFactorUpper << "], unconstrained " << Describe(Cert.UnconstrainedBodies) << ", hinging " << Describe(Cert.HingingInterfaces) << ", mechanism bodies " << Cert.Mechanism.size());
	const AuthorityResult Combined = Judge(Model, AuthorityOptions{});
	MESSAGE("combined: " << DescribeAuthority(Combined) << ", disagreements " << Combined.Disagreements);
}

void CheckReleasedAsOneCluster(Structure& Model, const std::vector<BodyId>& Floating)
{
	ReportLayers(Model);
	const std::chrono::steady_clock::time_point Start = std::chrono::steady_clock::now();
	const CascadeVerdict V = MakeCascade(MakeAuthority())->Run(Model, {}, CascadeOptions{});
	const double Ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Start).count();
	MESSAGE("cascade " << Ms << " ms: " << DescribeVerdict(V));
	INFO(DescribeVerdict(V));
	CHECK(V.State == CascadeState::Stands);
	CHECK(V.FirstPass == 1u);
	CHECK(V.Passes == 1);
	REQUIRE(V.Clusters.size() == 1u);
	CHECK(V.Clusters[0].Pass == 1u);
	CHECK(SameIdList(V.Clusters[0].Bodies, Floating));
	CHECK(V.Failures.empty());
	for (const BodyId& Id : Floating)
	{
		CHECK_FALSE(Model.IsLive(Id));
	}
}

struct FloatingWall
{
	Structure Model;
	geometry::Wall Value;
};

void BuildWithoutBase(FloatingWall& W, const StrengthProfile& Bed, const StrengthProfile& Perpend, int Courses, int BricksPerCourse)
{
	geometry::WallSpec Spec;
	Spec.Bed = AddProfileOk(W.Model, Bed);
	Spec.Perpend = AddProfileOk(W.Model, Perpend);
	Spec.Courses = Courses;
	Spec.BricksPerCourse = BricksPerCourse;
	W.Value = geometry::BuildRunningBondWall(W.Model, Spec);
	REQUIRE(W.Value.bComplete);
	for (const BodyId& Id : W.Value.Courses[0])
	{
		REQUIRE(W.Model.RemoveBody(Id));
	}
}

struct Size
{
	int Courses, BricksPerCourse;
};

constexpr Size Sizes[] = {{2, 2}, {3, 2}, {3, 3}};
}

TEST_CASE("Cascade.FloatingWall.TwoStackedFloatingBricksReleaseAsOneCluster")
{
	Structure Model;
	const ProfileId Profile = AddProfileOk(Model, DryJoint(0.7));
	AddGroundPlane(Model);
	const Box Lower = BrickShape(0.0, 1);
	const Box Upper = BrickShape(0.0, 2);
	const BodyId A = AddBox(Model, Lower);
	const BodyId B = AddBox(Model, Upper);
	AddInterfaceOk(Model, *StackedFaces(A, Lower, B, Upper, Profile));
	CheckReleasedAsOneCluster(Model, {A, B});
}

TEST_CASE("Cascade.FloatingWall.DryWallWithoutItsBaseCourseReleasesAsOneCluster")
{
	for (const Size& S : Sizes)
	{
		CAPTURE(S.Courses);
		CAPTURE(S.BricksPerCourse);
		MESSAGE("dry " << S.Courses << " x " << S.BricksPerCourse);
		FloatingWall W;
		BuildWithoutBase(W, DryJoint(0.7), DryJoint(0.7), S.Courses, S.BricksPerCourse);
		CheckReleasedAsOneCluster(W.Model, LiveFreeBodies(W.Model, W.Value.Courses));
	}
}

TEST_CASE("Cascade.FloatingWall.BondedWallWithoutItsBaseCourseReleasesAsOneCluster")
{
	const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
	REQUIRE(Loaded.Ok());
	for (const Size& S : Sizes)
	{
		CAPTURE(S.Courses);
		CAPTURE(S.BricksPerCourse);
		MESSAGE("bonded " << S.Courses << " x " << S.BricksPerCourse);
		FloatingWall W;
		BuildWithoutBase(W, JointOrFail(Loaded.Value, "clay_gp_mortar_bed_ec6").Profile, JointOrFail(Loaded.Value, "clay_gp_mortar_perpend_ec6").Profile, S.Courses, S.BricksPerCourse);
		CheckReleasedAsOneCluster(W.Model, LiveFreeBodies(W.Model, W.Value.Courses));
	}
}

TEST_CASE("Slow.Cascade.FloatingWall.DryTenByEightWithoutItsBaseCourseReleasesAsOneCluster")
{
	FloatingWall W;
	BuildWithoutBase(W, DryJoint(0.7), DryJoint(0.7), 10, 8);
	CheckReleasedAsOneCluster(W.Model, LiveFreeBodies(W.Model, W.Value.Courses));
}
