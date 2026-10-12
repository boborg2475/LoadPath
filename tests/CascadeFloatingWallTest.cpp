#include "CascadeTestSupport.h"
#include "CatalogueTestSupport.h"

#include "loadpath/Catalogue.h"
#include "loadpath/geometry/RunningBondWall.h"

#include "doctest.h"

#include <algorithm>
#include <chrono>
#include <span>
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

void CheckReleasedAsOneCluster(Structure& Model, const std::vector<BodyId>& Floating, std::span<const BodyLoad> Loads = {})
{
	ReportLayers(Model);
	const std::chrono::steady_clock::time_point Start = std::chrono::steady_clock::now();
	const CascadeVerdict V = MakeCascade(MakeAuthority())->Run(Model, Loads, CascadeOptions{});
	const double Ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Start).count();
	MESSAGE("cascade " << Ms << " ms: " << DescribeVerdict(V));
	INFO(DescribeVerdict(V));
	CHECK(V.State == CascadeState::Stands);
	CHECK(V.FirstPass == 1u);
	CHECK(V.Passes == 1);
	REQUIRE(V.Clusters.size() == 1u);
	if (V.Clusters.size() != 1u)
	{
		return;
	}
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

// A load with zero force adds nothing, so the floating wall falls exactly as it does unloaded.
TEST_CASE("Cascade.FloatingWall.AZeroForceLoadDoesNotStopTheRigidFall")
{
	const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
	REQUIRE(Loaded.Ok());
	FloatingWall W;
	SUBCASE("bonded ec6 4 x 4")
	{
		BuildWithoutBase(W, JointOrFail(Loaded.Value, "clay_gp_mortar_bed_ec6").Profile, JointOrFail(Loaded.Value, "clay_gp_mortar_perpend_ec6").Profile, 4, 4);
	}
	SUBCASE("dry 4 x 4")
	{
		BuildWithoutBase(W, DryJoint(0.7), DryJoint(0.7), 4, 4);
	}
	const BodyId Top = W.Value.Courses.back().front();
	const BodyLoad Loads[] = {BodyLoad{Top, Vec3{0.0, 0.0, 0.0}, W.Model.Body(Top).CentroidM}};
	CheckReleasedAsOneCluster(W.Model, LiveFreeBodies(W.Model, W.Value.Courses), Loads);
}

TEST_CASE("Slow.Cascade.FloatingWall.DryTenByEightWithoutItsBaseCourseReleasesAsOneCluster")
{
	FloatingWall W;
	BuildWithoutBase(W, DryJoint(0.7), DryJoint(0.7), 10, 8);
	CheckReleasedAsOneCluster(W.Model, LiveFreeBodies(W.Model, W.Value.Courses));
}

namespace
{
struct FloatingPair
{
	Structure Model;
	BodyId Lower, Upper;
	Box LowerShape, UpperShape;
	InterfaceId Joint;
};

FloatingPair MakeFloatingPair()
{
	FloatingPair P;
	const ProfileId Profile = AddProfileOk(P.Model, DryJoint(0.7));
	AddGroundPlane(P.Model);
	P.LowerShape = BrickShape(0.0, 1);
	P.UpperShape = BrickShape(0.0, 2);
	P.Lower = AddBox(P.Model, P.LowerShape);
	P.Upper = AddBox(P.Model, P.UpperShape);
	P.Joint = AddInterfaceOk(P.Model, *StackedFaces(P.Lower, P.LowerShape, P.Upper, P.UpperShape, Profile));
	return P;
}

bool OneClusterHoldsBoth(const CascadeVerdict& V, BodyId A, BodyId B)
{
	for (const ReleasedCluster& C : V.Clusters)
	{
		if (HasBody(C.Bodies, A) && HasBody(C.Bodies, B))
		{
			return true;
		}
	}
	return false;
}

void CheckUpperBrickLeavesAlone(const Vec3& ForceN)
{
	FloatingPair P = MakeFloatingPair();
	const BodyLoad Loads[] = {BodyLoad{P.Upper, ForceN, P.UpperShape.CentreM}};
	const CertificateResult Cert = DecideWith(P.Model, CertificateOptions{}, Loads);
	INFO("certificate: " << doctest::String(CertificateStateName(Cert.State)) << ", t [" << Cert.LoadFactorLower << ", " << Cert.LoadFactorUpper << "], unconstrained " << Describe(Cert.UnconstrainedBodies) << ", hinging " << Describe(Cert.HingingInterfaces));
	CHECK(Cert.State == CertificateState::Collapses);
	CHECK(std::any_of(Cert.HingingInterfaces.begin(), Cert.HingingInterfaces.end(), [&P](const InterfaceId& Id) { return SameHandle(Id, P.Joint); }));
	const CascadeVerdict V = MakeCascade(MakeAuthority())->Run(P.Model, Loads, CascadeOptions{});
	INFO(DescribeVerdict(V));
	CHECK_FALSE(OneClusterHoldsBoth(V, P.Lower, P.Upper));
	CHECK_FALSE(P.Model.IsLive(P.Upper));
}
}

TEST_CASE("Cascade.FloatingWall.AnUpliftedUpperBrickLiftsOffItsFloatingDryPartner")
{
	CheckUpperBrickLeavesAlone(Vec3{0.0, 0.0, 100.0});
}

TEST_CASE("Cascade.FloatingWall.ALateralPushSlidesTheUpperBrickOffItsFallingDryPartner")
{
	CheckUpperBrickLeavesAlone(Vec3{30.0, 0.0, 0.0});
}

TEST_CASE("Certificate.Floating.AWeightlessPairSqueezedByLoadsWithARoundingResidueStands")
{
	CertificateOptions Weightless;
	Weightless.Gravity = Vec3{0.0, 0.0, 0.0};
	SUBCASE("an exact split")
	{
		FloatingPair P = MakeFloatingPair();
		const BodyLoad Loads[] = {BodyLoad{P.Lower, Vec3{0.0, 0.0, 0.3}, P.LowerShape.CentreM}, BodyLoad{P.Upper, Vec3{0.0, 0.0, -0.3}, P.UpperShape.CentreM}};
		const CertificateResult Cert = DecideWith(P.Model, Weightless, Loads);
		INFO("state " << Cert.State << ", t [" << Cert.LoadFactorLower << ", " << Cert.LoadFactorUpper << "]");
		CHECK(Cert.State == CertificateState::Stands);
	}
	SUBCASE("0.1 + 0.2 against 0.3")
	{
		CHECK(0.1 + 0.2 != 0.3);
		FloatingPair P = MakeFloatingPair();
		const BodyLoad Loads[] = {BodyLoad{P.Lower, Vec3{0.0, 0.0, 0.1}, P.LowerShape.CentreM}, BodyLoad{P.Lower, Vec3{0.0, 0.0, 0.2}, P.LowerShape.CentreM}, BodyLoad{P.Upper, Vec3{0.0, 0.0, -0.3}, P.UpperShape.CentreM}};
		const CertificateResult Cert = DecideWith(P.Model, Weightless, Loads);
		INFO("state " << Cert.State << ", t [" << Cert.LoadFactorLower << ", " << Cert.LoadFactorUpper << "]");
		CHECK(Cert.State == CertificateState::Stands);
	}
}

TEST_CASE("Certificate.Floating.AFloatingPairUnderGravityFallsStraightDownWithUnitSpeed")
{
	FloatingPair P = MakeFloatingPair();
	const CertificateResult Cert = DecideWith(P.Model, CertificateOptions{});
	INFO("state " << Cert.State << ", mechanism bodies " << Cert.Mechanism.size());
	REQUIRE(Cert.State == CertificateState::Collapses);
	REQUIRE(Cert.Mechanism.size() == 2u);
	if (Cert.State != CertificateState::Collapses || Cert.Mechanism.size() != 2u)
	{
		return;
	}
	for (const MechanismMotion& M : Cert.Mechanism)
	{
		INFO("body " << M.Body.Index << ": translation (" << M.TranslationM.X << ", " << M.TranslationM.Y << ", " << M.TranslationM.Z << "), rotation (" << M.RotationRad.X << ", " << M.RotationRad.Y << ", " << M.RotationRad.Z << ")");
		CHECK(M.TranslationM.X == 0.0);
		CHECK(M.TranslationM.Y == 0.0);
		CHECK(M.TranslationM.Z == -1.0);
		CHECK(M.RotationRad.X == 0.0);
		CHECK(M.RotationRad.Y == 0.0);
		CHECK(M.RotationRad.Z == 0.0);
	}
	const CascadeVerdict V = MakeCascade(MakeAuthority())->Run(P.Model, {}, CascadeOptions{});
	INFO(DescribeVerdict(V));
	REQUIRE(V.Clusters.size() == 1u);
	if (V.Clusters.size() != 1u)
	{
		return;
	}
	const MechanismMotion& M = V.Clusters[0].Motion;
	CHECK(SameHandle(M.Body, P.Lower));
	CHECK(M.TranslationM.X == 0.0);
	CHECK(M.TranslationM.Y == 0.0);
	CHECK(M.TranslationM.Z == -1.0);
	CHECK(M.RotationRad.X == 0.0);
	CHECK(M.RotationRad.Y == 0.0);
	CHECK(M.RotationRad.Z == 0.0);
}

TEST_CASE("Slow.Cascade.FloatingWall.BondedTenByEightWithoutItsBaseCourseReleasesAsOneCluster")
{
	const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
	REQUIRE(Loaded.Ok());
	FloatingWall W;
	BuildWithoutBase(W, JointOrFail(Loaded.Value, "clay_gp_mortar_bed_ec6").Profile, JointOrFail(Loaded.Value, "clay_gp_mortar_perpend_ec6").Profile, 10, 8);
	CheckReleasedAsOneCluster(W.Model, LiveFreeBodies(W.Model, W.Value.Courses));
}

TEST_CASE("Deferred.Slice6.BondedWallHangingFromADryCeilingContactReleasesAsOneCluster")
{
	const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
	REQUIRE(Loaded.Ok());
	FloatingWall W;
	BuildWithoutBase(W, JointOrFail(Loaded.Value, "clay_gp_mortar_bed_ec6").Profile, JointOrFail(Loaded.Value, "clay_gp_mortar_perpend_ec6").Profile, 3, 8);
	const ProfileId Dry = AddProfileOk(W.Model, DryJoint(0.7));
	const geometry::WallSpec Spec;
	const Box CeilingShape{Vec3{0.5 * W.Value.LengthM, 0.0, W.Value.HeightM + 0.5 * Spec.BrickM.Z}, Vec3{0.5 * W.Value.LengthM, 0.5 * Spec.BrickM.Y, 0.5 * Spec.BrickM.Z}, 0.0};
	const BodyId Ceiling = AddBox(W.Model, CeilingShape, true);
	const double FullKg = Spec.DensityKgPerM3 * Spec.BrickM.X * Spec.BrickM.Y * Spec.BrickM.Z;
	for (const BodyId& Id : W.Value.Courses.back())
	{
		const BodyDesc& R = W.Model.Body(Id);
		const Box Shape{R.CentroidM, Vec3{0.5 * Spec.BrickM.X * R.MassKg / FullKg, 0.5 * Spec.BrickM.Y, 0.5 * Spec.BrickM.Z}, R.MassKg};
		AddInterfaceOk(W.Model, *StackedFaces(Id, Shape, Ceiling, CeilingShape, Dry));
	}
	const CertificateResult Cert = DecideWith(W.Model, CertificateOptions{});
	INFO("certificate: " << doctest::String(CertificateStateName(Cert.State)) << ", t [" << Cert.LoadFactorLower << ", " << Cert.LoadFactorUpper << "]");
	CHECK(Cert.State == CertificateState::Collapses);
	CheckReleasedAsOneCluster(W.Model, LiveFreeBodies(W.Model, W.Value.Courses));
}

TEST_CASE("Deferred.Slice6.DryPairHangingFromADryCeilingContactReleasesAsOneCluster")
{
	FloatingPair P = MakeFloatingPair();
	const ProfileId Dry = AddProfileOk(P.Model, DryJoint(0.7));
	const Box CeilingShape = BrickShape(0.0, 3);
	const BodyId Ceiling = AddBox(P.Model, Box{CeilingShape.CentreM, CeilingShape.HalfExtentsM, 0.0}, true);
	AddInterfaceOk(P.Model, *StackedFaces(P.Upper, P.UpperShape, Ceiling, CeilingShape, Dry));
	CheckReleasedAsOneCluster(P.Model, {P.Lower, P.Upper});
}
