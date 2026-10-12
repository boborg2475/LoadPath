#include "BrickFixtures.h"
#include "CascadeTestSupport.h"
#include "CatalogueTestSupport.h"
#include "EventRegionTestSupport.h"
#include "StaticsTestSupport.h"

#include "loadpath/Catalogue.h"
#include "loadpath/geometry/RunningBondWall.h"

#include "doctest.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <span>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
struct FloatingPair
{
	BodyId Lower, Upper;
	InterfaceId Joint;
};

FloatingPair AddFloatingPair(Structure& Model, ProfileId Profile, double CentreXM)
{
	FloatingPair P;
	const Box LowerShape = BrickShape(CentreXM, 1);
	const Box UpperShape = BrickShape(CentreXM, 2);
	P.Lower = AddBox(Model, LowerShape);
	P.Upper = AddBox(Model, UpperShape);
	P.Joint = AddInterfaceOk(Model, *StackedFaces(P.Lower, LowerShape, P.Upper, UpperShape, Profile));
	return P;
}

std::vector<BodyId> AddGroundedStack(Structure& Model, ProfileId Profile, BodyId Ground, int Count)
{
	std::vector<BodyId> Bricks;
	for (int Course = 0; Course < Count; ++Course)
	{
		Bricks.push_back(AddBox(Model, BrickShape(0.0, Course)));
	}
	AddInterfaceOk(Model, OnGroundPlane(Ground, Bricks[0], BrickShape(0.0, 0), Profile));
	for (int Course = 1; Course < Count; ++Course)
	{
		AddInterfaceOk(Model, *StackedFaces(Bricks[static_cast<std::size_t>(Course - 1)], BrickShape(0.0, Course - 1), Bricks[static_cast<std::size_t>(Course)], BrickShape(0.0, Course), Profile));
	}
	return Bricks;
}

void Characterise(const StandingResult& R)
{
	MESSAGE("state " << R.State << ", iterations " << R.ActiveSetIterations << ", unconstrained " << R.UnconstrainedBodies.size() << ", mechanism bodies " << R.Mechanism.size() << ", readings " << R.Readings.size() << ", components " << R.Components.size());
	for (const StandingComponent& C : R.Components)
	{
		MESSAGE("component of " << C.Bodies.size() << " bodies: " << C.State << ", iterations " << C.ActiveSetIterations);
	}
	for (std::size_t K = 0; K < R.Mechanism.size() && K < 3; ++K)
	{
		MESSAGE("motion body " << R.Mechanism[K].Body.Index << ": translation " << R.Mechanism[K].TranslationM << ", rotation " << R.Mechanism[K].RotationRad);
	}
	for (std::size_t K = 0; K < R.Readings.size() && K < 3; ++K)
	{
		const InterfaceReading& Reading = R.Readings[K];
		MESSAGE("reading " << K << ": utilisation " << Reading.Utilisation << " " << Reading.Axis << ", normal " << Reading.NormalForceN << ", shear " << Reading.ShearForceN << ", moment " << Reading.MomentNm << ", corner normals " << Reading.Corners[0].NormalForceN << " " << Reading.Corners[1].NormalForceN << " " << Reading.Corners[2].NormalForceN << " " << Reading.Corners[3].NormalForceN);
	}
}

void CheckRigidFall(const StandingResult& R, const std::vector<BodyId>& Bodies, const Vec3& Fall)
{
	CHECK(R.State == StandingState::Mechanism);
	CHECK(R.ActiveSetIterations == 0);
	CHECK(SameIdList(R.UnconstrainedBodies, Bodies));
	REQUIRE(R.Mechanism.size() == Bodies.size());
	for (const MechanismMotion& M : R.Mechanism)
	{
		INFO("body " << M.Body.Index << ": translation " << M.TranslationM << ", rotation " << M.RotationRad);
		CHECK(NearVec(M.TranslationM, Fall, 1.0e-12, 1.0e-12));
		CHECK(NearVec(M.RotationRad, Vec3{0.0, 0.0, 0.0}, 0.0, 1.0e-12));
	}
}

std::vector<BodyId> LiveFree(const Structure& Model, const geometry::Wall& W)
{
	std::vector<BodyId> Out;
	for (const std::vector<BodyId>& Course : W.Courses)
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

geometry::Wall BuildFloatingWall(Structure& Model, const StrengthProfile& Bed, const StrengthProfile& Perpend, int Courses, int BricksPerCourse)
{
	geometry::WallSpec Spec;
	Spec.Bed = AddProfileOk(Model, Bed);
	Spec.Perpend = AddProfileOk(Model, Perpend);
	Spec.Courses = Courses;
	Spec.BricksPerCourse = BricksPerCourse;
	geometry::Wall W = geometry::BuildRunningBondWall(Model, Spec);
	REQUIRE(W.bComplete);
	for (const BodyId& Id : W.Courses[0])
	{
		REQUIRE(Model.RemoveBody(Id));
	}
	return W;
}
void CheckUncertifiedReadings(const StandingResult& R, std::size_t Expected)
{
	CHECK(R.Readings.size() == Expected);
	for (const InterfaceReading& Reading : R.Readings)
	{
		INFO("reading " << Reading.Id.Index);
		CHECK(Reading.Utilisation == 0.0);
		CHECK(Reading.Axis == GoverningAxis::None);
		CHECK(Reading.NormalForceN == 0.0);
		CHECK(NearVec(Reading.ShearForceN, Vec3{}, 0.0, 0.0));
		CHECK(NearVec(Reading.MomentNm, Vec3{}, 0.0, 0.0));
		for (const CornerReading& Corner : Reading.Corners)
		{
			CHECK(Corner.NormalForceN == 0.0);
		}
	}
}
}

TEST_CASE("Statics.Ungrounded.AFloatingPairIsAMechanismWithoutIterating")
{
	Structure Model;
	const ProfileId Profile = AddProfileOk(Model, DryJoint(0.7));
	AddGroundPlane(Model);
	const FloatingPair P = AddFloatingPair(Model, Profile, 0.0);
	SUBCASE("gravity only")
	{
		const StandingResult R = SolveWith(Model, StandingOptions{});
		Characterise(R);
		CheckRigidFall(R, {P.Lower, P.Upper}, Vec3{0.0, 0.0, -1.0});
		CheckUncertifiedReadings(R, 1);
	}
	SUBCASE("a sideways push on the upper brick")
	{
		const BodyLoad Loads[] = {BodyLoad{P.Upper, Vec3{30.0, 0.0, 0.0}, BrickShape(0.0, 2).CentreM}};
		const StandingResult R = SolveWith(Model, StandingOptions{}, Loads);
		Characterise(R);
		CheckRigidFall(R, {P.Lower, P.Upper}, Vec3{0.0, 0.0, -1.0});
		CheckUncertifiedReadings(R, 1);
	}
}

TEST_CASE("Statics.Ungrounded.TheFloatingRemainderOfTheWallReadsTheSame")
{
	Structure Model;
	const geometry::Wall W = BuildFloatingWall(Model, DryJoint(0.7), DryJoint(0.7), 10, 8);
	const std::vector<BodyId> Floating = LiveFree(Model, W);
	CHECK(Floating.size() == 77u);
	const StandingResult R = SolveWith(Model, StandingOptions{});
	Characterise(R);
	CheckRigidFall(R, Floating, Vec3{0.0, 0.0, -1.0});
}

TEST_CASE("Statics.Ungrounded.AGroundedComponentBesideAFloatingOneIsUnchanged")
{
	Structure Alone;
	const ProfileId AloneProfile = AddProfileOk(Alone, DryJoint(0.7));
	const BodyId AloneGround = AddGroundPlane(Alone);
	AddGroundedStack(Alone, AloneProfile, AloneGround, 3);
	Structure Both;
	const ProfileId BothProfile = AddProfileOk(Both, DryJoint(0.7));
	const BodyId BothGround = AddGroundPlane(Both);
	const std::vector<BodyId> Stack = AddGroundedStack(Both, BothProfile, BothGround, 3);
	const FloatingPair P = AddFloatingPair(Both, BothProfile, 2.0);
	const BodyLoad Push[] = {BodyLoad{Stack[2], Vec3{5.0, 0.0, 0.0}, BrickShape(0.0, 2).CentreM}};
	const StandingResult A = SolveWith(Alone, StandingOptions{}, Push);
	const StandingResult R = SolveWith(Both, StandingOptions{}, Push);
	Characterise(A);
	Characterise(R);
	REQUIRE(R.Components.size() == 2u);
	REQUIRE(A.Components.size() == 1u);
	const StandingComponent& Grounded = R.Components[0];
	CHECK(SameIdList(Grounded.Bodies, Stack));
	CHECK(Grounded.State == A.Components[0].State);
	CHECK(Grounded.ActiveSetIterations == A.Components[0].ActiveSetIterations);
	REQUIRE(Grounded.ReadingSlots.size() == A.Readings.size());
	for (std::size_t K = 0; K < A.Readings.size(); ++K)
	{
		INFO("reading " << K);
		CHECK(SameReading(R.Readings[Grounded.ReadingSlots[K]], A.Readings[K], 0.0, 0.0));
	}
	CHECK(SameIdList(R.Components[1].Bodies, {P.Lower, P.Upper}));
	CHECK(R.Components[1].State == StandingState::Mechanism);
	CHECK(SameIdList(R.UnconstrainedBodies, {P.Lower, P.Upper}));
	REQUIRE(R.Mechanism.size() == 2u);
	for (const MechanismMotion& M : R.Mechanism)
	{
		INFO("body " << M.Body.Index << ": translation " << M.TranslationM << ", rotation " << M.RotationRad);
		CHECK(NearVec(M.TranslationM, Vec3{0.0, 0.0, -1.0}, 1.0e-12, 1.0e-12));
		CHECK(NearVec(M.RotationRad, Vec3{0.0, 0.0, 0.0}, 0.0, 1.0e-12));
	}
}

TEST_CASE("Slow.Statics.Ungrounded.TheFloatingWallRemainderSolvesInsideItsBudget")
{
	const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
	REQUIRE(Loaded.Ok());
	Structure Model;
	const geometry::Wall W = BuildFloatingWall(Model, JointOrFail(Loaded.Value, "clay_gp_mortar_bed_ec6").Profile, JointOrFail(Loaded.Value, "clay_gp_mortar_perpend_ec6").Profile, 32, 40);
	const std::vector<BodyId> Floating = LiveFree(Model, W);
	const std::chrono::steady_clock::time_point Start = std::chrono::steady_clock::now();
	const StandingResult R = SolveWith(Model, StandingOptions{});
	const double Ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Start).count();
	MESSAGE("floating remainder of " << Floating.size() << " bricks solved in " << Ms << " ms");
	Characterise(R);
	CHECK(R.State == StandingState::Mechanism);
	CHECK(SameIdList(R.UnconstrainedBodies, Floating));
	CHECK(Ms < 2000.0);
}
