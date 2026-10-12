#include "ModelTestSupport.h"

#include "doctest.h"

using namespace loadpath;
using namespace loadpath::test;

namespace
{
struct StackWithSideBrick
{
	Structure Model;
	ProfileId Profile;
	BodyId B0;
	BodyId B1;
	BodyId B2;
	BodyId B3;
	BodyId Side;
	InterfaceId J01;
	InterfaceId J12;
	InterfaceId J23;
	InterfaceId J1Side;
};

StackWithSideBrick MakeStackWithSideBrick()
{
	StackWithSideBrick F;
	F.Profile = F.Model.AddProfile(DistinctFieldProfile()).Value;
	F.B0 = F.Model.AddBody(BrickAt(StackedBrickCentroidM(0), true)).Value;
	F.B1 = F.Model.AddBody(BrickAt(StackedBrickCentroidM(1))).Value;
	F.B2 = F.Model.AddBody(BrickAt(StackedBrickCentroidM(2))).Value;
	F.B3 = F.Model.AddBody(BrickAt(StackedBrickCentroidM(3))).Value;
	F.Side = F.Model.AddBody(BrickAt(Vec3{BrickLengthM, 0.0, StackedBrickCentroidM(1).Z})).Value;
	F.J01 = F.Model.AddInterface(BedJoint(F.B0, F.B1, 1, F.Profile)).Value;
	F.J12 = F.Model.AddInterface(BedJoint(F.B1, F.B2, 2, F.Profile)).Value;
	F.J23 = F.Model.AddInterface(BedJoint(F.B2, F.B3, 3, F.Profile)).Value;
	F.J1Side = F.Model.AddInterface(HeadJointAlongX(F.B1, F.Side, 1, F.Profile)).Value;
	return F;
}

void CheckRemovedUnstamped(const Structure& Model, InterfaceId Id, const char* What)
{
	INFO(doctest::String(What));
	CHECK_FALSE(Model.IsLive(Id));
	CHECK(Model.Interface(Id).State == InterfaceState::Removed);
	CHECK(Model.Interface(Id).CrackPass == 0u);
	CHECK(Model.Interface(Id).SeverPass == 0u);
}

void CheckLiveIntact(const Structure& Model, InterfaceId Id, const char* What)
{
	INFO(doctest::String(What));
	CHECK(Model.IsLive(Id));
	CHECK(Model.Interface(Id).State == InterfaceState::Intact);
}
}

TEST_CASE("Model.Structure.RemovedBodyIsNoLongerLive")
{
	StackWithSideBrick F = MakeStackWithSideBrick();
	REQUIRE(F.Model.LiveBodyCount() == 5u);
	REQUIRE(F.Model.IsLive(F.B2));

	CHECK(F.Model.RemoveBody(F.B2));

	CHECK_FALSE(F.Model.IsLive(F.B2));
	CHECK(F.Model.LiveBodyCount() == 5u - 1u);
	CHECK(F.Model.IsLive(F.B0));
	CHECK(F.Model.IsLive(F.B1));
	CHECK(F.Model.IsLive(F.B3));
	CHECK(F.Model.IsLive(F.Side));
}

TEST_CASE("Model.Structure.RemovingABodyMarksItsInterfacesRemovedWithNoStamp")
{
	StackWithSideBrick F = MakeStackWithSideBrick();
	REQUIRE(F.Model.LiveInterfaceCount() == 4u);

	REQUIRE(F.Model.RemoveBody(F.B2));

	CheckRemovedUnstamped(F.Model, F.J12, "joint below the removed body");
	CheckRemovedUnstamped(F.Model, F.J23, "joint above the removed body");
	CheckLiveIntact(F.Model, F.J01, "joint not touching the removed body");
	CheckLiveIntact(F.Model, F.J1Side, "head joint not touching the removed body");
	CHECK(F.Model.LiveInterfaceCount() == 4u - 2u);
}

TEST_CASE("Model.Structure.AddAfterRemovalNeverReturnsTheRemovedHandle")
{
	StackWithSideBrick F = MakeStackWithSideBrick();
	const BodyId Removed = F.B2;
	REQUIRE(F.Model.RemoveBody(Removed));

	constexpr int AddsAfterRemoval = 8;
	for (int Course = 0; Course < AddsAfterRemoval; ++Course)
	{
		INFO("add after removal #" << Course);
		const Result<BodyId> Added = F.Model.AddBody(BrickAt(StackedBrickCentroidM(10 + Course)));
		CHECK(Added.Ok());
		CHECK(F.Model.IsLive(Added.Value));
		CHECK_MESSAGE(!SameHandle(Added.Value, Removed), "a later add returned the removed body's handle " << Removed);
		CHECK_FALSE(F.Model.IsLive(Removed));
	}
	CHECK(F.Model.LiveBodyCount() == static_cast<std::size_t>(4 + AddsAfterRemoval));
}

TEST_CASE("Model.Structure.RemoveBodyOnAStaleHandleReturnsFalseAndChangesNothing")
{
	StackWithSideBrick F = MakeStackWithSideBrick();
	REQUIRE(F.Model.RemoveBody(F.B2));
	REQUIRE(F.Model.LiveBodyCount() == 4u);
	REQUIRE(F.Model.LiveInterfaceCount() == 2u);
	const std::string BodiesBefore = Describe(F.Model.LiveBodies());
	const std::string InterfacesBefore = Describe(F.Model.LiveInterfaces());

	CHECK_FALSE(F.Model.RemoveBody(F.B2));
	CHECK_FALSE(F.Model.RemoveBody(BodyId{}));
	CHECK_FALSE(F.Model.RemoveBody(BodyId{1000u, 1u}));

	CHECK(F.Model.LiveBodyCount() == 4u);
	CHECK(F.Model.LiveInterfaceCount() == 2u);
	CHECK(Describe(F.Model.LiveBodies()) == BodiesBefore);
	CHECK(Describe(F.Model.LiveInterfaces()) == InterfacesBefore);
	CheckLiveIntact(F.Model, F.J01, "surviving bed joint");
	CheckLiveIntact(F.Model, F.J1Side, "surviving head joint");
}

TEST_CASE("Model.Structure.LiveListsAreAscendingAndExcludeTheDead")
{
	StackWithSideBrick F = MakeStackWithSideBrick();
	CHECK(Describe(F.Model.LiveBodies()) == DescribeAscending(std::vector<BodyId>{F.B0, F.B1, F.B2, F.B3, F.Side}));
	CHECK(Describe(F.Model.LiveInterfaces()) == DescribeAscending(std::vector<InterfaceId>{F.J01, F.J12, F.J23, F.J1Side}));

	REQUIRE(F.Model.RemoveBody(F.B2));

	CHECK(Describe(F.Model.LiveBodies()) == DescribeAscending(std::vector<BodyId>{F.B0, F.B1, F.B3, F.Side}));
	CHECK(Describe(F.Model.LiveInterfaces()) == DescribeAscending(std::vector<InterfaceId>{F.J01, F.J1Side}));
}

TEST_CASE("Model.Structure.InterfacesOfTheSurvivingNeighbourDropsTheRemovedJoint")
{
	StackWithSideBrick F = MakeStackWithSideBrick();
	CHECK(Describe(F.Model.InterfacesOf(F.B1)) == DescribeAscending(std::vector<InterfaceId>{F.J01, F.J12, F.J1Side}));
	CHECK(Describe(F.Model.InterfacesOf(F.B3)) == DescribeAscending(std::vector<InterfaceId>{F.J23}));

	REQUIRE(F.Model.RemoveBody(F.B2));

	CHECK(Describe(F.Model.InterfacesOf(F.B1)) == DescribeAscending(std::vector<InterfaceId>{F.J01, F.J1Side}));
	CHECK(F.Model.InterfacesOf(F.B3).empty());
	CHECK(F.Model.InterfacesOf(F.B2).empty());
	CHECK(Describe(F.Model.InterfacesOf(F.Side)) == DescribeAscending(std::vector<InterfaceId>{F.J1Side}));
}

TEST_CASE("Model.Structure.InterfacesOfListsOnlyLiveJointsAscendingAfterSeverAndRemoval")
{
	StackWithSideBrick F = MakeStackWithSideBrick();
	REQUIRE(F.Model.Sever(F.J01, 1));
	CHECK(Describe(F.Model.InterfacesOf(F.B1)) == DescribeAscending(std::vector<InterfaceId>{F.J12, F.J1Side}));
	REQUIRE(F.Model.RemoveBody(F.B2));
	CHECK(Describe(F.Model.InterfacesOf(F.B1)) == DescribeAscending(std::vector<InterfaceId>{F.J1Side}));
	CHECK(F.Model.InterfacesOf(F.B0).empty());
	CHECK(F.Model.Interface(F.J01).State == InterfaceState::Severed);
	CHECK(F.Model.LiveInterfaceCount() == 1u);
	REQUIRE(F.Model.RemoveBody(F.B1));
	CHECK(F.Model.Interface(F.J01).State == InterfaceState::Severed);
	CHECK(F.Model.Interface(F.J1Side).State == InterfaceState::Removed);
	CHECK(F.Model.InterfacesOf(F.Side).empty());
	CHECK(F.Model.LiveInterfaceCount() == 0u);
}

TEST_CASE("Model.Structure.ACopyIsIndependentOfTheOriginal")
{
	StackWithSideBrick F = MakeStackWithSideBrick();
	Structure Copy = F.Model;
	REQUIRE(Copy.RemoveBody(F.B1));
	REQUIRE(Copy.Sever(F.J23, 1));
	CHECK(Copy.InterfacesOf(F.Side).empty());
	CHECK(Copy.LiveInterfaceCount() == 0u);
	CHECK(Describe(F.Model.InterfacesOf(F.B1)) == DescribeAscending(std::vector<InterfaceId>{F.J01, F.J12, F.J1Side}));
	CHECK(Describe(F.Model.InterfacesOf(F.B3)) == DescribeAscending(std::vector<InterfaceId>{F.J23}));
	CHECK(F.Model.LiveInterfaceCount() == 4u);
	REQUIRE(F.Model.RemoveBody(F.B3));
	CHECK(Copy.Interface(F.J23).State == InterfaceState::Severed);
	CHECK(F.Model.Interface(F.J23).State == InterfaceState::Removed);
}
