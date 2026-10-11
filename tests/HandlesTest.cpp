#include "ModelTestSupport.h"

#include "doctest.h"

using namespace loadpath;
using namespace loadpath::test;

namespace
{
template <class Id>
void CheckEqualityComparesIndexAndGeneration()
{
	const Id Handle{3u, 7u};
	const Id SameFields{3u, 7u};
	const Id OtherGeneration{3u, 8u};
	const Id OtherIndex{4u, 7u};
	CHECK_MESSAGE(Handle == SameFields, "same index and generation must compare equal");
	CHECK_MESSAGE(!(Handle != SameFields), "same index and generation must not compare unequal");
	CHECK_MESSAGE(!(Handle == OtherGeneration), "a different generation must compare unequal");
	CHECK_MESSAGE(Handle != OtherGeneration, "a different generation must compare unequal");
	CHECK_MESSAGE(!(Handle == OtherIndex), "a different index must compare unequal");
	CHECK_MESSAGE(Handle != OtherIndex, "a different index must compare unequal");
}
}

TEST_CASE("Model.Handles.ValueInitialisedHandleIsInvalid")
{
	CHECK(BodyId::Invalid().Generation == 0u);
	CHECK(InterfaceId::Invalid().Generation == 0u);
	CHECK(ProfileId::Invalid().Generation == 0u);

	CHECK(BodyId{} == BodyId::Invalid());
	CHECK(InterfaceId{} == InterfaceId::Invalid());
	CHECK(ProfileId{} == ProfileId::Invalid());

	Structure Model;
	const Result<ProfileId> Profile = Model.AddProfile(DistinctFieldProfile());
	const Result<BodyId> Lower = Model.AddBody(BrickAt(StackedBrickCentroidM(0), true));
	const Result<BodyId> Upper = Model.AddBody(BrickAt(StackedBrickCentroidM(1)));
	const Result<InterfaceId> Joint = Model.AddInterface(BedJoint(Lower.Value, Upper.Value, 1, Profile.Value));

	REQUIRE_MESSAGE(Model.IsValid(Profile.Value), "the first profile added must be valid");
	REQUIRE_MESSAGE(Model.IsLive(Lower.Value), "the first body added must be live");
	REQUIRE_MESSAGE(Model.IsLive(Joint.Value), "the first interface added must be live");

	CHECK_MESSAGE(!Model.IsValid(ProfileId{}), "a value-initialised ProfileId must not alias the first profile added");
	CHECK_MESSAGE(!Model.IsLive(BodyId{}), "a value-initialised BodyId must not alias the first body added");
	CHECK_MESSAGE(!Model.IsLive(InterfaceId{}), "a value-initialised InterfaceId must not alias the first interface added");
}

TEST_CASE("Model.Handles.EqualityComparesIndexAndGeneration")
{
	SUBCASE("BodyId") { CheckEqualityComparesIndexAndGeneration<BodyId>(); }
	SUBCASE("InterfaceId") { CheckEqualityComparesIndexAndGeneration<InterfaceId>(); }
	SUBCASE("ProfileId") { CheckEqualityComparesIndexAndGeneration<ProfileId>(); }
}
