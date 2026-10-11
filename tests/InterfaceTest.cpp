#include "ModelTestSupport.h"

#include "doctest.h"

#include <iterator>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
struct BrickStack
{
	Structure Model;
	ProfileId Profile;
	BodyId Bottom;
	BodyId Middle;
	BodyId Top;
	InterfaceId BottomMiddle;
	InterfaceDesc Desc;
};

BrickStack MakeBrickStack()
{
	BrickStack Stack;
	Stack.Profile = Stack.Model.AddProfile(DistinctFieldProfile()).Value;
	Stack.Bottom = Stack.Model.AddBody(BrickAt(StackedBrickCentroidM(0), true)).Value;
	Stack.Middle = Stack.Model.AddBody(BrickAt(StackedBrickCentroidM(1))).Value;
	Stack.Top = Stack.Model.AddBody(BrickAt(StackedBrickCentroidM(2))).Value;
	Stack.BottomMiddle = Stack.Model.AddInterface(BedJoint(Stack.Bottom, Stack.Middle, 1, Stack.Profile)).Value;
	Stack.Desc = BedJoint(Stack.Middle, Stack.Top, 2, Stack.Profile);
	return Stack;
}

void CheckSameVec3(const Vec3& Got, const Vec3& Want, const char* What)
{
	INFO(doctest::String(What));
	CHECK(Got.X == Want.X);
	CHECK(Got.Y == Want.Y);
	CHECK(Got.Z == Want.Z);
}

struct RefusalCase
{
	const char* Name;
	void (*Mutate)(BrickStack&);
	ModelError Expected;
};

const RefusalCase Refusals[] = {
	{"BodyA value-initialised", [](BrickStack& S) { S.Desc.BodyA = BodyId{}; }, ModelError::UnknownBody},
	{"BodyB value-initialised", [](BrickStack& S) { S.Desc.BodyB = BodyId{}; }, ModelError::UnknownBody},
	{"BodyB index never allocated", [](BrickStack& S) { S.Desc.BodyB = BodyId{1000u, 1u}; }, ModelError::UnknownBody},
	{"BodyB removed", [](BrickStack& S) { S.Model.RemoveBody(S.Top); }, ModelError::BodyRemoved},
	{"BodyA removed", [](BrickStack& S) { S.Desc.BodyA = S.Top; S.Desc.BodyB = S.Middle; S.Model.RemoveBody(S.Top); }, ModelError::BodyRemoved},
	{"BodyA equals BodyB", [](BrickStack& S) { S.Desc.BodyB = S.Desc.BodyA; }, ModelError::SameBody},
	{"profile value-initialised", [](BrickStack& S) { S.Desc.Profile = ProfileId{}; }, ModelError::UnknownProfile},
	{"profile index never allocated", [](BrickStack& S) { S.Desc.Profile = ProfileId{1000u, 1u}; }, ModelError::UnknownProfile},
	{"NaN normal X", [](BrickStack& S) { S.Desc.NormalAToB.X = NaN; }, ModelError::NonFiniteInput},
	{"NaN normal Y", [](BrickStack& S) { S.Desc.NormalAToB.Y = NaN; }, ModelError::NonFiniteInput},
	{"NaN normal Z", [](BrickStack& S) { S.Desc.NormalAToB.Z = NaN; }, ModelError::NonFiniteInput},
	{"NaN centre X", [](BrickStack& S) { S.Desc.CentreM.X = NaN; }, ModelError::NonFiniteInput},
	{"NaN centre Y", [](BrickStack& S) { S.Desc.CentreM.Y = NaN; }, ModelError::NonFiniteInput},
	{"NaN centre Z", [](BrickStack& S) { S.Desc.CentreM.Z = NaN; }, ModelError::NonFiniteInput},
	{"NaN axis U X", [](BrickStack& S) { S.Desc.AxisU.X = NaN; }, ModelError::NonFiniteInput},
	{"NaN axis U Y", [](BrickStack& S) { S.Desc.AxisU.Y = NaN; }, ModelError::NonFiniteInput},
	{"NaN axis U Z", [](BrickStack& S) { S.Desc.AxisU.Z = NaN; }, ModelError::NonFiniteInput},
	{"NaN axis V X", [](BrickStack& S) { S.Desc.AxisV.X = NaN; }, ModelError::NonFiniteInput},
	{"NaN axis V Y", [](BrickStack& S) { S.Desc.AxisV.Y = NaN; }, ModelError::NonFiniteInput},
	{"NaN axis V Z", [](BrickStack& S) { S.Desc.AxisV.Z = NaN; }, ModelError::NonFiniteInput},
	{"NaN half-extent U", [](BrickStack& S) { S.Desc.HalfExtentUM = NaN; }, ModelError::NonFiniteInput},
	{"NaN half-extent V", [](BrickStack& S) { S.Desc.HalfExtentVM = NaN; }, ModelError::NonFiniteInput},
	{"infinite half-extent U", [](BrickStack& S) { S.Desc.HalfExtentUM = Infinity; }, ModelError::NonFiniteInput},
	{"infinite normal Z", [](BrickStack& S) { S.Desc.NormalAToB.Z = Infinity; }, ModelError::NonFiniteInput},
	{"zero normal", [](BrickStack& S) { S.Desc.NormalAToB = Vec3{}; }, ModelError::DegenerateNormal},
	{"U not unit length", [](BrickStack& S) { S.Desc.AxisU = Vec3{2.0, 0.0, 0.0}; }, ModelError::AxesNotOrthonormal},
	{"U length off by 1e-6", [](BrickStack& S) { S.Desc.AxisU = Vec3{1.0 + 1.0e-6, 0.0, 0.0}; }, ModelError::AxesNotOrthonormal},
	{"V not unit length", [](BrickStack& S) { S.Desc.AxisV = Vec3{0.0, 0.5, 0.0}; }, ModelError::AxesNotOrthonormal},
	{"U not orthogonal to N", [](BrickStack& S) { S.Desc.AxisU = Vec3{0.6, 0.0, 0.8}; }, ModelError::AxesNotOrthonormal},
	{"V not orthogonal to N", [](BrickStack& S) { S.Desc.AxisV = Vec3{0.0, 0.6, 0.8}; }, ModelError::AxesNotOrthonormal},
	{"U not orthogonal to V", [](BrickStack& S) { S.Desc.AxisV = Vec3{0.6, 0.8, 0.0}; }, ModelError::AxesNotOrthonormal},
	{"U x V antiparallel to N", [](BrickStack& S) { S.Desc.AxisU = Vec3{0.0, 1.0, 0.0}; S.Desc.AxisV = Vec3{1.0, 0.0, 0.0}; }, ModelError::AxesNotOrthonormal},
	{"zero half-extent U", [](BrickStack& S) { S.Desc.HalfExtentUM = 0.0; }, ModelError::NonPositiveExtent},
	{"zero half-extent V", [](BrickStack& S) { S.Desc.HalfExtentVM = 0.0; }, ModelError::NonPositiveExtent},
	{"negative half-extent U", [](BrickStack& S) { S.Desc.HalfExtentUM = -S.Desc.HalfExtentUM; }, ModelError::NonPositiveExtent},
	{"negative half-extent V", [](BrickStack& S) { S.Desc.HalfExtentVM = -S.Desc.HalfExtentVM; }, ModelError::NonPositiveExtent},
};

const RefusalCase OrderCases[] = {
	{"unknown BodyA with removed BodyB", [](BrickStack& S) { S.Desc.BodyA = BodyId{}; S.Model.RemoveBody(S.Top); }, ModelError::UnknownBody},
	{"removed body on both sides", [](BrickStack& S) { S.Desc.BodyA = S.Top; S.Desc.BodyB = S.Top; S.Model.RemoveBody(S.Top); }, ModelError::BodyRemoved},
	{"same body with unknown profile", [](BrickStack& S) { S.Desc.BodyB = S.Desc.BodyA; S.Desc.Profile = ProfileId{}; }, ModelError::SameBody},
	{"unknown profile with NaN half-extent", [](BrickStack& S) { S.Desc.Profile = ProfileId{}; S.Desc.HalfExtentUM = NaN; }, ModelError::UnknownProfile},
	{"NaN centre with zero normal", [](BrickStack& S) { S.Desc.CentreM.Z = NaN; S.Desc.NormalAToB = Vec3{}; }, ModelError::NonFiniteInput},
	{"NaN normal with zero half-extent", [](BrickStack& S) { S.Desc.NormalAToB.Z = NaN; S.Desc.HalfExtentUM = 0.0; }, ModelError::NonFiniteInput},
	{"zero normal with zero half-extent", [](BrickStack& S) { S.Desc.NormalAToB = Vec3{}; S.Desc.HalfExtentUM = 0.0; }, ModelError::DegenerateNormal},
	{"zero normal with axes not orthonormal", [](BrickStack& S) { S.Desc.NormalAToB = Vec3{}; S.Desc.AxisU = Vec3{2.0, 0.0, 0.0}; }, ModelError::DegenerateNormal},
	{"axes not orthonormal with zero half-extent", [](BrickStack& S) { S.Desc.AxisU = Vec3{0.6, 0.0, 0.8}; S.Desc.HalfExtentVM = 0.0; }, ModelError::AxesNotOrthonormal},
};

void CheckRefusalTable(const RefusalCase* Begin, const RefusalCase* End)
{
	for (const RefusalCase* Case = Begin; Case != End; ++Case)
	{
		INFO(doctest::String(Case->Name));
		BrickStack Stack = MakeBrickStack();
		Case->Mutate(Stack);
		const Result<InterfaceId> Added = Stack.Model.AddInterface(Stack.Desc);
		CHECK(Added.Error == Case->Expected);
		CHECK_FALSE(Added.Ok());
		CHECK_MESSAGE(Added.Value.Generation == 0u, "a refused add must return the invalid handle");
		CHECK_FALSE(Stack.Model.IsLive(Added.Value));
	}
}
}

TEST_CASE("Model.Interface.ValidBedJointReturnsALiveIdAndReadsBack")
{
	BrickStack Stack = MakeBrickStack();
	REQUIRE(Stack.Model.LiveInterfaceCount() == 1u);
	const Result<InterfaceId> Added = Stack.Model.AddInterface(Stack.Desc);

	CHECK(Added.Error == ModelError::None);
	CHECK_MESSAGE(Added.Value.Generation != 0u, "an accepted add must return a valid handle");
	CHECK(Stack.Model.IsLive(Added.Value));
	CHECK(Stack.Model.LiveInterfaceCount() == 2u);

	const InterfaceRecord& Stored = Stack.Model.Interface(Added.Value);
	CHECK(SameHandle(Stored.BodyA, Stack.Middle));
	CHECK(SameHandle(Stored.BodyB, Stack.Top));
	CheckSameVec3(Stored.NormalAToB, Vec3{0.0, 0.0, 1.0}, "normal");
	CheckSameVec3(Stored.CentreM, Vec3{0.0, 0.0, 2.0 * BrickHeightM}, "centre");
	CheckSameVec3(Stored.AxisU, Vec3{1.0, 0.0, 0.0}, "axis U");
	CheckSameVec3(Stored.AxisV, Vec3{0.0, 1.0, 0.0}, "axis V");
	CHECK(Stored.HalfExtentUM == BrickLengthM / 2.0);
	CHECK(Stored.HalfExtentVM == BrickWidthM / 2.0);
	CHECK(Stored.AreaM2 == 4.0 * (BrickLengthM / 2.0) * (BrickWidthM / 2.0));
	CHECK(SameHandle(Stored.Profile, Stack.Profile));
	CHECK(Stored.State == InterfaceState::Intact);
	CHECK(Stored.CrackPass == 0u);
	CHECK(Stored.SeverPass == 0u);
}

TEST_CASE("Model.Interface.NonUnitNormalIsNormalisedOnAdd")
{
	BrickStack Stack = MakeBrickStack();
	Stack.Desc.NormalAToB = Vec3{0.0, 0.0, 2.0};
	const Result<InterfaceId> Added = Stack.Model.AddInterface(Stack.Desc);

	CHECK(Added.Error == ModelError::None);
	REQUIRE(Stack.Model.IsLive(Added.Value));
	CheckSameVec3(Stack.Model.Interface(Added.Value).NormalAToB, Vec3{0.0, 0.0, 1.0}, "normal");
}

TEST_CASE("Model.Interface.AxesWithinToleranceAreAccepted")
{
	BrickStack Stack = MakeBrickStack();
	Stack.Desc.AxisU = Vec3{1.0 + 1.0e-12, 0.0, 0.0};
	const Result<InterfaceId> Added = Stack.Model.AddInterface(Stack.Desc);

	CHECK_MESSAGE(Added.Error == ModelError::None, "a unit-length error of 1e-12 is inside the 1e-9 tolerance");
	CHECK(Stack.Model.IsLive(Added.Value));
}

TEST_CASE("Model.Interface.InvalidInputIsRefusedWithItsReason")
{
	CheckRefusalTable(std::begin(Refusals), std::end(Refusals));
}

TEST_CASE("Model.Interface.FirstFailureInContractOrderWins")
{
	CheckRefusalTable(std::begin(OrderCases), std::end(OrderCases));
}

TEST_CASE("Model.Interface.RefusedAddLeavesTheLiveInterfacesUnchanged")
{
	for (const RefusalCase& Case : Refusals)
	{
		INFO(doctest::String(Case.Name));
		BrickStack Stack = MakeBrickStack();
		REQUIRE(Stack.Model.LiveInterfaceCount() == 1u);
		Case.Mutate(Stack);
		const Result<InterfaceId> Added = Stack.Model.AddInterface(Stack.Desc);
		CHECK_FALSE(Added.Ok());
		CHECK(Stack.Model.LiveInterfaceCount() == 1u);
		CHECK(Describe(Stack.Model.LiveInterfaces()) == Describe(std::vector<InterfaceId>{Stack.BottomMiddle}));
		CHECK(Stack.Model.InterfacesOf(Stack.Top).empty());
	}
}
