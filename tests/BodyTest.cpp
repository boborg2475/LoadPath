#include "ModelTestSupport.h"

#include "doctest.h"

using namespace loadpath;
using namespace loadpath::test;

namespace
{
struct BodyCase
{
	const char* Name;
	BodyDesc Desc;
	ModelError Expected;
};

BodyDesc WithMass(double MassKg)
{
	BodyDesc Desc = BrickAt(StackedBrickCentroidM(1));
	Desc.MassKg = MassKg;
	return Desc;
}

BodyDesc WithCentroid(double XM, double YM, double ZM)
{
	return BrickAt(Vec3{XM, YM, ZM});
}

const BodyCase RefusedBodies[] = {
	{"NaN mass", WithMass(NaN), ModelError::NonFiniteInput},
	{"infinite mass", WithMass(Infinity), ModelError::NonFiniteInput},
	{"NaN centroid X", WithCentroid(NaN, 0.0, BrickHeightM), ModelError::NonFiniteInput},
	{"NaN centroid Y", WithCentroid(0.0, NaN, BrickHeightM), ModelError::NonFiniteInput},
	{"NaN centroid Z", WithCentroid(0.0, 0.0, NaN), ModelError::NonFiniteInput},
	{"infinite centroid Z", WithCentroid(0.0, 0.0, Infinity), ModelError::NonFiniteInput},
	{"negative mass", WithMass(-BrickMassKg), ModelError::NegativeMass},
};
}

TEST_CASE("Model.Body.ValidAddReturnsALiveIdAndReadsBack")
{
	Structure Model;
	const BodyDesc Desc = BrickAt(Vec3{0.1, -0.2, 1.5 * BrickHeightM});
	const Result<BodyId> Added = Model.AddBody(Desc);

	CHECK(Added.Error == ModelError::None);
	CHECK_MESSAGE(Added.Value.Generation != 0u, "an accepted add must return a valid handle");
	CHECK(Model.IsLive(Added.Value));
	CHECK(Model.LiveBodyCount() == 1u);

	const BodyDesc& Stored = Model.Body(Added.Value);
	CHECK(Stored.MassKg == Desc.MassKg);
	CHECK(Stored.CentroidM.X == Desc.CentroidM.X);
	CHECK(Stored.CentroidM.Y == Desc.CentroidM.Y);
	CHECK(Stored.CentroidM.Z == Desc.CentroidM.Z);
	CHECK(Stored.bGrounded == false);
}

TEST_CASE("Model.Body.ZeroMassIsAccepted")
{
	Structure Model;
	const Result<BodyId> Added = Model.AddBody(WithMass(0.0));

	CHECK_MESSAGE(Added.Error == ModelError::None, "a massless body is legitimate and must not be refused");
	CHECK(Model.IsLive(Added.Value));
	CHECK(Model.LiveBodyCount() == 1u);
	CHECK(Model.Body(Added.Value).MassKg == 0.0);
	CHECK(Model.Body(Added.Value).CentroidM.Z == StackedBrickCentroidM(1).Z);
}

TEST_CASE("Model.Body.GroundedFlagReadsBack")
{
	Structure Model;
	const Result<BodyId> Grounded = Model.AddBody(BrickAt(StackedBrickCentroidM(0), true));
	const Result<BodyId> Free = Model.AddBody(BrickAt(StackedBrickCentroidM(1), false));

	REQUIRE(Model.IsLive(Grounded.Value));
	REQUIRE(Model.IsLive(Free.Value));
	CHECK(Model.Body(Grounded.Value).bGrounded == true);
	CHECK(Model.Body(Free.Value).bGrounded == false);
}

TEST_CASE("Model.Body.InvalidInputIsRefusedWithItsReason")
{
	for (const BodyCase& Case : RefusedBodies)
	{
		INFO(doctest::String(Case.Name));
		Structure Model;
		const Result<BodyId> Added = Model.AddBody(Case.Desc);
		CHECK(Added.Error == Case.Expected);
		CHECK_FALSE(Added.Ok());
		CHECK_MESSAGE(Added.Value.Generation == 0u, "a refused add must return the invalid handle");
		CHECK_FALSE(Model.IsLive(Added.Value));
	}
}

TEST_CASE("Model.Body.RefusedAddLeavesTheLiveBodiesUnchanged")
{
	Structure Model;
	const Result<BodyId> Kept = Model.AddBody(BrickAt(StackedBrickCentroidM(0), true));
	REQUIRE(Model.LiveBodyCount() == 1u);
	const std::string LiveBefore = Describe(Model.LiveBodies());

	for (const BodyCase& Case : RefusedBodies)
	{
		INFO(doctest::String(Case.Name));
		const Result<BodyId> Added = Model.AddBody(Case.Desc);
		CHECK_FALSE(Added.Ok());
		CHECK(Model.LiveBodyCount() == 1u);
		CHECK(Describe(Model.LiveBodies()) == LiveBefore);
	}
	CHECK(Model.IsLive(Kept.Value));
}
