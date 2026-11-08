#include "BitwiseTestSupport.h"
#include "BrickFixtures.h"
#include "CascadeTestSupport.h"
#include "StaticsTestSupport.h"

#include "loadpath/Cascade.h"
#include "loadpath/StandingAuthority.h"
#include "loadpath/StructuralAuthority.h"

#include "doctest.h"

#include <cstddef>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double LooseBrickXM = 0.5;

struct StackWithRemovedBrick
{
	Structure Model;
	BodyId Lower;
	BodyId Upper;
	BodyId Removed;
};

StackWithRemovedBrick MakeStackWithRemovedBrick()
{
	TwoBrickStack Stack = MakeTwoBrickStack(BondedJoint(2.0e5, 3.0e5, 0.7), 0.02);
	StackWithRemovedBrick F{std::move(Stack.Model), Stack.Lower, Stack.Upper, BodyId::Invalid()};
	F.Removed = AddBox(F.Model, BrickShape(LooseBrickXM, 0));
	REQUIRE(F.Model.RemoveBody(F.Removed));
	return F;
}

Vec3 CentroidOf(const Structure& Model, BodyId Body)
{
	return Model.Body(Body).CentroidM;
}

std::vector<BodyLoad> WithStaleMiddle(const StackWithRemovedBrick& F)
{
	return {
		BodyLoad{F.Upper, Vec3{3.0, 1.0, 0.0}, Plus(CentroidOf(F.Model, F.Upper), Vec3{0.01, 0.0, 0.02})},
		BodyLoad{F.Removed, Vec3{50.0, 0.0, -20.0}, Vec3{LooseBrickXM, 0.0, 0.03}},
		BodyLoad{F.Lower, Vec3{0.0, -2.0, -15.0}, CentroidOf(F.Model, F.Lower)},
	};
}

std::vector<BodyLoad> WithoutStale(const std::vector<BodyLoad>& Loads)
{
	return {Loads[0], Loads[2]};
}

const std::vector<std::size_t> StaleIsTheMiddle{1};
}

TEST_CASE("Loads.Stale.TheElasticAuthoritySkipsALoadOnARemovedBodyAndReportsIt")
{
	const StackWithRemovedBrick F = MakeStackWithRemovedBrick();
	const std::vector<BodyLoad> Loads = WithStaleMiddle(F);
	const std::vector<BodyLoad> Live = WithoutStale(Loads);
	const std::unique_ptr<IStandingAuthority> Authority = MakeRbsmStandingAuthority();
	const StandingResult Got = Authority->Solve(F.Model, Loads, StandingOptions{}, {});
	const StandingResult Want = Authority->Solve(F.Model, Live, StandingOptions{}, {});
	INFO("got " << std::string(StandingStateName(Got.State)) << ", want " << std::string(StandingStateName(Want.State)));
	REQUIRE(Want.State != StandingState::InvalidInput);
	CHECK(Got.State != StandingState::InvalidInput);
	CHECK(bitwise::Same(Got.StaleLoads, StaleIsTheMiddle));
	CHECK(Want.StaleLoads.empty());
	CHECK(bitwise::Same(Got, Want));
}

TEST_CASE("Loads.Stale.TheStructuralAuthoritySkipsALoadOnARemovedBodyAndReportsIt")
{
	const StackWithRemovedBrick F = MakeStackWithRemovedBrick();
	const std::vector<BodyLoad> Loads = WithStaleMiddle(F);
	const std::vector<BodyLoad> Live = WithoutStale(Loads);
	const std::unique_ptr<IStructuralAuthority> Authority = MakeAuthority();
	const AuthorityResult Got = Authority->Judge(F.Model, Loads, AuthorityOptions{}, nullptr);
	const AuthorityResult Want = Authority->Judge(F.Model, Live, AuthorityOptions{}, nullptr);
	INFO("got " << DescribeAuthority(Got) << "; want " << DescribeAuthority(Want));
	REQUIRE(Want.Worst != ComponentVerdict::InvalidInput);
	CHECK(Got.Worst != ComponentVerdict::InvalidInput);
	CHECK(bitwise::Same(Got.StaleLoads, StaleIsTheMiddle));
	CHECK(Want.StaleLoads.empty());
	CHECK(bitwise::Same(Got, Want));
}

TEST_CASE("Loads.Stale.TheCascadeReportsLoadsWhoseBodyWasDeadWhenTheRunStarted")
{
	StackWithRemovedBrick GotFixture = MakeStackWithRemovedBrick();
	StackWithRemovedBrick WantFixture = MakeStackWithRemovedBrick();
	const std::vector<BodyLoad> Loads = WithStaleMiddle(GotFixture);
	const std::vector<BodyLoad> Live = WithoutStale(WithStaleMiddle(WantFixture));
	const CascadeVerdict Got = MakeCascade(MakeAuthority())->Run(GotFixture.Model, Loads, CascadeOptions{}, nullptr);
	const CascadeVerdict Want = MakeCascade(MakeAuthority())->Run(WantFixture.Model, Live, CascadeOptions{}, nullptr);
	INFO("got " << DescribeVerdict(Got) << "; want " << DescribeVerdict(Want));
	REQUIRE(Want.State != CascadeState::InvalidInput);
	CHECK(Got.State != CascadeState::InvalidInput);
	CHECK(bitwise::Same(Got.StaleLoads, StaleIsTheMiddle));
	CHECK(Want.StaleLoads.empty());
	CHECK(bitwise::Same(Got, Want));
	CHECK(bitwise::Same(GotFixture.Model.LiveBodies(), WantFixture.Model.LiveBodies()));
}

TEST_CASE("Loads.Stale.ABodyTheCascadeReleasesIsNotAStaleLoad")
{
	TwoBrickStack Stack = MakeTwoBrickStack(DryJoint(0.7), 0.0);
	const std::vector<BodyLoad> Loads{BodyLoad{Stack.Upper, Vec3{0.0, 0.0, -1.0}, CentroidOf(Stack.Model, Stack.Upper)}};
	CascadeOptions Options;
	Options.Authority = AuthorityLateral(2.0);
	const CascadeVerdict Got = MakeCascade(MakeAuthority())->Run(Stack.Model, Loads, Options, nullptr);
	INFO(DescribeVerdict(Got));
	REQUIRE_FALSE(Got.Clusters.empty());
	REQUIRE_FALSE(Stack.Model.IsLive(Stack.Upper));
	CHECK(Got.State != CascadeState::InvalidInput);
	CHECK(Got.StaleLoads.empty());
}

namespace
{
struct EntryPoint
{
	const char* Name;
	bool (*ReadsInvalid)(Structure& Model, std::span<const BodyLoad> Loads);
};

const EntryPoint EntryPoints[] = {
	{"Solve", [](Structure& Model, std::span<const BodyLoad> Loads) { return MakeRbsmStandingAuthority()->Solve(Model, Loads, StandingOptions{}, {}).State == StandingState::InvalidInput; }},
	{"Judge", [](Structure& Model, std::span<const BodyLoad> Loads) { return MakeAuthority()->Judge(Model, Loads, AuthorityOptions{}, nullptr).Worst == ComponentVerdict::InvalidInput; }},
	{"Run", [](Structure& Model, std::span<const BodyLoad> Loads) { return MakeCascade(MakeAuthority())->Run(Model, Loads, CascadeOptions{}, nullptr).State == CascadeState::InvalidInput; }},
};
}

TEST_CASE("Loads.Stale.AHandleThisStructureNeverIssuedIsStillInvalidInput")
{
	for (const EntryPoint& Entry : EntryPoints)
	{
		const BodyId Probe = MakeStackWithRemovedBrick().Lower;
		const BodyId Bads[] = {BodyId{Probe.Index + 10u, Probe.Generation}, BodyId{Probe.Index, Probe.Generation + 1u}};
		for (const BodyId Bad : Bads)
		{
			StackWithRemovedBrick F = MakeStackWithRemovedBrick();
			std::vector<BodyLoad> Loads = WithStaleMiddle(F);
			Loads[1].Body = Bad;
			INFO(std::string(Entry.Name) << " body {" << Bad.Index << ", " << Bad.Generation << "}");
			CHECK(Entry.ReadsInvalid(F.Model, Loads));
		}
	}
}

TEST_CASE("Loads.Stale.ANonFiniteLoadOnALiveBodyIsStillInvalidInput")
{
	for (const EntryPoint& Entry : EntryPoints)
	{
		StackWithRemovedBrick ForceFixture = MakeStackWithRemovedBrick();
		std::vector<BodyLoad> ForceNaN = WithoutStale(WithStaleMiddle(ForceFixture));
		ForceNaN[0].ForceN.X = NaN;
		StackWithRemovedBrick PointFixture = MakeStackWithRemovedBrick();
		std::vector<BodyLoad> PointNaN = WithoutStale(WithStaleMiddle(PointFixture));
		PointNaN[1].PointM.Z = NaN;
		INFO(std::string(Entry.Name));
		CHECK(Entry.ReadsInvalid(ForceFixture.Model, ForceNaN));
		CHECK(Entry.ReadsInvalid(PointFixture.Model, PointNaN));
	}
}

TEST_CASE("Loads.Stale.ANonFiniteLoadOnARemovedBodyIsStillInvalidInput")
{
	for (const EntryPoint& Entry : EntryPoints)
	{
		StackWithRemovedBrick ForceFixture = MakeStackWithRemovedBrick();
		std::vector<BodyLoad> ForceNaN = WithStaleMiddle(ForceFixture);
		ForceNaN[1].ForceN.X = NaN;
		StackWithRemovedBrick PointFixture = MakeStackWithRemovedBrick();
		std::vector<BodyLoad> PointNaN = WithStaleMiddle(PointFixture);
		PointNaN[1].PointM.Z = NaN;
		INFO(std::string(Entry.Name));
		CHECK(Entry.ReadsInvalid(ForceFixture.Model, ForceNaN));
		CHECK(Entry.ReadsInvalid(PointFixture.Model, PointNaN));
	}
}
