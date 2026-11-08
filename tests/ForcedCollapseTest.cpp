#include "BrickFixtures.h"
#include "JsonReader.h"
#include "CascadeTestSupport.h"
#include "StaticsTestSupport.h"

#include "loadpath/Cascade.h"
#include "loadpath/Settings.h"
#include "loadpath/StandingAuthority.h"
#include "loadpath/Viewer.h"

#include "doctest.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
const DebugSettings RecordingOn{true};

CascadeOptions CappedStack(UndecidedResolution Resolution, int Cap, const DebugSettings* Debug)
{
	CascadeOptions Options;
	Options.Authority.Standing.Gravity = Vec3{1.375 * GravityMPerS2, 0.625 * GravityMPerS2, -GravityMPerS2};
	Options.Authority.Standing.MaxActiveSetIterations = Cap;
	Options.Authority.Standing.Debug = Debug;
	Options.OnUndecided = Resolution;
	return Options;
}

std::vector<BodyId> ReleasedBodies(const CascadeVerdict& V)
{
	std::vector<BodyId> Out;
	for (const ReleasedCluster& Cluster : V.Clusters)
	{
		Out.insert(Out.end(), Cluster.Bodies.begin(), Cluster.Bodies.end());
	}
	std::sort(Out.begin(), Out.end(), [](BodyId A, BodyId B) { return A.Index < B.Index; });
	return Out;
}

const ComponentReport& FirstJudged(const CascadeVerdict& V)
{
	return V.PassRecords.front().Components.front();
}
}

TEST_CASE("Cascade.Forced.AnUndecidedComponentCollapsesWholeUnderCollapse")
{
	TwoBrickStack S = MakeTwoBrickStack(DryJoint(5.0), 0.03);
	const CascadeVerdict V = MakeCascade(MakeAuthority())->Run(S.Model, {}, CappedStack(UndecidedResolution::Collapse, 2, &RecordingOn), nullptr);
	INFO(DescribeVerdict(V));
	CHECK(V.State == CascadeState::Stands);
	REQUIRE_FALSE(V.PassRecords.empty());
	if (V.PassRecords.empty())
	{
		return;
	}
	REQUIRE(V.PassRecords.front().Components.size() == 1u);
	if (!(V.PassRecords.front().Components.size() == 1u))
	{
		return;
	}
	const ComponentReport& Judged = FirstJudged(V);
	CHECK(Judged.Verdict == ComponentVerdict::Collapses);
	CHECK(Judged.Elastic == StandingState::Undecided);
	CHECK(Judged.Forced);
	CHECK(Judged.ForcedReason == SolveEnd::IterationCap);
	CHECK(Judged.PassCount == 2);
	CHECK_FALSE(Judged.UnsettledJoints.empty());
	const std::vector<BodyId> Released = ReleasedBodies(V);
	REQUIRE(Released.size() == 2u);
	if (!(Released.size() == 2u))
	{
		return;
	}
	CHECK(Released[0].Index == S.Lower.Index);
	CHECK(Released[1].Index == S.Upper.Index);
	REQUIRE(V.Clusters.size() == 1u);
	if (!(V.Clusters.size() == 1u))
	{
		return;
	}
	CHECK(V.Clusters[0].bForced);
	CHECK(V.ForcedByReason[static_cast<std::size_t>(SolveEnd::IterationCap)] == 1u);
	std::uint32_t Total = 0;
	for (const std::uint32_t Count : V.ForcedByReason)
	{
		Total += Count;
	}
	CHECK(Total == 1u);
}

TEST_CASE("Cascade.Forced.UnsettledJointsNeedTheRecorderAtTheIterationCap")
{
	TwoBrickStack S = MakeTwoBrickStack(DryJoint(5.0), 0.03);
	const CascadeVerdict V = MakeCascade(MakeAuthority())->Run(S.Model, {}, CappedStack(UndecidedResolution::Stands, 2, nullptr), nullptr);
	INFO(DescribeVerdict(V));
	REQUIRE(V.Components.size() == 1u);
	if (!(V.Components.size() == 1u))
	{
		return;
	}
	CHECK(V.Components[0].Forced);
	CHECK(V.Components[0].ForcedReason == SolveEnd::IterationCap);
	CHECK(V.Components[0].PassCount == 2);
	CHECK(V.Components[0].UnsettledJoints.empty());
}

TEST_CASE("Cascade.Forced.AnUndecidedComponentStandsUnderStandsAndSaysSo")
{
	TwoBrickStack S = MakeTwoBrickStack(DryJoint(5.0), 0.03);
	const CascadeVerdict V = MakeCascade(MakeAuthority())->Run(S.Model, {}, CappedStack(UndecidedResolution::Stands, 2, &RecordingOn), nullptr);
	INFO(DescribeVerdict(V));
	CHECK(V.State == CascadeState::Stands);
	CHECK(V.Clusters.empty());
	CHECK(V.Passes == 0);
	REQUIRE(V.Components.size() == 1u);
	if (!(V.Components.size() == 1u))
	{
		return;
	}
	const ComponentReport& Report = V.Components[0];
	CHECK(Report.Verdict == ComponentVerdict::Stands);
	CHECK(Report.Elastic == StandingState::Undecided);
	CHECK(Report.Forced);
	CHECK(Report.ForcedReason == SolveEnd::IterationCap);
	CHECK(Report.PassCount == 2);
	CHECK(Report.Release.empty());
	CHECK_FALSE(Report.UnsettledJoints.empty());
	CHECK(V.ForcedByReason[static_cast<std::size_t>(SolveEnd::IterationCap)] == 1u);
	CHECK(S.Model.LiveBodyCount() == 3u);
}

TEST_CASE("Cascade.Forced.AMechanismCollapsesUnforced")
{
	Structure Model;
	AddGroundPlane(Model);
	const BodyId Floating = AddBox(Model, Box{Vec3{1.0, 0.0, 1.0}, BrickShape().HalfExtentsM, BrickMassKg});
	CascadeOptions Options;
	Options.Authority.Standing.Debug = &RecordingOn;
	const CascadeVerdict V = MakeCascade(MakeAuthority())->Run(Model, {}, Options, nullptr);
	INFO(DescribeVerdict(V));
	REQUIRE_FALSE(V.PassRecords.empty());
	if (V.PassRecords.empty())
	{
		return;
	}
	REQUIRE(V.PassRecords.front().Components.size() == 1u);
	if (!(V.PassRecords.front().Components.size() == 1u))
	{
		return;
	}
	const ComponentReport& Judged = FirstJudged(V);
	CHECK(Judged.Verdict == ComponentVerdict::Collapses);
	CHECK_FALSE(Judged.Forced);
	CHECK(Judged.ForcedReason == SolveEnd::None);
	CHECK(Judged.UnsettledJoints.empty());
	REQUIRE(V.Clusters.size() == 1u);
	if (!(V.Clusters.size() == 1u))
	{
		return;
	}
	CHECK_FALSE(V.Clusters[0].bForced);
	CHECK(V.Clusters[0].Bodies.front().Index == Floating.Index);
	for (const std::uint32_t Count : V.ForcedByReason)
	{
		CHECK(Count == 0u);
	}
}

TEST_CASE("Cascade.Forced.TheDefaultResolutionIsCollapse")
{
	CHECK(CascadeOptions{}.OnUndecided == UndecidedResolution::Collapse);
	CHECK(CascadeSettings{}.OnUndecided == UndecidedResolution::Collapse);
	Settings S;
	S.Cascade.OnUndecided = UndecidedResolution::Stands;
	CHECK(ToCascadeOptions(S, Vec3{0.0, 0.0, -GravityMPerS2}).OnUndecided == UndecidedResolution::Stands);
}

namespace
{
const detail::JsonValue& Member(const detail::JsonValue& Object, std::string_view Key)
{
	static const detail::JsonValue Missing{};
	const detail::JsonValue* Value = detail::Find(Object, Key);
	INFO("viewer json key " << std::string(Key));
	CHECK(Value != nullptr);
	return Value != nullptr ? *Value : Missing;
}

void CheckForcedFields(const detail::JsonValue& Component)
{
	CHECK(Member(Component, "forced").Bool);
	CHECK(Member(Component, "forced_reason").Text == "IterationCap");
	CHECK(Member(Component, "pass_count").Number == 2.0);
	CHECK_FALSE(Member(Component, "unsettled_joints").Items.empty());
}
}

TEST_CASE("Scenario.Viewer.ForcedComponentsCarryTheirReasonPassCountAndUnsettledJoints")
{
	ViewerScene Scene;
	TwoBrickStack S = MakeTwoBrickStack(DryJoint(5.0), 0.03);
	Scene.After.push_back(MakeCascade(MakeAuthority())->Run(S.Model, {}, CappedStack(UndecidedResolution::Stands, 2, &RecordingOn), nullptr));
	Scene.Model = std::move(S.Model);
	const std::string Json = WriteViewerJson(Scene);
	detail::JsonParser Parser(Json);
	detail::JsonValue Root;
	REQUIRE(Parser.ParseDocument(Root));
	if (!(Parser.ParseDocument(Root)))
	{
		return;
	}
	const detail::JsonValue& After = Member(Root, "after");
	const detail::JsonValue& Components = Member(After, "components");
	REQUIRE(Components.Items.size() == 1u);
	if (!(Components.Items.size() == 1u))
	{
		return;
	}
	CheckForcedFields(Components.Items[0]);
	const detail::JsonValue& Records = Member(After, "pass_records");
	REQUIRE_FALSE(Records.Items.empty());
	if (Records.Items.empty())
	{
		return;
	}
	CheckForcedFields(Member(Records.Items[0], "components").Items[0]);

	ViewerScene Plain;
	Structure Model;
	AddGroundPlane(Model);
	AddBox(Model, Box{Vec3{1.0, 0.0, 1.0}, BrickShape().HalfExtentsM, BrickMassKg});
	Plain.After.push_back(MakeCascade(MakeAuthority())->Run(Model, {}, CascadeOptions{}, nullptr));
	Plain.Model = std::move(Model);
	const std::string PlainJson = WriteViewerJson(Plain);
	detail::JsonParser PlainParser(PlainJson);
	detail::JsonValue PlainRoot;
	REQUIRE(PlainParser.ParseDocument(PlainRoot));
	if (!(PlainParser.ParseDocument(PlainRoot)))
	{
		return;
	}
	for (const detail::JsonValue& Cluster : Member(Member(PlainRoot, "after"), "clusters").Items)
	{
		CHECK(detail::Find(Cluster, "forced") == nullptr);
	}
}
