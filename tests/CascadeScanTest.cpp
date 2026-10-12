#include "CascadeTestSupport.h"
#include "ModelTestSupport.h"

#include "loadpath/Cascade.h"

#include "doctest.h"

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
using Clock = std::chrono::steady_clock;

constexpr std::size_t FieldPairs = 2000;
constexpr double QuadraticRatioBar = 8.0;

Structure MakeFloatingPairField(std::size_t Pairs)
{
	Structure Model;
	const ProfileId Profile = AddProfileOk(Model, DryJoint(0.7));
	AddGroundPlane(Model);
	for (std::size_t I = 0; I < Pairs; ++I)
	{
		const double CentreXM = 2.0 * BrickLengthM * static_cast<double>(I);
		const Box LowerShape = BrickShape(CentreXM, 1);
		const Box UpperShape = BrickShape(CentreXM, 2);
		const BodyId Lower = AddBox(Model, LowerShape);
		const BodyId Upper = AddBox(Model, UpperShape);
		AddInterfaceOk(Model, *StackedFaces(Lower, LowerShape, Upper, UpperShape, Profile));
	}
	return Model;
}

double TimeFieldCascade(std::size_t Pairs)
{
	Structure Model = MakeFloatingPairField(Pairs);
	const std::unique_ptr<ICascade> Cascade = MakeCascade(MakeAuthority());
	const Clock::time_point Start = Clock::now();
	const CascadeVerdict V = Cascade->Run(Model, {}, CascadeOptions{}, nullptr);
	const double Ms = std::chrono::duration<double, std::milli>(Clock::now() - Start).count();
	INFO("pairs " << Pairs << ": state " << CascadeStateName(V.State) << ", passes " << V.Passes << ", clusters " << V.Clusters.size() << ", failures " << V.Failures.size());
	CHECK(V.State == CascadeState::Stands);
	CHECK(V.Passes == 1);
	CHECK(V.Failures.empty());
	CHECK(V.Clusters.size() == Pairs);
	std::size_t Severed = 0;
	std::size_t NotPairs = 0;
	for (const ReleasedCluster& C : V.Clusters)
	{
		Severed += C.Severed.size();
		NotPairs += C.Bodies.size() == 2u ? 0u : 1u;
	}
	CHECK(Severed == 0u);
	CHECK(NotPairs == 0u);
	return Ms;
}

struct HingedClusters
{
	Structure Model;
	BodyId A;
	BodyId B;
	BodyId C;
	BodyId D;
	InterfaceId GroundB;
	InterfaceId Hinge;
	InterfaceId AD;
	InterfaceId BC;
	InterfaceId GroundA;
};

HingedClusters MakeHingedClusters()
{
	HingedClusters F;
	const ProfileId Profile = AddProfileOk(F.Model, DryJoint(0.7));
	const BodyId Ground = F.Model.AddBody(BrickAt(StackedBrickCentroidM(0), true)).Value;
	F.A = F.Model.AddBody(BrickAt(StackedBrickCentroidM(1))).Value;
	F.B = F.Model.AddBody(BrickAt(StackedBrickCentroidM(2))).Value;
	F.C = F.Model.AddBody(BrickAt(StackedBrickCentroidM(3))).Value;
	F.D = F.Model.AddBody(BrickAt(StackedBrickCentroidM(4))).Value;
	F.GroundB = AddInterfaceOk(F.Model, BedJoint(Ground, F.B, 1, Profile));
	F.Hinge = AddInterfaceOk(F.Model, BedJoint(F.D, F.C, 2, Profile));
	F.AD = AddInterfaceOk(F.Model, BedJoint(F.A, F.D, 3, Profile));
	F.BC = AddInterfaceOk(F.Model, BedJoint(F.B, F.C, 4, Profile));
	F.GroundA = AddInterfaceOk(F.Model, BedJoint(Ground, F.A, 5, Profile));
	return F;
}

void SetReading(AuthorityResult& Script, InterfaceId Id, GoverningAxis Axis, double Ratio)
{
	for (InterfaceReading& R : Script.Readings)
	{
		if (R.Id == Id)
		{
			R.Axis = Axis;
			R.Utilisation = Ratio;
		}
	}
}
}

TEST_CASE("Slow.Cascade.Scans.AFieldOfFloatingPairsCascadesInLinearTime")
{
	TimeFieldCascade(FieldPairs);
	const double SmallMs = TimeFieldCascade(FieldPairs);
	const double LargeMs = TimeFieldCascade(4 * FieldPairs);
	const double Ratio = LargeMs / SmallMs;
	MESSAGE("floating pairs: " << FieldPairs << " in " << SmallMs << " ms, " << 4 * FieldPairs << " in " << LargeMs << " ms, ratio " << Ratio);
	CHECK(Ratio < QuadraticRatioBar);
}

TEST_CASE("Cascade.Scans.HingedClustersComeInFrontBodyOrderWithTheHingeUnderTheLowerCluster")
{
	HingedClusters F = MakeHingedClusters();
	AuthorityResult Collapse = CollapsesScript(F.Model, {F.C, F.D, F.B, F.A});
	Collapse.Components[0].Hinging = {F.Hinge};
	SetReading(Collapse, F.GroundB, GoverningAxis::Shear, 1.25);
	SetReading(Collapse, F.Hinge, GoverningAxis::Tension, 2.5);
	SetReading(Collapse, F.GroundA, GoverningAxis::Compression, 3.75);
	const CascadeVerdict V = RunScripted(F.Model, {Collapse, StandsScript()});
	INFO(DescribeVerdict(V));
	CHECK(V.State == CascadeState::Stands);
	CHECK(V.Passes == 1);
	REQUIRE(V.Clusters.size() == 2u);
	CHECK(SameIdList(V.Clusters[0].Bodies, {F.A, F.D}));
	CHECK(SameIdList(V.Clusters[0].Severed, {F.Hinge, F.GroundA}));
	CHECK(SameIdList(V.Clusters[1].Bodies, {F.B, F.C}));
	CHECK(SameIdList(V.Clusters[1].Severed, {F.GroundB}));
	REQUIRE(V.Failures.size() == 3u);
	CHECK(SameHandle(V.Failures[0].Id, F.GroundB));
	CHECK(V.Failures[0].Axis == GoverningAxis::Shear);
	CHECK(V.Failures[0].Ratio == 1.25);
	CHECK(SameHandle(V.Failures[1].Id, F.Hinge));
	CHECK(V.Failures[1].Axis == GoverningAxis::Tension);
	CHECK(V.Failures[1].Ratio == 2.5);
	CHECK(SameHandle(V.Failures[2].Id, F.GroundA));
	CHECK(V.Failures[2].Axis == GoverningAxis::Compression);
	CHECK(V.Failures[2].Ratio == 3.75);
	for (const FailedInterface& Failure : V.Failures)
	{
		CHECK(Failure.bSevered);
		CHECK(Failure.Pass == 1u);
	}
	CheckStamps(F.Model, F.Hinge, InterfaceState::Severed, 0, 1);
	CheckStamps(F.Model, F.GroundA, InterfaceState::Severed, 0, 1);
	CheckStamps(F.Model, F.GroundB, InterfaceState::Severed, 0, 1);
	CHECK(F.Model.Interface(F.AD).State == InterfaceState::Removed);
	CHECK(F.Model.Interface(F.BC).State == InterfaceState::Removed);
}

namespace
{
constexpr int CrackFieldBricks = 2000;

struct CrackFieldRun
{
	double Ms = 0.0;
	CascadeVerdict Verdict;
};

CrackFieldRun TimeCrackField(int Bricks)
{
	Structure Model;
	const ProfileId Profile = AddProfileOk(Model, BondedJoint(1.0e5, 1.0e5, 0.7));
	const BodyId Ground = AddGroundPlane(Model);
	AuthorityResult Script;
	Script.Worst = ComponentVerdict::Overstressed;
	for (int I = 0; I < Bricks; ++I)
	{
		const Box Shape = BrickShape(2.0 * BrickLengthM * static_cast<double>(I), 0);
		const BodyId Brick = AddBox(Model, Shape);
		const InterfaceId Joint = AddInterfaceOk(Model, OnGroundPlane(Ground, Brick, Shape, Profile));
		ComponentReport Component;
		Component.Bodies = {Brick};
		Component.Verdict = ComponentVerdict::Overstressed;
		Script.Components.push_back(Component);
		InterfaceReading Reading;
		Reading.Id = Joint;
		Reading.TensionUtilisation = 2.0;
		Reading.Utilisation = 2.0;
		Reading.Axis = GoverningAxis::Tension;
		Script.Readings.push_back(Reading);
	}
	const std::unique_ptr<ICascade> Cascade = MakeCascade(std::make_unique<ScriptedAuthority>(std::vector<AuthorityResult>{Script, StandsScript()}));
	CrackFieldRun Run;
	const Clock::time_point Start = Clock::now();
	Run.Verdict = Cascade->Run(Model, {}, CascadeOptions{}, nullptr);
	Run.Ms = std::chrono::duration<double, std::milli>(Clock::now() - Start).count();
	INFO("bricks " << Bricks << ": " << DescribeVerdict(Run.Verdict));
	CHECK(Run.Verdict.Failures.size() == static_cast<std::size_t>(Bricks));
	CHECK(Run.Verdict.Clusters.empty());
	CHECK(Run.Verdict.State == CascadeState::Stands);
	std::size_t Severed = 0;
	for (const FailedInterface& Failure : Run.Verdict.Failures)
	{
		Severed += Failure.bSevered ? 1u : 0u;
	}
	CHECK(Severed == 0u);
	return Run;
}

struct CrossFixture
{
	Structure Model;
	BodyId B1;
	BodyId B2;
	BodyId B3;
	BodyId B4;
	InterfaceId GroundB2;
	InterfaceId Cross;
	InterfaceId InsideY;
};

CrossFixture MakeCrossFixture(bool bCross, bool bInsideY)
{
	CrossFixture F;
	const ProfileId Profile = AddProfileOk(F.Model, DryJoint(0.7));
	const BodyId Ground = F.Model.AddBody(BrickAt(StackedBrickCentroidM(0), true)).Value;
	F.B1 = F.Model.AddBody(BrickAt(StackedBrickCentroidM(1))).Value;
	F.B2 = F.Model.AddBody(BrickAt(StackedBrickCentroidM(2))).Value;
	F.B3 = F.Model.AddBody(BrickAt(StackedBrickCentroidM(3))).Value;
	F.B4 = F.Model.AddBody(BrickAt(StackedBrickCentroidM(4))).Value;
	F.GroundB2 = AddInterfaceOk(F.Model, BedJoint(Ground, F.B2, 1, Profile));
	if (bCross)
	{
		F.Cross = AddInterfaceOk(F.Model, BedJoint(F.B1, F.B4, 2, Profile));
	}
	if (bInsideY)
	{
		F.InsideY = AddInterfaceOk(F.Model, BedJoint(F.B2, F.B3, 3, Profile));
	}
	return F;
}

AuthorityResult TwoCollapsesScript(const CrossFixture& F)
{
	AuthorityResult Script;
	Script.Worst = ComponentVerdict::Collapses;
	ComponentReport X;
	X.Bodies = {F.B1};
	X.Verdict = ComponentVerdict::Collapses;
	X.Release = {F.B1};
	ComponentReport Y;
	Y.Bodies = {F.B2, F.B3, F.B4};
	Y.Verdict = ComponentVerdict::Collapses;
	Y.Release = {F.B2, F.B3, F.B4};
	Script.Components = {X, Y};
	for (const InterfaceId Id : F.Model.LiveInterfaces())
	{
		InterfaceReading Reading;
		Reading.Id = Id;
		Script.Readings.push_back(Reading);
	}
	return Script;
}
}

TEST_CASE("Slow.Cascade.Scans.AFieldOfOverstressedBricksCracksInLinearTime")
{
	TimeCrackField(CrackFieldBricks);
	const double SmallMs = TimeCrackField(CrackFieldBricks).Ms;
	const double LargeMs = TimeCrackField(4 * CrackFieldBricks).Ms;
	const double Ratio = LargeMs / SmallMs;
	MESSAGE("overstressed bricks: " << CrackFieldBricks << " in " << SmallMs << " ms, " << 4 * CrackFieldBricks << " in " << LargeMs << " ms, ratio " << Ratio);
	CHECK(Ratio < QuadraticRatioBar);
}

TEST_CASE("Cascade.Scans.AnInterfaceBetweenTwoComponentsMergesNothing")
{
	SUBCASE("an interface from one component's released body to another's")
	{
		CrossFixture F = MakeCrossFixture(true, false);
		const CascadeVerdict V = RunScripted(F.Model, {TwoCollapsesScript(F), StandsScript()});
		INFO(DescribeVerdict(V));
		CHECK(V.State == CascadeState::Stands);
		REQUIRE(V.Clusters.size() == 4u);
		CHECK(SameIdList(V.Clusters[0].Bodies, {F.B1}));
		CHECK(SameIdList(V.Clusters[1].Bodies, {F.B2}));
		CHECK(SameIdList(V.Clusters[2].Bodies, {F.B3}));
		CHECK(SameIdList(V.Clusters[3].Bodies, {F.B4}));
		CHECK(SameIdList(V.Clusters[0].Severed, {F.Cross}));
		CHECK(SameIdList(V.Clusters[1].Severed, {F.GroundB2}));
		CHECK(V.Clusters[2].Severed.empty());
		CHECK(V.Clusters[3].Severed.empty());
		std::size_t CrossFailures = 0;
		for (const FailedInterface& Failure : V.Failures)
		{
			CrossFailures += SameHandle(Failure.Id, F.Cross) ? 1u : 0u;
		}
		CHECK(CrossFailures == 1u);
	}
	SUBCASE("one component's hinge naming an interface inside another")
	{
		CrossFixture F = MakeCrossFixture(false, true);
		AuthorityResult Script = TwoCollapsesScript(F);
		Script.Components[0].Hinging = {F.InsideY};
		const CascadeVerdict V = RunScripted(F.Model, {Script, StandsScript()});
		INFO(DescribeVerdict(V));
		CHECK(V.State == CascadeState::Stands);
		REQUIRE(V.Clusters.size() == 3u);
		CHECK(SameIdList(V.Clusters[0].Bodies, {F.B1}));
		CHECK(SameIdList(V.Clusters[1].Bodies, {F.B2, F.B3}));
		CHECK(SameIdList(V.Clusters[2].Bodies, {F.B4}));
		CHECK(SameIdList(V.Clusters[1].Severed, {F.GroundB2}));
		for (const FailedInterface& Failure : V.Failures)
		{
			CHECK_FALSE(SameHandle(Failure.Id, F.InsideY));
		}
	}
}
