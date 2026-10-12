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
