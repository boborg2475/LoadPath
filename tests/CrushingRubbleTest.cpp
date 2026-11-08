#include "BitwiseTestSupport.h"
#include "CascadeTestSupport.h"

#include "loadpath/Cascade.h"
#include "loadpath/Settings.h"
#include "loadpath/StructuralAuthority.h"

#include "doctest.h"

#include <cstddef>
#include <memory>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
BodyId AddBrick(Structure& Model, const Box& B, double CompressiveStrengthPa)
{
	BodyDesc Desc;
	Desc.MassKg = B.MassKg;
	Desc.CentroidM = B.CentreM;
	Desc.CompressiveStrengthPa = CompressiveStrengthPa;
	const Result<BodyId> Added = Model.AddBody(Desc);
	CHECK(Added.Ok());
	return Added.Value;
}

struct Row
{
	Structure Model;
	std::vector<BodyId> Bricks;
	std::vector<InterfaceId> Beds;
};

Row MakeRow(const StrengthProfile& Profile, int Count)
{
	Row R;
	const ProfileId P = AddProfileOk(R.Model, Profile);
	const BodyId Ground = AddGroundPlane(R.Model);
	for (int K = 0; K < Count; ++K)
	{
		const Box B = BrickShape(1.0 * K);
		R.Bricks.push_back(AddBrick(R.Model, B, 0.0));
		R.Beds.push_back(AddInterfaceOk(R.Model, OnGroundPlane(Ground, R.Bricks.back(), B, P)));
	}
	return R;
}

struct Stack
{
	Structure Model;
	BodyId Lower, Upper;
	InterfaceId Bed, Joint;
};

Stack MakeStack(const StrengthProfile& BedProfile, const StrengthProfile& JointProfile, double LowerPa, double UpperPa)
{
	Stack S;
	const ProfileId BedId = AddProfileOk(S.Model, BedProfile);
	const ProfileId JointId = AddProfileOk(S.Model, JointProfile);
	const BodyId Ground = AddGroundPlane(S.Model);
	S.Lower = AddBrick(S.Model, BrickShape(0.0, 0), LowerPa);
	S.Upper = AddBrick(S.Model, BrickShape(0.0, 1), UpperPa);
	S.Bed = AddInterfaceOk(S.Model, OnGroundPlane(Ground, S.Lower, BrickShape(0.0, 0), BedId));
	S.Joint = AddInterfaceOk(S.Model, *StackedFaces(S.Lower, BrickShape(0.0, 0), S.Upper, BrickShape(0.0, 1), JointId));
	return S;
}

std::vector<BodyId> CrushedIds(const CascadeVerdict& V)
{
	std::vector<BodyId> Ids;
	for (const CrushedBody& C : V.CrushedBodies)
	{
		Ids.push_back(C.Body);
	}
	return Ids;
}

std::vector<InterfaceId> FailedOfKind(const CascadeVerdict& V, FailureKind Kind)
{
	std::vector<InterfaceId> Ids;
	for (const FailedInterface& F : V.Failures)
	{
		if (F.Kind == Kind)
		{
			Ids.push_back(F.Id);
		}
	}
	return Ids;
}

StrengthProfile CrushingBed()
{
	StrengthProfile Profile = DryJoint(0.7);
	Profile.CompressiveStrengthPa = 0.5 * BrickMassKg * GravityMPerS2 / (BrickLengthM * BrickWidthM);
	return Profile;
}

StrengthProfile StrongDry()
{
	StrengthProfile Profile = DryJoint(0.7);
	Profile.CompressiveStrengthPa = 1.0e9;
	return Profile;
}
}

TEST_CASE("Cascade.Crushing.OnlyTheBodyInTheBandCrushes")
{
	Row R = MakeRow(DryJoint(0.7), 3);
	const AuthorityResult First = OverstressedScript(R.Model, {{R.Beds[0], 0.0, 0.0, 3.0}, {R.Beds[1], 0.0, 0.0, 1.2}, {R.Beds[2], 0.0, 0.0, 0.5}});
	const CascadeVerdict V = RunScripted(R.Model, {First, StandsScript()});
	INFO(DescribeVerdict(V));
	CHECK(V.State == CascadeState::Stands);
	CHECK(V.Passes == 1);
	CHECK(V.Clusters.empty());
	CHECK(V.CrushedBodies.size() == 1u);
	if (!(V.CrushedBodies.size() == 1u))
	{
		return;
	}
	CHECK(V.CrushedBodies[0].Pass == 1u);
	CHECK(SameHandle(V.CrushedBodies[0].Body, R.Bricks[0]));
	CHECK(SameHandle(V.CrushedBodies[0].Interface, R.Beds[0]));
	CHECK(V.CrushedBodies[0].Ratio == 3.0);
	CHECK(V.Failures.size() == 1u);
	if (!(V.Failures.size() == 1u))
	{
		return;
	}
	CHECK(SameHandle(V.Failures[0].Id, R.Beds[0]));
	CHECK(V.Failures[0].Kind == FailureKind::Crushed);
	CHECK_FALSE(V.Failures[0].bSevered);
	CHECK(V.Failures[0].Axis == GoverningAxis::Compression);
	CHECK(V.Failures[0].Ratio == 3.0);
	CHECK_FALSE(R.Model.IsLive(R.Bricks[0]));
	CHECK(R.Model.IsLive(R.Bricks[1]));
	CHECK(R.Model.IsLive(R.Bricks[2]));
}

TEST_CASE("Cascade.Crushing.TwoInsideTheBandCrushInOnePass")
{
	Row R = MakeRow(DryJoint(0.7), 3);
	const AuthorityResult First = OverstressedScript(R.Model, {{R.Beds[0], 0.0, 0.0, 2.0}, {R.Beds[1], 0.0, 0.0, 1.6}, {R.Beds[2], 0.0, 0.0, 1.2}});
	const CascadeVerdict V = RunScripted(R.Model, {First, StandsScript()}, 0.25);
	INFO(DescribeVerdict(V));
	CHECK(V.Passes == 1);
	CHECK(SameIdList(CrushedIds(V), {R.Bricks[0], R.Bricks[1]}));
	for (const CrushedBody& C : V.CrushedBodies)
	{
		CHECK(C.Pass == 1u);
	}
	CHECK(SameIdList(FailedOfKind(V, FailureKind::Crushed), {R.Beds[0], R.Beds[1]}));
	CHECK(R.Model.IsLive(R.Bricks[2]));
}

TEST_CASE("Cascade.Crushing.CrackingAndCrushingShareTheBand")
{
	SUBCASE("the crack leads and the crush is below its band")
	{
		Row R = MakeRow(BondedJoint(1.0e5, 1.0e5, 0.7), 2);
		const CascadeVerdict V = RunScripted(R.Model, {OverstressedScript(R.Model, {{R.Beds[0], 3.0, 0.0, 0.0}, {R.Beds[1], 0.0, 0.0, 1.2}}), StandsScript()});
		INFO(DescribeVerdict(V));
		CHECK(SameIdList(FailedOfKind(V, FailureKind::Cracked), {R.Beds[0]}));
		CHECK(V.CrushedBodies.empty());
		CHECK(R.Model.IsLive(R.Bricks[1]));
	}
	SUBCASE("the crush leads and the crack is below its band")
	{
		Row R = MakeRow(BondedJoint(1.0e5, 1.0e5, 0.7), 2);
		const CascadeVerdict V = RunScripted(R.Model, {OverstressedScript(R.Model, {{R.Beds[0], 1.2, 0.0, 0.0}, {R.Beds[1], 0.0, 0.0, 3.0}}), StandsScript()});
		INFO(DescribeVerdict(V));
		CHECK(FailedOfKind(V, FailureKind::Cracked).empty());
		CHECK(SameIdList(CrushedIds(V), {R.Bricks[1]}));
		CheckStamps(R.Model, R.Beds[0], InterfaceState::Intact, 0, 0);
	}
	SUBCASE("both inside the band fail in the same pass")
	{
		Row R = MakeRow(BondedJoint(1.0e5, 1.0e5, 0.7), 2);
		const CascadeVerdict V = RunScripted(R.Model, {OverstressedScript(R.Model, {{R.Beds[0], 2.0, 0.0, 0.0}, {R.Beds[1], 0.0, 0.0, 1.8}}), StandsScript()});
		INFO(DescribeVerdict(V));
		CHECK(V.Passes == 1);
		CHECK(SameIdList(FailedOfKind(V, FailureKind::Cracked), {R.Beds[0]}));
		CHECK(SameIdList(CrushedIds(V), {R.Bricks[1]}));
	}
}

TEST_CASE("Cascade.Crushing.TheWeakerMaterialCrushesElseTheLowerBody")
{
	const auto CrushedAtTheJoint = [](double LowerPa, double UpperPa) {
		Stack S = MakeStack(StrongDry(), StrongDry(), LowerPa, UpperPa);
		const CascadeVerdict V = RunScripted(S.Model, {OverstressedScript(S.Model, {{S.Joint, 0.0, 0.0, 2.0}}), StandsScript()});
		INFO(DescribeVerdict(V));
		CHECK(V.CrushedBodies.size() == 1u);
		if (!(V.CrushedBodies.size() == 1u))
		{
			return -1;
		}
		CHECK(SameHandle(V.CrushedBodies[0].Interface, S.Joint));
		return SameHandle(V.CrushedBodies[0].Body, S.Lower) ? 0 : SameHandle(V.CrushedBodies[0].Body, S.Upper) ? 1 : -1;
	};
	CHECK(CrushedAtTheJoint(0.0, 0.0) == 0);
	CHECK(CrushedAtTheJoint(20.0e6, 20.0e6) == 0);
	CHECK(CrushedAtTheJoint(20.0e6, 2.5e6) == 1);
	CHECK(CrushedAtTheJoint(2.5e6, 20.0e6) == 0);
	CHECK(CrushedAtTheJoint(0.0, 2.5e6) == 0);

	Stack S = MakeStack(StrongDry(), StrongDry(), 100.0e6, 1.0e6);
	const CascadeVerdict V = RunScripted(S.Model, {OverstressedScript(S.Model, {{S.Bed, 0.0, 0.0, 2.0}}), StandsScript()});
	INFO("ground bed: " << DescribeVerdict(V));
	CHECK(SameIdList(CrushedIds(V), {S.Lower}));
}

TEST_CASE("Cascade.Crushing.ACrushedBodyIsRemovedAndWhatItCarriedFallsNextPass")
{
	Stack S = MakeStack(CrushingBed(), StrongDry(), 0.0, 0.0);
	const CascadeVerdict V = MakeCascade(MakeAuthority())->Run(S.Model, {}, CascadeOptions{}, nullptr);
	INFO(DescribeVerdict(V));
	CHECK(V.State == CascadeState::Stands);
	CHECK(V.Passes == 2);
	CHECK(V.CrushedBodies.size() == 1u);
	if (!(V.CrushedBodies.size() == 1u))
	{
		return;
	}
	CHECK(V.CrushedBodies[0].Pass == 1u);
	CHECK(SameHandle(V.CrushedBodies[0].Body, S.Lower));
	CHECK(SameHandle(V.CrushedBodies[0].Interface, S.Bed));
	CheckRelative(V.CrushedBodies[0].Ratio, 4.0, 1.0e-6, "bed compression utilisation");
	CHECK(V.Clusters.size() == 1u);
	if (!(V.Clusters.size() == 1u))
	{
		return;
	}
	CHECK(V.Clusters[0].Pass == 2u);
	CHECK(SameIdList(V.Clusters[0].Bodies, {S.Upper}));
	CHECK_FALSE(S.Model.IsLive(S.Lower));
	CHECK_FALSE(S.Model.IsLive(S.Upper));
}

TEST_CASE("Cascade.Recorder.CrushesAreRecordedInTheirPass")
{
	const DebugSettings RecordingOn{true};
	Stack S = MakeStack(CrushingBed(), StrongDry(), 0.0, 0.0);
	CascadeOptions Options;
	Options.Authority.Standing.Debug = &RecordingOn;
	const CascadeVerdict V = MakeCascade(MakeAuthority())->Run(S.Model, {}, Options, nullptr);
	INFO(DescribeVerdict(V));
	CHECK(V.PassRecords.size() == 3u);
	if (!(V.PassRecords.size() == 3u))
	{
		return;
	}
	CHECK(bitwise::Same(V.PassRecords[0].Crushed, V.CrushedBodies));
	CHECK(V.PassRecords[0].Clusters.empty());
	CHECK(V.PassRecords[1].Crushed.empty());
	CHECK(V.PassRecords[1].Clusters.size() == 1u);
	CHECK(V.PassRecords[2].Crushed.empty());
}
