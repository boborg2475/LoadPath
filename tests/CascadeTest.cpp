#include "AnchorFixtures.h"
#include "ArchFixtures.h"
#include "CascadeTestSupport.h"
#include "SlowInDebug.h"

#include "loadpath/StructureFile.h"

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double PastFriction = 0.6;
constexpr double PastFrictionLateral = 0.8;
constexpr double PierFriction = 0.7;
constexpr double PierCohesionPa = 1.0e5;
constexpr double PierPushN = 50.0;

CascadeVerdict RunCascade(Structure& Model, const CascadeOptions& Options, std::span<const BodyLoad> Loads = {})
{
	return MakeCascade(MakeAuthority())->Run(Model, Loads, Options, nullptr);
}

CascadeOptions Lateral(double LambdaX)
{
	CascadeOptions Options;
	Options.Authority = AuthorityLateral(LambdaX);
	return Options;
}

double WeightN()
{
	return BrickMassKg * GravityMPerS2;
}

double CornerAreaM2()
{
	return BrickLengthM * BrickWidthM / 4.0;
}

double PierCornerTensionN(int Course, double PushN)
{
	const double ArmM = (3 - Course) * BrickHeightM;
	const double WeightAboveN = (3 - Course) * WeightN();
	return PushN * ArmM / (2.0 * BrickLengthM) - WeightAboveN / 4.0;
}

struct Pier
{
	Structure Model;
	std::vector<BodyId> Bricks;
	std::vector<InterfaceId> JointsBottomUp;
	std::vector<BodyLoad> Loads;
};

Pier MakePier(const double (&TensilePa)[3], double PushN)
{
	Pier P;
	const BodyId Ground = AddGroundPlane(P.Model);
	for (int Course = 0; Course < 3; ++Course)
	{
		P.Bricks.push_back(AddBox(P.Model, BrickShape(0.0, Course)));
	}
	for (int Course = 0; Course < 3; ++Course)
	{
		const ProfileId Profile = AddProfileOk(P.Model, BondedJoint(TensilePa[Course], PierCohesionPa, PierFriction));
		const Box Shape = BrickShape(0.0, Course);
		if (Course == 0)
		{
			P.JointsBottomUp.push_back(AddInterfaceOk(P.Model, OnGroundPlane(Ground, P.Bricks[0], Shape, Profile)));
		}
		else
		{
			P.JointsBottomUp.push_back(AddInterfaceOk(P.Model, *StackedFaces(P.Bricks[static_cast<std::size_t>(Course - 1)], BrickShape(0.0, Course - 1), P.Bricks[static_cast<std::size_t>(Course)], Shape, Profile)));
		}
	}
	P.Loads.push_back(BodyLoad{P.Bricks[2], Vec3{PushN, 0.0, 0.0}, Vec3{0.0, 0.0, 3.0 * BrickHeightM}});
	return P;
}

constexpr double PierBaseRatio = 1.3;

Pier MakeCrackingPier()
{
	const double FtPa = PierCornerTensionN(0, PierPushN) / (PierBaseRatio * CornerAreaM2());
	const double Ft[3] = {FtPa, FtPa, FtPa};
	return MakePier(Ft, PierPushN);
}

Pier MakeBandPier()
{
	const double BaseTensionN = PierCornerTensionN(0, PierPushN);
	const double FtPa = BaseTensionN / (3.0 * CornerAreaM2());
	const double TopFtPa = (BaseTensionN / 3.0) / (1.2 * CornerAreaM2());
	const double Ft[3] = {FtPa, FtPa, TopFtPa};
	return MakePier(Ft, PierPushN);
}

void CheckPierFixture()
{
	CHECK(PierPushN > WeightN() * BrickLengthM / (2.0 * BrickHeightM));
	CHECK(PierPushN < PierFriction * 3.0 * WeightN());
	CHECK(PierCornerTensionN(0, PierPushN) > 0.0);
	CHECK(std::abs(PierCornerTensionN(1, PierPushN) - 2.0 / 3.0 * PierCornerTensionN(0, PierPushN)) <= 1.0e-9);
	CHECK(PierPushN / 4.0 < 0.1 * PierCohesionPa * CornerAreaM2());
}

const InterfaceReading* FindReadingOf(const std::vector<InterfaceReading>& Readings, InterfaceId Id)
{
	for (const InterfaceReading& Reading : Readings)
	{
		if (SameHandle(Reading.Id, Id))
		{
			return &Reading;
		}
	}
	return nullptr;
}

bool SameOrBothNaN(double A, double B)
{
	return A == B || (std::isnan(A) && std::isnan(B));
}

struct PierInModel
{
	std::vector<BodyId> Bricks;
	std::vector<InterfaceId> JointsBottomUp;
	BodyLoad Load;
};

Box BrickShapeAtY(int Course, double YM)
{
	Box Shape = BrickShape(0.0, Course);
	Shape.CentreM.Y = YM;
	return Shape;
}

PierInModel AddUniformPier(Structure& Model, double YM, double BaseRatio)
{
	PierInModel P;
	const double FtPa = PierCornerTensionN(0, PierPushN) / (BaseRatio * CornerAreaM2());
	const ProfileId Profile = AddProfileOk(Model, BondedJoint(FtPa, PierCohesionPa, PierFriction));
	const BodyId Ground = AddGroundPlane(Model);
	for (int Course = 0; Course < 3; ++Course)
	{
		P.Bricks.push_back(AddBox(Model, BrickShapeAtY(Course, YM)));
	}
	P.JointsBottomUp.push_back(AddInterfaceOk(Model, OnGroundPlane(Ground, P.Bricks[0], BrickShapeAtY(0, YM), Profile)));
	for (int Course = 1; Course < 3; ++Course)
	{
		const std::size_t Below = static_cast<std::size_t>(Course - 1);
		P.JointsBottomUp.push_back(AddInterfaceOk(Model, *StackedFaces(P.Bricks[Below], BrickShapeAtY(Course - 1, YM), P.Bricks[Below + 1], BrickShapeAtY(Course, YM), Profile)));
	}
	P.Load = BodyLoad{P.Bricks[2], Vec3{PierPushN, 0.0, 0.0}, Vec3{0.0, YM, 3.0 * BrickHeightM}};
	return P;
}

struct DryStack
{
	Structure Model;
	BodyId Lower;
	BodyId Upper;
	InterfaceId Bed;
	InterfaceId Joint;
};

DryStack MakeDryStackSlidingAtTheGround()
{
	DryStack S;
	const ProfileId BedProfile = AddProfileOk(S.Model, DryJoint(PastFriction));
	const ProfileId JointProfile = AddProfileOk(S.Model, DryJoint(0.9));
	const BodyId Ground = AddGroundPlane(S.Model);
	S.Lower = AddBox(S.Model, BrickShape(0.0, 0));
	S.Upper = AddBox(S.Model, BrickShape(0.0, 1));
	S.Bed = AddInterfaceOk(S.Model, OnGroundPlane(Ground, S.Lower, BrickShape(0.0, 0), BedProfile));
	S.Joint = AddInterfaceOk(S.Model, *StackedFaces(S.Lower, BrickShape(0.0, 0), S.Upper, BrickShape(0.0, 1), JointProfile));
	return S;
}

std::vector<InterfaceId> CrackedOnPass(const CascadeVerdict& V, std::uint32_t Pass)
{
	std::vector<InterfaceId> Ids;
	for (const FailedInterface& F : V.Failures)
	{
		if (F.Pass == Pass && !F.bSevered)
		{
			Ids.push_back(F.Id);
		}
	}
	return Ids;
}

void CheckNothingChanged(const CascadeVerdict& V)
{
	CHECK(V.Passes == 0);
	CHECK(V.Failures.empty());
	CHECK(V.Clusters.empty());
}

void CheckIdenticalVec3(const Vec3& A, const Vec3& B)
{
	CHECK(A.X == B.X);
	CHECK(A.Y == B.Y);
	CHECK(A.Z == B.Z);
}
}

TEST_CASE("Cascade.DryBrickPastFrictionReleasesTheBrickWithTheBedSevered")
{
	BrickOnGround F = MakeBrickOnGround(DryJoint(PastFriction));
	const CascadeVerdict V = RunCascade(F.Model, Lateral(PastFrictionLateral));
	INFO(DescribeVerdict(V));
	CHECK(V.State == CascadeState::Stands);
	CHECK(V.Passes == 1);
	CHECK(V.FirstPass == 1u);
	CHECK(V.LastPass == 1u);
	CHECK(V.Clusters.size() == 1u);
	if (!(V.Clusters.size() == 1u))
	{
		return;
	}
	CHECK(V.Clusters[0].Pass == 1u);
	CHECK(SameIdList(V.Clusters[0].Bodies, {F.Brick}));
	CHECK(SameIdList(V.Clusters[0].Severed, {F.Bed}));
	CHECK(SameHandle(V.Clusters[0].Motion.Body, F.Brick));
	CHECK(V.Failures.size() == 1u);
	if (!(V.Failures.size() == 1u))
	{
		return;
	}
	CHECK(SameHandle(V.Failures[0].Id, F.Bed));
	CHECK(V.Failures[0].Pass == 1u);
	CHECK(V.Failures[0].bSevered);
	CHECK_FALSE(F.Model.IsLive(F.Brick));
	CHECK(F.Model.LiveInterfaceCount() == 0u);
	CheckStamps(F.Model, F.Bed, InterfaceState::Severed, 0, 1);
	CHECK(F.Model.HighestPass() == 1u);
}

TEST_CASE("Cascade.StackSlidingAtTheGroundIsReleasedAsOneCluster")
{
	DryStack S = MakeDryStackSlidingAtTheGround();
	CHECK(PastFrictionLateral > PastFriction / std::cos(Pi / 16.0));
	CHECK(PastFrictionLateral < 0.9);
	const CascadeVerdict V = RunCascade(S.Model, Lateral(PastFrictionLateral));
	INFO(DescribeVerdict(V));
	CHECK(V.State == CascadeState::Stands);
	CHECK(V.Passes == 1);
	CHECK(V.Clusters.size() == 1u);
	if (!(V.Clusters.size() == 1u))
	{
		return;
	}
	CHECK(SameIdList(V.Clusters[0].Bodies, {S.Lower, S.Upper}));
	CHECK(SameIdList(V.Clusters[0].Severed, {S.Bed}));
	CHECK(V.Failures.size() == 1u);
	if (!(V.Failures.size() == 1u))
	{
		return;
	}
	CHECK(SameHandle(V.Failures[0].Id, S.Bed));
	CHECK(V.Failures[0].bSevered);
	CheckStamps(S.Model, S.Bed, InterfaceState::Severed, 0, 1);
	CheckStamps(S.Model, S.Joint, InterfaceState::Removed, 0, 0);
	CHECK_FALSE(S.Model.IsLive(S.Lower));
	CHECK_FALSE(S.Model.IsLive(S.Upper));
}

TEST_CASE("Cascade.StackWithOnlyTheTopPastItsEdgeReleasesTheTopAlone")
{
	Stack S = BuildOnTable({PlacedBlock{0.0, 0}, PlacedBlock{0.0, 1}, PlacedBlock{0.6, 2}}, 1.0);
	CHECK(S.Blocks.size() == 3u);
	if (!(S.Blocks.size() == 3u))
	{
		return;
	}
	const std::vector<InterfaceId> UnderTop = S.Model.InterfacesOf(S.Blocks[2]);
	CHECK(UnderTop.size() == 1u);
	if (!(UnderTop.size() == 1u))
	{
		return;
	}
	const std::vector<InterfaceId> UnderMiddle = S.Model.InterfacesOf(S.Blocks[0]);
	CascadeOptions Options;
	const CascadeVerdict V = RunCascade(S.Model, Options);
	INFO(DescribeVerdict(V));
	CHECK(V.State == CascadeState::Stands);
	CHECK(V.Passes == 1);
	CHECK(V.Clusters.size() == 1u);
	if (!(V.Clusters.size() == 1u))
	{
		return;
	}
	CHECK(SameIdList(V.Clusters[0].Bodies, {S.Blocks[2]}));
	CHECK(SameIdList(V.Clusters[0].Severed, UnderTop));
	CheckStamps(S.Model, UnderTop[0], InterfaceState::Severed, 0, 1);
	CHECK(S.Model.IsLive(S.Blocks[0]));
	CHECK(S.Model.IsLive(S.Blocks[1]));
	CHECK_FALSE(S.Model.IsLive(S.Blocks[2]));
	for (const InterfaceId Id : UnderMiddle)
	{
		CheckStamps(S.Model, Id, InterfaceState::Intact, 0, 0);
	}
}

TEST_CASE("Cascade.BondedPierCracksItsBaseThenTipsAsOneCluster")
{
	CheckPierFixture();
	Pier P = MakeCrackingPier();
	const AuthorityResult Before = Judge(P.Model, AuthorityOptions{}, P.Loads);
	INFO("pass-1 judge: " << DescribeAuthority(Before) << "; readings" << DescribeReadings(Before.Readings));
	CHECK(Before.Worst == ComponentVerdict::Overstressed);
	const CascadeVerdict V = RunCascade(P.Model, CascadeOptions{}, P.Loads);
	INFO(DescribeVerdict(V));
	CHECK(V.State == CascadeState::Stands);
	CHECK(V.Passes == 2);
	CHECK(V.FirstPass == 1u);
	CHECK(V.LastPass == 2u);
	CHECK(V.Failures.size() == 2u);
	if (!(V.Failures.size() == 2u))
	{
		return;
	}
	CHECK(SameHandle(V.Failures[0].Id, P.JointsBottomUp[0]));
	CHECK(V.Failures[0].Pass == 1u);
	CHECK_FALSE(V.Failures[0].bSevered);
	CHECK(V.Failures[0].Axis == GoverningAxis::Tension);
	CHECK(V.Failures[0].Ratio > 1.0);
	CheckRelative(V.Failures[0].Ratio, PierBaseRatio, 1.0e-6, "base crack ratio");
	CHECK(SameHandle(V.Failures[1].Id, P.JointsBottomUp[0]));
	CHECK(V.Failures[1].Pass == 2u);
	CHECK(V.Failures[1].bSevered);
	CHECK(V.Clusters.size() == 1u);
	if (!(V.Clusters.size() == 1u))
	{
		return;
	}
	CHECK(V.Clusters[0].Pass == 2u);
	CHECK(SameIdList(V.Clusters[0].Bodies, P.Bricks));
	CHECK(SameIdList(V.Clusters[0].Severed, {P.JointsBottomUp[0]}));
	CheckStamps(P.Model, P.JointsBottomUp[0], InterfaceState::Severed, 1, 2);
	CheckStamps(P.Model, P.JointsBottomUp[1], InterfaceState::Removed, 0, 0);
	CheckStamps(P.Model, P.JointsBottomUp[2], InterfaceState::Removed, 0, 0);
}

TEST_CASE("Cascade.TopBandCracksJointsWithinTheBandOfTheWorstRatio")
{
	CheckPierFixture();
	struct Case
	{
		double Delta;
		bool Cracks[3];
	};
	const Case Cases[] = {{0.5, {true, true, false}}, {0.0, {true, false, false}}, {1.0, {true, true, true}}};
	for (const Case& Each : Cases)
	{
		INFO("delta " << Each.Delta);
		Pier P = MakeBandPier();
		const AuthorityResult Before = Judge(P.Model, AuthorityOptions{}, P.Loads);
		INFO("pass-1 judge: " << DescribeAuthority(Before) << "; readings" << DescribeReadings(Before.Readings));
		CHECK(Before.Worst == ComponentVerdict::Overstressed);
		CHECK(Before.Components.size() == 1u);
		const double WantRatios[3] = {3.0, 2.0, 1.2};
		for (std::size_t K = 0; K < 3; ++K)
		{
			const InterfaceReading* Reading = FindReadingOf(Before.Readings, P.JointsBottomUp[K]);
			CHECK(Reading != nullptr);
			if (Reading != nullptr)
			{
				CheckRelative(std::max(Reading->TensionUtilisation, Reading->ShearUtilisation), WantRatios[K], 1.0e-9, "band pre-judge crack ratio");
			}
		}
		CascadeOptions Options;
		Options.CrackBandFraction = Each.Delta;
		const CascadeVerdict V = RunCascade(P.Model, Options, P.Loads);
		INFO(DescribeVerdict(V));
		std::vector<InterfaceId> Want;
		for (std::size_t K = 0; K < 3; ++K)
		{
			if (Each.Cracks[K])
			{
				Want.push_back(P.JointsBottomUp[K]);
			}
		}
		CHECK(SameIdList(CrackedOnPass(V, 1), Want));
		for (std::size_t K = 0; K < 3; ++K)
		{
			INFO("joint " << K);
			CHECK((P.Model.Interface(P.JointsBottomUp[K]).CrackPass == 1u) == Each.Cracks[K]);
		}
	}
}

TEST_CASE("Cascade.CrushingOverloadCracksNothingAndReleasesTheBrick")
{
	BrickOnGround F = MakeBrickOnGround(CrushedByOwnWeight());
	const CascadeVerdict V = RunCascade(F.Model, CascadeOptions{});
	INFO(DescribeVerdict(V));
	CHECK(V.State == CascadeState::Stands);
	CHECK(V.Passes == 1);
	CHECK(V.Failures.size() == 1u);
	if (!(V.Failures.size() == 1u))
	{
		return;
	}
	CHECK(SameHandle(V.Failures[0].Id, F.Bed));
	CHECK(V.Failures[0].Pass == 1u);
	CHECK(V.Failures[0].bSevered);
	CHECK(V.Failures[0].Axis == GoverningAxis::Compression);
	CHECK(V.Clusters.size() == 1u);
	if (!(V.Clusters.size() == 1u))
	{
		return;
	}
	CHECK(SameIdList(V.Clusters[0].Bodies, {F.Brick}));
	CHECK(SameIdList(V.Clusters[0].Severed, {F.Bed}));
	CheckStamps(F.Model, F.Bed, InterfaceState::Severed, 0, 1);
}

TEST_CASE("Cascade.UndecidedComponentChangesNothing")
{
	const double Lambda = 0.7;
	CHECK(Lambda > PastFriction);
	BrickOnGround F = MakeBrickOnGround(DryJoint(PastFriction));
	CascadeOptions Options = Lateral(Lambda);
	Options.Authority.Standing.MaxActiveSetIterations = 1;
	{
		const AuthorityResult Judged = Judge(F.Model, Options.Authority);
		INFO("judge: " << DescribeAuthority(Judged));
		CHECK(Judged.Worst == ComponentVerdict::Undecided);
	}
	const CascadeVerdict V = RunCascade(F.Model, Options);
	INFO(DescribeVerdict(V));
	CHECK(V.State == CascadeState::Undecided);
	CheckNothingChanged(V);
	CheckStamps(F.Model, F.Bed, InterfaceState::Intact, 0, 0);
	CHECK(F.Model.IsLive(F.Brick));
	CHECK(V.Components.size() == 1u);
}

TEST_CASE("Cascade.PassCapEndsAfterTheCrackAndASecondRunContinues")
{
	CheckPierFixture();
	Pier P = MakeCrackingPier();
	CascadeOptions Capped;
	Capped.MaxPasses = 1;
	const CascadeVerdict First = RunCascade(P.Model, Capped, P.Loads);
	{
		INFO("first run: " << DescribeVerdict(First));
		CHECK(First.State == CascadeState::PassCapReached);
		CHECK(First.Passes == 1);
		CHECK(First.FirstPass == 1u);
		CHECK(First.LastPass == 1u);
		CHECK(First.Clusters.empty());
		CHECK(First.Failures.size() == 1u);
		if (!(First.Failures.size() == 1u))
		{
			return;
		}
		CHECK(SameHandle(First.Failures[0].Id, P.JointsBottomUp[0]));
		CHECK_FALSE(First.Failures[0].bSevered);
		CheckStamps(P.Model, P.JointsBottomUp[0], InterfaceState::Cracked, 1, 0);
		CHECK(P.Model.LiveBodyCount() == 4u);
		const AuthorityResult Fresh = Judge(P.Model, AuthorityOptions{}, P.Loads);
		INFO("fresh judge: " << DescribeAuthority(Fresh) << "; readings" << DescribeReadings(Fresh.Readings) << "; returned readings" << DescribeReadings(First.Readings));
		const InterfaceReading* Base = FindReadingOf(First.Readings, P.JointsBottomUp[0]);
		CHECK(Base != nullptr);
		if (Base != nullptr)
		{
			CHECK_FALSE(std::abs(Base->TensionUtilisation - PierBaseRatio) <= 1.0e-6);
		}
		CHECK(First.Readings.size() == Fresh.Readings.size());
		for (std::size_t I = 0; I < std::min(First.Readings.size(), Fresh.Readings.size()); ++I)
		{
			INFO("reading " << I);
			CHECK(SameHandle(First.Readings[I].Id, Fresh.Readings[I].Id));
			CHECK(SameOrBothNaN(First.Readings[I].Utilisation, Fresh.Readings[I].Utilisation));
			CHECK(SameOrBothNaN(First.Readings[I].TensionUtilisation, Fresh.Readings[I].TensionUtilisation));
		}
		CHECK(First.Components.size() == Fresh.Components.size());
		for (std::size_t I = 0; I < std::min(First.Components.size(), Fresh.Components.size()); ++I)
		{
			INFO("component " << I);
			CHECK(First.Components[I].Verdict == Fresh.Components[I].Verdict);
			CHECK(SameIdList(First.Components[I].Bodies, Fresh.Components[I].Bodies));
		}
	}
	const CascadeVerdict Second = RunCascade(P.Model, CascadeOptions{}, P.Loads);
	INFO("second run: " << DescribeVerdict(Second));
	CHECK(Second.State == CascadeState::Stands);
	CHECK(Second.Passes == 1);
	CHECK(Second.FirstPass == 2u);
	CHECK(Second.LastPass == 2u);
	CHECK(Second.Clusters.size() == 1u);
	if (!(Second.Clusters.size() == 1u))
	{
		return;
	}
	CHECK(Second.Clusters[0].Pass == 2u);
	CHECK(SameIdList(Second.Clusters[0].Bodies, P.Bricks));
	CheckStamps(P.Model, P.JointsBottomUp[0], InterfaceState::Severed, 1, 2);
}

TEST_CASE("Cascade.GroundedOnlyStructureStands")
{
	Structure Model;
	const ProfileId Profile = AddProfileOk(Model, DryJoint(0.7));
	const BodyId Ground = AddGroundPlane(Model);
	const BodyId Footing = AddBox(Model, BrickShape(), true);
	const InterfaceId Bed = AddInterfaceOk(Model, OnGroundPlane(Ground, Footing, BrickShape(), Profile));
	const CascadeVerdict V = RunCascade(Model, CascadeOptions{});
	INFO(DescribeVerdict(V));
	CHECK(V.State == CascadeState::Stands);
	CheckNothingChanged(V);
	CheckStamps(Model, Bed, InterfaceState::Intact, 0, 0);
}

TEST_CASE("Cascade.InvalidOptionsAreInvalidInputAndStampNothing")
{
	struct Case
	{
		const char* Name;
		double Delta;
		int MaxPasses;
	};
	const Case Cases[] = {{"band NaN", std::numeric_limits<double>::quiet_NaN(), 32}, {"band 1.5", 1.5, 32}, {"band -0.1", -0.1, 32}, {"max passes 0", 0.5, 0}};
	for (const Case& Each : Cases)
	{
		INFO("case: " << doctest::String(Each.Name));
		BrickOnGround F = MakeBrickOnGround(DryJoint(PastFriction));
		CascadeOptions Options = Lateral(PastFrictionLateral);
		Options.CrackBandFraction = Each.Delta;
		Options.MaxPasses = Each.MaxPasses;
		const CascadeVerdict V = RunCascade(F.Model, Options);
		INFO(DescribeVerdict(V));
		CHECK(V.State == CascadeState::InvalidInput);
		CheckNothingChanged(V);
		CheckStamps(F.Model, F.Bed, InterfaceState::Intact, 0, 0);
		CHECK(F.Model.IsLive(F.Brick));
		CHECK(F.Model.HighestPass() == 0u);
	}
}

TEST_CASE("Cascade.FreeBodyIsReleasedOnPassOneAsItsOwnCluster")
{
	Structure Model;
	AddGroundPlane(Model);
	const BodyId Floating = AddBox(Model, BrickShape(0.0, 3));
	const CascadeVerdict V = RunCascade(Model, CascadeOptions{});
	INFO(DescribeVerdict(V));
	CHECK(V.State == CascadeState::Stands);
	CHECK(V.Passes == 1);
	CHECK(V.Clusters.size() == 1u);
	if (!(V.Clusters.size() == 1u))
	{
		return;
	}
	CHECK(V.Clusters[0].Pass == 1u);
	CHECK(SameIdList(V.Clusters[0].Bodies, {Floating}));
	CHECK(V.Clusters[0].Severed.empty());
	CHECK(V.Failures.empty());
	CHECK_FALSE(Model.IsLive(Floating));
	CHECK(V.LastPass == 1u);
	CHECK(Model.HighestPass() == 1u);
	const BodyId Second = AddBox(Model, BrickShape(1.0, 3));
	const CascadeVerdict Again = RunCascade(Model, CascadeOptions{});
	INFO("second run: " << DescribeVerdict(Again));
	CHECK(Again.FirstPass == 2u);
	CHECK(Again.LastPass == 2u);
	CHECK(Model.HighestPass() == 2u);
	CHECK(Again.Clusters.size() == 1u);
	if (!(Again.Clusters.size() == 1u))
	{
		return;
	}
	CHECK(Again.Clusters[0].Pass == 2u);
	CHECK(SameIdList(Again.Clusters[0].Bodies, {Second}));
}

TEST_CASE("Cascade.TwoIndependentCollapsesGiveTwoClustersInSlotOrder")
{
	Structure Model;
	const ProfileId Profile = AddProfileOk(Model, DryJoint(PastFriction));
	const BodyId Ground = AddGroundPlane(Model);
	const BodyId Left = AddBox(Model, BrickShape(0.0));
	const BodyId Right = AddBox(Model, BrickShape(1.0));
	const InterfaceId LeftBed = AddInterfaceOk(Model, OnGroundPlane(Ground, Left, BrickShape(0.0), Profile));
	const InterfaceId RightBed = AddInterfaceOk(Model, OnGroundPlane(Ground, Right, BrickShape(1.0), Profile));
	const CascadeVerdict V = RunCascade(Model, Lateral(PastFrictionLateral));
	INFO(DescribeVerdict(V));
	CHECK(V.State == CascadeState::Stands);
	CHECK(V.Passes == 1);
	CHECK(V.Clusters.size() == 2u);
	if (!(V.Clusters.size() == 2u))
	{
		return;
	}
	CHECK(SameIdList(V.Clusters[0].Bodies, {Left}));
	CHECK(SameIdList(V.Clusters[0].Severed, {LeftBed}));
	CHECK(SameIdList(V.Clusters[1].Bodies, {Right}));
	CHECK(SameIdList(V.Clusters[1].Severed, {RightBed}));
	CHECK(V.Clusters[0].Pass == 1u);
	CHECK(V.Clusters[1].Pass == 1u);
	CHECK(V.Failures.size() == 2u);
	if (!(V.Failures.size() == 2u))
	{
		return;
	}
	CHECK(SameHandle(V.Failures[0].Id, LeftBed));
	CHECK(SameHandle(V.Failures[1].Id, RightBed));
	CheckStamps(Model, LeftBed, InterfaceState::Severed, 0, 1);
	CheckStamps(Model, RightBed, InterfaceState::Severed, 0, 1);
}

TEST_CASE("Cascade.SameInputGivesIdenticalVerdicts")
{
	Pier A = MakeCrackingPier();
	Pier B = MakeCrackingPier();
	const CascadeVerdict VA = RunCascade(A.Model, CascadeOptions{}, A.Loads);
	const CascadeVerdict VB = RunCascade(B.Model, CascadeOptions{}, B.Loads);
	INFO("first " << DescribeVerdict(VA) << "; second " << DescribeVerdict(VB));
	CHECK(VA.State == VB.State);
	CHECK(VA.Passes == VB.Passes);
	CHECK(VA.FirstPass == VB.FirstPass);
	CHECK(VA.LastPass == VB.LastPass);
	CHECK_FALSE(VA.Failures.empty());
	CHECK_FALSE(VA.Clusters.empty());
	CHECK(VA.Failures.size() == VB.Failures.size());
	if (!(VA.Failures.size() == VB.Failures.size()))
	{
		return;
	}
	for (std::size_t I = 0; I < VA.Failures.size(); ++I)
	{
		INFO("failure " << I);
		CHECK(SameHandle(VA.Failures[I].Id, VB.Failures[I].Id));
		CHECK(VA.Failures[I].Pass == VB.Failures[I].Pass);
		CHECK(VA.Failures[I].Axis == VB.Failures[I].Axis);
		CHECK(VA.Failures[I].Ratio == VB.Failures[I].Ratio);
		CHECK(VA.Failures[I].bSevered == VB.Failures[I].bSevered);
	}
	CHECK(VA.Clusters.size() == VB.Clusters.size());
	if (!(VA.Clusters.size() == VB.Clusters.size()))
	{
		return;
	}
	for (std::size_t I = 0; I < VA.Clusters.size(); ++I)
	{
		INFO("cluster " << I);
		const ReleasedCluster& CA = VA.Clusters[I];
		const ReleasedCluster& CB = VB.Clusters[I];
		CHECK(CA.Pass == CB.Pass);
		CHECK(SameIdList(CA.Bodies, CB.Bodies));
		CHECK(SameIdList(CA.Severed, CB.Severed));
		CHECK(SameHandle(CA.Motion.Body, CB.Motion.Body));
		CheckIdenticalVec3(CA.Motion.TranslationM, CB.Motion.TranslationM);
		CheckIdenticalVec3(CA.Motion.RotationRad, CB.Motion.RotationRad);
	}
}

TEST_CASE("Cascade.PassCounterSurvivesTheStructureFile")
{
	BrickOnGround F = MakeBrickOnGround(DryJoint(PastFriction));
	const CascadeVerdict V = RunCascade(F.Model, Lateral(PastFrictionLateral));
	INFO(DescribeVerdict(V));
	CHECK(V.Clusters.size() == 1u);
	CHECK(F.Model.HighestPass() == 1u);
	LoadedStructure Original;
	Original.Model = F.Model;
	const std::string Text = WriteStructure(Original);
	INFO(Text);
	LoadResult Loaded = LoadStructure(Text);
	INFO("line " << Loaded.Error.Line << ": " << Loaded.Error.Message);
	CHECK(Loaded.Ok());
	if (!Loaded.Ok())
	{
		return;
	}
	CHECK(Loaded.Value.Model.HighestPass() == 1u);
	const CascadeVerdict Next = RunCascade(Loaded.Value.Model, CascadeOptions{});
	INFO("reloaded run: " << DescribeVerdict(Next));
	CHECK(Next.FirstPass == 2u);
}

TEST_CASE("Cascade.RunIsInvalidInputWhenThePassCounterCannotAdvance")
{
	BrickOnGround F = MakeBrickOnGround(DryJoint(PastFriction));
	CHECK(F.Model.MarkPass(std::numeric_limits<std::uint32_t>::max()));
	const CascadeVerdict V = RunCascade(F.Model, Lateral(PastFrictionLateral));
	INFO(DescribeVerdict(V));
	CHECK(V.State == CascadeState::InvalidInput);
	CheckNothingChanged(V);
	CheckStamps(F.Model, F.Bed, InterfaceState::Intact, 0, 0);
	CHECK(F.Model.IsLive(F.Brick));
}

TEST_CASE("Cascade.ARunThatExhaustsThePassCounterStopsAtTheCap")
{
	CheckPierFixture();
	Pier P = MakeCrackingPier();
	constexpr std::uint32_t Last = std::numeric_limits<std::uint32_t>::max();
	CHECK(P.Model.MarkPass(Last - 1));
	const CascadeVerdict V = RunCascade(P.Model, CascadeOptions{}, P.Loads);
	INFO(DescribeVerdict(V));
	CHECK(V.State == CascadeState::PassCapReached);
	CHECK(V.Passes == 1);
	CHECK(V.FirstPass == Last);
	CHECK(V.LastPass == Last);
	CHECK(P.Model.HighestPass() == Last);
	for (const FailedInterface& Failure : V.Failures)
	{
		CHECK(Failure.Pass == Last);
	}
	for (const ReleasedCluster& Cluster : V.Clusters)
	{
		CHECK(Cluster.Pass == Last);
	}
	for (const InterfaceId Id : P.JointsBottomUp)
	{
		const InterfaceRecord& R = P.Model.Interface(Id);
		INFO("joint " << Id.Index << ": state " << doctest::String(InterfaceStateName(R.State)) << ", crack " << R.CrackPass << ", sever " << R.SeverPass);
		CHECK((R.State != InterfaceState::Cracked || R.CrackPass == Last));
		CHECK((R.State != InterfaceState::Severed || R.SeverPass == Last));
	}
	for (const BodyId Brick : P.Bricks)
	{
		if (P.Model.IsLive(Brick))
		{
			continue;
		}
		INFO("brick " << Brick.Index << " not live");
		const bool bRecorded = std::any_of(V.Clusters.begin(), V.Clusters.end(), [&](const ReleasedCluster& C) { return std::any_of(C.Bodies.begin(), C.Bodies.end(), [&](BodyId B) { return SameHandle(B, Brick); }); });
		CHECK(bRecorded);
	}
	CHECK_FALSE(V.Readings.empty());
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Cascade.ArchBelowLeastThicknessReleasesVoussoirRunsBetweenHinges")
{
	SemicircularArch Arch = MakeSemicircularArch(0.1075 - 0.002, DryJoint(0.7));
	const std::vector<double> HingeAnglesDeg = {-90.0, -ArchHaunchJointDeg, -54.0, 0.0, 54.0, ArchHaunchJointDeg, 90.0};
	const CascadeVerdict V = RunCascade(Arch.Model, CascadeOptions{});
	INFO(DescribeVerdict(V));
	CHECK(V.Passes == 1);
	std::vector<bool> bSevered(Arch.Joints.size(), false);
	std::size_t SeveredCount = 0;
	for (std::size_t K = 0; K < Arch.Joints.size(); ++K)
	{
		INFO("joint " << K << " at " << Arch.JointAnglesDeg[K] << " deg");
		const bool bHinge = std::any_of(HingeAnglesDeg.begin(), HingeAnglesDeg.end(), [&](double A) { return std::abs(A - Arch.JointAnglesDeg[K]) < 1.0e-9; });
		bSevered[K] = bHinge;
		SeveredCount += bHinge ? 1u : 0u;
		int Listed = 0;
		for (const FailedInterface& Failure : V.Failures)
		{
			if (SameHandle(Failure.Id, Arch.Joints[K]))
			{
				++Listed;
				CHECK(Failure.bSevered);
				CHECK(Failure.Pass == 1u);
			}
		}
		CHECK(Listed == (bHinge ? 1 : 0));
		if (bHinge)
		{
			CheckStamps(Arch.Model, Arch.Joints[K], InterfaceState::Severed, 0, 1);
		}
		else
		{
			CHECK(Arch.Model.Interface(Arch.Joints[K]).CrackPass == 0u);
			CHECK(Arch.Model.Interface(Arch.Joints[K]).SeverPass == 0u);
		}
	}
	CHECK(V.Failures.size() == SeveredCount);
	CHECK(V.Clusters.size() == SeveredCount - 1u);
	std::vector<int> ClusterOf(Arch.Voussoirs.size(), 0);
	for (const ReleasedCluster& Cluster : V.Clusters)
	{
		INFO("cluster bodies " << Describe(Cluster.Bodies));
		std::vector<std::size_t> Indices;
		for (const BodyId Body : Cluster.Bodies)
		{
			const auto Found = std::find_if(Arch.Voussoirs.begin(), Arch.Voussoirs.end(), [&](BodyId Each) { return SameHandle(Each, Body); });
			CHECK(Found != Arch.Voussoirs.end());
			if (Found != Arch.Voussoirs.end())
			{
				const std::size_t I = static_cast<std::size_t>(Found - Arch.Voussoirs.begin());
				Indices.push_back(I);
				++ClusterOf[I];
			}
		}
		if (Indices.empty())
		{
			continue;
		}
		std::sort(Indices.begin(), Indices.end());
		const std::size_t First = Indices.front();
		const std::size_t Last = Indices.back();
		CHECK(Last - First + 1u == Indices.size());
		CHECK(bSevered[First]);
		CHECK(bSevered[Last + 1u]);
		for (std::size_t K = First + 1u; K <= Last; ++K)
		{
			INFO("inner joint " << K);
			CHECK_FALSE(bSevered[K]);
		}
	}
	for (std::size_t I = 0; I < ClusterOf.size(); ++I)
	{
		INFO("voussoir " << I);
		CHECK(ClusterOf[I] == 1);
	}
	for (const BodyId Body : Arch.Model.LiveBodies())
	{
		CHECK(Arch.Model.Body(Body).bGrounded);
	}
}

TEST_CASE("Cascade.Crack.BandIsTakenPerOverstressedComponent")
{
	CheckPierFixture();
	Structure Model;
	const PierInModel Strong = AddUniformPier(Model, 0.0, 3.0);
	const PierInModel Weak = AddUniformPier(Model, 5.0, PierBaseRatio);
	const std::vector<BodyLoad> Loads = {Strong.Load, Weak.Load};
	{
		const AuthorityResult Before = Judge(Model, AuthorityOptions{}, Loads);
		INFO("pass-1 judge: " << DescribeAuthority(Before) << "; readings" << DescribeReadings(Before.Readings));
		CHECK(Before.Components.size() == 2u);
		const InterfaceReading* StrongBase = FindReadingOf(Before.Readings, Strong.JointsBottomUp[0]);
		const InterfaceReading* WeakBase = FindReadingOf(Before.Readings, Weak.JointsBottomUp[0]);
		CHECK(StrongBase != nullptr);
		CHECK(WeakBase != nullptr);
		if (StrongBase != nullptr && WeakBase != nullptr)
		{
			CheckRelative(StrongBase->TensionUtilisation, 3.0, 1.0e-6, "strong pier base ratio");
			CheckRelative(WeakBase->TensionUtilisation, PierBaseRatio, 1.0e-6, "weak pier base ratio");
		}
	}
	CascadeOptions Options;
	Options.CrackBandFraction = 0.0;
	const CascadeVerdict V = RunCascade(Model, Options, Loads);
	INFO(DescribeVerdict(V));
	CHECK(SameIdList(CrackedOnPass(V, 1), {Strong.JointsBottomUp[0], Weak.JointsBottomUp[0]}));
}

namespace
{
struct BondedRow
{
	Structure Model;
	std::vector<InterfaceId> Joints;
	std::vector<BodyId> Bricks;
};

BondedRow MakeBondedRow(std::size_t Count, const StrengthProfile& Profile = BondedJoint(1.0e5, 1.0e5, 0.7))
{
	BondedRow Row;
	const ProfileId P = AddProfileOk(Row.Model, Profile);
	const BodyId Ground = AddGroundPlane(Row.Model);
	for (std::size_t I = 0; I < Count; ++I)
	{
		const Box Shape = BrickShape(0.5 * static_cast<double>(I));
		Row.Bricks.push_back(AddBox(Row.Model, Shape));
		Row.Joints.push_back(AddInterfaceOk(Row.Model, OnGroundPlane(Ground, Row.Bricks.back(), Shape, P)));
	}
	return Row;
}

constexpr double NaNRatio = std::numeric_limits<double>::quiet_NaN();
constexpr double InfRatio = std::numeric_limits<double>::infinity();
}

TEST_CASE("Cascade.Crack.NaNOnEitherAxisCracks")
{
	struct Case
	{
		const char* Name;
		double T;
		double S;
	};
	const Case Cases[] = {{"tension NaN", NaNRatio, 0.5}, {"shear NaN", 0.5, NaNRatio}};
	for (const Case& Each : Cases)
	{
		INFO(doctest::String(Each.Name));
		BondedRow Row = MakeBondedRow(1);
		const CascadeVerdict V = RunScripted(Row.Model, {OverstressedScript(Row.Model, {{Row.Joints[0], Each.T, Each.S}})});
		INFO(DescribeVerdict(V));
		CHECK(SameIdList(CrackedOnPass(V, 1), {Row.Joints[0]}));
		CheckStamps(Row.Model, Row.Joints[0], InterfaceState::Cracked, 1, 0);
	}
}

TEST_CASE("Cascade.Crack.InfiniteRatioCracksOutsideTheBand")
{
	BondedRow Row = MakeBondedRow(3);
	const CascadeVerdict V = RunScripted(Row.Model, {OverstressedScript(Row.Model, {{Row.Joints[0], InfRatio, 0.0}, {Row.Joints[1], 3.0, 0.0}, {Row.Joints[2], 2.0, 0.0}}), StandsScript()}, 0.0);
	INFO(DescribeVerdict(V));
	CHECK(SameIdList(CrackedOnPass(V, 1), {Row.Joints[0], Row.Joints[1]}));
	CheckStamps(Row.Model, Row.Joints[0], InterfaceState::Cracked, 1, 0);
}

TEST_CASE("Cascade.Crack.BandMaximumIgnoresNonFiniteRatios")
{
	BondedRow Row = MakeBondedRow(4);
	const CascadeVerdict V = RunScripted(Row.Model, {OverstressedScript(Row.Model, {{Row.Joints[0], NaNRatio, 0.0}, {Row.Joints[1], InfRatio, 0.0}, {Row.Joints[2], 3.0, 0.0}, {Row.Joints[3], 1.2, 0.0}}), StandsScript()}, 0.5);
	INFO(DescribeVerdict(V));
	CHECK(SameIdList(CrackedOnPass(V, 1), {Row.Joints[0], Row.Joints[1], Row.Joints[2]}));
	CheckStamps(Row.Model, Row.Joints[3], InterfaceState::Intact, 0, 0);
}

TEST_CASE("Cascade.Crack.ATensionShearTieRecordsTension")
{
	BondedRow Row = MakeBondedRow(1);
	const CascadeVerdict V = RunScripted(Row.Model, {OverstressedScript(Row.Model, {{Row.Joints[0], 2.0, 2.0}})});
	INFO(DescribeVerdict(V));
	CHECK(V.Failures.size() == 1u);
	if (!(V.Failures.size() == 1u))
	{
		return;
	}
	CHECK(V.Failures[0].Axis == GoverningAxis::Tension);
	CHECK(V.Failures[0].Ratio == 2.0);
}

TEST_CASE("Cascade.Crack.ADryInterfaceIsNeverCracked")
{
	BondedRow Row = MakeBondedRow(1, DryJoint(0.7));
	const CascadeVerdict V = RunScripted(Row.Model, {OverstressedScript(Row.Model, {{Row.Joints[0], 3.0, 2.0}})});
	INFO(DescribeVerdict(V));
	CHECK(V.Failures.empty());
	CheckStamps(Row.Model, Row.Joints[0], InterfaceState::Intact, 0, 0);
}

TEST_CASE("Cascade.CollapsesWithAnEmptyReleaseEndsUndecided")
{
	BondedRow Row = MakeBondedRow(1);
	const CascadeVerdict V = RunScripted(Row.Model, {CollapsesScript(Row.Model, {})});
	INFO(DescribeVerdict(V));
	CHECK(V.State == CascadeState::Undecided);
	CheckNothingChanged(V);
	CheckStamps(Row.Model, Row.Joints[0], InterfaceState::Intact, 0, 0);
	CHECK(Row.Model.IsLive(Row.Bricks[0]));
	CHECK(Row.Model.HighestPass() == 0u);
}

TEST_CASE("Cascade.AnOverturnedBrickSeversItsBedInTension")
{
	constexpr double HighFriction = 5.0;
	constexpr double LambdaY = 2.0;
	CHECK(LambdaY > BrickWidthM / BrickHeightM);
	CHECK(LambdaY < HighFriction * std::cos(Pi / 16.0));
	BrickOnGround F = MakeBrickOnGround(DryJoint(HighFriction));
	CascadeOptions Options;
	Options.Authority = AuthorityUnderGravity(Vec3{0.0, LambdaY * GravityMPerS2, -GravityMPerS2});
	const CascadeVerdict V = RunCascade(F.Model, Options);
	INFO(DescribeVerdict(V));
	CHECK(V.Failures.size() == 1u);
	if (!(V.Failures.size() == 1u))
	{
		return;
	}
	CHECK(SameHandle(V.Failures[0].Id, F.Bed));
	CHECK(V.Failures[0].bSevered);
	CHECK(V.Failures[0].Axis == GoverningAxis::Tension);
}

TEST_CASE("Cascade.ASlidingBrickSeversItsBedInShear")
{
	CHECK(PastFrictionLateral > PastFriction / std::cos(Pi / 16.0));
	CHECK(PastFrictionLateral < BrickLengthM / BrickHeightM);
	BrickOnGround F = MakeBrickOnGround(DryJoint(PastFriction));
	const CascadeVerdict V = RunCascade(F.Model, Lateral(PastFrictionLateral));
	INFO(DescribeVerdict(V));
	CHECK(V.Failures.size() == 1u);
	if (!(V.Failures.size() == 1u))
	{
		return;
	}
	CHECK(SameHandle(V.Failures[0].Id, F.Bed));
	CHECK(V.Failures[0].bSevered);
	CHECK(V.Failures[0].Axis == GoverningAxis::Shear);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Cascade.ArchBelowLeastThicknessOpensEveryHingeItSevers")
{
	SemicircularArch Arch = MakeSemicircularArch(0.1075 - 0.002, DryJoint(0.7));
	const CascadeVerdict V = RunCascade(Arch.Model, CascadeOptions{});
	INFO(DescribeVerdict(V));
	CHECK(V.Failures.size() == 7u);
	for (const FailedInterface& Failure : V.Failures)
	{
		INFO("interface " << Failure.Id.Index << " axis " << Failure.Axis << " ratio " << Failure.Ratio);
		CHECK(Failure.bSevered);
		CHECK(Failure.Axis == GoverningAxis::Tension);
	}
}

namespace
{
struct ScriptedSever
{
	CascadeVerdict Verdict;
	InterfaceId Bed;
};

ScriptedSever SeverUnderMotion(const StrengthProfile& Profile, const Vec3& TranslationM, const Vec3& RotationRad, GoverningAxis ReadingAxis)
{
	BrickOnGround F = MakeBrickOnGround(Profile);
	AuthorityResult Script = CollapsesScript(F.Model, {F.Brick});
	Script.Components[0].Mechanism = {MechanismMotion{F.Brick, TranslationM, RotationRad}};
	for (InterfaceReading& Reading : Script.Readings)
	{
		Reading.Axis = ReadingAxis;
	}
	ScriptedSever Result{RunScripted(F.Model, {Script, StandsScript()}), F.Bed};
	return Result;
}

Vec3 BedCornerVelocity(const Vec3& TranslationM, const Vec3& RotationRad, double XM, double YM)
{
	const Vec3 Arm{XM, YM, -0.5 * BrickHeightM};
	return Vec3{TranslationM.X + RotationRad.Y * Arm.Z - RotationRad.Z * Arm.Y, TranslationM.Y + RotationRad.Z * Arm.X - RotationRad.X * Arm.Z,
		TranslationM.Z + RotationRad.X * Arm.Y - RotationRad.Y * Arm.X};
}

void CheckSeveredAxis(const ScriptedSever& Got, GoverningAxis Want)
{
	INFO(DescribeVerdict(Got.Verdict));
	INFO("want " << doctest::String(GoverningAxisName(Want)));
	REQUIRE(Got.Verdict.Failures.size() == 1u);
	CHECK(SameHandle(Got.Verdict.Failures[0].Id, Got.Bed));
	CHECK(Got.Verdict.Failures[0].bSevered);
	CHECK(Got.Verdict.Failures[0].Axis == Want);
}

double LiftForMargin(double Friction, double Margin)
{
	double Lift = Friction;
	for (int I = 0; I < 50; ++I)
	{
		Lift = Friction + Margin * std::sqrt(1.0 + Lift * Lift);
	}
	return Lift;
}
}

TEST_CASE("Cascade.AJointThatOpensWhileItSlidesSeversInTension")
{
	const double SpinRad = 0.1 / BrickLengthM;
	const Vec3 Rotation{0.0, SpinRad, 0.0};
	const Vec3 Translation{1.0 + 0.5 * BrickHeightM * SpinRad, 0.0, 0.6 + 0.05};
	for (const double YM : {-0.5 * BrickWidthM, 0.5 * BrickWidthM})
	{
		const Vec3 Toe = BedCornerVelocity(Translation, Rotation, 0.5 * BrickLengthM, YM);
		const Vec3 Heel = BedCornerVelocity(Translation, Rotation, -0.5 * BrickLengthM, YM);
		INFO("toe (" << Toe.X << ", " << Toe.Y << ", " << Toe.Z << "), heel (" << Heel.X << ", " << Heel.Y << ", " << Heel.Z << ")");
		CHECK(std::abs(Toe.X - 1.0) < 1.0e-12);
		CHECK(std::abs(Toe.Z - 0.6) < 1.0e-12);
		CHECK(std::abs(Heel.X - 1.0) < 1.0e-12);
		CHECK(std::abs(Heel.Z - 0.7) < 1.0e-12);
	}
	CheckSeveredAxis(SeverUnderMotion(DryJoint(PastFriction), Translation, Rotation, GoverningAxis::Shear), GoverningAxis::Tension);
}

TEST_CASE("Cascade.ALiftJustAboveTheDilatancyReadsTension")
{
	const double Lift = LiftForMargin(PastFriction, 2.0e-3);
	CHECK(std::abs(Lift - PastFriction - 2.0e-3 * std::sqrt(1.0 + Lift * Lift)) < 1.0e-14);
	CheckSeveredAxis(SeverUnderMotion(DryJoint(PastFriction), Vec3{1.0, 0.0, Lift}, Vec3{}, GoverningAxis::Shear), GoverningAxis::Tension);
}

TEST_CASE("Cascade.ALiftJustBelowTheThresholdReadsShear")
{
	const double Lift = LiftForMargin(PastFriction, 5.0e-4);
	CHECK(std::abs(Lift - PastFriction - 5.0e-4 * std::sqrt(1.0 + Lift * Lift)) < 1.0e-14);
	CheckSeveredAxis(SeverUnderMotion(DryJoint(PastFriction), Vec3{1.0, 0.0, Lift}, Vec3{}, GoverningAxis::Tension), GoverningAxis::Shear);
}

TEST_CASE("Cascade.AJointTheMechanismClosesSeversInCompression")
{
	for (const GoverningAxis ReadingAxis : {GoverningAxis::Shear, GoverningAxis::None})
	{
		INFO("reading axis " << doctest::String(GoverningAxisName(ReadingAxis)));
		CheckSeveredAxis(SeverUnderMotion(DryJoint(PastFriction), Vec3{0.0, 0.0, -1.0}, Vec3{}, ReadingAxis), GoverningAxis::Compression);
	}
}

TEST_CASE("Cascade.AZeroCapJointSubtractsNoFrictionLift")
{
	StrengthProfile Profile = DryJoint(0.7);
	Profile.MaxShearStrengthPa = 0.0;
	CheckSeveredAxis(SeverUnderMotion(Profile, Vec3{1.0, 0.0, 0.5}, Vec3{}, GoverningAxis::Shear), GoverningAxis::Tension);
}

TEST_CASE("Cascade.AJointRockingAboutItsCentreLineSeversInTension")
{
	const double SpinRad = 1.0 / BrickLengthM;
	const Vec3 Rotation{0.0, SpinRad, 0.0};
	const Vec3 Translation{0.5 * BrickHeightM * SpinRad, 0.0, 0.0};
	for (const double YM : {-0.5 * BrickWidthM, 0.5 * BrickWidthM})
	{
		const Vec3 Toe = BedCornerVelocity(Translation, Rotation, 0.5 * BrickLengthM, YM);
		const Vec3 Heel = BedCornerVelocity(Translation, Rotation, -0.5 * BrickLengthM, YM);
		INFO("toe (" << Toe.X << ", " << Toe.Y << ", " << Toe.Z << "), heel (" << Heel.X << ", " << Heel.Y << ", " << Heel.Z << ")");
		CHECK(std::abs(Toe.X) < 1.0e-12);
		CHECK(std::abs(Toe.Y) < 1.0e-12);
		CHECK(std::abs(Toe.Z + 0.5) < 1.0e-12);
		CHECK(std::abs(Heel.X) < 1.0e-12);
		CHECK(std::abs(Heel.Y) < 1.0e-12);
		CHECK(std::abs(Heel.Z - 0.5) < 1.0e-12);
	}
	CheckSeveredAxis(SeverUnderMotion(DryJoint(PastFriction), Translation, Rotation, GoverningAxis::Shear), GoverningAxis::Tension);
}

TEST_CASE("Cascade.AJointThatClosesWhileItSlidesSeversInCompression")
{
	CheckSeveredAxis(SeverUnderMotion(DryJoint(PastFriction), Vec3{1.0, 0.0, -1.0}, Vec3{}, GoverningAxis::Shear), GoverningAxis::Compression);
}
