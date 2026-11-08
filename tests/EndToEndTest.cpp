#include "EndToEndSupport.h"
#include "SlowInDebug.h"
#include "WarehouseTestSupport.h"

#include "loadpath/StructureFile.h"

#include "doctest.h"

#include <cmath>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

using namespace loadpath;
using namespace loadpath::test::e2e;

namespace
{
constexpr std::string_view ColonnadeDryMinusPost2 = "colonnade-dry-minus-post2";
constexpr std::string_view ColonnadeDryMinusPost0 = "colonnade-dry-minus-post0";
constexpr std::string_view ArchKeystoneIntact = "arch-keystone-intact";
constexpr std::string_view ArchKeystoneMinusCrownPair = "arch-keystone-minus-crown-pair";
constexpr std::string_view ArchKeystoneMinusOneCrownVoussoir = "arch-keystone-minus-one-crown-voussoir";
constexpr std::string_view WallEc6MinusTopEndBrick = "wall-ec6-minus-top-end-brick";
constexpr std::string_view WallEc6MinusMidBrick = "wall-ec6-minus-mid-brick";
constexpr std::string_view WallDryMinusTopEndBrick = "wall-dry-minus-top-end-brick";
constexpr std::string_view ColonnadeScrewed2MinusPost2 = "colonnade-screwed-2-screws-minus-post2";
constexpr std::string_view ColonnadeScrewed3MinusPost2 = "colonnade-screwed-3-screws-minus-post2";
constexpr std::string_view TwoBayScrewed2MinusMiddlePost = "two-bay-screwed-2-screws-minus-middle-post";
constexpr std::string_view TwoBayScrewed3MinusMiddlePost = "two-bay-screwed-3-screws-minus-middle-post";
constexpr std::string_view TwoBayDryMinusMiddlePost = "two-bay-dry-minus-middle-post";
constexpr std::string_view BondedWallPush098 = "bonded-wall-push-0-98";
constexpr std::string_view BondedWallPush102 = "bonded-wall-push-1-02";
constexpr std::string_view DryWallPush098 = "dry-wall-push-0-98";
constexpr std::string_view DryWallPush105 = "dry-wall-push-1-05";
constexpr std::string_view TimberBearingLoad08 = "timber-bearing-load-0-8";
constexpr std::string_view TimberBearingLoad12 = "timber-bearing-load-1-2";
constexpr std::string_view ShedMinusSidePosts = "shed-minus-side-posts";
constexpr std::string_view ShedMinusMiddlePost = "shed-minus-middle-post";
constexpr std::string_view ShedMinusCornerPost = "shed-minus-corner-post";
constexpr std::string_view CantileverNoCounterweight = "cantilever-no-counterweight";
constexpr std::string_view Cantilever16kg = "cantilever-16kg-counterweight";
constexpr std::string_view Cantilever4kg = "cantilever-4kg-counterweight";
constexpr std::string_view Cantilever16kgRemoved = "cantilever-16kg-counterweight-removed";
constexpr std::string_view FileColonnadeIntact = "file-colonnade-intact";
constexpr std::string_view FileColonnadeMinusPost2 = "file-colonnade-minus-post2";
constexpr std::string_view DryLintelIntact = "dry-lintel-intact";
constexpr std::string_view DryLintelRemoved = "dry-lintel-removed-expected";
constexpr std::string_view BondedLintelIntact = "bonded-lintel-intact";
constexpr std::string_view BondedLintelRemoved = "bonded-lintel-removed";
constexpr std::string_view DryWallMinusTwoBaseBricks = "dry-wall-minus-two-base-bricks";
constexpr std::string_view DryTwoBrickLintelRemoved = "dry-two-brick-lintel-removed";
constexpr std::string_view BuildOrderShuffled = "build-order-shuffled-screwed-colonnade";
constexpr std::string_view CornerStraightJointPush = "corner-straight-joint-push";
constexpr std::string_view CornerInterlockedPush = "corner-interlocked-push";

std::string PairName(const Scene& S, InterfaceId Id)
{
	const InterfaceRecord& R = S.Model.Interface(Id);
	std::string A = NameIn(S, R.BodyA);
	std::string B = NameIn(S, R.BodyB);
	return A < B ? A + "-" + B : B + "-" + A;
}

struct Cracks
{
	NameSet Pairs;
	bool bAllPassOne = true;
	bool bAllTension = true;
};

Cracks CrackedJoints(const Scene& S, const CascadeVerdict& V)
{
	Cracks Out;
	for (const FailedInterface& F : V.Failures)
	{
		if (F.bSevered)
		{
			continue;
		}
		Out.Pairs.push_back(PairName(S, F.Id));
		Out.bAllPassOne = Out.bAllPassOne && F.Pass == 1;
		Out.bAllTension = Out.bAllTension && F.Axis == GoverningAxis::Tension;
	}
	Out.Pairs = Sorted(std::move(Out.Pairs));
	return Out;
}

NameSet FreeNamesWhere(const Scene& S, const std::function<bool(const std::string&)>& Keep)
{
	NameSet Out;
	for (const SceneBox& B : S.Boxes)
	{
		if (!B.bGrounded && Keep(B.Name))
		{
			Out.push_back(B.Name);
		}
	}
	return Sorted(std::move(Out));
}

bool StartsWith(const std::string& Text, std::string_view Prefix) { return Text.compare(0, Prefix.size(), Prefix) == 0; }

bool Contains(const NameSet& Set, const std::string& Name) { return std::find(Set.begin(), Set.end(), Name) != Set.end(); }

std::size_t StandingChainLength(const ArchScene& A, const std::vector<std::size_t>& Voussoirs, const std::vector<std::size_t>& JointsBelow, double Friction)
{
	const double InnerM = test::ArchRadiusM - 0.5 * A.ThicknessM;
	const double OuterM = test::ArchRadiusM + 0.5 * A.ThicknessM;
	std::size_t Longest = 0;
	for (std::size_t Top = 0; Top < Voussoirs.size(); ++Top)
	{
		bool bStands = true;
		for (std::size_t J = 0; J <= Top && bStands; ++J)
		{
			double MassKg = 0.0;
			double MomentKgM = 0.0;
			for (std::size_t K = J; K <= Top; ++K)
			{
				const BodyDesc& Body = A.S.Model.Body(A.Voussoirs[Voussoirs[K]]);
				MassKg += Body.MassKg;
				MomentKgM += Body.MassKg * Body.CentroidM.X;
			}
			const double CentroidXM = MomentKgM / MassKg;
			const double AngleRad = A.JointAnglesDeg[JointsBelow[J]] * test::RadPerDegree;
			const double EndA = InnerM * std::sin(AngleRad);
			const double EndB = OuterM * std::sin(AngleRad);
			const bool bInside = CentroidXM >= std::min(EndA, EndB) && CentroidXM <= std::max(EndA, EndB);
			const bool bHolds = std::abs(std::cos(AngleRad)) <= Friction * std::abs(std::sin(AngleRad));
			bStands = bInside && bHolds;
		}
		if (bStands)
		{
			Longest = Top + 1;
		}
	}
	return Longest;
}

NameSet ArchOracleSurvivors(const ArchScene& A, std::size_t FirstRemoved, std::size_t LastRemoved, double Friction)
{
	std::vector<std::size_t> Left, LeftJoints, Right, RightJoints;
	for (std::size_t V = 0; V < FirstRemoved; ++V)
	{
		Left.push_back(V);
		LeftJoints.push_back(V);
	}
	for (std::size_t V = A.Voussoirs.size() - 1; V > LastRemoved; --V)
	{
		Right.push_back(V);
		RightJoints.push_back(V + 1);
	}
	const std::size_t LeftStanding = StandingChainLength(A, Left, LeftJoints, Friction);
	const std::size_t RightStanding = StandingChainLength(A, Right, RightJoints, Friction);
	INFO("oracle: " << LeftStanding << " stand on the left, " << RightStanding << " on the right");
	CHECK(LeftStanding == 12);
	CHECK(RightStanding == 12);
	NameSet Out;
	for (std::size_t K = 0; K < LeftStanding; ++K)
	{
		Out.push_back("v" + std::to_string(Left[K]));
	}
	for (std::size_t K = 0; K < RightStanding; ++K)
	{
		Out.push_back("v" + std::to_string(Right[K]));
	}
	return Sorted(std::move(Out));
}

std::string ReadText(const std::string& Path)
{
	std::ifstream File(Path, std::ios::binary);
	std::ostringstream Text;
	Text << File.rdbuf();
	return Text.str();
}

Scene SceneFromFile(const LoadedStructure& Loaded)
{
	Scene S;
	S.Model = Loaded.Model;
	for (const BodyId Id : S.Model.LiveBodies())
	{
		const std::string& Name = Loaded.BodyNames[Id.Index];
		const Vec3 Half = Name == "ground" ? Vec3{9.4, 5.0, 0.5} : (StartsWith(Name, "post") ? Vec3{0.1, 0.1, 1.0} : Vec3{1.0995, 0.1, 0.15});
		Record(S, Id, Name, Name == "ground" ? "ground" : "C24 timber", Half);
	}
	return S;
}

void CheckArchRemoval(std::size_t FirstRemoved, std::size_t LastRemoved, std::string_view Slug)
{
	ArchScene A = BuildArch(0.15, 0.7);
	CHECK(A.Voussoirs.size() == 62);
	const NameSet Want = A.Voussoirs.size() == 62 ? ArchOracleSurvivors(A, FirstRemoved, LastRemoved, 0.7) : NameSet{};
	for (std::size_t V = FirstRemoved; V <= LastRemoved && V < A.Voussoirs.size(); ++V)
	{
		RemoveBodyOf(A.S, A.Voussoirs[V]);
	}
	const CaseRun R = Run(A.S, Slug);
	RecordVisual(A.S, R);
	INFO("visual " << std::string(Slug) << ": " << Canonical(A.S, R.Verdict));
	INFO("survivors got " << Names(LiveFreeNames(A.S)) << " want " << Names(Want));
	CHECK(LiveFreeNames(A.S) == Want);
	CHECK(std::string(StateName(R.Verdict.State)) == "Stands");
	CHECK(JudgeRemainder(A.S) == ComponentVerdict::Stands);
}

void CheckNothingReleased(Scene& S, std::string_view Slug)
{
	ExpectedOutcome Want;
	Want.Passes = 0;
	Want.bNoFailures = true;
	const CaseRun R = Run(S, Slug);
	RecordVisual(S, R, Want);
	CheckOutcome(S, R, Want);
}

void CheckLateralPush(bool bBonded, double Factor, std::string_view Slug)
{
	WallScene W = BuildWall(bBonded, 5, 4);
	const double HeightM = 5 * BrickZM;
	const double BaseAreaM2 = 4 * BrickXM * BrickYM;
	const double WeightN = BrickDensity * 5 * 4 * BrickXM * BrickYM * BrickZM * GravityMPerS2;
	const double TensilePa = Ec6Bed().TensileStrengthPa;
	const double Limit = bBonded ? (BrickYM / HeightM) * (1.0 + TensilePa * BaseAreaM2 / WeightN) : BrickYM / HeightM;
	const double Lambda = Factor * Limit;
	W.S.Gravity = Vec3{0.0, Lambda * GravityMPerS2, -GravityMPerS2};
	const CaseRun R = Run(W.S, Slug);
	const NameSet AllBricks = FreeNamesWhere(W.S, [](const std::string&) { return true; });
	ExpectedOutcome Pictured;
	if (Factor >= 1.0)
	{
		Pictured.Clusters = {AllBricks};
	}
	RecordVisual(W.S, R, Pictured);
	INFO("lambda_c " << Limit << ", lambda " << Lambda);
	if (Factor < 1.0)
	{
		ExpectedOutcome Want;
		Want.Passes = 0;
		Want.bNoFailures = true;
		CheckOutcome(W.S, R, Want);
		return;
	}
	REQUIRE(AllBricks.size() == 22);
	INFO("visual " << std::string(Slug) << ": " << Canonical(W.S, R.Verdict));
	if (!bBonded)
	{
		CHECK(ReleasedNames(W.S, R.Verdict) == AllBricks);
		CHECK(LiveFreeNames(W.S).empty());
		return;
	}
	const Cracks Cracked = CrackedJoints(W.S, R.Verdict);
	const NameSet BaseBeds = Sorted({"c0b0-ground", "c0b1-ground", "c0b2-ground", "c0b3-ground"});
	INFO("cracked " << Names(Cracked.Pairs));
	CHECK(Cracked.Pairs == BaseBeds);
	CHECK(Cracked.bAllPassOne);
	CHECK(Cracked.bAllTension);
	ExpectedOutcome Want;
	Want.Clusters = {AllBricks};
	Want.ClusterPass = 2;
	Want.Passes = 2;
	CheckOutcome(W.S, R, Want);
}

void CheckLintelRemovalKeepsThePierBricks(Scene& S)
{
	RemoveNamed(S, "lintel");
	const NameSet MustFall = {"c6x2.50", "c6x3.50", "c6x4.50", "c6x5.50", "c7x3.00", "c7x4.00", "c7x5.00", "c8x3.50", "c8x4.50", "c9x4.00"};
	const CaseRun R = Run(S, DryLintelRemoved);
	NameSet MustStand = FreeNamesWhere(S, [](const std::string& Name) { return Name.size() > 1 && Name[0] == 'c' && Name[1] >= '0' && Name[1] <= '5'; });
	for (const char* Name : {"c6x0.50", "c6x7.50", "c7x0.25", "c7x7.75"})
	{
		MustStand.push_back(Name);
	}
	ExpectedOutcome Pictured;
	Pictured.Clusters = {MustFall};
	Pictured.Either = FreeNamesWhere(S, [&](const std::string& Name) { return !Contains(MustFall, Name) && !Contains(MustStand, Name); });
	RecordVisual(S, R, Pictured);
	INFO("visual " << std::string(DryLintelRemoved) << ": " << Canonical(S, R.Verdict));
	const NameSet Released = ReleasedNames(S, R.Verdict);
	INFO("released " << Names(Released));
	for (const std::string& Name : MustFall)
	{
		INFO("must fall " << Name);
		CHECK(Contains(Released, Name));
	}
	for (const std::string& Name : MustStand)
	{
		INFO("must stand " << Name);
		CHECK(S.Model.IsLive(SceneBody(S, Name)));
	}
}

std::vector<std::pair<std::string, std::function<Scene(unsigned)>>> ShuffleFixtures()
{
	auto Removing = [](Scene S, const char* Name) {
		RemoveNamed(S, Name);
		return S;
	};
	return {
		{"3.1 post2", [=](unsigned Seed) { return Removing(BuildColonnade(4, 0, Seed), "post2"); }},
		{"3.1 post0", [=](unsigned Seed) { return Removing(BuildColonnade(4, 0, Seed), "post0"); }},
		{"3.4 two screws", [=](unsigned Seed) { return Removing(BuildColonnade(4, 2, Seed), "post2"); }},
		{"3.4 three screws", [=](unsigned Seed) { return Removing(BuildColonnade(4, 3, Seed), "post2"); }},
		{"3.5 two screws", [=](unsigned Seed) { return Removing(BuildColonnade(2, 2, Seed), "post1"); }},
		{"3.5 three screws", [=](unsigned Seed) { return Removing(BuildColonnade(2, 3, Seed), "post1"); }},
		{"3.8 0.8", [](unsigned Seed) { return BuildTimberOnPiers(0.8, Seed); }},
		{"3.8 1.2", [](unsigned Seed) { return BuildTimberOnPiers(1.2, Seed); }},
		{"3.11 none", [](unsigned Seed) { return BuildCantilever(Counterweight::None, Seed); }},
		{"3.11 16 kg", [](unsigned Seed) { return BuildCantilever(Counterweight::Heavy, Seed); }},
		{"3.11 4 kg", [](unsigned Seed) { return BuildCantilever(Counterweight::Light, Seed); }},
		{"3.11 16 kg removed", [=](unsigned Seed) { return Removing(BuildCantilever(Counterweight::Heavy, Seed), "counterweight"); }},
	};
}
}

TEST_CASE("Parked.E2E.Colonnade.RemovingAPostDropsOnlyTheBeamsItCarried")
{
	SUBCASE("post 2")
	{
		Scene S = BuildColonnade(4, 0);
		RemoveNamed(S, "post2");
		const CaseRun R = Run(S, ColonnadeDryMinusPost2);
		const ExpectedOutcome Want{{{"beam1"}, {"beam2"}}, 1u};
		RecordVisual(S, R, Want);
		CheckOutcome(S, R, Want);
	}
	SUBCASE("post 0")
	{
		Scene S = BuildColonnade(4, 0);
		RemoveNamed(S, "post0");
		const CaseRun R = Run(S, ColonnadeDryMinusPost0);
		const ExpectedOutcome Want{{{"beam0"}}, 1u};
		RecordVisual(S, R, Want);
		CheckOutcome(S, R, Want);
	}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Parked.E2E.Arch.LosingTheKeystoneLeavesOnlyTheHaunchesFrictionHolds")
{
	SUBCASE("intact")
	{
		ArchScene A = BuildArch(0.15, 0.7);
		CheckNothingReleased(A.S, ArchKeystoneIntact);
	}
	SUBCASE("two crown voussoirs removed")
	{
		CheckArchRemoval(30, 31, ArchKeystoneMinusCrownPair);
	}
	SUBCASE("one crown voussoir removed")
	{
		CheckArchRemoval(31, 31, ArchKeystoneMinusOneCrownVoussoir);
	}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "E2E.Wall.RemovingANonStructuralBrickReleasesNothing")
{
	SUBCASE("EC6, top-course end brick")
	{
		WallScene W = BuildWall(true, 10, 8);
		RemoveWallBricks(W, 7.0, 8.0, 9, 10);
		CHECK(W.S.RemovedBeforeRun.size() == 1);
		CheckNothingReleased(W.S, WallEc6MinusTopEndBrick);
	}
	SUBCASE("EC6, mid-height middle brick")
	{
		WallScene W = BuildWall(true, 10, 8);
		RemoveWallBricks(W, 3.6, 4.4, 5, 6);
		CHECK(W.S.RemovedBeforeRun.size() == 1);
		CheckNothingReleased(W.S, WallEc6MinusMidBrick);
	}
	SUBCASE("dry, top-course end brick")
	{
		WallScene W = BuildWall(false, 10, 8);
		RemoveWallBricks(W, 7.0, 8.0, 9, 10);
		CHECK(W.S.RemovedBeforeRun.size() == 1);
		CheckNothingReleased(W.S, WallDryMinusTopEndBrick);
	}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Parked.E2E.Colonnade.ScrewedBearingsBridgeALostPostAboveTheWithdrawalThreshold")
{
	SUBCASE("two screws")
	{
		Scene S = BuildColonnade(4, 2);
		RemoveNamed(S, "post2");
		const CaseRun R = Run(S, ColonnadeScrewed2MinusPost2);
		const ExpectedOutcome Want{{{"beam1"}, {"beam2"}}, 2u, 2};
		RecordVisual(S, R, Want);
		const Cracks Cracked = CrackedJoints(S, R.Verdict);
		INFO("cracked " << Names(Cracked.Pairs) << "; " << Canonical(S, R.Verdict));
		CHECK(Cracked.Pairs == Sorted({"beam1-post1", "beam2-post3"}));
		CHECK(Cracked.bAllPassOne);
		CHECK(Cracked.bAllTension);
		CheckOutcome(S, R, Want);
	}
	SUBCASE("three screws")
	{
		Scene S = BuildColonnade(4, 3);
		RemoveNamed(S, "post2");
		CheckNothingReleased(S, ColonnadeScrewed3MinusPost2);
	}
}

TEST_CASE("Parked.E2E.Colonnade.AScrewedEndBayTipsWithItsPost")
{
	SUBCASE("two screws")
	{
		Scene S = BuildColonnade(2, 2);
		RemoveNamed(S, "post1");
		const CaseRun R = Run(S, TwoBayScrewed2MinusMiddlePost);
		const ExpectedOutcome Want{{{"post0", "beam0"}, {"post2", "beam1"}}, 1u};
		RecordVisual(S, R, Want);
		CheckOutcome(S, R, Want);
	}
	SUBCASE("three screws")
	{
		Scene S = BuildColonnade(2, 3);
		RemoveNamed(S, "post1");
		const CaseRun R = Run(S, TwoBayScrewed3MinusMiddlePost);
		const ExpectedOutcome Want{{{"post0", "beam0"}, {"post2", "beam1"}}, 1u};
		RecordVisual(S, R, Want);
		CheckOutcome(S, R, Want);
	}
	SUBCASE("dry bearings")
	{
		Scene S = BuildColonnade(2, 0);
		RemoveNamed(S, "post1");
		const CaseRun R = Run(S, TwoBayDryMinusMiddlePost);
		const ExpectedOutcome Want{{{"beam0"}, {"beam1"}}, 1u};
		RecordVisual(S, R, Want);
		CheckOutcome(S, R, Want);
	}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "E2E.Wall.BondedWallPushedPastItsLimitFallsAsOneSlab")
{
	SUBCASE("0.98 of the limit")
	{
		CheckLateralPush(true, 0.98, BondedWallPush098);
	}
	SUBCASE("1.02 of the limit")
	{
		CheckLateralPush(true, 1.02, BondedWallPush102);
	}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "E2E.Wall.DryWallPushedPastThicknessOverHeightFallsEntirely")
{
	SUBCASE("0.98 t/H")
	{
		CheckLateralPush(false, 0.98, DryWallPush098);
	}
	SUBCASE("1.05 t/H")
	{
		CheckLateralPush(false, 1.05, DryWallPush105);
	}
}

TEST_CASE("E2E.Bearing.AnOverloadedTimberBearingDropsTheBeamAndSparesThePiers")
{
	SUBCASE("0.8 of capacity")
	{
		Scene S = BuildTimberOnPiers(0.8);
		CheckNothingReleased(S, TimberBearingLoad08);
	}
	SUBCASE("1.2 of capacity")
	{
		Scene S = BuildTimberOnPiers(1.2);
		const CaseRun R = Run(S, TimberBearingLoad12);
		const ExpectedOutcome Want{{{"beam"}}, 1u};
		RecordVisual(S, R, Want);
		CheckOutcome(S, R, Want);
	}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Parked.E2E.Shed.LosingOneSideOfPostsDropsTheRoofBayByBay")
{
	ShedScene H = BuildShed(test::ShedSpec());
	for (const char* Post : {"post0", "post1", "post2"})
	{
		RemoveNamed(H.S, Post);
	}
	const CaseRun R = Run(H.S, ShedMinusSidePosts);
	const ExpectedOutcome Want{{{"beam0", "beam2", "plank0", "plank1", "plank2"}, {"beam1", "beam3", "plank3", "plank4", "plank5"}}};
	RecordVisual(H.S, R, Want);
	CheckOutcome(H.S, R, Want);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Parked.E2E.Shed.LosingOnePostLeavesTheScrewedRoofStanding")
{
	SUBCASE("middle post")
	{
		ShedScene H = BuildShed(test::ShedSpec());
		RemoveNamed(H.S, "post1");
		const CaseRun R = Run(H.S, ShedMinusMiddlePost);
		const ExpectedOutcome Want{};
		RecordVisual(H.S, R, Want);
		CheckOutcome(H.S, R, Want);
	}
	SUBCASE("corner post")
	{
		ShedScene H = BuildShed(test::ShedSpec());
		RemoveNamed(H.S, "post0");
		const CaseRun R = Run(H.S, ShedMinusCornerPost);
		const ExpectedOutcome Want{};
		RecordVisual(H.S, R, Want);
		CheckOutcome(H.S, R, Want);
	}
}

TEST_CASE("Parked.E2E.Cantilever.TheCounterweightHoldsTheBeamAndItsRemovalTipsIt")
{
	SUBCASE("no counterweight")
	{
		Scene S = BuildCantilever(Counterweight::None);
		const CaseRun R = Run(S, CantileverNoCounterweight);
		const ExpectedOutcome Want{{{"beam"}}};
		RecordVisual(S, R, Want);
		CheckOutcome(S, R, Want);
	}
	SUBCASE("16 kg")
	{
		Scene S = BuildCantilever(Counterweight::Heavy);
		CheckNothingReleased(S, Cantilever16kg);
	}
	SUBCASE("4 kg")
	{
		Scene S = BuildCantilever(Counterweight::Light);
		const CaseRun R = Run(S, Cantilever4kg);
		const ExpectedOutcome Want{{{"beam", "counterweight"}}};
		RecordVisual(S, R, Want);
		CheckOutcome(S, R, Want);
	}
	SUBCASE("16 kg removed")
	{
		Scene S = BuildCantilever(Counterweight::Heavy);
		RemoveNamed(S, "counterweight");
		const CaseRun R = Run(S, Cantilever16kgRemoved);
		const ExpectedOutcome Want{{{"beam"}}};
		RecordVisual(S, R, Want);
		CheckOutcome(S, R, Want);
	}
}

TEST_CASE("Parked.E2E.File.ALoadedColonnadeFallsLikeTheBuiltOne")
{
	const LoadResult Loaded = LoadStructure(ReadText(std::string(LOADPATH_DATA_PATH) + "/structures/colonnade.json"));
	INFO("load error line " << Loaded.Error.Line << ": " << Loaded.Error.Message);
	REQUIRE(Loaded.Ok());
	Scene S = SceneFromFile(Loaded.Value);
	CheckNothingReleased(S, FileColonnadeIntact);
	RemoveNamed(S, "post2");
	const CaseRun R = Run(S, FileColonnadeMinusPost2);
	const ExpectedOutcome Want{{{"beam1"}, {"beam2"}}, 1u};
	RecordVisual(S, R, Want);
	CheckOutcome(S, R, Want);
	Scene Built = BuildColonnade(4, 0);
	RemoveNamed(Built, "post2");
	const CaseRun BuiltRun = Run(Built, ColonnadeDryMinusPost2);
	CHECK(ClusterNames(S, R.Verdict) == ClusterNames(Built, BuiltRun.Verdict));
	LoadedStructure End = Loaded.Value;
	End.Model = S.Model;
	const LoadResult Reloaded = LoadStructure(WriteStructure(End));
	INFO("reload error line " << Reloaded.Error.Line << ": " << Reloaded.Error.Message);
	REQUIRE(Reloaded.Ok());
	Scene Again = SceneFromFile(Reloaded.Value);
	CHECK(LiveFreeNames(Again) == LiveFreeNames(S));
	CHECK(JudgeRemainder(Again) == ComponentVerdict::Stands);
}

TEST_CASE("Deferred.Slice6.DryWallLosingItsLintelKeepsTheBricksOverThePiers")
{
	SUBCASE("lintel in place")
	{
		Scene S = BuildLintelWall(false, 2, 6);
		CheckNothingReleased(S, DryLintelIntact);
	}
	SUBCASE("lintel removed")
	{
		Scene S = BuildLintelWall(false, 2, 6);
		CheckLintelRemovalKeepsThePierBricks(S);
	}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "E2E.Wall.BondedWallStandsOverItsOpeningWithoutTheLintel")
{
	SUBCASE("lintel in place")
	{
		Scene S = BuildLintelWall(true, 2, 6);
		CheckNothingReleased(S, BondedLintelIntact);
	}
	SUBCASE("lintel removed")
	{
		Scene S = BuildLintelWall(true, 2, 6);
		RemoveNamed(S, "lintel");
		CheckNothingReleased(S, BondedLintelRemoved);
	}
}

TEST_CASE("Deferred.Wall.DryWallWithAGapInItsBaseReleasesNothing")
{
	SUBCASE("two base bricks removed")
	{
		WallScene W = BuildWall(false, 10, 8);
		RemoveWallBricks(W, 3.0, 5.0, 0, 1);
		CHECK(W.S.RemovedBeforeRun.size() == 2);
		CheckNothingReleased(W.S, DryWallMinusTwoBaseBricks);
	}
	SUBCASE("two-brick opening, lintel removed")
	{
		Scene S = BuildLintelWall(false, 3, 5);
		RemoveNamed(S, "lintel");
		CheckNothingReleased(S, DryTwoBrickLintelRemoved);
	}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "E2E.Property.BuildOrderChangesNoOutcome")
{
	bool bPictured = false;
	for (const auto& [Fixture, Make] : ShuffleFixtures())
	{
		Scene Reference = Make(0);
		const std::string Want = Canonical(Reference, Run(Reference, BuildOrderShuffled).Verdict);
		for (unsigned Seed = 1; Seed <= 8; ++Seed)
		{
			Scene Shuffled = Make(Seed);
			const CaseRun ShuffledRun = Run(Shuffled, BuildOrderShuffled);
			if (!bPictured)
			{
				RecordVisual(Shuffled, ShuffledRun);
				bPictured = true;
			}
			const std::string Got = Canonical(Shuffled, ShuffledRun.Verdict);
			INFO("fixture " << Fixture << ", seed " << Seed << "\n  want " << Want << "\n  got  " << Got);
			CHECK(Got == Want);
		}
	}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "E2E.Corner.InterlockingRaisesTheOverturningLoad")
{
	const double Lambda = 1.1 * BrickYM / (6 * BrickZM);
	SUBCASE("straight joint")
	{
		Scene S = BuildCorner(false);
		S.Gravity = Vec3{0.0, -Lambda * GravityMPerS2, -GravityMPerS2};
		const CaseRun R = Run(S, CornerStraightJointPush);
		ExpectedOutcome Pictured;
		Pictured.Clusters = {FreeNamesWhere(S, [](const std::string& Name) { return StartsWith(Name, "f_"); })};
		RecordVisual(S, R, Pictured);
		INFO("visual " << std::string(CornerStraightJointPush) << ": " << Canonical(S, R.Verdict));
		CHECK(ReleasedNames(S, R.Verdict) == FreeNamesWhere(S, [](const std::string& Name) { return StartsWith(Name, "f_"); }));
		CHECK(LiveFreeNames(S) == FreeNamesWhere(S, [](const std::string& Name) { return StartsWith(Name, "r_"); }));
	}
	SUBCASE("interlocked")
	{
		Scene S = BuildCorner(true);
		S.Gravity = Vec3{0.0, -Lambda * GravityMPerS2, -GravityMPerS2};
		const CaseRun R = Run(S, CornerInterlockedPush);
		RecordVisual(S, R, ExpectedOutcome{});
		INFO("visual " << std::string(CornerInterlockedPush) << ": " << Canonical(S, R.Verdict));
		CHECK(R.Verdict.Clusters.empty());
		CHECK(LiveFreeNames(S).size() == R.Boxes.size() - 1);
	}
}
