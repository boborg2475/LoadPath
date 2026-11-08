#include "BrickFixtures.h"
#include "CascadeTestSupport.h"
#include "CatalogueTestSupport.h"
#include "EventRegionTestSupport.h"
#include "SlowInDebug.h"
#include "StaticsTestSupport.h"

#include "loadpath/Catalogue.h"
#include "loadpath/StandingAuthority.h"
#include "loadpath/StructuralAuthority.h"
#include "loadpath/geometry/RunningBondWall.h"

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
template <class R>
constexpr bool HasComponentResults = requires(const R& X) {
	X.Components.size();
	X.Components[0].Bodies;
	X.Components[0].State;
	X.Components[0].ActiveSetIterations;
	X.Components[0].ReadingSlots;
};

template <class S>
constexpr bool HasCachedPartition = requires(const S& Model) { Model.Components().FreeBodies.size(); };

template <class S>
std::optional<std::pair<const void*, std::size_t>> PartitionOf(const S& Model)
{
	if constexpr (HasCachedPartition<S>)
	{
		const auto& Partition = Model.Components();
		return std::pair<const void*, std::size_t>{static_cast<const void*>(&Partition), Partition.FreeBodies.size()};
	}
	else
	{
		(void)Model;
		return std::nullopt;
	}
}

bool SameMotions(const std::vector<MechanismMotion>& Got, const std::vector<MechanismMotion>& Want)
{
	if (Got.size() != Want.size())
	{
		return false;
	}
	for (std::size_t I = 0; I < Got.size(); ++I)
	{
		if (!SameHandle(Got[I].Body, Want[I].Body) || !NearVec(Got[I].TranslationM, Want[I].TranslationM, 1.0e-9, 1.0e-12) || !NearVec(Got[I].RotationRad, Want[I].RotationRad, 1.0e-9, 1.0e-12))
		{
			return false;
		}
	}
	return true;
}

template <class R>
void CheckEveryComponent(const R& Whole, const ThreeComponents& F, const IStandingAuthority& Authority)
{
	if constexpr (HasComponentResults<R>)
	{
		CHECK(Whole.Components.size() == F.Parts.size());
		if (Whole.Components.size() != F.Parts.size())
		{
			return;
		}
		std::vector<int> Owner(Whole.Readings.size(), -1);
		for (std::size_t K = 0; K < F.Parts.size(); ++K)
		{
			const auto& Component = Whole.Components[K];
			const Structure Alone = KeepOnly(F.Model, F.Parts[K]);
			const std::vector<BodyLoad> AloneLoads = LoadsOn(F.Loads, F.Parts[K]);
			const StandingResult Reference = Authority.Solve(Alone, AloneLoads, StandingOptions{}, {});
			INFO("component " << K << ": bodies " << Describe(Component.Bodies) << " want " << Describe(F.Parts[K]) << ", state " << Component.State << " alone " << Reference.State << ", iterations " << Component.ActiveSetIterations << " alone " << Reference.ActiveSetIterations);
			CHECK(SameIdList(Component.Bodies, F.Parts[K]));
			CHECK(Component.State == Reference.State);
			CHECK(Component.ActiveSetIterations == Reference.ActiveSetIterations);
			for (const std::size_t Slot : Component.ReadingSlots)
			{
				CHECK(Slot < Whole.Readings.size());
				if (Slot >= Whole.Readings.size())
				{
					continue;
				}
				CHECK_MESSAGE(Owner[Slot] == -1, "reading slot " << Slot << " is in components " << Owner[Slot] << " and " << K);
				Owner[Slot] = static_cast<int>(K);
			}
		}
		for (std::size_t Slot = 0; Slot < Whole.Readings.size(); ++Slot)
		{
			const bool bTouches = TouchesFreeBody(F.Model, Whole.Readings[Slot].Id);
			CHECK_MESSAGE((Owner[Slot] >= 0) == bTouches, "reading slot " << Slot << " (interface " << Whole.Readings[Slot].Id.Index << ") touches a free body " << bTouches << " but is owned by component " << Owner[Slot]);
		}
	}
	else
	{
		(void)Whole;
		(void)F;
		(void)Authority;
		CHECK_MESSAGE(false, "StandingResult has no Components (Bodies, State, ActiveSetIterations, ReadingSlots)");
	}
}

constexpr int JudgeDraws = 40;
constexpr std::uint64_t JudgeSeedBase = 0x6b1b2ull;

void CheckJudgeMatchesEachComponentAlone(std::uint64_t Seed, int& ComponentsSeen, int& Collapsed, int& Mismatches)
{
	MultiDraw D = MakeMultiDraw(Seed);
	INFO("seed " << Seed << ": " << D.Description);
	const AuthorityResult Whole = Judge(D.Model, D.Options, D.Loads);
	INFO("whole: " << DescribeAuthority(Whole));
	CHECK(Whole.Worst != ComponentVerdict::InvalidInput);
	CHECK(Whole.Components.size() >= 2u);
	for (std::size_t K = 0; K < Whole.Components.size(); ++K)
	{
		const ComponentReport& Got = Whole.Components[K];
		const Structure Alone = KeepOnly(D.Model, Got.Bodies);
		const std::vector<BodyLoad> AloneLoads = LoadsOn(D.Loads, Got.Bodies);
		const AuthorityResult Reference = Judge(Alone, D.Options, AloneLoads);
		INFO("component " << K << " alone: " << DescribeAuthority(Reference));
		++ComponentsSeen;
		Collapsed += Got.Verdict == ComponentVerdict::Collapses ? 1 : 0;
		CHECK(Reference.Components.size() == 1u);
		if (Reference.Components.size() != 1u)
		{
			++Mismatches;
			continue;
		}
		const ComponentReport& Want = Reference.Components[0];
		int Mismatched = 0;
		for (const InterfaceReading& WantReading : Reference.Readings)
		{
			const InterfaceReading* GotReading = FindReading(Whole.Readings, WantReading.Id);
			if (GotReading == nullptr || !SameReading(*GotReading, WantReading))
			{
				++Mismatched;
			}
		}
		const bool bSame = SameIdList(Got.Bodies, Want.Bodies) && Got.Verdict == Want.Verdict && Got.Elastic == Want.Elastic && SameIdList(Got.Release, Want.Release) && Mismatched == 0;
		Mismatches += bSame ? 0 : 1;
		CHECK(SameIdList(Got.Bodies, Want.Bodies));
		CHECK(Got.Verdict == Want.Verdict);
		CHECK(Got.Elastic == Want.Elastic);
		CHECK(SameIdList(Got.Release, Want.Release));
		CHECK(SameMotions(Got.Mechanism, Want.Mechanism));
		CHECK_MESSAGE(Mismatched == 0, Mismatched << " of " << Reference.Readings.size() << " readings differ from the component alone");
	}
}

struct PairsAndBrick
{
	Structure Model;
	std::vector<BodyLoad> Loads;
	std::vector<std::vector<BodyId>> Pairs;
	std::vector<BodyId> Brick;
};

PairsAndBrick MakePairsAndBrick()
{
	PairsAndBrick F;
	const ProfileId Profile = AddProfileOk(F.Model, DryJoint(0.7));
	const BodyId Ground = AddGroundPlane(F.Model);
	for (int I = 0; I < 3; ++I)
	{
		const double CentreXM = 2.0 * BrickLengthM * static_cast<double>(I);
		const Box LowerShape = BrickShape(CentreXM, 1);
		const Box UpperShape = BrickShape(CentreXM, 2);
		const BodyId Lower = AddBox(F.Model, LowerShape);
		const BodyId Upper = AddBox(F.Model, UpperShape);
		AddInterfaceOk(F.Model, *StackedFaces(Lower, LowerShape, Upper, UpperShape, Profile));
		F.Pairs.push_back({Lower, Upper});
	}
	const Box BrickBox = BrickShape(8.0 * BrickLengthM, 0);
	const BodyId Brick = AddBox(F.Model, BrickBox);
	AddInterfaceOk(F.Model, OnGroundPlane(Ground, Brick, BrickBox, Profile));
	F.Brick.push_back(Brick);
	return F;
}

const ComponentReport* ReportFor(const AuthorityResult& Result, const std::vector<BodyId>& Bodies)
{
	for (const ComponentReport& Report : Result.Components)
	{
		if (SameIdList(Report.Bodies, Bodies))
		{
			return &Report;
		}
	}
	return nullptr;
}

void CheckSameReport(const ComponentReport& Got, const ComponentReport& Want)
{
	CHECK(Got.Verdict == Want.Verdict);
	CHECK(SameMotions(Got.Mechanism, Want.Mechanism));
	CHECK(SameIdList(Got.Release, Want.Release));
}
}

TEST_CASE("Authority.Judge.SolvesTheElasticFieldOnce")
{
	const ThreeComponents F = MakeThreeComponents();
	std::vector<SolveCall> Solves;
	const std::unique_ptr<IStructuralAuthority> Authority = MakeElasticAuthority(std::make_unique<CountingStanding>(Solves));
	const AuthorityResult Result = Authority->Judge(F.Model, F.Loads, AuthorityOptions{}, nullptr);
	INFO(DescribeAuthority(Result));
	CHECK(Result.Components.size() == 3u);
	if (Result.Components.size() == 3u)
	{
		CHECK(Result.Components[0].Elastic == StandingState::Stands);
		CHECK(Result.Components[1].Elastic == StandingState::Mechanism);
		CHECK(Result.Components[2].Elastic == StandingState::Stands);
		CHECK(Result.Components[1].Verdict == ComponentVerdict::Collapses);
	}
	std::ostringstream SolveLog;
	for (const SolveCall& Call : Solves)
	{
		SolveLog << " " << Call.LiveBodies;
	}
	INFO("Solve calls, live bodies each:" << SolveLog.str());
	CHECK(Solves.size() == 1u);
	if (!Solves.empty())
	{
		CHECK(Solves[0].LiveBodies == F.Model.LiveBodyCount());
	}
}

TEST_CASE("Statics.StandingResult.ReportsEveryComponent")
{
	const ThreeComponents F = MakeThreeComponents();
	const std::unique_ptr<IStandingAuthority> Authority = MakeRbsmStandingAuthority();
	const StandingResult Whole = Authority->Solve(F.Model, F.Loads, StandingOptions{}, {});
	INFO("whole state " << Whole.State << ", readings " << Whole.Readings.size());
	CheckEveryComponent(Whole, F, *Authority);
}

TEST_CASE("Structure.Components.IsCachedUntilAMutation")
{
	Structure Model;
	const BodyId Ground = AddGroundPlane(Model);
	const ProfileId Dry = AddProfileOk(Model, DryJoint(0.7));
	std::vector<BodyId> Row;
	for (int K = 0; K < 3; ++K)
	{
		const Box Shape = BrickShape(K * BrickLengthM);
		Row.push_back(AddBox(Model, Shape));
		AddInterfaceOk(Model, OnGroundPlane(Ground, Row.back(), Shape, Dry));
	}
	const InterfaceId FirstBed = Model.InterfacesOf(Row[0])[0];
	for (int K = 0; K < 2; ++K)
	{
		InterfaceDesc Head = HeadJointAlongX(Row[static_cast<std::size_t>(K)], Row[static_cast<std::size_t>(K + 1)], 0, Dry);
		Head.CentreM.X = (K + 0.5) * BrickLengthM;
		AddInterfaceOk(Model, Head);
	}
	std::vector<BodyId> Pair;
	for (int K = 0; K < 2; ++K)
	{
		const Box Shape = BrickShape(2.0 + K * BrickLengthM);
		Pair.push_back(AddBox(Model, Shape));
		AddInterfaceOk(Model, OnGroundPlane(Ground, Pair.back(), Shape, Dry));
	}
	InterfaceDesc PairHead = HeadJointAlongX(Pair[0], Pair[1], 0, Dry);
	PairHead.CentreM.X = 2.0 + 0.5 * BrickLengthM;
	const InterfaceId PairJoint = AddInterfaceOk(Model, PairHead);

	const auto First = PartitionOf(Model);
	CHECK_MESSAGE(First.has_value(), "Structure has no Components() returning a partition with FreeBodies");
	if (!First)
	{
		return;
	}
	const auto Second = PartitionOf(Model);
	CHECK(First->second == 2u);
	CHECK(Second->first == First->first);
	CHECK(Second->second == 2u);
	AddBox(Model, BrickShape(5.0));
	const auto Grown = PartitionOf(Model);
	CHECK(Grown->second == 3u);
	CHECK(Model.AddProfile(DryJoint(0.5)).Ok());
	const auto AfterProfile = PartitionOf(Model);
	CHECK(AfterProfile->first == Grown->first);
	CHECK(AfterProfile->second == 3u);
	CHECK(Model.MarkPass(5));
	const auto AfterPass = PartitionOf(Model);
	CHECK(AfterPass->first == Grown->first);
	CHECK(AfterPass->second == 3u);
	CHECK(Model.RemoveBody(Row[1]));
	CHECK(PartitionOf(Model)->second == 4u);
	CHECK(Model.Sever(PairJoint, 6));
	CHECK(PartitionOf(Model)->second == 5u);
	AddInterfaceOk(Model, PairHead);
	CHECK(PartitionOf(Model)->second == 4u);
	CHECK(Model.Crack(FirstBed, 7));
	CHECK(PartitionOf(Model)->second == 4u);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Authority.Judge.MatchesSolvingEachComponentAlone")
{
	int ComponentsSeen = 0;
	int Collapsed = 0;
	int Mismatches = 0;
	for (int Draw = 0; Draw < JudgeDraws; ++Draw)
	{
		CheckJudgeMatchesEachComponentAlone(JudgeSeedBase + static_cast<std::uint64_t>(Draw), ComponentsSeen, Collapsed, Mismatches);
	}
	MESSAGE("judge per component: " << JudgeDraws << " draws, " << ComponentsSeen << " components, " << Collapsed << " collapse, " << Mismatches << " differ from the component alone");
	CHECK(Collapsed >= 1);
}

TEST_CASE("Authority.Judge.EachCollapsingPairReleasesAsItDoesAlone")
{
	PairsAndBrick F = MakePairsAndBrick();
	const std::unique_ptr<IStructuralAuthority> Plain = MakeAuthority();
	const AuthorityResult First = Plain->Judge(F.Model, F.Loads, AuthorityOptions{}, nullptr);
	INFO("first: " << DescribeAuthority(First));
	REQUIRE(First.Components.size() == 4u);
	for (const std::vector<BodyId>& Pair : F.Pairs)
	{
		INFO("pair " << Describe(Pair));
		const ComponentReport* Got = ReportFor(First, Pair);
		REQUIRE(Got != nullptr);
		CHECK(Got->Elastic == StandingState::Mechanism);
		CHECK(Got->Verdict == ComponentVerdict::Collapses);
		CHECK(SameIdList(Got->Release, Pair));
		const Structure Alone = KeepOnly(F.Model, Pair);
		const AuthorityResult AloneResult = Plain->Judge(Alone, LoadsOn(F.Loads, Pair), AuthorityOptions{}, nullptr);
		REQUIRE(AloneResult.Components.size() == 1u);
		CheckSameReport(*Got, AloneResult.Components[0]);
	}
	const ComponentReport* Brick = ReportFor(First, F.Brick);
	REQUIRE(Brick != nullptr);
	CHECK(Brick->Verdict == ComponentVerdict::Stands);
}
