#include "BrickFixtures.h"
#include "CascadeTestSupport.h"
#include "EventRegionTestSupport.h"
#include "SlowInDebug.h"
#include "StaticsTestSupport.h"

#include "loadpath/Cascade.h"
#include "loadpath/CollapseCertificate.h"
#include "loadpath/StandingAuthority.h"
#include "loadpath/StructuralAuthority.h"
#include "loadpath/Structure.h"

#include "doctest.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
template <class C>
constexpr bool HasRunPrevious = requires(const C& X, Structure& Model, std::span<const BodyLoad> Loads, const CascadeOptions& Options, const AuthorityResult* Previous) { X.Run(Model, Loads, Options, Previous); };

template <class C>
CascadeVerdict RunFrom(const C& Cascade, Structure& Model, std::span<const BodyLoad> Loads, const CascadeOptions& Options)
{
	if constexpr (HasRunPrevious<C>)
	{
		return Cascade.Run(Model, Loads, Options, nullptr);
	}
	else
	{
		return Cascade.Run(Model, Loads, Options);
	}
}

const std::string NoPrevious = "IStructuralAuthority::Judge takes no Previous (const AuthorityResult*)";

AuthorityResult JudgeWithPrevious(const IStructuralAuthority& Authority, const Structure& Model, std::span<const BodyLoad> Loads, const AuthorityOptions& Options, const AuthorityResult* Previous)
{
	std::optional<AuthorityResult> Got = JudgeAfter(Authority, Model, Loads, Options, Previous);
	CHECK_MESSAGE(Got.has_value(), NoPrevious);
	return Got ? std::move(*Got) : JudgeFresh(Authority, Model, Loads, Options);
}

struct Counted
{
	std::vector<SolveCall> Solves;
	std::vector<DecideCall> Decides;
	std::unique_ptr<IStructuralAuthority> Authority;
};

std::unique_ptr<Counted> MakeCounted()
{
	auto C = std::make_unique<Counted>();
	C->Authority = MakeCombinedAuthority(std::make_unique<CountingStanding>(C->Solves), std::make_unique<CountingCertificate>(C->Decides));
	return C;
}

std::vector<InterfaceId> InterfacesTouching(const Structure& Model, const std::vector<BodyId>& Bodies)
{
	std::vector<InterfaceId> Found;
	for (const InterfaceId Id : Model.LiveInterfaces())
	{
		const InterfaceRecord& Record = Model.Interface(Id);
		if (Holds(Bodies, Record.BodyA) || Holds(Bodies, Record.BodyB))
		{
			Found.push_back(Id);
		}
	}
	return Found;
}

InterfaceId InterfaceBetween(const Structure& Model, BodyId A, BodyId B)
{
	for (const InterfaceId Id : Model.InterfacesOf(A))
	{
		const InterfaceRecord& Record = Model.Interface(Id);
		if ((SameHandle(Record.BodyA, A) && SameHandle(Record.BodyB, B)) || (SameHandle(Record.BodyA, B) && SameHandle(Record.BodyB, A)))
		{
			return Id;
		}
	}
	return InterfaceId::Invalid();
}

const ComponentReport* ComponentOf(const AuthorityResult& Result, const std::vector<BodyId>& Bodies)
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

std::vector<BodyId> Union(std::vector<BodyId> A, const std::vector<BodyId>& B)
{
	A.insert(A.end(), B.begin(), B.end());
	std::sort(A.begin(), A.end(), [](BodyId L, BodyId R) { return L.Index < R.Index; });
	return A;
}

void CheckSameReport(const ComponentReport& Got, const ComponentReport& Want, double Rel)
{
	INFO("got bodies " << Describe(Got.Bodies) << " " << VerdictName(Got.Verdict) << " elastic " << StandingStateName(Got.Elastic) << " kinematic " << CertificateStateName(Got.Kinematic) << " t [" << Got.LoadFactorLower << ", " << Got.LoadFactorUpper << "]");
	INFO("want bodies " << Describe(Want.Bodies) << " " << VerdictName(Want.Verdict) << " elastic " << StandingStateName(Want.Elastic) << " kinematic " << CertificateStateName(Want.Kinematic) << " t [" << Want.LoadFactorLower << ", " << Want.LoadFactorUpper << "]");
	CHECK(SameIdList(Got.Bodies, Want.Bodies));
	CHECK(Got.Verdict == Want.Verdict);
	CHECK(Got.Elastic == Want.Elastic);
	CHECK(Got.Kinematic == Want.Kinematic);
	CHECK(Got.bConsultedCertificate == Want.bConsultedCertificate);
	CHECK(Got.bDisagreed == Want.bDisagreed);
	CHECK(Near(Got.LoadFactorLower, Want.LoadFactorLower, Rel));
	CHECK(Near(Got.LoadFactorUpper, Want.LoadFactorUpper, Rel));
	CHECK(SameIdList(Got.Release, Want.Release));
	CHECK(SameIdList(Got.Hinging, Want.Hinging));
}

void CheckSameReadings(const std::vector<InterfaceReading>& Got, const std::vector<InterfaceReading>& Want, const std::vector<InterfaceId>& Ids, double Rel, double AbsN)
{
	int Mismatched = 0;
	std::string First;
	for (const InterfaceId Id : Ids)
	{
		const InterfaceReading* G = FindReading(Got, Id);
		const InterfaceReading* W = FindReading(Want, Id);
		if (G == nullptr || W == nullptr || !SameReading(*G, *W, Rel, AbsN))
		{
			if (Mismatched++ == 0)
			{
				First = "interface " + std::to_string(Id.Index) + (G == nullptr ? " missing from got" : W == nullptr ? " missing from want" : " differs");
			}
		}
	}
	CHECK_MESSAGE(Mismatched == 0, Mismatched << " of " << Ids.size() << " readings differ; first: " << First);
}

void CheckSameVerdict(const AuthorityResult& Got, const AuthorityResult& Want, const Structure& Model, double Rel, double AbsN)
{
	CHECK(Got.Worst == Want.Worst);
	CHECK(Got.Components.size() == Want.Components.size());
	if (Got.Components.size() != Want.Components.size())
	{
		return;
	}
	for (std::size_t K = 0; K < Got.Components.size(); ++K)
	{
		INFO("component " << K);
		CheckSameReport(Got.Components[K], Want.Components[K], Rel);
	}
	CHECK(Got.Readings.size() == Want.Readings.size());
	CheckSameReadings(Got.Readings, Want.Readings, Model.LiveInterfaces(), Rel, AbsN);
}

void CheckReused(const AuthorityResult& Got, const AuthorityResult& Previous, const Structure& Model, const std::vector<BodyId>& Bodies)
{
	INFO("component " << Describe(Bodies) << " should be copied from the previous verdict");
	const ComponentReport* G = ComponentOf(Got, Bodies);
	const ComponentReport* P = ComponentOf(Previous, Bodies);
	CHECK(G != nullptr);
	CHECK(P != nullptr);
	if (G == nullptr || P == nullptr)
	{
		return;
	}
	const std::optional<bool> Reused = ReusedOf(*G);
	CHECK_MESSAGE(Reused.has_value(), "ComponentReport has no bReused");
	CHECK(Reused.value_or(false));
	CheckSameReport(*G, *P, 0.0);
	CheckSameReadings(Got.Readings, Previous.Readings, InterfacesTouching(Model, Bodies), 0.0, 0.0);
}

void CheckFresh(const AuthorityResult& Got, const std::vector<BodyId>& Bodies)
{
	INFO("component " << Describe(Bodies) << " should be judged fresh");
	const ComponentReport* G = ComponentOf(Got, Bodies);
	CHECK(G != nullptr);
	if (G == nullptr)
	{
		return;
	}
	const std::optional<bool> Reused = ReusedOf(*G);
	CHECK_MESSAGE(Reused.has_value(), "ComponentReport has no bReused");
	CHECK_FALSE(Reused.value_or(true));
}

void CheckSolvedOnly(const std::vector<SolveCall>& Solves, const std::vector<BodyId>& Only)
{
	INFO("Solve calls:" << DescribeSolves(Solves) << "; want one with only " << Describe(Only));
	CHECK(Solves.size() == 1u);
	if (Solves.size() == 1u)
	{
		CHECK(SameIdList(Solves[0].Only, Only));
	}
}

struct JudgeRecord
{
	Structure Model;
	std::vector<BodyLoad> Loads;
	AuthorityOptions Options;
	AuthorityResult Result;
};

class RecordingAuthority final : public IStructuralAuthority
{
public:
	explicit RecordingAuthority(std::vector<JudgeRecord>& InRecords)
		: Records(InRecords)
		, Inner(MakeAuthority())
	{
	}

	AuthorityResult Judge(const Structure& Model, std::span<const BodyLoad> Loads, const AuthorityOptions& Options) const
	{
		return Record(Model, Loads, Options, JudgeFresh(*Inner, Model, Loads, Options));
	}

	AuthorityResult Judge(const Structure& Model, std::span<const BodyLoad> Loads, const AuthorityOptions& Options, const AuthorityResult* Previous) const
	{
		std::optional<AuthorityResult> Got = JudgeAfter(*Inner, Model, Loads, Options, Previous);
		return Record(Model, Loads, Options, Got ? std::move(*Got) : JudgeFresh(*Inner, Model, Loads, Options));
	}

private:
	AuthorityResult Record(const Structure& Model, std::span<const BodyLoad> Loads, const AuthorityOptions& Options, AuthorityResult Result) const
	{
		Records.push_back(JudgeRecord{Model, std::vector<BodyLoad>(Loads.begin(), Loads.end()), Options, Result});
		return Result;
	}

	std::vector<JudgeRecord>& Records;
	std::unique_ptr<IStructuralAuthority> Inner;
};

constexpr int EventRegionDraws = 16;
constexpr std::uint64_t EventRegionSeedBase = 0x6b2e0ull;
}

TEST_CASE("Structure.Revision.CountsMutationsAndStampsTouchedBodies")
{
	Structure Model;
	const std::optional<std::uint64_t> Start = RevisionOf(Model);
	CHECK_MESSAGE(Start.has_value(), "Structure has no Revision() and LastTouched(BodyId) returning std::uint64_t");
	if (!Start)
	{
		return;
	}
	const auto Rev = [&] { return RevisionOf(Model).value_or(0); };
	const auto Touched = [&](BodyId Body) { return LastTouchedOf(Model, Body).value_or(0); };
	CHECK(*Start == 0u);
	const ProfileId Dry = AddProfileOk(Model, DryJoint(0.7));
	CHECK(Rev() == 0u);
	const BodyId Ground = AddGroundPlane(Model);
	CHECK(Rev() == 1u);
	CHECK(Touched(Ground) == 1u);
	const BodyId A = AddBox(Model, BrickShape(0.0, 0));
	const BodyId B = AddBox(Model, BrickShape(0.0, 1));
	const BodyId C = AddBox(Model, BrickShape(1.0, 0));
	const BodyId D = AddBox(Model, BrickShape(1.0, 1));
	CHECK(Rev() == 5u);
	CHECK(Touched(A) == 2u);
	CHECK(Touched(D) == 5u);
	BodyDesc Bad;
	Bad.MassKg = NaN;
	CHECK_FALSE(Model.AddBody(Bad).Ok());
	CHECK(Rev() == 5u);
	AddInterfaceOk(Model, OnGroundPlane(Ground, A, BrickShape(0.0, 0), Dry));
	CHECK(Rev() == 6u);
	CHECK(Touched(Ground) == 6u);
	CHECK(Touched(A) == 6u);
	CHECK(Touched(B) == 3u);
	const InterfaceId Joint = AddInterfaceOk(Model, *StackedFaces(A, BrickShape(0.0, 0), B, BrickShape(0.0, 1), Dry));
	CHECK(Rev() == 7u);
	CHECK(Touched(A) == 7u);
	CHECK(Touched(B) == 7u);
	CHECK(Touched(Ground) == 6u);
	AddInterfaceOk(Model, OnGroundPlane(Ground, C, BrickShape(1.0, 0), Dry));
	AddInterfaceOk(Model, *StackedFaces(C, BrickShape(1.0, 0), D, BrickShape(1.0, 1), Dry));
	CHECK(Rev() == 9u);
	CHECK(Touched(Ground) == 8u);
	CHECK(Touched(C) == 9u);
	CHECK(Touched(D) == 9u);
	CHECK_FALSE(Model.AddInterface(OnGroundPlane(Ground, BodyId{999u, 1u}, BrickShape(3.0, 0), Dry)).Ok());
	CHECK(Rev() == 9u);
	CHECK(Touched(Ground) == 8u);

	CHECK_FALSE(Model.Crack(Joint, 0));
	CHECK(Rev() == 9u);
	CHECK(Touched(A) == 7u);
	CHECK(Model.Crack(Joint, 1));
	CHECK(Rev() == 10u);
	CHECK(Touched(A) == 10u);
	CHECK(Touched(B) == 10u);
	CHECK(Touched(Ground) == 8u);
	CHECK(Touched(C) == 9u);
	CHECK(Model.Sever(Joint, 1));
	CHECK(Rev() == 11u);
	CHECK(Touched(A) == 11u);
	CHECK(Touched(B) == 11u);
	CHECK_FALSE(Model.Sever(Joint, 2));
	CHECK(Rev() == 11u);
	CHECK(Touched(B) == 11u);

	const Structure Copy = Model;
	CHECK(RevisionOf(Copy).value_or(0) == 11u);
	CHECK(LastTouchedOf(Copy, A).value_or(0) == 11u);
	CHECK(LastTouchedOf(Copy, C).value_or(0) == 9u);

	CHECK(Model.RemoveBody(D));
	CHECK(Rev() == 12u);
	CHECK(Touched(C) == 12u);
	CHECK(Touched(A) == 11u);
	CHECK(Touched(Ground) == 8u);
	CHECK_FALSE(Model.RemoveBody(D));
	CHECK(Rev() == 12u);
	CHECK(Touched(C) == 12u);
	CHECK(RevisionOf(Copy).value_or(0) == 11u);
	CHECK(LastTouchedOf(Copy, C).value_or(0) == 9u);
}

TEST_CASE("Authority.Judge.ReusesCleanComponentsFromThePreviousVerdict")
{
	ThreeComponents F = MakeThreeComponents();
	const auto C = MakeCounted();
	const AuthorityResult First = JudgeWithPrevious(*C->Authority, F.Model, F.Loads, AuthorityOptions{}, nullptr);
	INFO("first: " << DescribeAuthority(First));
	CheckSolvedOnly(C->Solves, {});
	REQUIRE(First.Components.size() == 3u);
	CHECK(First.Components[0].Elastic == StandingState::Stands);
	const InterfaceId Bed = InterfaceBetween(F.Model, F.Ground, F.Parts[0][0]);
	CHECK(F.Model.Sever(Bed, 1));
	C->Solves.clear();
	C->Decides.clear();
	const AuthorityResult Second = JudgeWithPrevious(*C->Authority, F.Model, F.Loads, AuthorityOptions{}, &First);
	INFO("second: " << DescribeAuthority(Second));
	CheckSolvedOnly(C->Solves, F.Parts[0]);
	INFO("Decide calls:" << DescribeDecides(C->Decides));
	CHECK(C->Decides.size() == 1u);
	if (C->Decides.size() == 1u)
	{
		CHECK(SameIdList(C->Decides[0].Only, F.Parts[0]));
	}
	CHECK(Second.Components.size() == 3u);
	CheckFresh(Second, F.Parts[0]);
	const ComponentReport* Mutated = ComponentOf(Second, F.Parts[0]);
	CHECK(Mutated != nullptr);
	if (Mutated != nullptr)
	{
		CHECK(Mutated->Elastic == StandingState::Mechanism);
	}
	CHECK(FindReading(Second.Readings, Bed) == nullptr);
	CheckReused(Second, First, F.Model, F.Parts[1]);
	CheckReused(Second, First, F.Model, F.Parts[2]);
}

TEST_CASE("Authority.Judge.ALoadChangeDirtiesOnlyItsComponent")
{
	ThreeComponents F = MakeThreeComponents();
	const auto C = MakeCounted();
	const AuthorityResult First = JudgeWithPrevious(*C->Authority, F.Model, F.Loads, AuthorityOptions{}, nullptr);
	std::vector<BodyLoad> Pushed = F.Loads;
	const BodyId Top = F.Parts[2][1];
	Pushed.push_back(BodyLoad{Top, Vec3{0.1 * BrickMassKg * GravityMPerS2, 0.0, 0.0}, F.Model.Body(Top).CentroidM});
	C->Solves.clear();
	C->Decides.clear();
	const AuthorityResult Second = JudgeWithPrevious(*C->Authority, F.Model, Pushed, AuthorityOptions{}, &First);
	INFO("first: " << DescribeAuthority(First));
	INFO("second: " << DescribeAuthority(Second));
	CheckSolvedOnly(C->Solves, F.Parts[2]);
	INFO("Decide calls:" << DescribeDecides(C->Decides));
	CHECK(C->Decides.empty());
	CheckFresh(Second, F.Parts[2]);
	const InterfaceId Joint = InterfaceBetween(F.Model, F.Parts[2][0], F.Parts[2][1]);
	const InterfaceReading* Before = FindReading(First.Readings, Joint);
	const InterfaceReading* After = FindReading(Second.Readings, Joint);
	CHECK(Before != nullptr);
	CHECK(After != nullptr);
	if (Before != nullptr && After != nullptr)
	{
		INFO("pushed joint shear before " << Before->ShearForceN << " after " << After->ShearForceN);
		CHECK_FALSE(NearVec(After->ShearForceN, Before->ShearForceN, 1.0e-6, 1.0e-6));
	}
	CheckReused(Second, First, F.Model, F.Parts[0]);
	CheckReused(Second, First, F.Model, F.Parts[1]);
}

TEST_CASE("Authority.Judge.ASplitComponentIsNeverReused")
{
	ThreeComponents F = MakeThreeComponents();
	const ProfileId Dry = AddProfileOk(F.Model, DryJoint(ThreeComponentFriction));
	const double BridgeXM = 4.0;
	const Box LeftShape = BrickShape(BridgeXM - BridgeHalfSpanM, 0);
	const Box RightShape = BrickShape(BridgeXM + BridgeHalfSpanM, 0);
	const Box DeckShape = BrickShape(BridgeXM, 1);
	const BodyId Left = AddBox(F.Model, LeftShape);
	const BodyId Right = AddBox(F.Model, RightShape);
	const BodyId Deck = AddBox(F.Model, DeckShape);
	AddInterfaceOk(F.Model, OnGroundPlane(F.Ground, Left, LeftShape, Dry));
	AddInterfaceOk(F.Model, OnGroundPlane(F.Ground, Right, RightShape, Dry));
	AddInterfaceOk(F.Model, *StackedFaces(Left, LeftShape, Deck, DeckShape, Dry));
	AddInterfaceOk(F.Model, *StackedFaces(Right, RightShape, Deck, DeckShape, Dry));
	const auto C = MakeCounted();
	const AuthorityResult First = JudgeWithPrevious(*C->Authority, F.Model, F.Loads, AuthorityOptions{}, nullptr);
	INFO("first: " << DescribeAuthority(First));
	CHECK(First.Components.size() == 4u);
	CHECK(F.Model.RemoveBody(Deck));
	C->Solves.clear();
	const AuthorityResult Second = JudgeWithPrevious(*C->Authority, F.Model, F.Loads, AuthorityOptions{}, &First);
	INFO("second: " << DescribeAuthority(Second));
	CHECK(Second.Components.size() == 5u);
	CheckSolvedOnly(C->Solves, {Left, Right});
	CheckFresh(Second, {Left});
	CheckFresh(Second, {Right});
	for (const std::vector<BodyId>& Part : F.Parts)
	{
		CheckReused(Second, First, F.Model, Part);
	}
}

TEST_CASE("Authority.Judge.AnUnmatchedPreviousReusesNothing")
{
	ThreeComponents F = MakeThreeComponents();
	const std::unique_ptr<IStructuralAuthority> Authority = MakeAuthority();
	const AuthorityResult Fresh = JudgeWithPrevious(*Authority, F.Model, F.Loads, AuthorityOptions{}, nullptr);
	Structure Other;
	const BodyId OtherGround = AddGroundPlane(Other);
	const ProfileId OtherDry = AddProfileOk(Other, DryJoint(ThreeComponentFriction));
	AddBox(Other, BrickShape(5.0, 3));
	AddDryStack(Other, OtherGround, OtherDry, 6.0);
	const AuthorityResult OtherVerdict = JudgeWithPrevious(*Authority, Other, {}, AuthorityOptions{}, nullptr);
	AuthorityResult Ahead = Fresh;
	CHECK_MESSAGE(SetResultRevision(Ahead, RevisionOf(F.Model).value_or(0) + 5u), "AuthorityResult has no Revision");
	const std::vector<std::pair<const char*, const AuthorityResult*>> Unmatched{{"a verdict on another structure", &OtherVerdict}, {"a verdict whose revision is ahead of the model", &Ahead}};
	for (const auto& [Name, Previous] : Unmatched)
	{
		INFO("previous: " << Name);
		const AuthorityResult Got = JudgeWithPrevious(*Authority, F.Model, F.Loads, AuthorityOptions{}, Previous);
		INFO("got: " << DescribeAuthority(Got));
		CHECK(Got.Components.size() == Fresh.Components.size());
		for (const ComponentReport& Report : Got.Components)
		{
			CheckFresh(Got, Report.Bodies);
		}
		CheckSameVerdict(Got, Fresh, F.Model, 0.0, 0.0);
	}
}

TEST_CASE("Authority.Judge.NothingDirtyMakesNoSolveCall")
{
	ThreeComponents F = MakeThreeComponents();
	const auto C = MakeCounted();
	const AuthorityResult First = JudgeWithPrevious(*C->Authority, F.Model, F.Loads, AuthorityOptions{}, nullptr);
	C->Solves.clear();
	C->Decides.clear();
	const AuthorityResult Second = JudgeWithPrevious(*C->Authority, F.Model, F.Loads, AuthorityOptions{}, &First);
	INFO("Solve calls:" << DescribeSolves(C->Solves) << "; Decide calls:" << DescribeDecides(C->Decides));
	CHECK(C->Solves.empty());
	CHECK(C->Decides.empty());
	CHECK(Second.Components.size() == 3u);
	for (const std::vector<BodyId>& Part : F.Parts)
	{
		CheckReused(Second, First, F.Model, Part);
	}
	CheckSameVerdict(Second, First, F.Model, 0.0, 0.0);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Cascade.EventRegion.ReusedComponentsEqualAFreshJudge")
{
	CHECK_MESSAGE(HasJudgePrevious<IStructuralAuthority>, NoPrevious);
	const std::unique_ptr<IStructuralAuthority> Referee = MakeAuthority();
	int Judges = 0;
	int Components = 0;
	int Reused = 0;
	for (int Draw = 0; Draw < EventRegionDraws; ++Draw)
	{
		const std::uint64_t Seed = EventRegionSeedBase + static_cast<std::uint64_t>(Draw);
		MultiDraw D = MakeMultiDraw(Seed);
		INFO("seed " << Seed << ": " << D.Description);
		std::vector<JudgeRecord> Records;
		CascadeOptions Options;
		Options.Authority = D.Options;
		Options.MaxPasses = 16;
		const std::unique_ptr<ICascade> Cascade = MakeCascade(std::make_unique<RecordingAuthority>(Records));
		const CascadeVerdict V = RunFrom(*Cascade, D.Model, D.Loads, Options);
		INFO("cascade: " << DescribeVerdict(V));
		for (std::size_t J = 0; J < Records.size(); ++J)
		{
			const JudgeRecord& R = Records[J];
			INFO("judge " << J << ": " << DescribeAuthority(R.Result));
			const AuthorityResult Fresh = JudgeFresh(*Referee, R.Model, R.Loads, R.Options);
			INFO("fresh: " << DescribeAuthority(Fresh));
			CheckSameVerdict(R.Result, Fresh, R.Model, 1.0e-9, 1.0e-9);
			++Judges;
			for (const ComponentReport& Report : R.Result.Components)
			{
				++Components;
				Reused += ReusedOf(Report).value_or(false) ? 1 : 0;
			}
		}
	}
	MESSAGE("event region: " << EventRegionDraws << " draws, " << Judges << " judges, " << Components << " components, " << Reused << " reused");
	CHECK(Reused >= 1);
}

TEST_CASE("Cascade.EventRegion.ASecondPassSolvesOnlyWhatTheFirstReleased")
{
	Structure Model;
	const BodyId Ground = AddGroundPlane(Model);
	const ProfileId Dry = AddProfileOk(Model, DryJoint(ThreeComponentFriction));
	const ProfileId Bonded = AddProfileOk(Model, BondedJoint(1.0e6, 1.0e6, ThreeComponentFriction));
	const Box LowerShape = BrickShape(0.0, 0);
	const Box UpperShape = BrickShape(0.0, 1);
	const BodyId Lower = AddBox(Model, LowerShape);
	const BodyId Upper = AddBox(Model, UpperShape);
	AddInterfaceOk(Model, OnGroundPlane(Ground, Lower, LowerShape, Bonded));
	AddInterfaceOk(Model, *StackedFaces(Lower, LowerShape, Upper, UpperShape, Dry));
	const std::vector<BodyId> Standing = AddDryStack(Model, Ground, Dry, 2.0);
	const std::vector<BodyLoad> Loads{BodyLoad{Upper, Vec3{PushOverWeight * BrickMassKg * GravityMPerS2, 0.0, 0.0}, UpperShape.CentreM}};
	const auto C = MakeCounted();
	const CascadeVerdict V = RunFrom(*MakeCascade(std::move(C->Authority)), Model, Loads, CascadeOptions{});
	INFO(DescribeVerdict(V));
	INFO("Solve calls:" << DescribeSolves(C->Solves));
	CHECK_MESSAGE(HasJudgePrevious<IStructuralAuthority>, NoPrevious);
	CHECK(V.State == CascadeState::Stands);
	REQUIRE(V.Clusters.size() == 1u);
	CHECK(SameIdList(V.Clusters[0].Bodies, {Upper}));
	REQUIRE(C->Solves.size() == 2u);
	CHECK(C->Solves[0].Only.empty());
	CHECK(SameIdList(C->Solves[1].Only, {Lower}));
	AuthorityResult Last;
	Last.Components = V.Components;
	CHECK(V.Components.size() == 2u);
	CheckFresh(Last, {Lower});
	const ComponentReport* Kept = ComponentOf(Last, Standing);
	CHECK(Kept != nullptr);
	if (Kept != nullptr)
	{
		CHECK(ReusedOf(*Kept).value_or(false));
	}
}
