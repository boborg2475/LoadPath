#include "BitwiseTestSupport.h"
#include "BrickFixtures.h"
#include "CascadeTestSupport.h"
#include "EventRegionTestSupport.h"
#include "SlowInDebug.h"
#include "StaticsTestSupport.h"

#include "loadpath/Cascade.h"
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
#include <utility>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
struct Counted
{
	std::vector<SolveCall> Solves;
	std::unique_ptr<IStructuralAuthority> Authority;
};

std::unique_ptr<Counted> MakeCounted()
{
	auto C = std::make_unique<Counted>();
	C->Authority = MakeElasticAuthority(std::make_unique<CountingStanding>(C->Solves));
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

void CheckSameReport(const ComponentReport& Got, const ComponentReport& Want, double Rel)
{
	INFO("got bodies " << Describe(Got.Bodies) << " " << std::string(VerdictName(Got.Verdict)) << " elastic " << std::string(StandingStateName(Got.Elastic)));
	INFO("want bodies " << Describe(Want.Bodies) << " " << std::string(VerdictName(Want.Verdict)) << " elastic " << std::string(StandingStateName(Want.Elastic)));
	CHECK(SameIdList(Got.Bodies, Want.Bodies));
	CHECK(Got.Verdict == Want.Verdict);
	CHECK(Got.Elastic == Want.Elastic);
	CHECK(SameIdList(Got.Release, Want.Release));
	CHECK(Got.Mechanism.size() == Want.Mechanism.size());
	for (std::size_t M = 0; M < Got.Mechanism.size() && M < Want.Mechanism.size(); ++M)
	{
		const MechanismMotion& G = Got.Mechanism[M];
		const MechanismMotion& W = Want.Mechanism[M];
		INFO("motion " << M << " body " << G.Body.Index << " t (" << G.TranslationM.X << ", " << G.TranslationM.Y << ", " << G.TranslationM.Z << ") want body " << W.Body.Index << " t (" << W.TranslationM.X << ", " << W.TranslationM.Y << ", " << W.TranslationM.Z << ")");
		CHECK(SameHandle(G.Body, W.Body));
		CHECK(NearVec(G.TranslationM, W.TranslationM, Rel, Rel));
		CHECK(NearVec(G.RotationRad, W.RotationRad, Rel, Rel));
	}
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
	std::vector<BodyId> ReusedBodies;
	for (std::size_t K = 0; K < Got.Components.size(); ++K)
	{
		const bool bReused = Got.Components[K].bReused;
		INFO("component " << K << std::string(bReused ? " (reused, exact)" : ""));
		CheckSameReport(Got.Components[K], Want.Components[K], bReused ? 0.0 : Rel);
		if (bReused)
		{
			ReusedBodies.insert(ReusedBodies.end(), Got.Components[K].Bodies.begin(), Got.Components[K].Bodies.end());
		}
	}
	CHECK(Got.Readings.size() == Want.Readings.size());
	std::vector<InterfaceId> Exact;
	std::vector<InterfaceId> Loose;
	for (const InterfaceId Id : Model.LiveInterfaces())
	{
		const InterfaceRecord& Record = Model.Interface(Id);
		(Holds(ReusedBodies, Record.BodyA) || Holds(ReusedBodies, Record.BodyB) ? Exact : Loose).push_back(Id);
	}
	{
		INFO("readings of reused components, exact");
		CheckSameReadings(Got.Readings, Want.Readings, Exact, 0.0, 0.0);
	}
	CheckSameReadings(Got.Readings, Want.Readings, Loose, Rel, AbsN);
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
	CHECK(G->bReused);
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
	CHECK_FALSE(G->bReused);
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
	explicit RecordingAuthority(std::vector<JudgeRecord>& InRecords, std::unique_ptr<IStructuralAuthority> InInner = MakeAuthority())
		: Records(InRecords)
		, Inner(std::move(InInner))
	{
	}

	AuthorityResult Judge(const Structure& Model, std::span<const BodyLoad> Loads, const AuthorityOptions& Options, const AuthorityResult* Previous) const override
	{
		AuthorityResult Result = Inner->Judge(Model, Loads, Options, Previous);
		Records.push_back(JudgeRecord{Model, std::vector<BodyLoad>(Loads.begin(), Loads.end()), Options, Result});
		return Result;
	}

private:
	std::vector<JudgeRecord>& Records;
	std::unique_ptr<IStructuralAuthority> Inner;
};

class PreviousBlindAuthority final : public IStructuralAuthority
{
public:
	AuthorityResult Judge(const Structure& Model, std::span<const BodyLoad> Loads, const AuthorityOptions& Options, const AuthorityResult* Previous) const override
	{
		(void)Previous;
		return Inner->Judge(Model, Loads, Options, nullptr);
	}

private:
	std::unique_ptr<IStructuralAuthority> Inner = MakeAuthority();
};

void CheckSameCascade(const CascadeVerdict& Got, const CascadeVerdict& Want, const Structure& GotModel, const Structure& WantModel)
{
	INFO("warm: " << DescribeVerdict(Got));
	INFO("cold: " << DescribeVerdict(Want));
	CHECK(Got.State == Want.State);
	CHECK(Got.Passes == Want.Passes);
	CHECK(Got.FirstPass == Want.FirstPass);
	CHECK(Got.LastPass == Want.LastPass);
	CHECK(Got.Failures.size() == Want.Failures.size());
	for (std::size_t K = 0; K < Got.Failures.size() && K < Want.Failures.size(); ++K)
	{
		const FailedInterface& G = Got.Failures[K];
		const FailedInterface& W = Want.Failures[K];
		INFO("failure " << K << ": interface " << G.Id.Index << " pass " << G.Pass << " ratio " << G.Ratio << "; want interface " << W.Id.Index << " pass " << W.Pass << " ratio " << W.Ratio);
		CHECK(SameHandle(G.Id, W.Id));
		CHECK(G.Pass == W.Pass);
		CHECK(G.Axis == W.Axis);
		CHECK(Near(G.Ratio, W.Ratio, 0.0));
		CHECK(G.bSevered == W.bSevered);
	}
	CHECK(Got.Clusters.size() == Want.Clusters.size());
	for (std::size_t K = 0; K < Got.Clusters.size() && K < Want.Clusters.size(); ++K)
	{
		INFO("cluster " << K);
		CHECK(SameIdList(Got.Clusters[K].Bodies, Want.Clusters[K].Bodies));
		CHECK(SameIdList(Got.Clusters[K].Severed, Want.Clusters[K].Severed));
		CHECK(Got.Clusters[K].Pass == Want.Clusters[K].Pass);
	}
	CHECK(SameIdList(GotModel.LiveBodies(), WantModel.LiveBodies()));
	CHECK(Got.Components.size() == Want.Components.size());
	for (std::size_t K = 0; K < Got.Components.size() && K < Want.Components.size(); ++K)
	{
		INFO("final component " << K);
		CheckSameReport(Got.Components[K], Want.Components[K], 0.0);
	}
	CHECK(Got.Readings.size() == Want.Readings.size());
	CheckSameReadings(Got.Readings, Want.Readings, WantModel.LiveInterfaces(), 0.0, 0.0);
}

struct SecondPassFixture
{
	Structure Model;
	BodyId Lower;
	BodyId Upper;
	std::vector<BodyId> Standing;
	std::vector<BodyLoad> Loads;
};

SecondPassFixture MakeSecondPass()
{
	SecondPassFixture F;
	const BodyId Ground = AddGroundPlane(F.Model);
	const ProfileId Dry = AddProfileOk(F.Model, DryJoint(ThreeComponentFriction));
	const ProfileId Bonded = AddProfileOk(F.Model, BondedJoint(1.0e6, 1.0e6, ThreeComponentFriction));
	const Box LowerShape = BrickShape(0.0, 0);
	const Box UpperShape = BrickShape(0.0, 1);
	F.Lower = AddBox(F.Model, LowerShape);
	F.Upper = AddBox(F.Model, UpperShape);
	AddInterfaceOk(F.Model, OnGroundPlane(Ground, F.Lower, LowerShape, Bonded));
	AddInterfaceOk(F.Model, *StackedFaces(F.Lower, LowerShape, F.Upper, UpperShape, Dry));
	F.Standing = AddDryStack(F.Model, Ground, Dry, 2.0);
	F.Loads.push_back(BodyLoad{F.Upper, Vec3{PushOverWeight * BrickMassKg * GravityMPerS2, 0.0, 0.0}, UpperShape.CentreM});
	return F;
}

constexpr int EventRegionDraws = 32;
constexpr std::uint64_t EventRegionSeedBase = 0x6b2e0ull;
}

TEST_CASE("Structure.Revision.CountsMutationsAndStampsTouchedBodies")
{
	Structure Model;
	const auto Rev = [&] { return Model.Revision(); };
	const auto Touched = [&](BodyId Body) { return Model.LastTouched(Body); };
	CHECK(Rev() == 0u);
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
	CHECK(Copy.Revision() == 11u);
	CHECK(Copy.LastTouched(A) == 11u);
	CHECK(Copy.LastTouched(C) == 9u);

	CHECK(Model.RemoveBody(D));
	CHECK(Rev() == 12u);
	CHECK(Touched(C) == 12u);
	CHECK(Touched(A) == 11u);
	CHECK(Touched(Ground) == 8u);
	CHECK_FALSE(Model.RemoveBody(D));
	CHECK(Rev() == 12u);
	CHECK(Touched(C) == 12u);
	CHECK(Copy.Revision() == 11u);
	CHECK(Copy.LastTouched(C) == 9u);
}

TEST_CASE("Structure.LastTouched.OfAnUnknownIdReadsTheLargestValue")
{
	Structure Model;
	const BodyId Ground = AddGroundPlane(Model);
	const BodyId Brick = AddBox(Model, BrickShape(0.0, 0));
	constexpr std::uint64_t Largest = std::numeric_limits<std::uint64_t>::max();
	CHECK(Model.LastTouched(Ground) == 1u);
	CHECK(Model.LastTouched(Brick) == 2u);
	CHECK(Model.LastTouched(BodyId{999u, 1u}) == Largest);
	CHECK(Model.LastTouched(BodyId{2u, 1u}) == Largest);
	CHECK(Model.LastTouched(BodyId::Invalid()) == Largest);
	CHECK(Structure{}.LastTouched(BodyId{0u, 1u}) == Largest);
}

TEST_CASE("Structure.Lineage.IsFreshOnEveryCopy")
{
	Structure First;
	Structure Second;
	CHECK(First.Lineage() != Second.Lineage());
	const std::uint64_t Original = First.Lineage();
	AddGroundPlane(First);
	AddBox(First, BrickShape(0.0, 0));
	CHECK(First.Lineage() == Original);
	const Structure Copied = First;
	CHECK(Copied.Lineage() != First.Lineage());
	CHECK(Copied.Lineage() != Second.Lineage());
	CHECK(First.Lineage() == Original);
	Structure Assigned;
	const std::uint64_t BeforeAssignment = Assigned.Lineage();
	Assigned = First;
	CHECK(Assigned.Lineage() != First.Lineage());
	CHECK(Assigned.Lineage() != BeforeAssignment);
	CHECK(Assigned.Lineage() != Copied.Lineage());
	CHECK(Assigned.Revision() == First.Revision());
	Structure Moved = std::move(First);
	CHECK(Moved.Lineage() == Original);
	Structure MoveAssigned;
	MoveAssigned = std::move(Moved);
	CHECK(MoveAssigned.Lineage() == Original);
}

TEST_CASE("Authority.Judge.AVerdictOnADivergedCopyReusesNothing")
{
	Structure Model;
	const BodyId Ground = AddGroundPlane(Model);
	const ProfileId Bonded = AddProfileOk(Model, BondedJoint(1.0e6, 1.0e6, ThreeComponentFriction));
	const ProfileId Dry = AddProfileOk(Model, DryJoint(ThreeComponentFriction));
	const Box AShape = BrickShape(0.0, 0);
	const Box BShape = BrickShape(2.0, 0);
	const BodyId A = AddBox(Model, AShape);
	const BodyId B = AddBox(Model, BShape);
	const InterfaceId BedA = AddInterfaceOk(Model, OnGroundPlane(Ground, A, AShape, Bonded));
	const InterfaceId BedB = AddInterfaceOk(Model, OnGroundPlane(Ground, B, BShape, Dry));
	const std::vector<BodyLoad> Loads{BodyLoad{A, Vec3{PushOverWeight * BrickMassKg * GravityMPerS2, 0.0, 0.0}, AShape.CentreM}};
	const std::unique_ptr<IStructuralAuthority> Authority = MakeAuthority();
	const AuthorityResult V0 = Authority->Judge(Model, Loads, AuthorityOptions{}, nullptr);
	Structure Copy = Model;
	REQUIRE(Copy.Crack(BedA, 1));
	const AuthorityResult VC = Authority->Judge(Copy, Loads, AuthorityOptions{}, &V0);
	INFO("copy verdict: " << DescribeAuthority(VC));
	const ComponentReport* OnCopy = ComponentOf(VC, {A});
	REQUIRE(OnCopy != nullptr);
	CHECK(OnCopy->Verdict == ComponentVerdict::Collapses);
	REQUIRE(Model.Crack(BedB, 1));
	CHECK(Model.Revision() == Copy.Revision());
	const AuthorityResult Fresh = Authority->Judge(Model, Loads, AuthorityOptions{}, nullptr);
	INFO("fresh: " << DescribeAuthority(Fresh));
	const ComponentReport* FreshA = ComponentOf(Fresh, {A});
	REQUIRE(FreshA != nullptr);
	CHECK(FreshA->Verdict == ComponentVerdict::Stands);
	const AuthorityResult Got = Authority->Judge(Model, Loads, AuthorityOptions{}, &VC);
	INFO("judged against the copy's verdict: " << DescribeAuthority(Got));
	CHECK(VC.Lineage == Copy.Lineage());
	CHECK(Got.Lineage == Model.Lineage());
	CHECK(Got.Components.size() == 2u);
	CheckFresh(Got, {A});
	CheckFresh(Got, {B});
	CheckSameVerdict(Got, Fresh, Model, 0.0, 0.0);
}

TEST_CASE("Authority.Judge.InvalidOptionsAreInvalidWhateverIsClean")
{
	ThreeComponents F = MakeThreeComponents();
	const auto C = MakeCounted();
	const AuthorityResult First = C->Authority->Judge(F.Model, F.Loads, AuthorityOptions{}, nullptr);
	REQUIRE(First.Worst != ComponentVerdict::InvalidInput);
	std::vector<std::pair<const char*, AuthorityOptions>> Variants;
	const auto Add = [&](const char* Name, auto Edit)
	{
		AuthorityOptions Options;
		Edit(Options.Standing);
		Variants.emplace_back(Name, Options);
	};
	Add("Standing.PivotToleranceRelative 0", [](StandingOptions& S) { S.PivotToleranceRelative = 0.0; });
	Add("Standing.PivotToleranceRelative -1e-10", [](StandingOptions& S) { S.PivotToleranceRelative = -1.0e-10; });
	Add("Standing.PivotToleranceRelative 1", [](StandingOptions& S) { S.PivotToleranceRelative = 1.0; });
	Add("Standing.PivotToleranceRelative NaN", [](StandingOptions& S) { S.PivotToleranceRelative = NaN; });
	Add("Standing.MaxActiveSetIterations 0", [](StandingOptions& S) { S.MaxActiveSetIterations = 0; });
	Add("Standing.MaxActiveSetIterations -1", [](StandingOptions& S) { S.MaxActiveSetIterations = -1; });
	for (const auto& [Name, Options] : Variants)
	{
		INFO("options: " << std::string(Name));
		const AuthorityResult Fresh = MakeAuthority()->Judge(F.Model, F.Loads, Options, nullptr);
		CHECK(Fresh.Worst == ComponentVerdict::InvalidInput);
		AuthorityResult Previous = First;
		Previous.Options = Options;
		C->Solves.clear();
		const AuthorityResult Got = C->Authority->Judge(F.Model, F.Loads, Options, &Previous);
		INFO("with Previous and nothing dirty: " << DescribeAuthority(Got) << "; Solve calls:" << DescribeSolves(C->Solves));
		CHECK(Got.Worst == ComponentVerdict::InvalidInput);
	}
}

TEST_CASE("Authority.Judge.ALoadOnANeverIssuedBodyIsInvalidWhateverIsClean")
{
	ThreeComponents F = MakeThreeComponents();
	const BodyId Live = F.Parts[0][1];
	const BodyId NeverIssued{Live.Index, Live.Generation + 1u};
	REQUIRE_FALSE(F.Model.IsKnown(NeverIssued));
	const std::unique_ptr<IStructuralAuthority> Authority = MakeAuthority();
	const AuthorityResult V1 = Authority->Judge(F.Model, F.Loads, AuthorityOptions{}, nullptr);
	REQUIRE(V1.Worst != ComponentVerdict::InvalidInput);
	std::vector<BodyLoad> Loads = F.Loads;
	Loads.push_back(BodyLoad{NeverIssued, Vec3{1.0, 0.0, 0.0}, F.Model.Body(Live).CentroidM});
	const AuthorityResult Got = Authority->Judge(F.Model, Loads, AuthorityOptions{}, &V1);
	INFO("got: " << DescribeAuthority(Got));
	CHECK(Got.Worst == ComponentVerdict::InvalidInput);
}

TEST_CASE("Authority.Judge.ALoadOnADeadBodyIsSkippedWhateverIsClean")
{
	ThreeComponents F = MakeThreeComponents();
	const BodyId Dead = F.Parts[0][1];
	const Vec3 DeadCentreM = F.Model.Body(Dead).CentroidM;
	REQUIRE(F.Model.RemoveBody(Dead));
	const std::unique_ptr<IStructuralAuthority> Authority = MakeAuthority();
	const AuthorityResult V1 = Authority->Judge(F.Model, F.Loads, AuthorityOptions{}, nullptr);
	REQUIRE(V1.Worst != ComponentVerdict::InvalidInput);
	std::vector<BodyLoad> Loads = F.Loads;
	Loads.push_back(BodyLoad{Dead, Vec3{1.0, 0.0, 0.0}, DeadCentreM});
	const AuthorityResult Got = Authority->Judge(F.Model, Loads, AuthorityOptions{}, &V1);
	INFO("got: " << DescribeAuthority(Got));
	CHECK(Got.Worst == V1.Worst);
	CHECK(Got.StaleLoads == std::vector<std::size_t>{F.Loads.size()});
	REQUIRE(Got.Components.size() == V1.Components.size());
	for (std::size_t I = 0; I < Got.Components.size(); ++I)
	{
		CHECK(Got.Components[I].Elastic == V1.Components[I].Elastic);
		CHECK(Got.Components[I].bReused);
	}
	const AuthorityResult Want = Authority->Judge(F.Model, F.Loads, AuthorityOptions{}, &V1);
	INFO("want: " << DescribeAuthority(Want));
	CHECK(bitwise::Same(Got, Want));
}

TEST_CASE("Authority.Judge.ReusesCleanComponentsFromThePreviousVerdict")
{
	ThreeComponents F = MakeThreeComponents();
	const auto C = MakeCounted();
	const AuthorityResult First = C->Authority->Judge(F.Model, F.Loads, AuthorityOptions{}, nullptr);
	INFO("first: " << DescribeAuthority(First));
	CheckSolvedOnly(C->Solves, {});
	REQUIRE(First.Components.size() == 3u);
	CHECK(First.Components[0].Elastic == StandingState::Stands);
	const InterfaceId Bed = InterfaceBetween(F.Model, F.Ground, F.Parts[0][0]);
	CHECK(F.Model.Sever(Bed, 1));
	C->Solves.clear();
	const AuthorityResult Second = C->Authority->Judge(F.Model, F.Loads, AuthorityOptions{}, &First);
	INFO("second: " << DescribeAuthority(Second));
	CheckSolvedOnly(C->Solves, F.Parts[0]);
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
	const AuthorityResult First = C->Authority->Judge(F.Model, F.Loads, AuthorityOptions{}, nullptr);
	std::vector<BodyLoad> Pushed = F.Loads;
	const BodyId Top = F.Parts[2][1];
	Pushed.push_back(BodyLoad{Top, Vec3{0.1 * BrickMassKg * GravityMPerS2, 0.0, 0.0}, F.Model.Body(Top).CentroidM});
	C->Solves.clear();
	const AuthorityResult Second = C->Authority->Judge(F.Model, Pushed, AuthorityOptions{}, &First);
	INFO("first: " << DescribeAuthority(First));
	INFO("second: " << DescribeAuthority(Second));
	CheckSolvedOnly(C->Solves, F.Parts[2]);
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

TEST_CASE("Authority.Judge.AMovedLoadPointDirtiesItsComponent")
{
	ThreeComponents F = MakeThreeComponents();
	const auto C = MakeCounted();
	const BodyId Top = F.Parts[2][1];
	const Vec3 PushN{0.1 * BrickMassKg * GravityMPerS2, 0.0, 0.0};
	const Vec3 CentreM = F.Model.Body(Top).CentroidM;
	std::vector<BodyLoad> Centred = F.Loads;
	Centred.push_back(BodyLoad{Top, PushN, CentreM});
	std::vector<BodyLoad> Moved = F.Loads;
	Moved.push_back(BodyLoad{Top, PushN, Vec3{CentreM.X, CentreM.Y, CentreM.Z + 0.03}});
	const AuthorityResult First = C->Authority->Judge(F.Model, Centred, AuthorityOptions{}, nullptr);
	C->Solves.clear();
	const AuthorityResult Second = C->Authority->Judge(F.Model, Moved, AuthorityOptions{}, &First);
	INFO("first: " << DescribeAuthority(First));
	INFO("second: " << DescribeAuthority(Second));
	CheckSolvedOnly(C->Solves, F.Parts[2]);
	CheckFresh(Second, F.Parts[2]);
	CheckReused(Second, First, F.Model, F.Parts[0]);
	CheckReused(Second, First, F.Model, F.Parts[1]);
}

TEST_CASE("Authority.Judge.AGroundContactAddedToALooseBrickIsFresh")
{
	Structure Model;
	const BodyId Ground = AddGroundPlane(Model);
	const ProfileId Dry = AddProfileOk(Model, DryJoint(ThreeComponentFriction));
	const std::vector<BodyId> Standing = AddDryStack(Model, Ground, Dry, 2.0);
	const Box LooseShape = BrickShape(0.0, 0);
	const BodyId Loose = AddBox(Model, LooseShape);
	const auto C = MakeCounted();
	const AuthorityResult V0 = C->Authority->Judge(Model, {}, AuthorityOptions{}, nullptr);
	INFO("first: " << DescribeAuthority(V0));
	const ComponentReport* Before = ComponentOf(V0, {Loose});
	REQUIRE(Before != nullptr);
	CHECK(Before->Verdict == ComponentVerdict::Collapses);
	AddInterfaceOk(Model, OnGroundPlane(Ground, Loose, LooseShape, Dry));
	C->Solves.clear();
	const AuthorityResult Got = C->Authority->Judge(Model, {}, AuthorityOptions{}, &V0);
	INFO("after the ground contact: " << DescribeAuthority(Got));
	CheckSolvedOnly(C->Solves, {Loose});
	CheckFresh(Got, {Loose});
	const ComponentReport* After = ComponentOf(Got, {Loose});
	REQUIRE(After != nullptr);
	CHECK(After->Verdict == ComponentVerdict::Stands);
	CheckReused(Got, V0, Model, Standing);
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
	const AuthorityResult First = C->Authority->Judge(F.Model, F.Loads, AuthorityOptions{}, nullptr);
	INFO("first: " << DescribeAuthority(First));
	CHECK(First.Components.size() == 4u);
	CHECK(F.Model.RemoveBody(Deck));
	C->Solves.clear();
	const AuthorityResult Second = C->Authority->Judge(F.Model, F.Loads, AuthorityOptions{}, &First);
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
	const AuthorityResult Fresh = Authority->Judge(F.Model, F.Loads, AuthorityOptions{}, nullptr);
	Structure Other;
	const BodyId OtherGround = AddGroundPlane(Other);
	const ProfileId OtherDry = AddProfileOk(Other, DryJoint(ThreeComponentFriction));
	AddBox(Other, BrickShape(5.0, 3));
	AddDryStack(Other, OtherGround, OtherDry, 6.0);
	const AuthorityResult OtherVerdict = Authority->Judge(Other, {}, AuthorityOptions{}, nullptr);
	AuthorityResult Ahead = Fresh;
	Ahead.Revision = F.Model.Revision() + 5u;
	const std::vector<std::pair<const char*, const AuthorityResult*>> Unmatched{{"a verdict on another structure", &OtherVerdict}, {"a verdict whose revision is ahead of the model", &Ahead}};
	for (const auto& [Name, Previous] : Unmatched)
	{
		INFO("previous: " << std::string(Name));
		const AuthorityResult Got = Authority->Judge(F.Model, F.Loads, AuthorityOptions{}, Previous);
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
	const AuthorityResult First = C->Authority->Judge(F.Model, F.Loads, AuthorityOptions{}, nullptr);
	C->Solves.clear();
	const AuthorityResult Second = C->Authority->Judge(F.Model, F.Loads, AuthorityOptions{}, &First);
	INFO("Solve calls:" << DescribeSolves(C->Solves));
	CHECK(C->Solves.empty());
	CHECK(Second.Components.size() == 3u);
	for (const std::vector<BodyId>& Part : F.Parts)
	{
		CheckReused(Second, First, F.Model, Part);
	}
	CheckSameVerdict(Second, First, F.Model, 0.0, 0.0);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Cascade.EventRegion.ReusedComponentsEqualAFreshJudge")
{
	const std::unique_ptr<IStructuralAuthority> Referee = MakeAuthority();
	int Judges = 0;
	int Components = 0;
	int Reused = 0;
	int WarmFirstReused = 0;
	for (int Draw = 0; Draw < EventRegionDraws; ++Draw)
	{
		const std::uint64_t Seed = EventRegionSeedBase + static_cast<std::uint64_t>(Draw);
		{
			MultiDraw D = MakeMultiDraw(Seed);
			INFO("every judge of a cascade, seed " << Seed << ": " << D.Description);
			std::vector<JudgeRecord> Records;
			CascadeOptions Options;
			Options.Authority = D.Options;
			Options.MaxPasses = 16;
			const std::unique_ptr<ICascade> Cascade = MakeCascade(std::make_unique<RecordingAuthority>(Records));
			const CascadeVerdict V = Cascade->Run(D.Model, D.Loads, Options, nullptr);
			INFO("cascade: " << DescribeVerdict(V));
			for (std::size_t J = 0; J < Records.size(); ++J)
			{
				const JudgeRecord& R = Records[J];
				INFO("judge " << J << ": " << DescribeAuthority(R.Result));
				const AuthorityResult Fresh = Referee->Judge(R.Model, R.Loads, R.Options, nullptr);
				INFO("fresh: " << DescribeAuthority(Fresh));
				CheckSameVerdict(R.Result, Fresh, R.Model, 1.0e-9, 1.0e-9);
				++Judges;
				for (const ComponentReport& Report : R.Result.Components)
				{
					++Components;
					Reused += Report.bReused ? 1 : 0;
				}
			}
		}
		{
			MultiDraw D = MakeMultiDraw(Seed);
			CascadeOptions Options;
			Options.Authority = D.Options;
			Options.MaxPasses = 16;
			Structure Warm = D.Model;
			const AuthorityResult V0 = Referee->Judge(Warm, D.Loads, D.Options, nullptr);
			std::vector<BodyId> Free;
			for (const BodyId Body : Warm.LiveBodies())
			{
				if (!Warm.Body(Body).bGrounded)
				{
					Free.push_back(Body);
				}
			}
			REQUIRE(!Free.empty());
			const BodyId Victim = Free[static_cast<std::size_t>(Seed % Free.size())];
			INFO("cascade parity, seed " << Seed << ": " << D.Description << ", body " << Victim.Index << " removed");
			Structure Cold = Warm;
			REQUIRE(Warm.RemoveBody(Victim));
			REQUIRE(Cold.RemoveBody(Victim));
			std::vector<BodyLoad> Loads;
			for (const BodyLoad& Load : D.Loads)
			{
				if (!SameHandle(Load.Body, Victim))
				{
					Loads.push_back(Load);
				}
			}
			std::vector<JudgeRecord> WarmRecords;
			const CascadeVerdict WarmV = MakeCascade(std::make_unique<RecordingAuthority>(WarmRecords))->Run(Warm, Loads, Options, &V0);
			const CascadeVerdict ColdV = MakeCascade(std::make_unique<PreviousBlindAuthority>())->Run(Cold, Loads, Options, nullptr);
			CheckSameCascade(WarmV, ColdV, Warm, Cold);
			if (!WarmRecords.empty())
			{
				for (const ComponentReport& Report : WarmRecords.front().Result.Components)
				{
					WarmFirstReused += Report.bReused ? 1 : 0;
				}
			}
		}
	}
	MESSAGE("event region: " << EventRegionDraws << " draws, " << Judges << " judges, " << Components << " components, " << Reused << " reused; cascade parity: " << WarmFirstReused << " components reused by the warm first pass");
	CHECK(Reused >= 1);
	CHECK(WarmFirstReused >= 1);
}

TEST_CASE("Cascade.EventRegion.ASecondPassSolvesOnlyWhatTheFirstReleased")
{
	SecondPassFixture F = MakeSecondPass();
	const auto C = MakeCounted();
	const CascadeVerdict V = MakeCascade(std::move(C->Authority))->Run(F.Model, F.Loads, CascadeOptions{}, nullptr);
	INFO(DescribeVerdict(V));
	INFO("Solve calls:" << DescribeSolves(C->Solves));
	CHECK(V.State == CascadeState::Stands);
	REQUIRE(V.Clusters.size() == 1u);
	CHECK(SameIdList(V.Clusters[0].Bodies, {F.Upper}));
	REQUIRE(C->Solves.size() == 2u);
	CHECK(C->Solves[0].Only.empty());
	CHECK(SameIdList(C->Solves[1].Only, {F.Lower}));
	AuthorityResult Last;
	Last.Components = V.Components;
	CHECK(V.Components.size() == 2u);
	CheckFresh(Last, {F.Lower});
	const ComponentReport* Kept = ComponentOf(Last, F.Standing);
	CHECK(Kept != nullptr);
	if (Kept != nullptr)
	{
		CHECK(Kept->bReused);
	}
}

TEST_CASE("Cascade.EventRegion.TheCapReJudgeIsFiltered")
{
	SecondPassFixture F = MakeSecondPass();
	const auto C = MakeCounted();
	CascadeOptions Options;
	Options.MaxPasses = 1;
	const CascadeVerdict V = MakeCascade(std::move(C->Authority))->Run(F.Model, F.Loads, Options, nullptr);
	INFO(DescribeVerdict(V));
	INFO("Solve calls:" << DescribeSolves(C->Solves));
	CHECK(V.State == CascadeState::PassCapReached);
	CHECK(V.Passes == 1);
	REQUIRE(C->Solves.size() == 2u);
	CHECK(C->Solves[0].Only.empty());
	CHECK(SameIdList(C->Solves[1].Only, {F.Lower}));
	AuthorityResult Last;
	Last.Components = V.Components;
	CheckFresh(Last, {F.Lower});
	const ComponentReport* Kept = ComponentOf(Last, F.Standing);
	CHECK(Kept != nullptr);
	if (Kept != nullptr)
	{
		CHECK(Kept->bReused);
	}
}

TEST_CASE("Cascade.EventRegion.TheFirstPassReadsTheHostPrevious")
{
	ThreeComponents F = MakeThreeComponents();
	const CascadeOptions Options;
	const AuthorityResult V0 = MakeAuthority()->Judge(F.Model, F.Loads, Options.Authority, nullptr);
	REQUIRE(F.Model.RemoveBody(F.Parts[0][1]));
	const auto C = MakeCounted();
	std::vector<JudgeRecord> Records;
	const CascadeVerdict V = MakeCascade(std::make_unique<RecordingAuthority>(Records, std::move(C->Authority)))->Run(F.Model, F.Loads, Options, &V0);
	INFO(DescribeVerdict(V));
	INFO("Solve calls:" << DescribeSolves(C->Solves));
	REQUIRE(!C->Solves.empty());
	CHECK(SameIdList(C->Solves[0].Only, {F.Parts[0][0]}));
	REQUIRE(!Records.empty());
	const JudgeRecord& FirstPass = Records.front();
	INFO("first pass: " << DescribeAuthority(FirstPass.Result));
	CheckFresh(FirstPass.Result, {F.Parts[0][0]});
	CheckReused(FirstPass.Result, V0, FirstPass.Model, F.Parts[1]);
	CheckReused(FirstPass.Result, V0, FirstPass.Model, F.Parts[2]);
}

TEST_CASE("Statics.Solve.AComponentFilterSolvesTheListedComponentsOnly")
{
	ThreeComponents F = MakeThreeComponents();
	const ProfileId Dry = AddProfileOk(F.Model, DryJoint(ThreeComponentFriction));
	const Box PlinthShape = BrickShape(4.0);
	const BodyId Plinth = AddBox(F.Model, PlinthShape, true);
	AddInterfaceOk(F.Model, OnGroundPlane(F.Ground, Plinth, PlinthShape, Dry));
	const std::unique_ptr<IStandingAuthority> Authority = MakeRbsmStandingAuthority();
	const StandingResult Whole = Authority->Solve(F.Model, F.Loads, StandingOptions{}, {});
	CHECK(Whole.Components.size() == 3u);
	const auto ExpectedFilteredIds = [&](const std::vector<BodyId>& Bodies)
	{
		std::vector<InterfaceId> Ids;
		for (const InterfaceId Id : F.Model.LiveInterfaces())
		{
			const InterfaceRecord& Record = F.Model.Interface(Id);
			const bool bBothGrounded = F.Model.Body(Record.BodyA).bGrounded && F.Model.Body(Record.BodyB).bGrounded;
			if (bBothGrounded || Holds(Bodies, Record.BodyA) || Holds(Bodies, Record.BodyB))
			{
				Ids.push_back(Id);
			}
		}
		return Ids;
	};
	const auto CheckFilteredSolve = [&](const StandingResult& Got, const std::vector<std::vector<BodyId>>& Wanted)
	{
		INFO("filtered state " << std::string(StandingStateName(Got.State)) << ", readings " << Got.Readings.size() << ", components " << Got.Components.size());
		std::vector<BodyId> AllWanted;
		for (const std::vector<BodyId>& Part : Wanted)
		{
			AllWanted.insert(AllWanted.end(), Part.begin(), Part.end());
		}
		const std::vector<InterfaceId> Ids = ExpectedFilteredIds(AllWanted);
		CHECK(Got.Components.size() == Wanted.size());
		StandingState WorstWanted = StandingState::Stands;
		for (std::size_t K = 0; K < Wanted.size() && K < Got.Components.size(); ++K)
		{
			INFO("component " << K << " want " << Describe(Wanted[K]) << " got " << Describe(Got.Components[K].Bodies));
			CHECK(SameIdList(Got.Components[K].Bodies, Wanted[K]));
			const StandingComponent* Alone = nullptr;
			for (const StandingComponent& Each : Whole.Components)
			{
				if (SameIdList(Each.Bodies, Wanted[K]))
				{
					Alone = &Each;
				}
			}
			CHECK(Alone != nullptr);
			if (Alone == nullptr)
			{
				continue;
			}
			CHECK(Got.Components[K].State == Alone->State);
			if (static_cast<int>(Alone->State) > static_cast<int>(WorstWanted))
			{
				WorstWanted = Alone->State;
			}
			const std::vector<InterfaceId> Own = InterfacesTouching(F.Model, Wanted[K]);
			CHECK(Got.Components[K].ReadingSlots.size() == Own.size());
			for (const std::size_t Slot : Got.Components[K].ReadingSlots)
			{
				INFO("reading slot " << Slot);
				CHECK(Slot < Got.Readings.size());
				if (Slot < Got.Readings.size())
				{
					CHECK(std::any_of(Own.begin(), Own.end(), [&](InterfaceId Id) { return SameHandle(Id, Got.Readings[Slot].Id); }));
				}
			}
		}
		CHECK(Got.State == WorstWanted);
		CHECK(Got.Readings.size() == Ids.size());
		for (std::size_t K = 0; K < Ids.size() && K < Got.Readings.size(); ++K)
		{
			INFO("reading " << K << " want interface " << Ids[K].Index << " got " << Got.Readings[K].Id.Index);
			CHECK(SameHandle(Got.Readings[K].Id, Ids[K]));
			const InterfaceReading* W = FindReading(Whole.Readings, Ids[K]);
			CHECK(W != nullptr);
			CHECK((W != nullptr && SameReading(Got.Readings[K], *W, 0.0, 0.0)));
		}
	};
	{
		INFO("Only = the first stack");
		CheckFilteredSolve(Authority->Solve(F.Model, F.Loads, StandingOptions{}, F.Parts[0]), {F.Parts[0]});
	}
	{
		INFO("Only = the last stack's upper brick");
		CheckFilteredSolve(Authority->Solve(F.Model, F.Loads, StandingOptions{}, std::vector<BodyId>{F.Parts[2][1]}), {F.Parts[2]});
	}
	{
		INFO("Only = the pushed brick and the last stack");
		const std::vector<BodyId> Only{F.Parts[2][0], F.Parts[1][0]};
		CheckFilteredSolve(Authority->Solve(F.Model, F.Loads, StandingOptions{}, Only), {F.Parts[1], F.Parts[2]});
	}
	{
		INFO("Only = the first stack with a duplicate entry");
		const std::vector<BodyId> Only{F.Parts[0][1], F.Parts[0][0], F.Parts[0][1]};
		CheckFilteredSolve(Authority->Solve(F.Model, F.Loads, StandingOptions{}, Only), {F.Parts[0]});
	}
	const std::vector<std::pair<const char*, std::vector<BodyId>>> BadFilters{
		{"the ground", {F.Ground}},
		{"a grounded plinth", {Plinth}},
		{"an unknown id", {BodyId{999u, 1u}}},
		{"the invalid handle", {BodyId::Invalid()}},
		{"a free body and the ground", {F.Parts[0][0], F.Ground}},
	};
	for (const auto& [Name, Only] : BadFilters)
	{
		const StandingResult Got = Authority->Solve(F.Model, F.Loads, StandingOptions{}, Only);
		INFO("filter naming " << std::string(Name) << ": state " << std::string(StandingStateName(Got.State)) << ", readings " << Got.Readings.size() << ", invalid interface " << Got.InvalidInterface.Index);
		CHECK(Got.State == StandingState::InvalidInput);
		CHECK(SameHandle(Got.InvalidInterface, InterfaceId::Invalid()));
		CHECK(Got.Readings.empty());
		CHECK(Got.Components.empty());
	}
}

TEST_CASE("Authority.Judge.DifferentOptionsReuseNothing")
{
	ThreeComponents F = MakeThreeComponents();
	const auto C = MakeCounted();
	const AuthorityOptions Base;
	const AuthorityResult First = C->Authority->Judge(F.Model, F.Loads, Base, nullptr);
	CHECK(NearVec(First.Options.Standing.Gravity, Base.Standing.Gravity, 0.0));
	std::vector<std::pair<const char*, AuthorityOptions>> Variants;
	Variants.emplace_back("gravity tilted 5 deg", AuthorityTilted(5.0 * RadPerDegree));
	AuthorityOptions Pivot = Base;
	Pivot.Standing.PivotToleranceRelative = 1.0e-9;
	Variants.emplace_back("Standing.PivotToleranceRelative 1e-9", Pivot);
	AuthorityOptions Dof = Base;
	Dof.Standing.DenseDofLimit = 6;
	Variants.emplace_back("Standing.DenseDofLimit 6", Dof);
	const auto Add = [&](const char* Name, auto Edit)
	{
		AuthorityOptions Options = Base;
		Edit(Options);
		Variants.emplace_back(Name, Options);
	};
	Add("Standing.MaxActiveSetIterations 49", [](AuthorityOptions& O) { O.Standing.MaxActiveSetIterations = 49; });
	Add("Standing.DryRecloseFactor 3", [](AuthorityOptions& O) { O.Standing.DryRecloseFactor = 3.0; });
	Add("Standing.SparseAcceptanceMargin 2", [](AuthorityOptions& O) { O.Standing.SparseAcceptanceMargin = 2.0; });
	Add("Standing.OpenTensionToleranceRelative 1e-9", [](AuthorityOptions& O) { O.Standing.OpenTensionToleranceRelative = 1e-9; });
	Add("Standing.CloseCompressionToleranceRelative 1e-9", [](AuthorityOptions& O) { O.Standing.CloseCompressionToleranceRelative = 1e-9; });
	for (const auto& [Name, Options] : Variants)
	{
		INFO("options differing in " << std::string(Name));
		const AuthorityResult Fresh = MakeAuthority()->Judge(F.Model, F.Loads, Options, nullptr);
		C->Solves.clear();
		const AuthorityResult Got = C->Authority->Judge(F.Model, F.Loads, Options, &First);
		INFO("got: " << DescribeAuthority(Got));
		CheckSolvedOnly(C->Solves, {});
		CHECK(Got.Components.size() == 3u);
		for (const std::vector<BodyId>& Part : F.Parts)
		{
			CheckFresh(Got, Part);
		}
		CheckSameVerdict(Got, Fresh, F.Model, 0.0, 0.0);
	}
}

TEST_CASE("Structure.Lineage.AMovedFromStructureTakesAFreshLineage")
{
	Structure A;
	AddGroundPlane(A);
	AddBox(A, BrickShape(0.0, 0));
	const std::uint64_t Original = A.Lineage();
	const std::uint64_t Revision = A.Revision();
	Structure B(std::move(A));
	CHECK(B.Lineage() == Original);
	CHECK(B.Revision() == Revision);
	CHECK(B.LiveBodyCount() == 2u);
	CHECK(A.Lineage() != B.Lineage());
	CHECK(A.Lineage() != Original);
	Structure C;
	C = std::move(B);
	CHECK(C.Lineage() == Original);
	CHECK(C.Revision() == Revision);
	CHECK(C.LiveBodyCount() == 2u);
	CHECK(B.Lineage() != C.Lineage());
	CHECK(B.Lineage() != Original);
	CHECK(B.Lineage() != A.Lineage());
}

TEST_CASE("Authority.Judge.AVerdictOnARebuiltMovedFromStructureReusesNothing")
{
	const auto Build = [](Structure& Model, const StrengthProfile& Bed, std::vector<BodyLoad>& Loads)
	{
		const BodyId Ground = AddGroundPlane(Model);
		const ProfileId P = AddProfileOk(Model, Bed);
		const Box Shape = BrickShape(0.0, 0);
		const BodyId Brick = AddBox(Model, Shape);
		AddInterfaceOk(Model, OnGroundPlane(Ground, Brick, Shape, P));
		Loads = {BodyLoad{Brick, Vec3{PushOverWeight * BrickMassKg * GravityMPerS2, 0.0, 0.0}, Shape.CentreM}};
		return Brick;
	};
	const std::unique_ptr<IStructuralAuthority> Authority = MakeAuthority();
	for (const bool bAssign : {false, true})
	{
		INFO(std::string(bAssign ? "move assignment" : "move construction"));
		Structure A;
		std::vector<BodyLoad> BondedLoads;
		const BodyId Brick = Build(A, BondedJoint(1.0e6, 1.0e6, ThreeComponentFriction), BondedLoads);
		std::optional<Structure> Moved;
		if (bAssign)
		{
			Moved.emplace();
			*Moved = std::move(A);
		}
		else
		{
			Moved.emplace(std::move(A));
		}
		Structure& B = *Moved;
		CHECK(B.Revision() == 3u);
		std::vector<BodyLoad> DryLoads;
		const BodyId Rebuilt = Build(A, DryJoint(ThreeComponentFriction), DryLoads);
		CHECK(SameHandle(Rebuilt, Brick));
		CHECK(SameHandle(DryLoads[0].Body, BondedLoads[0].Body));
		CHECK(NearVec(DryLoads[0].ForceN, BondedLoads[0].ForceN, 0.0));
		CHECK(NearVec(DryLoads[0].PointM, BondedLoads[0].PointM, 0.0));
		const AuthorityResult VA = Authority->Judge(A, DryLoads, AuthorityOptions{}, nullptr);
		INFO("rebuilt A: " << DescribeAuthority(VA));
		const ComponentReport* OnA = ComponentOf(VA, {Brick});
		REQUIRE(OnA != nullptr);
		CHECK(OnA->Verdict == ComponentVerdict::Collapses);
		for (int K = 1; K <= 3; ++K)
		{
			AddBox(B, BrickShape(static_cast<double>(K), 0));
		}
		CHECK(B.Revision() == 6u);
		const AuthorityResult Fresh = Authority->Judge(B, BondedLoads, AuthorityOptions{}, nullptr);
		INFO("fresh on B: " << DescribeAuthority(Fresh));
		const ComponentReport* FreshBrick = ComponentOf(Fresh, {Brick});
		REQUIRE(FreshBrick != nullptr);
		CHECK(FreshBrick->Verdict == ComponentVerdict::Stands);
		const AuthorityResult Got = Authority->Judge(B, BondedLoads, AuthorityOptions{}, &VA);
		INFO("B judged against A's verdict: " << DescribeAuthority(Got));
		CheckFresh(Got, {Brick});
		CheckSameVerdict(Got, Fresh, B, 0.0, 0.0);
	}
}

TEST_CASE("Authority.Judge.AChangedLoadForceDirtiesItsComponent")
{
	ThreeComponents F = MakeThreeComponents();
	const auto C = MakeCounted();
	const BodyId Top = F.Parts[2][1];
	const Vec3 CentreM = F.Model.Body(Top).CentroidM;
	std::vector<BodyLoad> Light = F.Loads;
	Light.push_back(BodyLoad{Top, Vec3{0.1 * BrickMassKg * GravityMPerS2, 0.0, 0.0}, CentreM});
	std::vector<BodyLoad> Heavy = F.Loads;
	Heavy.push_back(BodyLoad{Top, Vec3{PushOverWeight * BrickMassKg * GravityMPerS2, 0.0, 0.0}, CentreM});
	const AuthorityResult First = C->Authority->Judge(F.Model, Light, AuthorityOptions{}, nullptr);
	C->Solves.clear();
	const AuthorityResult Second = C->Authority->Judge(F.Model, Heavy, AuthorityOptions{}, &First);
	INFO("first: " << DescribeAuthority(First));
	INFO("second: " << DescribeAuthority(Second));
	CheckSolvedOnly(C->Solves, F.Parts[2]);
	CheckFresh(Second, F.Parts[2]);
	const ComponentReport* Pushed = ComponentOf(Second, F.Parts[2]);
	REQUIRE(Pushed != nullptr);
	CHECK(Pushed->Verdict == ComponentVerdict::Collapses);
	CheckReused(Second, First, F.Model, F.Parts[0]);
	CheckReused(Second, First, F.Model, F.Parts[1]);
}
