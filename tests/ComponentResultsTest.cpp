#include "BrickFixtures.h"
#include "CascadeTestSupport.h"
#include "CatalogueTestSupport.h"
#include "CertificateTestSupport.h"
#include "EventRegionTestSupport.h"
#include "SlowInDebug.h"
#include "StaticsTestSupport.h"

#include "loadpath/Catalogue.h"
#include "loadpath/CollapseCertificate.h"
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

template <class C>
constexpr bool HasComponentFilter = requires(const C& Cert, const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, std::span<const BodyId> Only) { Cert.Decide(Model, Loads, Options, Only); };

template <class S>
constexpr bool HasCachedPartition = requires(const S& Model) { Model.Components().FreeBodies.size(); };

template <class C>
CertificateResult ForwardDecide(const C& Inner, const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, std::span<const BodyId> Only)
{
	if constexpr (HasComponentFilter<C>)
	{
		return Inner.Decide(Model, Loads, Options, Only);
	}
	else
	{
		(void)Only;
		return Inner.Decide(Model, Loads, Options);
	}
}

template <class C>
std::optional<CertificateResult> DecideFiltered(const C& Cert, const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, std::span<const BodyId> Only)
{
	if constexpr (HasComponentFilter<C>)
	{
		return Cert.Decide(Model, Loads, Options, Only);
	}
	else
	{
		(void)Cert;
		(void)Model;
		(void)Loads;
		(void)Options;
		(void)Only;
		return std::nullopt;
	}
}

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

std::string DescribeCertificate(const CertificateResult& R)
{
	std::ostringstream Out;
	Out << CertificateStateName(R.State) << " t [" << R.LoadFactorLower << ", " << R.LoadFactorUpper << "], iterations " << R.Iterations << ", mechanism " << R.Mechanism.size() << ", unconstrained " << Describe(R.UnconstrainedBodies) << ", hinging " << Describe(R.HingingInterfaces);
	return Out.str();
}

void CheckSameCertificate(const CertificateResult& Got, const CertificateResult& Want)
{
	INFO("filtered: " << DescribeCertificate(Got));
	INFO("alone:    " << DescribeCertificate(Want));
	CHECK(Got.State == Want.State);
	CHECK(Near(Got.LoadFactorLower, Want.LoadFactorLower, 1.0e-9));
	CHECK(Near(Got.LoadFactorUpper, Want.LoadFactorUpper, 1.0e-9));
	CHECK(Got.Iterations == Want.Iterations);
	CHECK(SameMotions(Got.Mechanism, Want.Mechanism));
	CHECK(SameIdList(Got.UnconstrainedBodies, Want.UnconstrainedBodies));
	CHECK(SameIdList(Got.HingingInterfaces, Want.HingingInterfaces));
}

int CertificateSeverity(CertificateState State)
{
	switch (State)
	{
	case CertificateState::Stands: return 0;
	case CertificateState::Collapses: return 1;
	case CertificateState::Undecided: return 2;
	case CertificateState::InvalidInput: return 3;
	}
	return 3;
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

bool SameBound(double A, double B)
{
	return A == B || (std::isnan(A) && std::isnan(B));
}

void CheckJudgeMatchesEachComponentAlone(std::uint64_t Seed, double Cap, int& ComponentsSeen, int& Consulted, int& Mismatches)
{
	MultiDraw D = MakeMultiDraw(Seed);
	if (Cap > 0.0)
	{
		D.Options.Certificate.LoadFactorCap = Cap;
	}
	INFO("seed " << Seed << " (cap " << D.Options.Certificate.LoadFactorCap << "): " << D.Description);
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
		Consulted += Got.bConsultedCertificate ? 1 : 0;
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
		const bool bSame = SameIdList(Got.Bodies, Want.Bodies) && Got.Verdict == Want.Verdict && Got.Elastic == Want.Elastic && Got.Kinematic == Want.Kinematic && Got.bConsultedCertificate == Want.bConsultedCertificate && SameBound(Got.LoadFactorLower, Want.LoadFactorLower) && SameBound(Got.LoadFactorUpper, Want.LoadFactorUpper) && Mismatched == 0;
		Mismatches += bSame ? 0 : 1;
		CHECK(SameIdList(Got.Bodies, Want.Bodies));
		CHECK(Got.Verdict == Want.Verdict);
		CHECK(Got.Elastic == Want.Elastic);
		CHECK(Got.Kinematic == Want.Kinematic);
		CHECK(Got.bConsultedCertificate == Want.bConsultedCertificate);
		CHECK(SameBound(Got.LoadFactorLower, Want.LoadFactorLower));
		CHECK(SameBound(Got.LoadFactorUpper, Want.LoadFactorUpper));
		CHECK_MESSAGE(Mismatched == 0, Mismatched << " of " << Reference.Readings.size() << " readings differ from the component alone");
	}
}
}

TEST_CASE("Authority.Judge.SolvesTheElasticFieldOnceAndConsultsOnlyTheUndecided")
{
	const ThreeComponents F = MakeThreeComponents();
	std::vector<SolveCall> Solves;
	std::vector<DecideCall> Decides;
	const std::unique_ptr<IStructuralAuthority> Authority = MakeCombinedAuthority(std::make_unique<CountingStanding>(Solves), std::make_unique<CountingCertificate>(Decides));
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
	std::ostringstream DecideLog;
	for (const DecideCall& Call : Decides)
	{
		DecideLog << " {live " << Call.LiveBodies << ", only " << Describe(Call.Only) << "}";
	}
	INFO("Decide calls:" << DecideLog.str());
	CHECK(Decides.size() == 1u);
	if (Decides.size() != 1u)
	{
		return;
	}
	CHECK(Decides[0].LiveBodies == F.Model.LiveBodyCount());
	CHECK(SameIdList(Decides[0].Only, F.Parts[1]));
}

TEST_CASE("Statics.StandingResult.ReportsEveryComponent")
{
	const ThreeComponents F = MakeThreeComponents();
	const std::unique_ptr<IStandingAuthority> Authority = MakeRbsmStandingAuthority();
	const StandingResult Whole = Authority->Solve(F.Model, F.Loads, StandingOptions{}, {});
	INFO("whole state " << Whole.State << ", readings " << Whole.Readings.size());
	CheckEveryComponent(Whole, F, *Authority);
}

TEST_CASE("Certificate.Decide.AComponentFilterReadsAsTheComponentAlone")
{
	const ThreeComponents F = MakeThreeComponents();
	const std::unique_ptr<ICollapseCertificate> Certificate = MakeLimitAnalysisCertificate();
	const CertificateOptions Options;
	const std::optional<CertificateResult> Probe = DecideFiltered(*Certificate, F.Model, F.Loads, Options, {});
	CHECK_MESSAGE(Probe.has_value(), "ICollapseCertificate::Decide takes no component filter (std::span<const BodyId> Only)");
	if (!Probe)
	{
		return;
	}
	CertificateResult Aggregate;
	Aggregate.State = CertificateState::Stands;
	Aggregate.LoadFactorLower = Options.LoadFactorCap;
	Aggregate.LoadFactorUpper = Options.LoadFactorCap;
	for (std::size_t K = 0; K < F.Parts.size(); ++K)
	{
		INFO("component " << K << " bodies " << Describe(F.Parts[K]));
		const Structure Alone = KeepOnly(F.Model, F.Parts[K]);
		const CertificateResult Want = ForwardDecide(*Certificate, Alone, LoadsOn(F.Loads, F.Parts[K]), Options, {});
		const std::optional<CertificateResult> Got = DecideFiltered(*Certificate, F.Model, F.Loads, Options, F.Parts[K]);
		if (!Got)
		{
			return;
		}
		CheckSameCertificate(*Got, Want);
		if (CertificateSeverity(Want.State) > CertificateSeverity(Aggregate.State))
		{
			Aggregate.State = Want.State;
		}
		Aggregate.LoadFactorLower = std::min(Aggregate.LoadFactorLower, Want.LoadFactorLower);
		Aggregate.LoadFactorUpper = std::min(Aggregate.LoadFactorUpper, Want.LoadFactorUpper);
		Aggregate.Iterations = std::max(Aggregate.Iterations, Want.Iterations);
		Aggregate.Mechanism.insert(Aggregate.Mechanism.end(), Want.Mechanism.begin(), Want.Mechanism.end());
		Aggregate.UnconstrainedBodies.insert(Aggregate.UnconstrainedBodies.end(), Want.UnconstrainedBodies.begin(), Want.UnconstrainedBodies.end());
		Aggregate.HingingInterfaces.insert(Aggregate.HingingInterfaces.end(), Want.HingingInterfaces.begin(), Want.HingingInterfaces.end());
	}
	{
		INFO("empty filter against the three components aggregated");
		CheckSameCertificate(*Probe, Aggregate);
	}
	const std::vector<std::pair<const char*, std::vector<BodyId>>> BadFilters{
		{"the ground", {F.Ground}},
		{"an unknown id", {BodyId{999u, 1u}}},
		{"the invalid handle", {BodyId::Invalid()}},
		{"a free body and the ground", {F.Parts[0][0], F.Ground}},
	};
	for (const auto& [Name, Only] : BadFilters)
	{
		const std::optional<CertificateResult> Got = DecideFiltered(*Certificate, F.Model, F.Loads, Options, Only);
		if (!Got)
		{
			return;
		}
		INFO("filter naming " << Name << ": " << DescribeCertificate(*Got));
		CHECK(Got->State == CertificateState::InvalidInput);
	}
}

TEST_CASE("Certificate.Decide.AFilterNamingTwoComponentsReadsAsThoseTwoAlone")
{
	const ThreeComponents F = MakeThreeComponents();
	const std::unique_ptr<ICollapseCertificate> Certificate = MakeLimitAnalysisCertificate();
	const CertificateOptions Options;
	std::vector<BodyId> Both = F.Parts[0];
	Both.insert(Both.end(), F.Parts[1].begin(), F.Parts[1].end());
	const std::vector<BodyId> Only{F.Parts[1][0], F.Parts[0][0], F.Parts[0][0]};
	const CertificateResult StackAlone = ForwardDecide(*Certificate, KeepOnly(F.Model, F.Parts[0]), LoadsOn(F.Loads, F.Parts[0]), Options, {});
	const CertificateResult PushedAlone = ForwardDecide(*Certificate, KeepOnly(F.Model, F.Parts[1]), LoadsOn(F.Loads, F.Parts[1]), Options, {});
	CHECK(StackAlone.State == CertificateState::Stands);
	CHECK(PushedAlone.State == CertificateState::Collapses);
	const CertificateResult Want = ForwardDecide(*Certificate, KeepOnly(F.Model, Both), LoadsOn(F.Loads, Both), Options, {});
	const std::optional<CertificateResult> Got = DecideFiltered(*Certificate, F.Model, F.Loads, Options, Only);
	REQUIRE(Got.has_value());
	CheckSameCertificate(*Got, Want);
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
	for (const double Cap : {0.0, 1.2})
	{
		int ComponentsSeen = 0;
		int Consulted = 0;
		int Mismatches = 0;
		for (int Draw = 0; Draw < JudgeDraws; ++Draw)
		{
			CheckJudgeMatchesEachComponentAlone(JudgeSeedBase + static_cast<std::uint64_t>(Draw), Cap, ComponentsSeen, Consulted, Mismatches);
		}
		MESSAGE("judge per component (cap " << (Cap > 0.0 ? Cap : CertificateOptions{}.LoadFactorCap) << "): " << JudgeDraws << " draws, " << ComponentsSeen << " components, " << Consulted << " consulted the certificate, " << Mismatches << " differ from the component alone");
		CHECK(Consulted >= 1);
	}
}

namespace
{
std::string Hex(double X)
{
	std::ostringstream Out;
	Out << std::hexfloat << X;
	return Out.str();
}

struct CapCase
{
	std::string Name;
	double Cap;
	int MaxIterations;
};

void CheckConsultedBoundsAreTheCappedAggregate(std::uint64_t Seed, const CapCase& Case, int& Consulted, int& Mismatches)
{
	MultiDraw D = MakeMultiDraw(Seed);
	if (Case.Cap > 0.0)
	{
		D.Options.Certificate.LoadFactorCap = Case.Cap;
	}
	if (Case.MaxIterations > 0)
	{
		D.Options.Certificate.MaxIterations = Case.MaxIterations;
	}
	const double Cap = D.Options.Certificate.LoadFactorCap;
	INFO("seed " << Seed << " (" << Case.Name << ", cap " << Cap << "): " << D.Description);
	const AuthorityResult Whole = Judge(D.Model, D.Options, D.Loads);
	const std::unique_ptr<ICollapseCertificate> Certificate = MakeLimitAnalysisCertificate();
	for (std::size_t K = 0; K < Whole.Components.size(); ++K)
	{
		const ComponentReport& Got = Whole.Components[K];
		if (!Got.bConsultedCertificate)
		{
			continue;
		}
		++Consulted;
		const CertificateResult Want = Certificate->Decide(D.Model, D.Loads, D.Options.Certificate, Got.Bodies);
		INFO("component " << K << " bodies " << Describe(Got.Bodies) << ": report t [" << Hex(Got.LoadFactorLower) << ", " << Hex(Got.LoadFactorUpper) << "], filtered aggregate t [" << Hex(Want.LoadFactorLower) << ", " << Hex(Want.LoadFactorUpper) << "], cap " << Hex(Cap));
		const bool bSame = Got.LoadFactorLower == Want.LoadFactorLower && Got.LoadFactorUpper == Want.LoadFactorUpper;
		Mismatches += bSame ? 0 : 1;
		CHECK(Got.LoadFactorLower == Want.LoadFactorLower);
		CHECK(Got.LoadFactorUpper == Want.LoadFactorUpper);
		CHECK(Got.LoadFactorLower <= Cap);
		CHECK(Got.LoadFactorUpper <= Cap);
		for (const auto& Entry : Want.Components)
		{
			INFO("entry " << Describe(Entry.Bodies) << " t [" << Hex(Entry.LoadFactorLower) << ", " << Hex(Entry.LoadFactorUpper) << "]");
			CHECK(Entry.LoadFactorLower <= Cap);
			CHECK(Entry.LoadFactorUpper <= Cap);
		}
	}
}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Authority.Judge.ConsultedBoundsAreTheCappedFilteredAggregate")
{
	const CapCase Cases[] = {{"default cap", 0.0, 0}, {"cap 1.2", 1.2, 0}, {"cap 3.0, 12 iterations", 3.0, 12}};
	for (const CapCase& Case : Cases)
	{
		int Consulted = 0;
		int Mismatches = 0;
		for (int Draw = 0; Draw < JudgeDraws; ++Draw)
		{
			CheckConsultedBoundsAreTheCappedAggregate(JudgeSeedBase + static_cast<std::uint64_t>(Draw), Case, Consulted, Mismatches);
		}
		MESSAGE(Case.Name << ": " << JudgeDraws << " draws, " << Consulted << " consulted reports, " << Mismatches << " differ from the filtered aggregate");
		CHECK(Consulted >= 1);
	}
}
namespace
{
CertificateResult EntryAsResult(const CertificateResult& Whole, std::size_t K)
{
	CertificateResult R;
	R.State = Whole.Components[K].State;
	R.LoadFactorLower = Whole.Components[K].LoadFactorLower;
	R.LoadFactorUpper = Whole.Components[K].LoadFactorUpper;
	R.Iterations = Whole.Components[K].Iterations;
	R.Mechanism = Whole.Components[K].Mechanism;
	R.UnconstrainedBodies = Whole.Components[K].UnconstrainedBodies;
	R.HingingInterfaces = Whole.Components[K].HingingInterfaces;
	return R;
}

struct StackAndPushed
{
	Structure Model;
	std::vector<BodyLoad> Loads;
	std::vector<BodyId> Stack;
	std::vector<BodyId> Pushed;
	BodyId Ground;
};

StackAndPushed MakeStackAndPushed()
{
	const ThreeComponents F = MakeThreeComponents();
	std::vector<BodyId> Both = F.Parts[0];
	Both.insert(Both.end(), F.Parts[1].begin(), F.Parts[1].end());
	return StackAndPushed{KeepOnly(F.Model, Both), LoadsOn(F.Loads, Both), F.Parts[0], F.Parts[1], F.Ground};
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

std::vector<BodyId> SortedUnion(const std::vector<std::vector<BodyId>>& Parts)
{
	std::vector<BodyId> All;
	for (const std::vector<BodyId>& Part : Parts)
	{
		All.insert(All.end(), Part.begin(), Part.end());
	}
	std::sort(All.begin(), All.end(), [](BodyId A, BodyId B) { return A.Index < B.Index; });
	return All;
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
	CHECK(Got.Kinematic == Want.Kinematic);
	CHECK(Got.bConsultedCertificate == Want.bConsultedCertificate);
	CHECK(Near(Got.LoadFactorLower, Want.LoadFactorLower, 1.0e-9));
	CHECK(Near(Got.LoadFactorUpper, Want.LoadFactorUpper, 1.0e-9));
	CHECK(SameMotions(Got.Mechanism, Want.Mechanism));
	CHECK(SameIdList(Got.Release, Want.Release));
	CHECK(SameIdList(Got.Hinging, Want.Hinging));
}
}

TEST_CASE("Certificate.Components.EachEntryIsTheComponentDecidedAlone")
{
	const StackAndPushed F = MakeStackAndPushed();
	const std::unique_ptr<ICollapseCertificate> Certificate = MakeLimitAnalysisCertificate();
	const CertificateOptions Options;
	const CertificateResult Whole = Certificate->Decide(F.Model, F.Loads, Options, {});
	INFO("whole: " << DescribeCertificate(Whole));
	const std::vector<std::vector<BodyId>>& Partition = F.Model.Components().FreeBodies;
	REQUIRE(Partition.size() == 2u);
	REQUIRE(Whole.Components.size() == 2u);
	CertificateResult Aggregate;
	Aggregate.State = CertificateState::Stands;
	Aggregate.LoadFactorLower = Options.LoadFactorCap;
	Aggregate.LoadFactorUpper = Options.LoadFactorCap;
	for (std::size_t K = 0; K < 2u; ++K)
	{
		INFO("component " << K << " bodies " << Describe(Partition[K]));
		CHECK(SameIdList(Whole.Components[K].Bodies, Partition[K]));
		const CertificateResult Want = Certificate->Decide(F.Model, F.Loads, Options, Partition[K]);
		CheckSameCertificate(EntryAsResult(Whole, K), Want);
		if (CertificateSeverity(Want.State) > CertificateSeverity(Aggregate.State))
		{
			Aggregate.State = Want.State;
		}
		Aggregate.LoadFactorLower = std::min(Aggregate.LoadFactorLower, Want.LoadFactorLower);
		Aggregate.LoadFactorUpper = std::min(Aggregate.LoadFactorUpper, Want.LoadFactorUpper);
		Aggregate.Iterations = std::max(Aggregate.Iterations, Want.Iterations);
		Aggregate.Mechanism.insert(Aggregate.Mechanism.end(), Want.Mechanism.begin(), Want.Mechanism.end());
		Aggregate.UnconstrainedBodies.insert(Aggregate.UnconstrainedBodies.end(), Want.UnconstrainedBodies.begin(), Want.UnconstrainedBodies.end());
		Aggregate.HingingInterfaces.insert(Aggregate.HingingInterfaces.end(), Want.HingingInterfaces.begin(), Want.HingingInterfaces.end());
	}
	const std::size_t StackK = SameIdList(Partition[0], F.Stack) ? 0u : 1u;
	const std::size_t PushedK = 1u - StackK;
	CHECK(SameIdList(Partition[PushedK], F.Pushed));
	const auto& Standing = Whole.Components[StackK];
	CHECK(Standing.State == CertificateState::Stands);
	CHECK(Near(Standing.LoadFactorLower, Options.LoadFactorCap, 1.0e-9));
	CHECK(Standing.LoadFactorUpper == Options.LoadFactorCap);
	CHECK(Standing.Mechanism.empty());
	CHECK(Standing.UnconstrainedBodies.empty());
	CHECK(Standing.HingingInterfaces.empty());
	CHECK(Whole.Components[PushedK].State == CertificateState::Collapses);
	CHECK(SameIdList(Whole.Components[PushedK].UnconstrainedBodies, F.Pushed));
	INFO("aggregate of the alone-results");
	CheckSameCertificate(Whole, Aggregate);
}

TEST_CASE("Certificate.Components.AFilterNamingOneComponentGivesOneEntry")
{
	const StackAndPushed F = MakeStackAndPushed();
	const CertificateResult Got = MakeLimitAnalysisCertificate()->Decide(F.Model, F.Loads, CertificateOptions{}, F.Pushed);
	INFO(DescribeCertificate(Got));
	REQUIRE(Got.Components.size() == 1u);
	CHECK(SameIdList(Got.Components[0].Bodies, F.Pushed));
	CheckSameCertificate(EntryAsResult(Got, 0), Got);
}

TEST_CASE("Certificate.Components.InvalidInputHasNoEntries")
{
	StackAndPushed F = MakeStackAndPushed();
	const std::unique_ptr<ICollapseCertificate> Certificate = MakeLimitAnalysisCertificate();
	{
		INFO("filter naming the ground");
		const std::vector<BodyId> Only{F.Ground};
		const CertificateResult Got = Certificate->Decide(F.Model, F.Loads, CertificateOptions{}, Only);
		CHECK(Got.State == CertificateState::InvalidInput);
		CHECK(Got.Components.empty());
	}
	{
		INFO("a load on a never-issued body");
		std::vector<BodyLoad> Loads = F.Loads;
		Loads.push_back(BodyLoad{BodyId{F.Pushed[0].Index, F.Pushed[0].Generation + 1u},Vec3{1.0, 0.0, 0.0}, Vec3{}});
		const CertificateResult Got = Certificate->Decide(F.Model, Loads, CertificateOptions{}, {});
		CHECK(Got.State == CertificateState::InvalidInput);
		CHECK(Got.Components.empty());
	}
}

TEST_CASE("Authority.Combined.ConsultsTheCertificateOncePerJudge")
{
	PairsAndBrick F = MakePairsAndBrick();
	std::vector<SolveCall> Solves;
	std::vector<DecideCall> Decides;
	const std::unique_ptr<IStructuralAuthority> Authority = MakeCombinedAuthority(std::make_unique<CountingStanding>(Solves), std::make_unique<CountingCertificate>(Decides));
	const std::unique_ptr<IStructuralAuthority> Plain = MakeAuthority();
	const AuthorityResult First = Authority->Judge(F.Model, F.Loads, AuthorityOptions{}, nullptr);
	INFO("first: " << DescribeAuthority(First));
	REQUIRE(First.Components.size() == 4u);
	{
		INFO("Decide calls:" << DescribeDecides(Decides));
		CHECK(Decides.size() == 1u);
		if (Decides.size() == 1u)
		{
			CHECK(SameIdList(Decides[0].Only, SortedUnion(F.Pairs)));
		}
	}
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
	CHECK_FALSE(Brick->bConsultedCertificate);

	SUBCASE("a previous verdict with one pair's loads changed decides that pair only")
	{
		std::vector<BodyLoad> Loads = F.Loads;
		const Box Shape = BrickShape(2.0 * BrickLengthM, 2);
		Loads.push_back(BodyLoad{F.Pairs[1][1], Vec3{1.0, 0.0, 0.0}, Shape.CentreM});
		Decides.clear();
		const AuthorityResult Second = Authority->Judge(F.Model, Loads, AuthorityOptions{}, &First);
		INFO("second: " << DescribeAuthority(Second));
		INFO("Decide calls:" << DescribeDecides(Decides));
		CHECK(Decides.size() == 1u);
		if (Decides.size() == 1u)
		{
			CHECK(SameIdList(Decides[0].Only, F.Pairs[1]));
		}
	}
}

namespace
{
class EntrylessCertificate final : public ICollapseCertificate
{
public:
	CertificateResult Decide(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, std::span<const BodyId> Only) const override
	{
		CertificateResult Result = Inner->Decide(Model, Loads, Options, Only);
		Result.Components.clear();
		return Result;
	}

private:
	std::unique_ptr<ICollapseCertificate> Inner = MakeLimitAnalysisCertificate();
};
}

TEST_CASE("Authority.Combined.ACertificateWithNoEntriesReadsInvalidInput")
{
	PairsAndBrick F = MakePairsAndBrick();
	const std::unique_ptr<IStructuralAuthority> Authority = MakeCombinedAuthority(MakeRbsmStandingAuthority(), std::make_unique<EntrylessCertificate>());
	const AuthorityResult Result = Authority->Judge(F.Model, F.Loads, AuthorityOptions{}, nullptr);
	INFO(DescribeAuthority(Result));
	REQUIRE(Result.Components.size() == 4u);
	CHECK(Result.Worst == ComponentVerdict::InvalidInput);
	for (const std::vector<BodyId>& Pair : F.Pairs)
	{
		INFO("pair " << Describe(Pair));
		const ComponentReport* Got = ReportFor(Result, Pair);
		REQUIRE(Got != nullptr);
		CHECK(Got->bConsultedCertificate);
		CHECK(Got->Verdict == ComponentVerdict::InvalidInput);
	}
	const ComponentReport* Brick = ReportFor(Result, F.Brick);
	REQUIRE(Brick != nullptr);
	CHECK(Brick->Verdict == ComponentVerdict::Stands);
	CHECK_FALSE(Brick->bConsultedCertificate);
	const CascadeVerdict V = MakeCascade(MakeCombinedAuthority(MakeRbsmStandingAuthority(), std::make_unique<EntrylessCertificate>()))->Run(F.Model, F.Loads, CascadeOptions{}, nullptr);
	INFO(DescribeVerdict(V));
	CHECK(V.State == CascadeState::InvalidInput);
	CHECK(V.Clusters.empty());
}
