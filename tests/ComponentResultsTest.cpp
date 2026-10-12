#include "BrickFixtures.h"
#include "CascadeTestSupport.h"
#include "CatalogueTestSupport.h"
#include "CertificateTestSupport.h"
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

struct SolveCall
{
	std::size_t LiveBodies = 0;
};

struct DecideCall
{
	std::size_t LiveBodies = 0;
	std::vector<BodyId> Only;
};

class CountingStanding final : public IStandingAuthority
{
public:
	explicit CountingStanding(std::vector<SolveCall>& InCalls)
		: Calls(InCalls)
		, Inner(MakeRbsmStandingAuthority())
	{
	}

	StandingResult Solve(const Structure& Model, std::span<const BodyLoad> Loads, const StandingOptions& Options) const override
	{
		Calls.push_back(SolveCall{Model.LiveBodyCount()});
		return Inner->Solve(Model, Loads, Options);
	}

private:
	std::vector<SolveCall>& Calls;
	std::unique_ptr<IStandingAuthority> Inner;
};

class CountingCertificate final : public ICollapseCertificate
{
public:
	explicit CountingCertificate(std::vector<DecideCall>& InCalls)
		: Calls(InCalls)
		, Inner(MakeLimitAnalysisCertificate())
	{
	}

	CertificateResult Decide(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options) const
	{
		Calls.push_back(DecideCall{Model.LiveBodyCount(), {}});
		return ForwardDecide(*Inner, Model, Loads, Options, {});
	}

	CertificateResult Decide(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, std::span<const BodyId> Only) const
	{
		Calls.push_back(DecideCall{Model.LiveBodyCount(), std::vector<BodyId>(Only.begin(), Only.end())});
		return ForwardDecide(*Inner, Model, Loads, Options, Only);
	}

private:
	std::vector<DecideCall>& Calls;
	std::unique_ptr<ICollapseCertificate> Inner;
};

constexpr double ThreeComponentFriction = 0.7;
constexpr double PushOverWeight = 1.0;

struct ThreeComponents
{
	Structure Model;
	BodyId Ground;
	std::vector<std::vector<BodyId>> Parts;
	std::vector<BodyLoad> Loads;
};

std::vector<BodyId> AddDryStack(Structure& Model, BodyId Ground, ProfileId Profile, double XM)
{
	const Box LowerShape = BrickShape(XM, 0);
	const Box UpperShape = BrickShape(XM, 1);
	const BodyId Lower = AddBox(Model, LowerShape);
	const BodyId Upper = AddBox(Model, UpperShape);
	AddInterfaceOk(Model, OnGroundPlane(Ground, Lower, LowerShape, Profile));
	AddInterfaceOk(Model, *StackedFaces(Lower, LowerShape, Upper, UpperShape, Profile));
	return {Lower, Upper};
}

ThreeComponents MakeThreeComponents()
{
	ThreeComponents F;
	F.Ground = AddGroundPlane(F.Model);
	const ProfileId Dry = AddProfileOk(F.Model, DryJoint(ThreeComponentFriction));
	F.Parts.push_back(AddDryStack(F.Model, F.Ground, Dry, 0.0));
	const Box PushedShape = BrickShape(1.0);
	const BodyId Pushed = AddBox(F.Model, PushedShape);
	AddInterfaceOk(F.Model, OnGroundPlane(F.Ground, Pushed, PushedShape, Dry));
	F.Parts.push_back({Pushed});
	F.Loads.push_back(BodyLoad{Pushed, Vec3{PushOverWeight * BrickMassKg * GravityMPerS2, 0.0, 0.0}, PushedShape.CentreM});
	F.Parts.push_back(AddDryStack(F.Model, F.Ground, Dry, 2.0));
	return F;
}

bool Holds(const std::vector<BodyId>& Bodies, BodyId Body)
{
	return std::any_of(Bodies.begin(), Bodies.end(), [&](BodyId B) { return SameHandle(B, Body); });
}

Structure KeepOnly(const Structure& Model, const std::vector<BodyId>& Keep)
{
	Structure Sub = Model;
	for (const BodyId Body : Model.LiveBodies())
	{
		if (!Model.Body(Body).bGrounded && !Holds(Keep, Body))
		{
			Sub.RemoveBody(Body);
		}
	}
	return Sub;
}

std::vector<BodyLoad> LoadsOn(std::span<const BodyLoad> Loads, const std::vector<BodyId>& Keep)
{
	std::vector<BodyLoad> Kept;
	for (const BodyLoad& Load : Loads)
	{
		if (Holds(Keep, Load.Body))
		{
			Kept.push_back(Load);
		}
	}
	return Kept;
}

bool TouchesFreeBody(const Structure& Model, InterfaceId Id)
{
	const InterfaceRecord& Record = Model.Interface(Id);
	const auto Free = [&](BodyId B) { return Model.IsLive(B) && !Model.Body(B).bGrounded; };
	return Free(Record.BodyA) || Free(Record.BodyB);
}

bool Near(double Got, double Want, double Relative, double Absolute = 0.0)
{
	if (std::isnan(Got) || std::isnan(Want))
	{
		return std::isnan(Got) && std::isnan(Want);
	}
	if (Got == Want)
	{
		return true;
	}
	return std::abs(Got - Want) <= Absolute + Relative * std::max(std::abs(Got), std::abs(Want));
}

bool NearVec(const Vec3& Got, const Vec3& Want, double Relative, double Absolute = 0.0)
{
	return Near(Got.X, Want.X, Relative, Absolute) && Near(Got.Y, Want.Y, Relative, Absolute) && Near(Got.Z, Want.Z, Relative, Absolute);
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
			const StandingResult Reference = Authority.Solve(Alone, AloneLoads, StandingOptions{});
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

class SplitMix
{
public:
	explicit SplitMix(std::uint64_t Seed) : State(Seed) {}

	std::uint64_t Next()
	{
		std::uint64_t Z = (State += 0x9e3779b97f4a7c15ull);
		Z = (Z ^ (Z >> 30)) * 0xbf58476d1ce4e5b9ull;
		Z = (Z ^ (Z >> 27)) * 0x94d049bb133111ebull;
		return Z ^ (Z >> 31);
	}

	double Uniform(double Low, double High) { return Low + (High - Low) * (static_cast<double>(Next() >> 11) * 0x1.0p-53); }

	int Between(int Low, int High) { return Low + static_cast<int>(Next() % static_cast<std::uint64_t>(High - Low + 1)); }

private:
	std::uint64_t State;
};

const JointRow& CatalogueJoint(const char* Name)
{
	static const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
	REQUIRE(Loaded.Ok());
	return JointOrFail(Loaded.Value, Name);
}

struct MultiDraw
{
	Structure Model;
	AuthorityOptions Options;
	std::vector<BodyLoad> Loads;
	std::string Description;
};

MultiDraw MakeMultiDraw(std::uint64_t Seed)
{
	MultiDraw D;
	SplitMix Rng(Seed);
	std::ostringstream Out;
	const BodyId Ground = AddGroundPlane(D.Model);
	const int Parts = Rng.Between(2, 5);
	Out << Parts << " parts:";
	for (int Part = 0; Part < Parts; ++Part)
	{
		const double OriginXM = 3.0 * Part;
		const int Family = Rng.Between(0, 2);
		const double Friction = Rng.Uniform(0.4, 0.9);
		const double Weakening = std::pow(10.0, Rng.Uniform(-4.0, -2.0));
		const double Crushing = std::pow(10.0, Rng.Uniform(-4.0, -2.5));
		StrengthProfile Bed = DryJoint(Friction);
		StrengthProfile Perpend = DryJoint(Friction);
		if (Family > 0)
		{
			Bed = CatalogueJoint("clay_gp_mortar_bed_ec6").Profile;
			Perpend = CatalogueJoint("clay_gp_mortar_perpend_ec6").Profile;
			Out << " [bonded ec6";
		}
		else
		{
			Out << " [dry mu " << Friction;
		}
		if (Family == 2)
		{
			for (StrengthProfile* Profile : {&Bed, &Perpend})
			{
				Profile->TensileStrengthPa *= Weakening;
				Profile->ShearCohesionPa *= Weakening;
				Profile->CompressiveStrengthPa *= Crushing;
				Profile->MaxShearStrengthPa *= Weakening;
			}
			Out << " weakened x" << Weakening << ", crushing x" << Crushing;
		}
		if (Family == 2 || Rng.Between(0, 1) == 0)
		{
			const int Count = Rng.Between(2, 5);
			Out << ", stack of " << Count << " offsets";
			const ProfileId P = AddProfileOk(D.Model, Bed);
			double X = OriginXM;
			BodyId Lower{};
			Box LowerShape{};
			for (int K = 0; K < Count; ++K)
			{
				if (K > 0)
				{
					const double Shift = Rng.Uniform(-0.45, 0.45) * BrickLengthM;
					X += Shift;
					Out << " " << Shift;
				}
				const Box Shape = BrickShape(X, K);
				const BodyId Brick = AddBox(D.Model, Shape);
				if (K == 0)
				{
					AddInterfaceOk(D.Model, OnGroundPlane(Ground, Brick, Shape, P));
				}
				else
				{
					AddInterfaceOk(D.Model, *StackedFaces(Lower, LowerShape, Brick, Shape, P));
				}
				Lower = Brick;
				LowerShape = Shape;
			}
		}
		else
		{
			geometry::WallSpec Spec;
			Spec.Courses = Rng.Between(3, 6);
			Spec.BricksPerCourse = Rng.Between(3, 5);
			Spec.Bed = AddProfileOk(D.Model, Bed);
			Spec.Perpend = AddProfileOk(D.Model, Perpend);
			Spec.OriginM = Vec3{OriginXM, 0.0, 0.0};
			Spec.Ground = Ground;
			const geometry::Wall W = geometry::BuildRunningBondWall(D.Model, Spec);
			CHECK(W.bComplete);
			Out << ", wall " << Spec.Courses << " x " << Spec.BricksPerCourse << " removed";
			const int Removals = Rng.Between(0, 3);
			for (int R = 0; R < Removals; ++R)
			{
				const std::vector<BodyId>& Course = W.Courses[static_cast<std::size_t>(Rng.Between(0, Spec.Courses - 1))];
				const BodyId Victim = Course[static_cast<std::size_t>(Rng.Between(0, static_cast<int>(Course.size()) - 1))];
				if (D.Model.RemoveBody(Victim))
				{
					Out << " " << Victim.Index;
				}
			}
		}
		Out << "]";
	}
	const bool bTilt = Rng.Between(0, 1) == 0;
	const bool bAlongY = Rng.Between(0, 1) == 1;
	Vec3 Gravity{0.0, 0.0, -GravityMPerS2};
	double Horizontal = 0.0;
	if (bTilt)
	{
		const double TiltDeg = Rng.Uniform(0.0, 30.0);
		Out << ", tilt " << TiltDeg << " deg";
		Horizontal = GravityMPerS2 * std::sin(TiltDeg * RadPerDegree);
		Gravity.Z = -GravityMPerS2 * std::cos(TiltDeg * RadPerDegree);
	}
	else
	{
		const double Lambda = Rng.Uniform(0.0, 1.0);
		Out << ", lateral " << Lambda;
		Horizontal = Lambda * GravityMPerS2;
	}
	Out << (bAlongY ? " along y" : " along x");
	(bAlongY ? Gravity.Y : Gravity.X) = Horizontal;
	D.Options = AuthorityUnderGravity(Gravity);
	if (Rng.Between(0, 5) == 0)
	{
		const BodyId Loose = AddBox(D.Model, BrickShape(0.0, 40));
		Out << ", loose brick " << Loose.Index;
	}
	if (Rng.Between(0, 2) == 0)
	{
		std::vector<BodyId> Free;
		for (const BodyId Body : D.Model.LiveBodies())
		{
			if (!D.Model.Body(Body).bGrounded)
			{
				Free.push_back(Body);
			}
		}
		const BodyId Pushed = Free[static_cast<std::size_t>(Rng.Between(0, static_cast<int>(Free.size()) - 1))];
		const double PushN = Rng.Uniform(0.0, 1.0) * D.Model.Body(Pushed).MassKg * GravityMPerS2;
		D.Loads.push_back(BodyLoad{Pushed, Vec3{PushN, 0.0, 0.0}, D.Model.Body(Pushed).CentroidM});
		Out << ", push " << PushN << " N on " << Pushed.Index;
	}
	D.Description = Out.str();
	return D;
}

bool SameReading(const InterfaceReading& Got, const InterfaceReading& Want)
{
	constexpr double Rel = 1.0e-9;
	constexpr double AbsN = 1.0e-9;
	if (Got.Axis != Want.Axis || !Near(Got.Utilisation, Want.Utilisation, Rel, AbsN) || !Near(Got.CompressionUtilisation, Want.CompressionUtilisation, Rel, AbsN) || !Near(Got.TensionUtilisation, Want.TensionUtilisation, Rel, AbsN) || !Near(Got.ShearUtilisation, Want.ShearUtilisation, Rel, AbsN))
	{
		return false;
	}
	if (!Near(Got.NormalForceN, Want.NormalForceN, Rel, AbsN) || !NearVec(Got.ShearForceN, Want.ShearForceN, Rel, AbsN) || !NearVec(Got.MomentNm, Want.MomentNm, Rel, AbsN))
	{
		return false;
	}
	for (std::size_t C = 0; C < Got.Corners.size(); ++C)
	{
		const CornerReading& G = Got.Corners[C];
		const CornerReading& W = Want.Corners[C];
		if (G.bOpen != W.bOpen || G.bSlipping != W.bSlipping || !Near(G.NormalForceN, W.NormalForceN, Rel, AbsN) || !NearVec(G.ShearForceN, W.ShearForceN, Rel, AbsN))
		{
			return false;
		}
	}
	return true;
}

const InterfaceReading* FindReading(const std::vector<InterfaceReading>& Readings, InterfaceId Id)
{
	for (const InterfaceReading& R : Readings)
	{
		if (SameHandle(R.Id, Id))
		{
			return &R;
		}
	}
	return nullptr;
}

void CheckJudgeMatchesEachComponentAlone(std::uint64_t Seed, int& ComponentsSeen, int& Consulted, int& Mismatches)
{
	const MultiDraw D = MakeMultiDraw(Seed);
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
		const bool bSame = SameIdList(Got.Bodies, Want.Bodies) && Got.Verdict == Want.Verdict && Got.Elastic == Want.Elastic && Got.Kinematic == Want.Kinematic && Got.bConsultedCertificate == Want.bConsultedCertificate && Near(Got.LoadFactorLower, Want.LoadFactorLower, 1.0e-9) && Near(Got.LoadFactorUpper, Want.LoadFactorUpper, 1.0e-9) && Mismatched == 0;
		Mismatches += bSame ? 0 : 1;
		CHECK(SameIdList(Got.Bodies, Want.Bodies));
		CHECK(Got.Verdict == Want.Verdict);
		CHECK(Got.Elastic == Want.Elastic);
		CHECK(Got.Kinematic == Want.Kinematic);
		CHECK(Got.bConsultedCertificate == Want.bConsultedCertificate);
		CHECK(Near(Got.LoadFactorLower, Want.LoadFactorLower, 1.0e-9));
		CHECK(Near(Got.LoadFactorUpper, Want.LoadFactorUpper, 1.0e-9));
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
	const AuthorityResult Result = Authority->Judge(F.Model, F.Loads, AuthorityOptions{});
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
	const StandingResult Whole = Authority->Solve(F.Model, F.Loads, StandingOptions{});
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
	CHECK(Model.RemoveBody(Row[1]));
	CHECK(PartitionOf(Model)->second == 3u);
	CHECK(Model.Sever(PairJoint, 1));
	CHECK(PartitionOf(Model)->second == 4u);
	AddInterfaceOk(Model, PairHead);
	CHECK(PartitionOf(Model)->second == 3u);
	CHECK(Model.Crack(FirstBed, 2));
	CHECK(PartitionOf(Model)->second == 3u);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Authority.Judge.MatchesSolvingEachComponentAlone")
{
	int ComponentsSeen = 0;
	int Consulted = 0;
	int Mismatches = 0;
	for (int Draw = 0; Draw < JudgeDraws; ++Draw)
	{
		CheckJudgeMatchesEachComponentAlone(JudgeSeedBase + static_cast<std::uint64_t>(Draw), ComponentsSeen, Consulted, Mismatches);
	}
	MESSAGE("judge per component: " << JudgeDraws << " draws, " << ComponentsSeen << " components, " << Consulted << " consulted the certificate, " << Mismatches << " differ from the component alone");
	CHECK(Consulted >= 1);
}
