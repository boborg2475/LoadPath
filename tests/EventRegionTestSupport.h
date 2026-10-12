#pragma once

#include "BrickFixtures.h"
#include "CascadeTestSupport.h"
#include "CatalogueTestSupport.h"
#include "StaticsTestSupport.h"

#include "loadpath/Catalogue.h"
#include "loadpath/CollapseCertificate.h"
#include "loadpath/StandingAuthority.h"
#include "loadpath/StructuralAuthority.h"
#include "loadpath/Structure.h"
#include "loadpath/geometry/RunningBondWall.h"

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace loadpath::test
{
template <class S>
constexpr bool HasRevision = requires(const S& Model, BodyId Body) {
	{ Model.Revision() } -> std::same_as<std::uint64_t>;
	{ Model.LastTouched(Body) } -> std::same_as<std::uint64_t>;
};

template <class A>
constexpr bool HasSolveFilter = requires(const A& X, const Structure& Model, std::span<const BodyLoad> Loads, const StandingOptions& Options, std::span<const BodyId> Only) { X.Solve(Model, Loads, Options, Only); };

template <class A>
constexpr bool HasJudgePrevious = requires(const A& X, const Structure& Model, std::span<const BodyLoad> Loads, const AuthorityOptions& Options, const AuthorityResult* Previous) { X.Judge(Model, Loads, Options, Previous); };

template <class R>
constexpr bool HasReuseFlag = requires(const R& Report) { { Report.bReused } -> std::convertible_to<bool>; };

template <class R>
constexpr bool HasResultRevision = requires(R& Result) { Result.Revision = std::uint64_t{0}; };

template <class S>
std::optional<std::uint64_t> RevisionOf(const S& Model)
{
	if constexpr (HasRevision<S>)
	{
		return Model.Revision();
	}
	else
	{
		(void)Model;
		return std::nullopt;
	}
}

template <class S>
std::optional<std::uint64_t> LastTouchedOf(const S& Model, BodyId Body)
{
	if constexpr (HasRevision<S>)
	{
		return Model.LastTouched(Body);
	}
	else
	{
		(void)Model;
		(void)Body;
		return std::nullopt;
	}
}

template <class R>
std::optional<bool> ReusedOf(const R& Report)
{
	if constexpr (HasReuseFlag<R>)
	{
		return static_cast<bool>(Report.bReused);
	}
	else
	{
		(void)Report;
		return std::nullopt;
	}
}

template <class R>
bool SetResultRevision(R& Result, std::uint64_t Revision)
{
	if constexpr (HasResultRevision<R>)
	{
		Result.Revision = Revision;
		return true;
	}
	else
	{
		(void)Result;
		(void)Revision;
		return false;
	}
}

template <class A>
std::optional<AuthorityResult> JudgeAfter(const A& Authority, const Structure& Model, std::span<const BodyLoad> Loads, const AuthorityOptions& Options, const AuthorityResult* Previous)
{
	if constexpr (HasJudgePrevious<A>)
	{
		return Authority.Judge(Model, Loads, Options, Previous);
	}
	else
	{
		(void)Authority;
		(void)Model;
		(void)Loads;
		(void)Options;
		(void)Previous;
		return std::nullopt;
	}
}

template <class A>
AuthorityResult JudgeFresh(const A& Authority, const Structure& Model, std::span<const BodyLoad> Loads, const AuthorityOptions& Options)
{
	if constexpr (HasJudgePrevious<A>)
	{
		return Authority.Judge(Model, Loads, Options, nullptr);
	}
	else
	{
		return Authority.Judge(Model, Loads, Options);
	}
}

template <class A>
StandingResult ForwardSolve(const A& Inner, const Structure& Model, std::span<const BodyLoad> Loads, const StandingOptions& Options, std::span<const BodyId> Only)
{
	if constexpr (HasSolveFilter<A>)
	{
		return Inner.Solve(Model, Loads, Options, Only);
	}
	else
	{
		(void)Only;
		return Inner.Solve(Model, Loads, Options);
	}
}

struct SolveCall
{
	std::size_t LiveBodies = 0;
	std::vector<BodyId> Only;
};

struct DecideCall
{
	std::size_t LiveBodies = 0;
	std::vector<BodyId> Only;
};

// Both Solve signatures are declared so the decorator compiles before and after Solve takes a component filter.
class CountingStanding final : public IStandingAuthority
{
public:
	explicit CountingStanding(std::vector<SolveCall>& InCalls)
		: Calls(InCalls)
		, Inner(MakeRbsmStandingAuthority())
	{
	}

	StandingResult Solve(const Structure& Model, std::span<const BodyLoad> Loads, const StandingOptions& Options) const
	{
		Calls.push_back(SolveCall{Model.LiveBodyCount(), {}});
		return ForwardSolve(*Inner, Model, Loads, Options, {});
	}

	StandingResult Solve(const Structure& Model, std::span<const BodyLoad> Loads, const StandingOptions& Options, std::span<const BodyId> Only) const
	{
		Calls.push_back(SolveCall{Model.LiveBodyCount(), std::vector<BodyId>(Only.begin(), Only.end())});
		return ForwardSolve(*Inner, Model, Loads, Options, Only);
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

	CertificateResult Decide(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, std::span<const BodyId> Only) const override
	{
		Calls.push_back(DecideCall{Model.LiveBodyCount(), std::vector<BodyId>(Only.begin(), Only.end())});
		return Inner->Decide(Model, Loads, Options, Only);
	}

private:
	std::vector<DecideCall>& Calls;
	std::unique_ptr<ICollapseCertificate> Inner;
};

inline std::string DescribeSolves(const std::vector<SolveCall>& Calls)
{
	std::ostringstream Out;
	for (const SolveCall& Call : Calls)
	{
		Out << " {live " << Call.LiveBodies << ", only " << Describe(Call.Only) << "}";
	}
	return Out.str();
}

inline std::string DescribeDecides(const std::vector<DecideCall>& Calls)
{
	std::ostringstream Out;
	for (const DecideCall& Call : Calls)
	{
		Out << " {live " << Call.LiveBodies << ", only " << Describe(Call.Only) << "}";
	}
	return Out.str();
}

inline bool Near(double Got, double Want, double Relative, double Absolute = 0.0)
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

inline bool NearVec(const Vec3& Got, const Vec3& Want, double Relative, double Absolute = 0.0)
{
	return Near(Got.X, Want.X, Relative, Absolute) && Near(Got.Y, Want.Y, Relative, Absolute) && Near(Got.Z, Want.Z, Relative, Absolute);
}

inline bool SameReading(const InterfaceReading& Got, const InterfaceReading& Want, double Rel = 1.0e-9, double AbsN = 1.0e-9)
{
	if (!SameHandle(Got.Id, Want.Id) || Got.Axis != Want.Axis || !Near(Got.Utilisation, Want.Utilisation, Rel, AbsN) || !Near(Got.CompressionUtilisation, Want.CompressionUtilisation, Rel, AbsN) || !Near(Got.TensionUtilisation, Want.TensionUtilisation, Rel, AbsN) || !Near(Got.ShearUtilisation, Want.ShearUtilisation, Rel, AbsN))
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

inline const InterfaceReading* FindReading(const std::vector<InterfaceReading>& Readings, InterfaceId Id)
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

inline const JointRow& CatalogueJoint(const char* Name)
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

// Two to five separate parts (dry or bonded stacks, running-bond walls with removals) under one tilt or lateral load.
inline MultiDraw MakeMultiDraw(std::uint64_t Seed)
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
inline constexpr double ThreeComponentFriction = 0.7;
inline constexpr double PushOverWeight = 1.0;

struct ThreeComponents
{
	Structure Model;
	BodyId Ground;
	std::vector<std::vector<BodyId>> Parts;
	std::vector<BodyLoad> Loads;
};

inline std::vector<BodyId> AddDryStack(Structure& Model, BodyId Ground, ProfileId Profile, double XM)
{
	const Box LowerShape = BrickShape(XM, 0);
	const Box UpperShape = BrickShape(XM, 1);
	const BodyId Lower = AddBox(Model, LowerShape);
	const BodyId Upper = AddBox(Model, UpperShape);
	AddInterfaceOk(Model, OnGroundPlane(Ground, Lower, LowerShape, Profile));
	AddInterfaceOk(Model, *StackedFaces(Lower, LowerShape, Upper, UpperShape, Profile));
	return {Lower, Upper};
}

inline ThreeComponents MakeThreeComponents()
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

inline bool Holds(const std::vector<BodyId>& Bodies, BodyId Body)
{
	return std::any_of(Bodies.begin(), Bodies.end(), [&](BodyId B) { return SameHandle(B, Body); });
}

inline Structure KeepOnly(const Structure& Model, const std::vector<BodyId>& Keep)
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

inline std::vector<BodyLoad> LoadsOn(std::span<const BodyLoad> Loads, const std::vector<BodyId>& Keep)
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

inline bool TouchesFreeBody(const Structure& Model, InterfaceId Id)
{
	const InterfaceRecord& Record = Model.Interface(Id);
	const auto Free = [&](BodyId B) { return Model.IsLive(B) && !Model.Body(B).bGrounded; };
	return Free(Record.BodyA) || Free(Record.BodyB);
}
}
