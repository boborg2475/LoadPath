#include "loadpath/Cascade.h"

#include "loadpath/Settings.h"
#include "loadpath/Units.h"
#include "StaticsCommon.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <utility>

namespace loadpath
{
namespace
{
constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
constexpr double Infinity = std::numeric_limits<double>::infinity();

bool AreOptionsValid(const CascadeOptions& Options)
{
	return IsFinite(Options.CrackBandFraction) && Options.CrackBandFraction >= 0.0 && Options.CrackBandFraction <= 1.0 &&
		Options.MaxPasses >= 1 &&
		(Options.OnUndecided == UndecidedResolution::Collapse || Options.OnUndecided == UndecidedResolution::Stands);
}

bool Contains(const std::vector<BodyId>& Bodies, BodyId Id)
{
	return std::find(Bodies.begin(), Bodies.end(), Id) != Bodies.end();
}

bool IsLowerSlot(BodyId A, BodyId B)
{
	return A.Index < B.Index;
}

CascadeState FinalState(ComponentVerdict Worst)
{
	switch (Worst)
	{
	case ComponentVerdict::Stands: return CascadeState::Stands;
	case ComponentVerdict::Overstressed: return CascadeState::Overstressed;
	case ComponentVerdict::InvalidInput: return CascadeState::InvalidInput;
	case ComponentVerdict::Collapses:
	case ComponentVerdict::Undecided: return CascadeState::Undecided;
	}
	return CascadeState::Undecided;
}

constexpr std::size_t NoSlot = std::numeric_limits<std::size_t>::max();

template <typename IdType>
std::vector<std::size_t> PositionTable(const std::vector<IdType>& Ids)
{
	std::size_t Size = 0;
	for (const IdType Id : Ids)
	{
		Size = std::max(Size, static_cast<std::size_t>(Id.Index) + 1);
	}
	std::vector<std::size_t> Table(Size, NoSlot);
	for (std::size_t I = 0; I < Ids.size(); ++I)
	{
		if (Table[Ids[I].Index] == NoSlot)
		{
			Table[Ids[I].Index] = I;
		}
	}
	return Table;
}

template <typename IdType>
std::size_t Lookup(const std::vector<std::size_t>& Table, IdType Id)
{
	return Id.Index < Table.size() ? Table[Id.Index] : NoSlot;
}

struct PendingCluster
{
	std::vector<BodyId> Bodies;
	const ComponentReport* Component = nullptr;
};

template <typename IdType>
void MarkPositions(std::vector<std::size_t>& Table, const std::vector<IdType>& Ids)
{
	for (std::size_t I = 0; I < Ids.size(); ++I)
	{
		if (Ids[I].Index >= Table.size())
		{
			Table.resize(static_cast<std::size_t>(Ids[I].Index) + 1, NoSlot);
		}
		if (Table[Ids[I].Index] == NoSlot)
		{
			Table[Ids[I].Index] = I;
		}
	}
}

template <typename IdType>
void ClearPositions(std::vector<std::size_t>& Table, const std::vector<IdType>& Ids)
{
	for (const IdType Id : Ids)
	{
		Table[Id.Index] = NoSlot;
	}
}

std::vector<PendingCluster> ClustersOf(const Structure& Model, const ComponentReport& Component, const std::vector<std::size_t>& PositionOf)
{
	const std::vector<BodyId>& Released = Component.Release;
	std::vector<std::size_t> Parent(Released.size());
	std::iota(Parent.begin(), Parent.end(), std::size_t{0});
	const auto Root = [&Parent](std::size_t I)
	{
		while (Parent[I] != I)
		{
			Parent[I] = Parent[Parent[I]];
			I = Parent[I];
		}
		return I;
	};
	for (const BodyId Body : Released)
	{
		for (const InterfaceId Id : Model.InterfacesOf(Body))
		{
			const InterfaceRecord& Record = Model.Interface(Id);
			const std::size_t A = Lookup(PositionOf, Record.BodyA);
			const std::size_t B = Lookup(PositionOf, Record.BodyB);
			if (A == NoSlot || B == NoSlot)
			{
				continue;
			}
			const std::size_t RootA = Root(A);
			const std::size_t RootB = Root(B);
			Parent[std::max(RootA, RootB)] = std::min(RootA, RootB);
		}
	}
	std::vector<PendingCluster> Clusters;
	std::vector<std::size_t> ClusterOfRoot(Released.size(), Released.size());
	for (std::size_t I = 0; I < Released.size(); ++I)
	{
		const std::size_t R = Root(I);
		if (ClusterOfRoot[R] == Released.size())
		{
			ClusterOfRoot[R] = Clusters.size();
			Clusters.push_back(PendingCluster{{}, &Component});
		}
		Clusters[ClusterOfRoot[R]].Bodies.push_back(Released[I]);
	}
	for (PendingCluster& Cluster : Clusters)
	{
		std::sort(Cluster.Bodies.begin(), Cluster.Bodies.end(), IsLowerSlot);
	}
	return Clusters;
}

MechanismMotion MotionOf(const ComponentReport& Component, BodyId Body)
{
	for (const MechanismMotion& Motion : Component.Mechanism)
	{
		if (Motion.Body == Body)
		{
			return Motion;
		}
	}
	return MechanismMotion{Body, Vec3{NaN, NaN, NaN}, Vec3{NaN, NaN, NaN}};
}

MechanismMotion StillUnlessMoving(const ComponentReport& Component, BodyId Body)
{
	for (const MechanismMotion& Motion : Component.Mechanism)
	{
		if (Motion.Body == Body)
		{
			return Motion;
		}
	}
	return MechanismMotion{Body, Vec3{}, Vec3{}};
}

Vec3 PointVelocity(const MechanismMotion& Motion, const Vec3& CentroidM, const Vec3& PointM)
{
	const Vec3 Arm{PointM.X - CentroidM.X, PointM.Y - CentroidM.Y, PointM.Z - CentroidM.Z};
	const Vec3 Spin = Cross(Motion.RotationRad, Arm);
	return Vec3{Motion.TranslationM.X + Spin.X, Motion.TranslationM.Y + Spin.Y, Motion.TranslationM.Z + Spin.Z};
}

GoverningAxis MechanismAxis(const Structure& Model, const ComponentReport& Component, InterfaceId Id, GoverningAxis Fallback, double ThresholdRelative)
{
	const InterfaceRecord& Record = Model.Interface(Id);
	const MechanismMotion MotionA = StillUnlessMoving(Component, Record.BodyA);
	const MechanismMotion MotionB = StillUnlessMoving(Component, Record.BodyB);
	const Vec3 CentroidA = Model.Body(Record.BodyA).CentroidM;
	const Vec3 CentroidB = Model.Body(Record.BodyB).CentroidM;
	const StrengthProfile Profile = Model.EffectiveProfile(Id);
	const double Lift = Profile.MaxShearStrengthPa * (Record.AreaM2 / 4.0) > 0.0 ? Profile.FrictionCoefficient : 0.0;
	double Fastest = 0.0;
	double Opening = -Infinity;
	double Closing = Infinity;
	double Sliding = 0.0;
	for (const Vec3& Corner : statics::CornerPoints(Record))
	{
		const Vec3 VelocityA = PointVelocity(MotionA, CentroidA, Corner);
		const Vec3 VelocityB = PointVelocity(MotionB, CentroidB, Corner);
		const Vec3 Relative{VelocityB.X - VelocityA.X, VelocityB.Y - VelocityA.Y, VelocityB.Z - VelocityA.Z};
		const double Normal = Dot(Relative, Record.NormalAToB);
		const Vec3 Tangential{Relative.X - Normal * Record.NormalAToB.X, Relative.Y - Normal * Record.NormalAToB.Y, Relative.Z - Normal * Record.NormalAToB.Z};
		Fastest = std::max(Fastest, Length(Relative));
		Opening = std::max(Opening, Normal - Lift * Length(Tangential));
		Closing = std::min(Closing, Normal);
		Sliding = std::max(Sliding, Length(Tangential));
	}
	const double Threshold = ThresholdRelative * Fastest;
	if (!(Fastest > 0.0))
	{
		return Fallback;
	}
	if (Opening > Threshold)
	{
		return GoverningAxis::Tension;
	}
	if (Closing < -Threshold)
	{
		return GoverningAxis::Compression;
	}
	if (Sliding > Threshold)
	{
		return GoverningAxis::Shear;
	}
	return Fallback;
}

int Severity(ComponentVerdict Verdict)
{
	switch (Verdict)
	{
	case ComponentVerdict::Stands: return 0;
	case ComponentVerdict::Overstressed: return 1;
	case ComponentVerdict::Collapses: return 2;
	case ComponentVerdict::Undecided: return 3;
	case ComponentVerdict::InvalidInput: return 4;
	}
	return 4;
}

SolveEnd ForcedReasonOf(SolveEnd End)
{
	return End == SolveEnd::IterationCap || End == SolveEnd::RestoreFailedClosed || End == SolveEnd::PowerCheckFailed ? End : SolveEnd::None;
}

void ResolveUndecided(AuthorityResult& Judged, UndecidedResolution Resolution, std::array<std::uint32_t, SolveEndCount>& Tally)
{
	Judged.Worst = ComponentVerdict::Stands;
	for (ComponentReport& Component : Judged.Components)
	{
		if (Component.Verdict == ComponentVerdict::Undecided)
		{
			Component.Forced = true;
			Component.ForcedReason = ForcedReasonOf(Component.End);
			Component.Mechanism.clear();
			Component.Release.clear();
			if (Resolution == UndecidedResolution::Collapse)
			{
				Component.Verdict = ComponentVerdict::Collapses;
				Component.Release = Component.Bodies;
			}
			else
			{
				Component.Verdict = ComponentVerdict::Stands;
			}
			if (!Component.bReused)
			{
				++Tally[static_cast<std::size_t>(Component.ForcedReason)];
			}
		}
		if (Severity(Component.Verdict) > Severity(Judged.Worst))
		{
			Judged.Worst = Component.Verdict;
		}
	}
}

struct CrackCandidate
{
	InterfaceId Id;
	double Ratio = 0.0;
	GoverningAxis Axis = GoverningAxis::None;
};

bool Release(Structure& Model, const AuthorityResult& Judged, std::uint32_t Pass, const CascadeOptions& Options, CascadeVerdict& Verdict,
	std::vector<BodyId>& ReleasedThisRun)
{
	std::vector<std::size_t> PositionOf;
	std::vector<PendingCluster> Pending;
	for (const ComponentReport& Component : Judged.Components)
	{
		if (Component.Verdict != ComponentVerdict::Collapses)
		{
			continue;
		}
		MarkPositions(PositionOf, Component.Release);
		for (PendingCluster& Cluster : ClustersOf(Model, Component, PositionOf))
		{
			Pending.push_back(std::move(Cluster));
		}
		ClearPositions(PositionOf, Component.Release);
	}
	std::stable_sort(Pending.begin(), Pending.end(),
		[](const PendingCluster& A, const PendingCluster& B) { return A.Bodies.front().Index < B.Bodies.front().Index; });
	std::vector<InterfaceId> ReadingIds;
	ReadingIds.reserve(Judged.Readings.size());
	for (const InterfaceReading& Reading : Judged.Readings)
	{
		ReadingIds.push_back(Reading.Id);
	}
	const std::vector<std::size_t> ReadingAt = PositionTable(ReadingIds);
	std::vector<std::size_t> ClusterOfBody;
	for (std::size_t C = 0; C < Pending.size(); ++C)
	{
		for (const BodyId Body : Pending[C].Bodies)
		{
			if (Body.Index >= ClusterOfBody.size())
			{
				ClusterOfBody.resize(static_cast<std::size_t>(Body.Index) + 1, NoSlot);
			}
			if (ClusterOfBody[Body.Index] == NoSlot)
			{
				ClusterOfBody[Body.Index] = C;
			}
		}
	}
	std::vector<std::vector<InterfaceId>> Crossing(Pending.size());
	for (const InterfaceId Id : Model.LiveInterfaces())
	{
		const InterfaceRecord& Record = Model.Interface(Id);
		const std::size_t A = Lookup(ClusterOfBody, Record.BodyA);
		const std::size_t B = Lookup(ClusterOfBody, Record.BodyB);
		if (A != B)
		{
			Crossing[std::min(A, B)].push_back(Id);
		}
	}
	for (std::size_t C = 0; C < Pending.size(); ++C)
	{
		const PendingCluster& Cluster = Pending[C];
		ReleasedCluster Released;
		Released.Pass = Pass;
		Released.Bodies = Cluster.Bodies;
		Released.bForced = Cluster.Component->Forced;
		Released.Motion = Released.bForced ? MechanismMotion{Cluster.Bodies.front(), Vec3{}, Vec3{}} : MotionOf(*Cluster.Component, Cluster.Bodies.front());
		for (const InterfaceId Id : Crossing[C])
		{
			if (!Model.Sever(Id, Pass))
			{
				continue;
			}
			const std::size_t At = Lookup(ReadingAt, Id);
			const InterfaceReading* Reading = At != NoSlot ? &Judged.Readings[At] : nullptr;
			FailedInterface Failure;
			Failure.Id = Id;
			Failure.Pass = Pass;
			Failure.Axis = MechanismAxis(Model, *Cluster.Component, Id, Reading != nullptr ? Reading->Axis : GoverningAxis::None, Options.MotionAxisThresholdRelative);
			Failure.Ratio = Reading != nullptr ? Reading->Utilisation : NaN;
			Failure.bSevered = true;
			Failure.Kind = FailureKind::Severed;
			Verdict.Failures.push_back(Failure);
			Released.Severed.push_back(Id);
		}
		for (const BodyId Body : Cluster.Bodies)
		{
			Model.RemoveBody(Body);
			ReleasedThisRun.push_back(Body);
		}
		Verdict.Clusters.push_back(std::move(Released));
	}
	return !Pending.empty();
}

struct CrushCandidate
{
	InterfaceId Id;
	BodyId Body;
	double Ratio = 0.0;
};

double HeightOf(const BodyDesc& Body, const Vec3& Gravity)
{
	return -Dot(Body.CentroidM, Gravity);
}

bool IsKnownStrength(double StrengthPa)
{
	return IsFinite(StrengthPa) && StrengthPa > 0.0;
}

BodyId CrushingBody(const Structure& Model, const InterfaceRecord& Record, const Vec3& Gravity)
{
	const BodyDesc& A = Model.Body(Record.BodyA);
	const BodyDesc& B = Model.Body(Record.BodyB);
	if (A.bGrounded || B.bGrounded)
	{
		return A.bGrounded ? Record.BodyB : Record.BodyA;
	}
	if (IsKnownStrength(A.CompressiveStrengthPa) && IsKnownStrength(B.CompressiveStrengthPa) && A.CompressiveStrengthPa != B.CompressiveStrengthPa)
	{
		return A.CompressiveStrengthPa < B.CompressiveStrengthPa ? Record.BodyA : Record.BodyB;
	}
	const double HeightA = HeightOf(A, Gravity);
	const double HeightB = HeightOf(B, Gravity);
	if (HeightA != HeightB)
	{
		return HeightA < HeightB ? Record.BodyA : Record.BodyB;
	}
	return IsLowerSlot(Record.BodyA, Record.BodyB) ? Record.BodyA : Record.BodyB;
}

bool IsWorseCrush(const CrushCandidate& A, const CrushCandidate& B)
{
	return A.Ratio != B.Ratio ? A.Ratio > B.Ratio : A.Id.Index < B.Id.Index;
}

bool FailBand(Structure& Model, const AuthorityResult& Judged, std::uint32_t Pass, const CascadeOptions& Options, CascadeVerdict& Verdict, std::vector<BodyId>& ReleasedThisRun)
{
	std::vector<std::size_t> ComponentOf;
	for (std::size_t C = 0; C < Judged.Components.size(); ++C)
	{
		if (Judged.Components[C].Verdict != ComponentVerdict::Overstressed)
		{
			continue;
		}
		for (const BodyId Body : Judged.Components[C].Bodies)
		{
			if (Body.Index >= ComponentOf.size())
			{
				ComponentOf.resize(static_cast<std::size_t>(Body.Index) + 1, NoSlot);
			}
			if (ComponentOf[Body.Index] == NoSlot)
			{
				ComponentOf[Body.Index] = C;
			}
		}
	}
	std::vector<std::vector<std::size_t>> Bucket(Judged.Components.size());
	for (std::size_t R = 0; R < Judged.Readings.size(); ++R)
	{
		const InterfaceRecord& Record = Model.Interface(Judged.Readings[R].Id);
		std::size_t C = Lookup(ComponentOf, Record.BodyA);
		if (C == NoSlot)
		{
			C = Lookup(ComponentOf, Record.BodyB);
		}
		if (C != NoSlot)
		{
			Bucket[C].push_back(R);
		}
	}
	bool bChanged = false;
	for (std::size_t C = 0; C < Judged.Components.size(); ++C)
	{
		std::vector<CrackCandidate> Eligible;
		double RMax = -Infinity;
		std::vector<CrushCandidate> Crushing;
		for (const std::size_t R : Bucket[C])
		{
			const InterfaceReading& Reading = Judged.Readings[R];
			const InterfaceRecord& Record = Model.Interface(Reading.Id);
			if (Model.IsLive(Reading.Id) && IsFinite(Reading.CompressionUtilisation) && Reading.CompressionUtilisation > 1.0 + Options.CrushEdgeRelative)
			{
				Crushing.push_back(CrushCandidate{Reading.Id, CrushingBody(Model, Record, Options.Authority.Standing.Gravity), Reading.CompressionUtilisation});
				RMax = std::max(RMax, Reading.CompressionUtilisation);
			}
			if (!Model.IsLive(Reading.Id) || Record.State != InterfaceState::Intact)
			{
				continue;
			}
			const StrengthProfile Profile = Model.EffectiveProfile(Reading.Id);
			if (!(Profile.TensileStrengthPa > 0.0) && !(Profile.ShearCohesionPa > 0.0))
			{
				continue;
			}
			const bool bTensionGoverns = std::isnan(Reading.TensionUtilisation) ||
				(!std::isnan(Reading.ShearUtilisation) && !(Reading.TensionUtilisation < Reading.ShearUtilisation));
			const double Ratio = bTensionGoverns ? Reading.TensionUtilisation : Reading.ShearUtilisation;
			if (Ratio <= 1.0)
			{
				continue;
			}
			Eligible.push_back(CrackCandidate{Reading.Id, Ratio, bTensionGoverns ? GoverningAxis::Tension : GoverningAxis::Shear});
			if (IsFinite(Ratio))
			{
				RMax = std::max(RMax, Ratio);
			}
		}
		const double Threshold = RMax * (1.0 - Options.CrackBandFraction) * (1.0 - Options.CrackBandEdgeRelative);
		std::sort(Crushing.begin(), Crushing.end(), IsWorseCrush);
		for (const CrushCandidate& Each : Crushing)
		{
			if (Each.Ratio < Threshold || !Model.RemoveBody(Each.Body))
			{
				continue;
			}
			ReleasedThisRun.push_back(Each.Body);
			Verdict.CrushedBodies.push_back(CrushedBody{Pass, Each.Body, Each.Id, Each.Ratio});
			Verdict.Failures.push_back(FailedInterface{Each.Id, Pass, GoverningAxis::Compression, Each.Ratio, false, FailureKind::Crushed});
			bChanged = true;
		}
		for (const CrackCandidate& Each : Eligible)
		{
			const bool bInBand = !IsFinite(Each.Ratio) || Each.Ratio >= Threshold;
			if (!bInBand || !Model.Crack(Each.Id, Pass))
			{
				continue;
			}
			Verdict.Failures.push_back(FailedInterface{Each.Id, Pass, Each.Axis, Each.Ratio, false});
			bChanged = true;
		}
	}
	return bChanged;
}

class BrittleCascade final : public ICascade
{
public:
	explicit BrittleCascade(std::unique_ptr<IStructuralAuthority> InAuthority)
		: Authority(std::move(InAuthority))
	{
	}

	CascadeVerdict Run(Structure& Model, std::span<const BodyLoad> Loads, const CascadeOptions& Options, const AuthorityResult* Previous) const override
	{
		std::vector<std::size_t> Stale;
		const std::vector<BodyLoad> Live = statics::WithoutStaleLoads(Model, Loads, Stale);
		CascadeVerdict Verdict = RunLive(Model, Live, Options, Previous);
		Verdict.StaleLoads = std::move(Stale);
		return Verdict;
	}

private:
	CascadeVerdict RunLive(Structure& Model, std::span<const BodyLoad> Loads, const CascadeOptions& Options, const AuthorityResult* Previous) const
	{
		CascadeVerdict Verdict;
		Verdict.FirstPass = Model.HighestPass() + 1;
		Verdict.LastPass = Verdict.FirstPass - 1;
		if (!AreOptionsValid(Options) || Model.HighestPass() == std::numeric_limits<std::uint32_t>::max())
		{
			Verdict.State = CascadeState::InvalidInput;
			return Verdict;
		}
		std::vector<BodyId> ReleasedThisRun;
		AuthorityResult Last;
		const AuthorityResult* Before = Previous;
		const DebugSettings* Debug = Options.Authority.Standing.Debug;
		const bool bRecord = Debug != nullptr && Debug->bRecordPasses;
		while (true)
		{
			const std::uint32_t Pass = Model.HighestPass() + 1;
			std::vector<BodyLoad> PassLoads;
			for (const BodyLoad& Load : Loads)
			{
				if (!Contains(ReleasedThisRun, Load.Body))
				{
					PassLoads.push_back(Load);
				}
			}
			AuthorityResult Judged = Authority->Judge(Model, PassLoads, Options.Authority, Before);
			if (Judged.Worst == ComponentVerdict::InvalidInput)
			{
				Verdict.State = CascadeState::InvalidInput;
				Verdict.InvalidInterface = Judged.InvalidInterface;
				Verdict.Readings = std::move(Judged.Readings);
				Verdict.Components = std::move(Judged.Components);
				return Verdict;
			}
			ResolveUndecided(Judged, Options.OnUndecided, Verdict.ForcedByReason);
			const std::size_t FailuresBefore = Verdict.Failures.size();
			const std::size_t ClustersBefore = Verdict.Clusters.size();
			const bool bReleased = Release(Model, Judged, Pass, Options, Verdict, ReleasedThisRun);
			const std::size_t CrushedBefore = Verdict.CrushedBodies.size();
			const bool bFailed = FailBand(Model, Judged, Pass, Options, Verdict, ReleasedThisRun);
			std::stable_sort(Verdict.Failures.begin() + static_cast<std::ptrdiff_t>(FailuresBefore), Verdict.Failures.end(),
				[](const FailedInterface& A, const FailedInterface& B) { return A.Id.Index < B.Id.Index; });
			std::sort(Verdict.CrushedBodies.begin() + static_cast<std::ptrdiff_t>(CrushedBefore), Verdict.CrushedBodies.end(),
				[](const CrushedBody& A, const CrushedBody& B) { return A.Body.Index < B.Body.Index; });
			if (bRecord)
			{
				Verdict.PassRecords.push_back(CascadePassRecord{Pass, Judged.Components, Judged.Readings,
					{Verdict.Clusters.begin() + static_cast<std::ptrdiff_t>(ClustersBefore), Verdict.Clusters.end()},
					{Verdict.Failures.begin() + static_cast<std::ptrdiff_t>(FailuresBefore), Verdict.Failures.end()},
					{Verdict.CrushedBodies.begin() + static_cast<std::ptrdiff_t>(CrushedBefore), Verdict.CrushedBodies.end()}});
			}
			const ComponentVerdict Worst = Judged.Worst;
			Last = std::move(Judged);
			Before = &Last;
			Verdict.Readings = Last.Readings;
			Verdict.Components = Last.Components;
			if (!bReleased && !bFailed)
			{
				Verdict.State = FinalState(Worst);
				return Verdict;
			}
			Model.MarkPass(Pass);
			++Verdict.Passes;
			Verdict.LastPass = Pass;
			if (Verdict.Passes >= Options.MaxPasses || Pass == UINT32_MAX)
			{
				Verdict.State = CascadeState::PassCapReached;
				std::vector<BodyLoad> CapLoads;
				for (const BodyLoad& Load : Loads)
				{
					if (!Contains(ReleasedThisRun, Load.Body))
					{
						CapLoads.push_back(Load);
					}
				}
				AuthorityResult Rejudged = Authority->Judge(Model, CapLoads, Options.Authority, &Last);
				if (bRecord)
				{
					Verdict.PassRecords.push_back(CascadePassRecord{Pass + 1, Rejudged.Components, Rejudged.Readings, {}, {}, {}});
				}
				Verdict.Readings = std::move(Rejudged.Readings);
				Verdict.Components = std::move(Rejudged.Components);
				return Verdict;
			}
		}
	}

	std::unique_ptr<IStructuralAuthority> Authority;
};
}

std::unique_ptr<ICascade> MakeCascade(std::unique_ptr<IStructuralAuthority> Authority)
{
	return std::make_unique<BrittleCascade>(std::move(Authority));
}
}
