#include "loadpath/Cascade.h"

#include "loadpath/Units.h"

#include <algorithm>
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
constexpr double CrackBandRelativeEdge = 1.0e-9;

bool AreOptionsValid(const CascadeOptions& Options)
{
	return IsFinite(Options.CrackBandFraction) && Options.CrackBandFraction >= 0.0 && Options.CrackBandFraction <= 1.0 &&
		Options.MaxPasses >= 1;
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

std::vector<PendingCluster> ClustersOf(const Structure& Model, const ComponentReport& Component, const std::vector<std::size_t>& PositionOf, const std::vector<std::size_t>& HingingAt)
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
			if (Lookup(HingingAt, Id) != NoSlot)
			{
				continue;
			}
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

struct CrackCandidate
{
	InterfaceId Id;
	double Ratio = 0.0;
	GoverningAxis Axis = GoverningAxis::None;
};

bool Release(Structure& Model, const AuthorityResult& Judged, std::uint32_t Pass, CascadeVerdict& Verdict, std::vector<BodyId>& ReleasedThisRun)
{
	std::vector<std::size_t> PositionOf;
	std::vector<std::size_t> HingingAt;
	for (const ComponentReport& Component : Judged.Components)
	{
		if (Component.Verdict == ComponentVerdict::Collapses)
		{
			MarkPositions(PositionOf, Component.Release);
			MarkPositions(HingingAt, Component.Hinging);
		}
	}
	std::vector<PendingCluster> Pending;
	for (const ComponentReport& Component : Judged.Components)
	{
		if (Component.Verdict != ComponentVerdict::Collapses)
		{
			continue;
		}
		for (PendingCluster& Cluster : ClustersOf(Model, Component, PositionOf, HingingAt))
		{
			Pending.push_back(std::move(Cluster));
		}
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
			ClusterOfBody[Body.Index] = C;
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
		Released.Motion = MotionOf(*Cluster.Component, Cluster.Bodies.front());
		Released.LoadFactorLower = Cluster.Component->LoadFactorLower;
		Released.LoadFactorUpper = Cluster.Component->LoadFactorUpper;
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
			Failure.Axis = Reading != nullptr ? Reading->Axis : GoverningAxis::None;
			Failure.Ratio = Reading != nullptr ? Reading->Utilisation : NaN;
			Failure.bSevered = true;
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

bool CrackBand(Structure& Model, const AuthorityResult& Judged, std::uint32_t Pass, double BandFraction, CascadeVerdict& Verdict)
{
	bool bCracked = false;
	for (const ComponentReport& Component : Judged.Components)
	{
		if (Component.Verdict != ComponentVerdict::Overstressed)
		{
			continue;
		}
		std::vector<CrackCandidate> Eligible;
		double RMax = -Infinity;
		const std::vector<std::size_t> MemberAt = PositionTable(Component.Bodies);
		for (const InterfaceReading& Reading : Judged.Readings)
		{
			const InterfaceRecord& Record = Model.Interface(Reading.Id);
			if (!Model.IsLive(Reading.Id) || Record.State != InterfaceState::Intact ||
				(Lookup(MemberAt, Record.BodyA) == NoSlot && Lookup(MemberAt, Record.BodyB) == NoSlot))
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
		const double Threshold = RMax * (1.0 - BandFraction) * (1.0 - CrackBandRelativeEdge);
		for (const CrackCandidate& Each : Eligible)
		{
			const bool bInBand = !IsFinite(Each.Ratio) || Each.Ratio >= Threshold;
			if (!bInBand || !Model.Crack(Each.Id, Pass))
			{
				continue;
			}
			Verdict.Failures.push_back(FailedInterface{Each.Id, Pass, Each.Axis, Each.Ratio, false});
			bCracked = true;
		}
	}
	return bCracked;
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
			const std::size_t FailuresBefore = Verdict.Failures.size();
			const bool bReleased = Release(Model, Judged, Pass, Verdict, ReleasedThisRun);
			const bool bCracked = CrackBand(Model, Judged, Pass, Options.CrackBandFraction, Verdict);
			std::stable_sort(Verdict.Failures.begin() + static_cast<std::ptrdiff_t>(FailuresBefore), Verdict.Failures.end(),
				[](const FailedInterface& A, const FailedInterface& B) { return A.Id.Index < B.Id.Index; });
			const ComponentVerdict Worst = Judged.Worst;
			Last = std::move(Judged);
			Before = &Last;
			Verdict.Readings = Last.Readings;
			Verdict.Components = Last.Components;
			if (!bReleased && !bCracked)
			{
				Verdict.State = FinalState(Worst);
				return Verdict;
			}
			Model.MarkPass(Pass);
			++Verdict.Passes;
			Verdict.LastPass = Pass;
			if (Verdict.Passes >= Options.MaxPasses)
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
				Verdict.Readings = std::move(Rejudged.Readings);
				Verdict.Components = std::move(Rejudged.Components);
				return Verdict;
			}
		}
	}

private:
	std::unique_ptr<IStructuralAuthority> Authority;
};
}

std::unique_ptr<ICascade> MakeCascade(std::unique_ptr<IStructuralAuthority> Authority)
{
	return std::make_unique<BrittleCascade>(std::move(Authority));
}
}
