#include "loadpath/StructuralAuthority.h"

#include "PhaseClock.h"
#include "StaticsCommon.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>
#include <utility>

namespace loadpath
{
namespace
{
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

bool SameVector(const Vec3& A, const Vec3& B)
{
	return A.X == B.X && A.Y == B.Y && A.Z == B.Z;
}

bool SameStanding(const StandingOptions& A, const StandingOptions& B)
{
	const auto& [AGravity, AIterations, APivot, ADense, AStick, AHeld, ADry, AReclose, ASparseMargin, ANeutral, AAdmissible, AUnconstrained, ADebug] = A;
	const auto& [BGravity, BIterations, BPivot, BDense, BStick, BHeld, BDry, BReclose, BSparseMargin, BNeutral, BAdmissible, BUnconstrained, BDebug] = B;
	return SameVector(AGravity, BGravity) && AIterations == BIterations && APivot == BPivot && ADense == BDense && AStick == BStick && AHeld == BHeld &&
		ADry == BDry && AReclose == BReclose && ASparseMargin == BSparseMargin && ANeutral == BNeutral && AAdmissible == BAdmissible && AUnconstrained == BUnconstrained && ADebug == BDebug;
}

bool SameOptions(const AuthorityOptions& A, const AuthorityOptions& B)
{
	const auto& [AStanding] = A;
	const auto& [BStanding] = B;
	return SameStanding(AStanding, BStanding);
}

AuthorityResult Invalid(InterfaceId Interface)
{
	AuthorityResult Result;
	Result.Worst = ComponentVerdict::InvalidInput;
	Result.InvalidInterface = Interface;
	return Result;
}

bool SameLoads(const std::vector<BodyLoad>& A, const std::vector<BodyLoad>& B)
{
	if (A.size() != B.size())
	{
		return false;
	}
	for (std::size_t I = 0; I < A.size(); ++I)
	{
		if (A[I].Body != B[I].Body || !SameVector(A[I].ForceN, B[I].ForceN) || !SameVector(A[I].PointM, B[I].PointM))
		{
			return false;
		}
	}
	return true;
}

bool AreLoadsAttributable(const Structure& Model, std::span<const BodyLoad> Loads)
{
	for (const BodyLoad& Load : Loads)
	{
		if (!Model.IsLive(Load.Body) || !IsFinite(Load.ForceN) || !IsFinite(Load.PointM))
		{
			return false;
		}
	}
	return true;
}

bool IsUntouchedSince(const Structure& Model, BodyId Body, std::uint64_t Revision)
{
	return Model.LastTouched(Body) <= Revision;
}

const ComponentReport* FindClean(const Structure& Model, const AuthorityResult& Previous, const std::vector<int>& ReportOfFront, const std::vector<BodyId>& Bodies, const std::vector<BodyLoad>& Loads)
{
	for (const BodyId Body : Bodies)
	{
		if (!IsUntouchedSince(Model, Body, Previous.Revision))
		{
			return nullptr;
		}
	}
	const std::size_t Front = Bodies.front().Index;
	if (Front >= ReportOfFront.size() || ReportOfFront[Front] < 0)
	{
		return nullptr;
	}
	const ComponentReport& Report = Previous.Components[static_cast<std::size_t>(ReportOfFront[Front])];
	if (Report.Bodies != Bodies)
	{
		return nullptr;
	}
	return SameLoads(Report.Loads, Loads) ? &Report : nullptr;
}

const InterfaceReading* FindReading(const std::vector<InterfaceReading>& Readings, InterfaceId Id)
{
	const auto Found = std::lower_bound(Readings.begin(), Readings.end(), Id, [](const InterfaceReading& R, InterfaceId Key) { return R.Id.Index < Key.Index; });
	return Found != Readings.end() && Found->Id == Id ? &*Found : nullptr;
}

bool IsReadingUntouched(const Structure& Model, const InterfaceRecord& Record, std::uint64_t Revision)
{
	const bool bGroundedA = Model.Body(Record.BodyA).bGrounded;
	const bool bGroundedB = Model.Body(Record.BodyB).bGrounded;
	const bool bCheckA = !bGroundedA || bGroundedB;
	const bool bCheckB = !bGroundedB || bGroundedA;
	return (!bCheckA || IsUntouchedSince(Model, Record.BodyA, Revision)) && (!bCheckB || IsUntouchedSince(Model, Record.BodyB, Revision));
}

bool MergeReadings(const Structure& Model, const std::vector<InterfaceReading>& Solved, const AuthorityResult* Previous, std::vector<InterfaceReading>& Merged)
{
	const std::vector<InterfaceId> Live = Model.LiveInterfaces();
	if (Previous == nullptr)
	{
		Merged = Solved;
		return true;
	}
	Merged.reserve(Live.size());
	for (const InterfaceId Id : Live)
	{
		const InterfaceReading* Reading = FindReading(Solved, Id);
		if (Reading == nullptr)
		{
			const InterfaceRecord& Record = Model.Interface(Id);
			const bool bUntouched = IsReadingUntouched(Model, Record, Previous->Revision);
			Reading = bUntouched ? FindReading(Previous->Readings, Id) : nullptr;
		}
		if (Reading == nullptr)
		{
			return false;
		}
		Merged.push_back(*Reading);
	}
	return true;
}

class ElasticAuthority final : public IStructuralAuthority
{
public:
	explicit ElasticAuthority(std::unique_ptr<IStandingAuthority> InStanding)
		: Standing(std::move(InStanding))
	{
	}

	AuthorityResult Judge(const Structure& Model, std::span<const BodyLoad> Loads, const AuthorityOptions& Options, const AuthorityResult* Previous) const override
	{
		PhaseTimings Elastic;
		double JudgeSeconds = 0.0;
		AuthorityResult Result = profile::Timed(JudgeSeconds, [&] {
			std::vector<std::size_t> Stale;
			const std::vector<BodyLoad> Live = statics::WithoutStaleLoads(Model, Loads, Stale);
			AuthorityResult Judged = JudgeLive(Model, Live, Options, Previous, Elastic);
			Judged.StaleLoads = std::move(Stale);
			return Judged;
		});
		Result.ElasticTimings = Elastic;
		Result.JudgeSeconds = JudgeSeconds;
		return Result;
	}

private:
	AuthorityResult JudgeLive(const Structure& Model, std::span<const BodyLoad> Loads, const AuthorityOptions& Options, const AuthorityResult* Previous, PhaseTimings& Elastic) const
	{
		if (!statics::AreStandingOptionsValid(Options.Standing))
		{
			AuthorityResult Result = Invalid(InterfaceId{});
			Result.Options = Options;
			Result.Lineage = Model.Lineage();
			return Result;
		}
		const bool bMatchable = Previous != nullptr && SameOptions(Previous->Options, Options) && Previous->Lineage == Model.Lineage() && Previous->Revision <= Model.Revision() && AreLoadsAttributable(Model, Loads);
		if (bMatchable)
		{
			std::optional<AuthorityResult> Reused = JudgeAgainst(Model, Loads, Options, Previous, Elastic);
			if (Reused)
			{
				Reused->Options = Options;
				Reused->Lineage = Model.Lineage();
				return std::move(*Reused);
			}
		}
		AuthorityResult Result = std::move(*JudgeAgainst(Model, Loads, Options, nullptr, Elastic));
		Result.Options = Options;
		Result.Lineage = Model.Lineage();
		return Result;
	}

	std::optional<AuthorityResult> JudgeAgainst(const Structure& Model, std::span<const BodyLoad> Loads, const AuthorityOptions& Options, const AuthorityResult* Previous, PhaseTimings& Elastic) const
	{
		const ComponentPartition& Partition = Model.Components();
		const std::size_t Count = Partition.FreeBodies.size();
		std::vector<std::vector<BodyLoad>> LoadsOf(Count);
		for (const BodyLoad& Load : Loads)
		{
			if (Model.IsLive(Load.Body) && Partition.ComponentOfSlot[Load.Body.Index] >= 0)
			{
				LoadsOf[static_cast<std::size_t>(Partition.ComponentOfSlot[Load.Body.Index])].push_back(Load);
			}
		}
		std::vector<int> ReportOfFront;
		if (Previous != nullptr)
		{
			for (std::size_t R = 0; R < Previous->Components.size(); ++R)
			{
				const std::size_t Front = Previous->Components[R].Bodies.front().Index;
				if (Front >= ReportOfFront.size())
				{
					ReportOfFront.resize(Front + 1, -1);
				}
				ReportOfFront[Front] = static_cast<int>(R);
			}
		}
		std::vector<const ComponentReport*> CleanFrom(Count, nullptr);
		std::vector<BodyId> Only;
		std::size_t DirtyCount = 0;
		for (std::size_t K = 0; K < Count; ++K)
		{
			CleanFrom[K] = Previous != nullptr ? FindClean(Model, *Previous, ReportOfFront, Partition.FreeBodies[K], LoadsOf[K]) : nullptr;
			if (CleanFrom[K] == nullptr)
			{
				++DirtyCount;
				Only.insert(Only.end(), Partition.FreeBodies[K].begin(), Partition.FreeBodies[K].end());
			}
		}
		std::sort(Only.begin(), Only.end(), [](BodyId A, BodyId B) { return A.Index < B.Index; });
		if (DirtyCount == Count)
		{
			Only.clear();
		}
		StandingResult Whole;
		if (DirtyCount > 0 || Previous == nullptr)
		{
			Whole = Standing->Solve(Model, Loads, Options.Standing, Only);
			profile::Accumulate(Elastic, Whole.Timings);
			if (Whole.State == StandingState::InvalidInput)
			{
				return Invalid(Whole.InvalidInterface);
			}
		}
		AuthorityResult Result;
		Result.Revision = Model.Revision();
		if (!MergeReadings(Model, Whole.Readings, Previous, Result.Readings))
		{
			return std::nullopt;
		}
		std::vector<std::size_t> SolvedOfSlot;
		for (std::size_t Solved = 0; Solved < Whole.Components.size(); ++Solved)
		{
			for (const BodyId Body : Whole.Components[Solved].Bodies)
			{
				if (Body.Index >= SolvedOfSlot.size())
				{
					SolvedOfSlot.resize(static_cast<std::size_t>(Body.Index) + 1, Whole.Components.size());
				}
				SolvedOfSlot[Body.Index] = Solved;
			}
		}
		std::size_t NextSolved = 0;
		std::vector<ComponentReport> Reports(Count);
		for (std::size_t K = 0; K < Count; ++K)
		{
			if (CleanFrom[K] != nullptr)
			{
				Reports[K] = *CleanFrom[K];
				Reports[K].bReused = true;
				continue;
			}
			const std::size_t Solved = NextSolved++;
			Reports[K] = ReportOf(Whole.Components[Solved]);
			Reports[K].Loads = std::move(LoadsOf[K]);
			if (Reports[K].Verdict != ComponentVerdict::Collapses)
			{
				continue;
			}
			const auto IsHere = [&](BodyId Body) { return Body.Index < SolvedOfSlot.size() && SolvedOfSlot[Body.Index] == Solved; };
			for (const BodyId Body : Whole.UnconstrainedBodies)
			{
				if (IsHere(Body))
				{
					Reports[K].Release.push_back(Body);
				}
			}
			for (const MechanismMotion& Motion : Whole.Mechanism)
			{
				if (IsHere(Motion.Body))
				{
					Reports[K].Mechanism.push_back(Motion);
				}
			}
		}
		Result.Worst = ComponentVerdict::Stands;
		for (ComponentReport& Report : Reports)
		{
			if (Severity(Report.Verdict) > Severity(Result.Worst))
			{
				Result.Worst = Report.Verdict;
			}
			Result.Components.push_back(std::move(Report));
		}
		return Result;
	}

	static std::vector<InterfaceId> UnsettledOf(const StandingComponent& Part)
	{
		if (Part.End == SolveEnd::RestoreFailedClosed)
		{
			return Part.Blocking;
		}
		std::vector<InterfaceId> Out;
		if (Part.End != SolveEnd::IterationCap || Part.Passes.empty())
		{
			return Out;
		}
		for (const JointPassRecord& Joint : Part.Passes.back().Joints)
		{
			if (std::find(Joint.bFlipped.begin(), Joint.bFlipped.end(), true) != Joint.bFlipped.end())
			{
				Out.push_back(Joint.Id);
			}
		}
		return Out;
	}

	static ComponentReport ReportOf(StandingComponent& Part)
	{
		ComponentReport Report;
		Report.Bodies = std::move(Part.Bodies);
		Report.Elastic = Part.State;
		Report.End = Part.End;
		Report.PassCount = Part.ActiveSetIterations;
		if (Part.State == StandingState::Undecided)
		{
			Report.UnsettledJoints = UnsettledOf(Part);
		}
		Report.Passes = std::move(Part.Passes);
		switch (Part.State)
		{
		case StandingState::Stands: Report.Verdict = ComponentVerdict::Stands; break;
		case StandingState::Overstressed: Report.Verdict = ComponentVerdict::Overstressed; break;
		case StandingState::Mechanism: Report.Verdict = ComponentVerdict::Collapses; break;
		case StandingState::Undecided: Report.Verdict = ComponentVerdict::Undecided; break;
		case StandingState::InvalidInput: Report.Verdict = ComponentVerdict::InvalidInput; break;
		}
		return Report;
	}

	std::unique_ptr<IStandingAuthority> Standing;
};
}

std::unique_ptr<IStructuralAuthority> MakeElasticAuthority(std::unique_ptr<IStandingAuthority> Standing)
{
	return std::make_unique<ElasticAuthority>(std::move(Standing));
}
}
