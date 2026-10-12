#include "loadpath/StructuralAuthority.h"

#include "StaticsCommon.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>
#include <limits>
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
	const auto& [AGravity, AIterations, APivot, AFactor, ADense] = A;
	const auto& [BGravity, BIterations, BPivot, BFactor, BDense] = B;
	return SameVector(AGravity, BGravity) && AIterations == BIterations && APivot == BPivot && AFactor == BFactor && ADense == BDense;
}

bool SameCertificate(const CertificateOptions& A, const CertificateOptions& B)
{
	const auto& [AGravity, AGenerators, ACap, ATolerance, AIterations, AStallWindow, AStallImprovement, ARows, ACapSite, ACoupling] = A;
	const auto& [BGravity, BGenerators, BCap, BTolerance, BIterations, BStallWindow, BStallImprovement, BRows, BCapSite, BCoupling] = B;
	return SameVector(AGravity, BGravity) && AGenerators == BGenerators && ACap == BCap && ATolerance == BTolerance && AIterations == BIterations && AStallWindow == BStallWindow && AStallImprovement == BStallImprovement && ARows == BRows && ACapSite == BCapSite && ACoupling == BCoupling;
}

bool SameOptions(const AuthorityOptions& A, const AuthorityOptions& B)
{
	const auto& [AStanding, ACertificate] = A;
	const auto& [BStanding, BCertificate] = B;
	return SameStanding(AStanding, BStanding) && SameCertificate(ACertificate, BCertificate);
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

const ComponentReport* FindClean(const Structure& Model, const AuthorityResult& Previous, const std::vector<BodyId>& Bodies, const std::vector<BodyLoad>& Loads)
{
	for (const BodyId Body : Bodies)
	{
		if (!IsUntouchedSince(Model, Body, Previous.Revision))
		{
			return nullptr;
		}
	}
	for (const ComponentReport& Report : Previous.Components)
	{
		if (Report.Bodies == Bodies)
		{
			return SameLoads(Report.Loads, Loads) ? &Report : nullptr;
		}
	}
	return nullptr;
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

class CombinedAuthority final : public IStructuralAuthority
{
public:
	CombinedAuthority(std::unique_ptr<IStandingAuthority> InStanding, std::unique_ptr<ICollapseCertificate> InCertificate)
		: Standing(std::move(InStanding))
		, Certificate(std::move(InCertificate))
	{
	}

	AuthorityResult Judge(const Structure& Model, std::span<const BodyLoad> Loads, const AuthorityOptions& Options, const AuthorityResult* Previous) const override
	{
		if (!SameVector(Options.Standing.Gravity, Options.Certificate.Gravity) || !statics::AreCertificateOptionsValid(Options.Certificate) || !statics::AreStandingOptionsValid(Options.Standing))
		{
			AuthorityResult Result = Invalid(InterfaceId{});
			Result.Options = Options;
			Result.Lineage = Model.Lineage();
			return Result;
		}
		const bool bMatchable = Previous != nullptr && SameOptions(Previous->Options, Options) && Previous->Lineage == Model.Lineage() && Previous->Revision <= Model.Revision() && AreLoadsAttributable(Model, Loads);
		if (bMatchable)
		{
			std::optional<AuthorityResult> Reused = JudgeAgainst(Model, Loads, Options, Previous);
			if (Reused)
			{
				Reused->Options = Options;
				Reused->Lineage = Model.Lineage();
				return std::move(*Reused);
			}
		}
		AuthorityResult Result = std::move(*JudgeAgainst(Model, Loads, Options, nullptr));
		Result.Options = Options;
		Result.Lineage = Model.Lineage();
		return Result;
	}

private:
	std::optional<AuthorityResult> JudgeAgainst(const Structure& Model, std::span<const BodyLoad> Loads, const AuthorityOptions& Options, const AuthorityResult* Previous) const
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
		std::vector<const ComponentReport*> CleanFrom(Count, nullptr);
		std::vector<BodyId> Only;
		std::size_t DirtyCount = 0;
		for (std::size_t K = 0; K < Count; ++K)
		{
			CleanFrom[K] = Previous != nullptr ? FindClean(Model, *Previous, Partition.FreeBodies[K], LoadsOf[K]) : nullptr;
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
		Result.Worst = ComponentVerdict::Stands;
		std::size_t NextSolved = 0;
		for (std::size_t K = 0; K < Count; ++K)
		{
			ComponentReport Report;
			if (CleanFrom[K] != nullptr)
			{
				Report = *CleanFrom[K];
				Report.bReused = true;
			}
			else
			{
				Report = ReportOf(Model, Loads, Options.Certificate, Whole.Components[NextSolved++]);
				Report.Loads = std::move(LoadsOf[K]);
			}
			if (Report.bDisagreed)
			{
				++Result.Disagreements;
			}
			if (Severity(Report.Verdict) > Severity(Result.Worst))
			{
				Result.Worst = Report.Verdict;
			}
			Result.Components.push_back(std::move(Report));
		}
		return Result;
	}

	ComponentReport ReportOf(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, StandingComponent& Part) const
	{
		ComponentReport Report;
		Report.Bodies = std::move(Part.Bodies);
		Report.Elastic = Part.State;
		Report.LoadFactorLower = std::numeric_limits<double>::quiet_NaN();
		Report.LoadFactorUpper = std::numeric_limits<double>::quiet_NaN();
		if (Report.Elastic == StandingState::Stands)
		{
			Report.Verdict = ComponentVerdict::Stands;
		}
		else if (Report.Elastic == StandingState::Overstressed)
		{
			Report.Verdict = ComponentVerdict::Overstressed;
		}
		else
		{
			Consult(Model, Loads, Options, Report);
		}
		return Report;
	}

	void Consult(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, ComponentReport& Report) const
	{
		CertificateResult Kinematic = Certificate->Decide(Model, Loads, Options, Report.Bodies);
		Report.bConsultedCertificate = true;
		Report.Kinematic = Kinematic.State;
		Report.LoadFactorLower = Kinematic.LoadFactorLower;
		Report.LoadFactorUpper = Kinematic.LoadFactorUpper;
		switch (Kinematic.State)
		{
		case CertificateState::Stands:
			Report.Verdict = ComponentVerdict::Stands;
			Report.bDisagreed = Report.Elastic == StandingState::Mechanism;
			break;
		case CertificateState::Collapses:
			Report.Verdict = ComponentVerdict::Collapses;
			Report.Release = std::move(Kinematic.UnconstrainedBodies);
			Report.Mechanism = std::move(Kinematic.Mechanism);
			Report.Hinging = std::move(Kinematic.HingingInterfaces);
			break;
		case CertificateState::Undecided:
			Report.Verdict = ComponentVerdict::Undecided;
			break;
		case CertificateState::InvalidInput:
			Report.Verdict = ComponentVerdict::InvalidInput;
			break;
		}
	}

	std::unique_ptr<IStandingAuthority> Standing;
	std::unique_ptr<ICollapseCertificate> Certificate;
};
}

std::unique_ptr<IStructuralAuthority> MakeCombinedAuthority(std::unique_ptr<IStandingAuthority> Standing, std::unique_ptr<ICollapseCertificate> Certificate)
{
	return std::make_unique<CombinedAuthority>(std::move(Standing), std::move(Certificate));
}
}
