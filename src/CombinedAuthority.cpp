#include "loadpath/StructuralAuthority.h"

#include "StaticsCommon.h"

#include <algorithm>
#include <cstddef>
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

bool SameGravity(const Vec3& A, const Vec3& B)
{
	return A.X == B.X && A.Y == B.Y && A.Z == B.Z;
}

int ComponentOfBody(const statics::ComponentPartition& Partition, BodyId Body)
{
	const std::size_t Slot = static_cast<std::size_t>(Body.Index);
	return Slot < Partition.ComponentOfSlot.size() ? Partition.ComponentOfSlot[Slot] : -1;
}

double Ratio(double DemandN, double CapacityN)
{
	return DemandN == 0.0 ? 0.0 : DemandN / CapacityN;
}

bool IsReadingOverstressed(const Structure& Model, const InterfaceReading& Reading)
{
	const InterfaceRecord& Record = Model.Interface(Reading.Id);
	const StrengthProfile& Profile = Model.Profile(Record.Profile);
	const double CornerAreaM2 = Record.AreaM2 / static_cast<double>(statics::CornerCount);
	const bool bReleasesOpening = !(Profile.TensileStrengthPa > 0.0);
	const bool bReleasesSlip = !(Profile.ShearCohesionPa > 0.0);
	for (const CornerReading& Corner : Reading.Corners)
	{
		const double CompressionN = Corner.NormalForceN > 0.0 ? Corner.NormalForceN : 0.0;
		const double TensionN = Corner.NormalForceN < 0.0 ? -Corner.NormalForceN : 0.0;
		const double Compression = Ratio(CompressionN, Profile.CompressiveStrengthPa * CornerAreaM2);
		const double Tension = Ratio(TensionN, Profile.TensileStrengthPa * CornerAreaM2);
		const double ShearCapacityN = std::min(Profile.ShearCohesionPa * CornerAreaM2 + Profile.FrictionCoefficient * CompressionN, Profile.MaxShearStrengthPa * CornerAreaM2);
		const double Shear = Ratio(Length(Corner.ShearForceN), ShearCapacityN);
		if (!(Compression <= 1.0) || (!bReleasesOpening && !(Tension <= 1.0)) || (!bReleasesSlip && !(Shear <= 1.0)))
		{
			return true;
		}
	}
	return false;
}

class CombinedAuthority final : public IStructuralAuthority
{
public:
	CombinedAuthority(std::unique_ptr<IStandingAuthority> InStanding, std::unique_ptr<ICollapseCertificate> InCertificate)
		: Standing(std::move(InStanding))
		, Certificate(std::move(InCertificate))
	{
	}

	AuthorityResult Judge(const Structure& Model, std::span<const BodyLoad> Loads, const AuthorityOptions& Options) const override
	{
		AuthorityResult Result;
		if (!SameGravity(Options.Standing.Gravity, Options.Certificate.Gravity))
		{
			Result.Worst = ComponentVerdict::InvalidInput;
			return Result;
		}
		StandingResult Elastic = Standing->Solve(Model, Loads, Options.Standing);
		if (Elastic.State == StandingState::InvalidInput)
		{
			Result.Worst = ComponentVerdict::InvalidInput;
			Result.InvalidInterface = Elastic.InvalidInterface;
			return Result;
		}
		Result.Readings = std::move(Elastic.Readings);
		const statics::ComponentPartition Partition = statics::PartitionFreeBodies(Model);
		const std::size_t ComponentCount = Partition.FreeBodies.size();

		std::vector<StandingState> ElasticOf(ComponentCount, StandingState::Stands);
		for (const BodyId Body : Elastic.UnconstrainedBodies)
		{
			const int C = ComponentOfBody(Partition, Body);
			if (C >= 0)
			{
				ElasticOf[static_cast<std::size_t>(C)] = StandingState::Mechanism;
			}
		}
		bool bAnyOverstressed = false;
		for (const InterfaceReading& Reading : Result.Readings)
		{
			const int C = statics::ComponentOf(Partition, Model.Interface(Reading.Id));
			if (C >= 0 && IsReadingOverstressed(Model, Reading) && ElasticOf[static_cast<std::size_t>(C)] != StandingState::Mechanism)
			{
				ElasticOf[static_cast<std::size_t>(C)] = StandingState::Overstressed;
				bAnyOverstressed = true;
			}
		}
		for (StandingState& State : ElasticOf)
		{
			if (State == StandingState::Mechanism)
			{
				continue;
			}
			if (Elastic.State == StandingState::Undecided)
			{
				State = StandingState::Undecided;
			}
			else if (Elastic.State == StandingState::Overstressed && !bAnyOverstressed)
			{
				State = StandingState::Overstressed;
			}
		}

		Result.Worst = ComponentVerdict::Stands;
		for (std::size_t C = 0; C < ComponentCount; ++C)
		{
			ComponentReport Report;
			Report.Bodies = Partition.FreeBodies[C];
			Report.Elastic = ElasticOf[C];
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
				Consult(Model, Loads, Options.Certificate, Partition, C, Report);
				if (Report.bDisagreed)
				{
					++Result.Disagreements;
				}
			}
			if (Severity(Report.Verdict) > Severity(Result.Worst))
			{
				Result.Worst = Report.Verdict;
			}
			Result.Components.push_back(std::move(Report));
		}
		return Result;
	}

private:
	void Consult(const Structure& Model, std::span<const BodyLoad> Loads, const CertificateOptions& Options, const statics::ComponentPartition& Partition, std::size_t Component, ComponentReport& Report) const
	{
		Structure Sub = Model;
		for (std::size_t Other = 0; Other < Partition.FreeBodies.size(); ++Other)
		{
			if (Other == Component)
			{
				continue;
			}
			for (const BodyId Body : Partition.FreeBodies[Other])
			{
				Sub.RemoveBody(Body);
			}
		}
		std::vector<BodyLoad> SubLoads;
		for (const BodyLoad& Load : Loads)
		{
			if (ComponentOfBody(Partition, Load.Body) == static_cast<int>(Component))
			{
				SubLoads.push_back(Load);
			}
		}
		CertificateResult Kinematic = Certificate->Decide(Sub, SubLoads, Options);
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
