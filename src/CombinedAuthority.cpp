#include "loadpath/StructuralAuthority.h"

#include "StaticsCommon.h"

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

AuthorityResult Invalid(InterfaceId Interface)
{
	AuthorityResult Result;
	Result.Worst = ComponentVerdict::InvalidInput;
	Result.InvalidInterface = Interface;
	return Result;
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
		if (!SameGravity(Options.Standing.Gravity, Options.Certificate.Gravity) || !statics::AreCertificateOptionsValid(Options.Certificate))
		{
			return Invalid(InterfaceId{});
		}
		StandingResult Whole = Standing->Solve(Model, Loads, Options.Standing);
		if (Whole.State == StandingState::InvalidInput)
		{
			return Invalid(Whole.InvalidInterface);
		}
		AuthorityResult Result;
		Result.Readings = std::move(Whole.Readings);
		Result.Worst = ComponentVerdict::Stands;
		for (StandingComponent& Part : Whole.Components)
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
				Consult(Model, Loads, Options.Certificate, Report);
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
