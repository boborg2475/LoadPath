#pragma once

#include "BrickFixtures.h"
#include "StaticsTestSupport.h"

#include "loadpath/Cascade.h"
#include "loadpath/StructuralAuthority.h"

#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <memory>
#include <span>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace loadpath::test
{
inline const char* VerdictName(ComponentVerdict Verdict)
{
	switch (Verdict)
	{
	case ComponentVerdict::Stands: return "Stands";
	case ComponentVerdict::Overstressed: return "Overstressed";
	case ComponentVerdict::Collapses: return "Collapses";
	case ComponentVerdict::Undecided: return "Undecided";
	case ComponentVerdict::InvalidInput: return "InvalidInput";
	}
	return "ComponentVerdict(?)";
}

inline const char* CascadeStateName(CascadeState State)
{
	switch (State)
	{
	case CascadeState::Stands: return "Stands";
	case CascadeState::Overstressed: return "Overstressed";
	case CascadeState::Undecided: return "Undecided";
	case CascadeState::InvalidInput: return "InvalidInput";
	case CascadeState::PassCapReached: return "PassCapReached";
	}
	return "CascadeState(?)";
}

inline std::unique_ptr<IStructuralAuthority> MakeAuthority()
{
	return MakeElasticAuthority(MakeRbsmStandingAuthority());
}

inline AuthorityResult Judge(const Structure& Model, const AuthorityOptions& Options, std::span<const BodyLoad> Loads = {})
{
	return MakeAuthority()->Judge(Model, Loads, Options, nullptr);
}

inline AuthorityOptions AuthorityUnderGravity(const Vec3& GravityMPerS2Vector)
{
	AuthorityOptions Options;
	Options.Standing.Gravity = GravityMPerS2Vector;
	return Options;
}

inline AuthorityOptions AuthorityTilted(double TiltRad)
{
	return AuthorityUnderGravity(Vec3{GravityMPerS2 * std::sin(TiltRad), 0.0, -GravityMPerS2 * std::cos(TiltRad)});
}

inline AuthorityOptions AuthorityLateral(double LambdaX)
{
	return AuthorityUnderGravity(Vec3{LambdaX * GravityMPerS2, 0.0, -GravityMPerS2});
}

inline StrengthProfile BondedJoint(double TensileStrengthPa, double ShearCohesionPa, double Friction)
{
	StrengthProfile Profile = DryJoint(Friction);
	Profile.TensileStrengthPa = TensileStrengthPa;
	Profile.ShearCohesionPa = ShearCohesionPa;
	return Profile;
}

inline StrengthProfile CrushedByOwnWeight()
{
	StrengthProfile Profile = BondedJoint(1.0e4, 1.0e4, 0.7);
	Profile.CompressiveStrengthPa = 0.5 * BrickMassKg * GravityMPerS2 / (BrickLengthM * BrickWidthM);
	return Profile;
}

template <class Id>
bool SameIdList(const std::vector<Id>& Got, const std::vector<Id>& Want)
{
	if (Got.size() != Want.size())
	{
		return false;
	}
	for (std::size_t I = 0; I < Got.size(); ++I)
	{
		if (!SameHandle(Got[I], Want[I]))
		{
			return false;
		}
	}
	return true;
}

inline std::string DescribeAuthority(const AuthorityResult& Result)
{
	std::ostringstream Out;
	Out << "worst " << VerdictName(Result.Worst);
	for (const ComponentReport& Report : Result.Components)
	{
		Out << "; [bodies " << Describe(Report.Bodies) << ", verdict " << VerdictName(Report.Verdict) << ", elastic " << StandingStateName(Report.Elastic) << ", release " << Describe(Report.Release) << "]";
	}
	return Out.str();
}

inline std::string DescribeVerdict(const CascadeVerdict& V)
{
	std::ostringstream Out;
	Out << "state " << CascadeStateName(V.State) << ", passes " << V.Passes << ", first " << V.FirstPass << ", last " << V.LastPass << ", failures [";
	for (const FailedInterface& F : V.Failures)
	{
		Out << " {" << F.Id.Index << " pass " << F.Pass << " " << GoverningAxisName(F.Axis) << " " << F.Ratio << (F.Kind == FailureKind::Crushed ? " crushed" : F.bSevered ? " severed" : " cracked") << "}";
	}
	Out << " ], crushed [";
	for (const CrushedBody& C : V.CrushedBodies)
	{
		Out << " {pass " << C.Pass << " body " << C.Body.Index << " at " << C.Interface.Index << " ratio " << C.Ratio << "}";
	}
	Out << " ], clusters [";
	for (const ReleasedCluster& C : V.Clusters)
	{
		Out << " {pass " << C.Pass << " bodies " << Describe(C.Bodies) << " severed " << Describe(C.Severed) << "}";
	}
	Out << " ], readings " << V.Readings.size() << ", components " << V.Components.size();
	return Out.str();
}

inline std::string DescribeReadings(const std::vector<InterfaceReading>& Readings)
{
	std::ostringstream Out;
	for (const InterfaceReading& R : Readings)
	{
		Out << " {" << R.Id.Index << " u " << R.Utilisation << " " << GoverningAxisName(R.Axis) << " c " << R.CompressionUtilisation << " t " << R.TensionUtilisation << " s " << R.ShearUtilisation << "}";
	}
	return Out.str();
}

inline void CheckStamps(const Structure& Model, InterfaceId Id, InterfaceState State, std::uint32_t CrackPass, std::uint32_t SeverPass)
{
	const InterfaceRecord& R = Model.Interface(Id);
	INFO("interface " << Id.Index << ": state " << doctest::String(InterfaceStateName(R.State)) << ", crack " << R.CrackPass << ", sever " << R.SeverPass << "; want " << doctest::String(InterfaceStateName(State)) << ", " << CrackPass << ", " << SeverPass);
	CHECK(R.State == State);
	CHECK(R.CrackPass == CrackPass);
	CHECK(R.SeverPass == SeverPass);
}

struct ScriptedJoint
{
	InterfaceId Id;
	double TensionUtilisation = 0.0;
	double ShearUtilisation = 0.0;
	double CompressionUtilisation = 0.0;
};

// Returns Script[call] on each Judge, repeating the last entry once the script runs out.
class ScriptedAuthority final : public IStructuralAuthority
{
public:
	explicit ScriptedAuthority(std::vector<AuthorityResult> InScript)
		: Script(std::move(InScript))
	{
	}

	AuthorityResult Judge(const Structure&, std::span<const BodyLoad>, const AuthorityOptions&, const AuthorityResult*) const override
	{
		const std::size_t At = Calls < Script.size() ? Calls : Script.size() - 1;
		++Calls;
		return Script[At];
	}

	std::vector<AuthorityResult> Script;
	mutable std::size_t Calls = 0;
};

inline AuthorityResult OverstressedScript(const Structure& Model, const std::vector<ScriptedJoint>& Joints)
{
	AuthorityResult Result;
	Result.Worst = ComponentVerdict::Overstressed;
	ComponentReport Component;
	for (const BodyId Id : Model.LiveBodies())
	{
		if (!Model.Body(Id).bGrounded)
		{
			Component.Bodies.push_back(Id);
		}
	}
	Component.Verdict = ComponentVerdict::Overstressed;
	Component.Elastic = StandingState::Overstressed;
	Result.Components.push_back(Component);
	for (const ScriptedJoint& Joint : Joints)
	{
		InterfaceReading Reading;
		Reading.Id = Joint.Id;
		Reading.TensionUtilisation = Joint.TensionUtilisation;
		Reading.ShearUtilisation = Joint.ShearUtilisation;
		const bool bTension = !(Joint.TensionUtilisation < Joint.ShearUtilisation);
		Reading.Utilisation = bTension ? Joint.TensionUtilisation : Joint.ShearUtilisation;
		Reading.Axis = bTension ? GoverningAxis::Tension : GoverningAxis::Shear;
		Reading.CompressionUtilisation = Joint.CompressionUtilisation;
		if (Joint.CompressionUtilisation > Reading.Utilisation)
		{
			Reading.Utilisation = Joint.CompressionUtilisation;
			Reading.Axis = GoverningAxis::Compression;
		}
		Result.Readings.push_back(Reading);
	}
	return Result;
}

inline AuthorityResult CollapsesScript(const Structure& Model, const std::vector<BodyId>& Release)
{
	AuthorityResult Result = OverstressedScript(Model, {});
	Result.Worst = ComponentVerdict::Collapses;
	Result.Components[0].Verdict = ComponentVerdict::Collapses;
	Result.Components[0].Elastic = StandingState::Mechanism;
	Result.Components[0].Release = Release;
	for (const InterfaceId Id : Model.LiveInterfaces())
	{
		InterfaceReading Reading;
		Reading.Id = Id;
		Result.Readings.push_back(Reading);
	}
	return Result;
}

inline AuthorityResult StandsScript()
{
	AuthorityResult Result;
	Result.Worst = ComponentVerdict::Stands;
	return Result;
}

inline CascadeVerdict RunScripted(Structure& Model, std::vector<AuthorityResult> Script, double CrackBandFraction = 0.5)
{
	CascadeOptions Options;
	Options.CrackBandFraction = CrackBandFraction;
	return MakeCascade(std::make_unique<ScriptedAuthority>(std::move(Script)))->Run(Model, {}, Options, nullptr);
}
}
