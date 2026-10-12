#pragma once

#include "CascadeTestSupport.h"
#include "CatalogueTestSupport.h"
#include "StaticAdmissibilityOracle.h"
#include "StaticsTestSupport.h"

#include "loadpath/Catalogue.h"
#include "loadpath/geometry/RunningBondWall.h"

#include "doctest.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

namespace loadpath::test
{
using Clock = std::chrono::steady_clock;

inline double SecondsSince(Clock::time_point Start) { return std::chrono::duration<double>(Clock::now() - Start).count(); }

struct FactorCase
{
	std::string Name;
	Structure Model;
	StandingOptions Options;
	std::vector<BodyLoad> Loads;
};

inline double WeightN(const Structure& Model)
{
	double Sum = 0.0;
	for (const BodyId& Id : Model.LiveBodies())
	{
		if (!Model.Body(Id).bGrounded)
		{
			Sum += Model.Body(Id).MassKg * GravityMPerS2;
		}
	}
	return Sum;
}

struct Gaps
{
	double WorstRelative = 0.0;
	double WorstForceAbsoluteOverW = 0.0;
	double WorstUtilisationAbsolute = 0.0;
};

inline bool Near(double Got, double Want, double AbsoluteFloor, Gaps& G, bool bForce, double WeightScale)
{
	if (std::isnan(Got) || std::isnan(Want))
	{
		return std::isnan(Got) && std::isnan(Want);
	}
	if (Got == Want)
	{
		return true;
	}
	const double Gap = std::abs(Got - Want);
	const double Scale = std::max(std::abs(Got), std::abs(Want));
	if (std::isfinite(Gap))
	{
		if (Gap > AbsoluteFloor)
		{
			G.WorstRelative = std::max(G.WorstRelative, Gap / Scale);
		}
		if (bForce)
		{
			G.WorstForceAbsoluteOverW = std::max(G.WorstForceAbsoluteOverW, Gap / WeightScale);
		}
		else
		{
			G.WorstUtilisationAbsolute = std::max(G.WorstUtilisationAbsolute, Gap);
		}
	}
	return Gap <= 1e-7 * Scale + AbsoluteFloor;
}

inline std::vector<BodyId> MechanismBodies(const StandingResult& R)
{
	std::vector<BodyId> Out;
	for (const MechanismMotion& M : R.Mechanism)
	{
		Out.push_back(M.Body);
	}
	return Out;
}

inline bool Certified(StandingState S) { return S == StandingState::Stands || S == StandingState::Overstressed || S == StandingState::InvalidInput; }

struct Comparison
{
	int Mismatches = 0;
	bool bStateClassTraded = false;
	bool bIterationsDiffer = false;
	bool bCertified = false;
	int MotionMismatches = 0;
	double WorstMotionGap = 0.0;
	Gaps G;
	StandingState StateA = StandingState::Undecided;
	StandingState StateB = StandingState::Undecided;
};

inline Comparison CompareOptions(const FactorCase& C, const StandingOptions& OptionsA, const StandingOptions& OptionsB, const std::string& NameA, const std::string& NameB, bool bAllowTrade)
{
	const StandingResult Ref = SolveWith(C.Model, OptionsA, C.Loads);
	const StandingResult Blk = SolveWith(C.Model, OptionsB, C.Loads);
	const double W = WeightN(C.Model);
	const double ForceFloorN = 1e-9 * W;
	const double WeightScale = W > 0.0 ? W : 1.0;
	INFO(C.Name << ": " << NameA << " " << Ref.State << " after " << Ref.ActiveSetIterations << ", " << NameB << " " << Blk.State << " after " << Blk.ActiveSetIterations);
	Comparison Out;
	Out.StateA = Ref.State;
	Out.StateB = Blk.State;
	auto Expect = [&](bool bOk, const std::string& What) {
		if (!bOk)
		{
			++Out.Mismatches;
			FAIL_CHECK(C.Name << ": " << What);
		}
	};
	const bool bSameState = Ref.State == Blk.State;
	Out.bCertified = bSameState && (Ref.State == StandingState::Stands || Ref.State == StandingState::Overstressed);
	if (!bSameState && !Certified(Ref.State) && !Certified(Blk.State))
	{
		Out.bStateClassTraded = true;
		Expect(bAllowTrade, "mechanism and undecided traded on a fixture that allows no trade");
	}
	else
	{
		Expect(bSameState, "state");
	}
	Out.bIterationsDiffer = Ref.ActiveSetIterations != Blk.ActiveSetIterations;
	Expect(!Out.bCertified || std::abs(Ref.ActiveSetIterations - Blk.ActiveSetIterations) <= 2, "active-set iterations differ by more than 2");
	if (!bSameState || Ref.State == StandingState::Undecided)
	{
		return Out;
	}
	Expect(SameIdList(Ref.UnconstrainedBodies, Blk.UnconstrainedBodies), "unconstrained bodies");
	Expect(SameIdList(MechanismBodies(Ref), MechanismBodies(Blk)), "mechanism bodies");
	if (Ref.State == StandingState::Mechanism)
	{
		for (const MechanismMotion& A : Ref.Mechanism)
		{
			const auto Match = std::find_if(Blk.Mechanism.begin(), Blk.Mechanism.end(), [&](const MechanismMotion& B) { return SameHandle(A.Body, B.Body); });
			if (Match == Blk.Mechanism.end())
			{
				++Out.MotionMismatches;
				Expect(false, "mechanism body missing from the " + NameB + " motion");
				continue;
			}
			const double Gap = std::max({std::abs(A.TranslationM.X - Match->TranslationM.X), std::abs(A.TranslationM.Y - Match->TranslationM.Y), std::abs(A.TranslationM.Z - Match->TranslationM.Z), std::abs(A.RotationRad.X - Match->RotationRad.X), std::abs(A.RotationRad.Y - Match->RotationRad.Y), std::abs(A.RotationRad.Z - Match->RotationRad.Z)});
			Out.WorstMotionGap = std::max(Out.WorstMotionGap, std::isnan(Gap) ? HUGE_VAL : Gap);
			if (!(Gap <= 1e-6))
			{
				++Out.MotionMismatches;
				Expect(false, "mechanism motion differs by " + std::to_string(Gap));
			}
		}
		for (const MechanismMotion& B : Blk.Mechanism)
		{
			if (std::none_of(Ref.Mechanism.begin(), Ref.Mechanism.end(), [&](const MechanismMotion& A) { return SameHandle(A.Body, B.Body); }))
			{
				++Out.MotionMismatches;
				Expect(false, "mechanism body missing from the " + NameA + " motion");
			}
		}
	}
	Expect(Ref.Readings.size() == Blk.Readings.size(), "reading count");
	auto Force = [&](double X, double Y) { return Near(X, Y, ForceFloorN, Out.G, true, WeightScale); };
	auto Util = [&](double X, double Y) { return Near(X, Y, 1e-9, Out.G, false, WeightScale); };
	for (std::size_t I = 0; I < Ref.Readings.size() && I < Blk.Readings.size(); ++I)
	{
		const InterfaceReading& A = Ref.Readings[I];
		const InterfaceReading& B = Blk.Readings[I];
		const std::string At = "reading " + std::to_string(I) + " ";
		Expect(SameHandle(A.Id, B.Id), At + "id");
		for (std::size_t K = 0; K < 4; ++K)
		{
			Expect(!Out.bCertified || A.Corners[K].bOpen == B.Corners[K].bOpen, At + "corner " + std::to_string(K) + " open flag");
		}
		if (!Out.bCertified)
		{
			Force(A.NormalForceN, B.NormalForceN);
			Util(A.Utilisation, B.Utilisation);
			continue;
		}
		Expect(Force(A.NormalForceN, B.NormalForceN), At + "normal force " + std::to_string(A.NormalForceN) + " vs " + std::to_string(B.NormalForceN));
		const bool bShearX = Force(A.ShearForceN.X, B.ShearForceN.X);
		const bool bShearY = Force(A.ShearForceN.Y, B.ShearForceN.Y);
		const bool bShearZ = Force(A.ShearForceN.Z, B.ShearForceN.Z);
		Expect(bShearX && bShearY && bShearZ, At + "shear force");
		Expect(Util(A.Utilisation, B.Utilisation), At + "utilisation");
		Expect(Util(A.CompressionUtilisation, B.CompressionUtilisation), At + "compression utilisation");
		Expect(Util(A.TensionUtilisation, B.TensionUtilisation), At + "tension utilisation");
		Expect(Util(A.ShearUtilisation, B.ShearUtilisation), At + "shear utilisation");
	}
	return Out;
}

inline StandingOptions Biaxial(double LambdaX, double LambdaY)
{
	StandingOptions Options;
	Options.Gravity = oracle::BiaxialGravity(LambdaX, LambdaY);
	return Options;
}

inline Structure RunningBondWall(const StrengthProfile& Bed, const StrengthProfile& Perpend, int Courses, int BricksPerCourse, geometry::Wall* Out = nullptr)
{
	Structure Model;
	geometry::WallSpec Spec;
	Spec.Bed = AddProfileOk(Model, Bed);
	Spec.Perpend = AddProfileOk(Model, Perpend);
	Spec.Courses = Courses;
	Spec.BricksPerCourse = BricksPerCourse;
	const geometry::Wall W = geometry::BuildRunningBondWall(Model, Spec);
	if (Out != nullptr)
	{
		*Out = W;
	}
	return Model;
}

inline Structure Ec6Wall(int Courses, int BricksPerCourse)
{
	const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
	INFO("line " << Loaded.Error.Line << ": " << Loaded.Error.Message);
	REQUIRE(Loaded.Ok());
	return RunningBondWall(JointOrFail(Loaded.Value, "clay_gp_mortar_bed_ec6").Profile, JointOrFail(Loaded.Value, "clay_gp_mortar_perpend_ec6").Profile, Courses, BricksPerCourse);
}

}
