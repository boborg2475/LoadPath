#pragma once

#include "StaticsTestSupport.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <span>
#include <vector>

namespace loadpath::test
{
inline constexpr double OwnFieldRelative = 1.0e-9;
inline constexpr double OwnFieldMomentArmM = 0.1;

struct OwnFieldCheck
{
	int Violations = 0;
	double WorstResidualOverW = 0.0;
	double WorstTensionOverW = 0.0;
	double WorstConeExcessOverW = 0.0;
};

// Sums gravity, host loads and every reading corner's force on each free body of a Stands result, recomputed here from the readings alone.
inline OwnFieldCheck CheckOwnField(const Structure& Model, const StandingResult& Result, const Vec3& GravityVectorMPerS2, std::span<const BodyLoad> Loads = {})
{
	OwnFieldCheck Out;
	std::size_t Slots = 0;
	for (const BodyId& Body : Model.LiveBodies())
	{
		Slots = std::max<std::size_t>(Slots, static_cast<std::size_t>(Body.Index) + 1);
	}
	std::vector<std::array<double, 6>> Sum(Slots, std::array<double, 6>{});
	std::vector<double> WeightOfSlot(Slots, 0.0);
	const auto Add = [&](BodyId Body, const Vec3& PointM, const Vec3& ForceN) {
		const BodyDesc& Desc = Model.Body(Body);
		if (Desc.bGrounded)
		{
			return;
		}
		const Vec3 MomentNm = Cross(Minus(PointM, Desc.CentroidM), ForceN);
		std::array<double, 6>& S = Sum[Body.Index];
		S[0] += ForceN.X;
		S[1] += ForceN.Y;
		S[2] += ForceN.Z;
		S[3] += MomentNm.X / OwnFieldMomentArmM;
		S[4] += MomentNm.Y / OwnFieldMomentArmM;
		S[5] += MomentNm.Z / OwnFieldMomentArmM;
	};
	for (const StandingComponent& Component : Result.Components)
	{
		double WeightN = 0.0;
		for (const BodyId& Body : Component.Bodies)
		{
			WeightN += Model.Body(Body).MassKg * GravityMPerS2;
		}
		for (const BodyId& Body : Component.Bodies)
		{
			WeightOfSlot[Body.Index] = WeightN;
			Add(Body, Model.Body(Body).CentroidM, Times(Model.Body(Body).MassKg, GravityVectorMPerS2));
		}
	}
	for (const BodyLoad& Load : Loads)
	{
		Add(Load.Body, Load.PointM, Load.ForceN);
	}
	for (const InterfaceReading& Reading : Result.Readings)
	{
		const InterfaceRecord& Record = Model.Interface(Reading.Id);
		const StrengthProfile Profile = Model.EffectiveProfile(Reading.Id);
		const bool bDry = !(Profile.TensileStrengthPa > 0.0) && !(Profile.ShearCohesionPa > 0.0);
		const double WeightN = std::max(Model.Body(Record.BodyA).bGrounded ? 0.0 : WeightOfSlot[Record.BodyA.Index], Model.Body(Record.BodyB).bGrounded ? 0.0 : WeightOfSlot[Record.BodyB.Index]);
		for (const CornerReading& Corner : Reading.Corners)
		{
			const Vec3 ForceN = Plus(Times(Corner.NormalForceN, Record.NormalAToB), Corner.ShearForceN);
			Add(Record.BodyB, Corner.PointM, ForceN);
			Add(Record.BodyA, Corner.PointM, Times(-1.0, ForceN));
			if (!bDry || !(WeightN > 0.0))
			{
				continue;
			}
			const double TensionOverW = -Corner.NormalForceN / WeightN;
			const double ConeExcessOverW = (Length(Corner.ShearForceN) - Profile.FrictionCoefficient * Corner.NormalForceN) / WeightN;
			Out.WorstTensionOverW = std::max(Out.WorstTensionOverW, std::isnan(TensionOverW) ? HUGE_VAL : TensionOverW);
			Out.WorstConeExcessOverW = std::max(Out.WorstConeExcessOverW, std::isnan(ConeExcessOverW) ? HUGE_VAL : ConeExcessOverW);
			Out.Violations += (TensionOverW <= OwnFieldRelative && ConeExcessOverW <= OwnFieldRelative) ? 0 : 1;
		}
	}
	for (const StandingComponent& Component : Result.Components)
	{
		for (const BodyId& Body : Component.Bodies)
		{
			const std::array<double, 6>& S = Sum[Body.Index];
			double SquaredN2 = 0.0;
			for (const double Entry : S)
			{
				SquaredN2 += Entry * Entry;
			}
			const double ResidualOverW = std::sqrt(SquaredN2) / WeightOfSlot[Body.Index];
			Out.WorstResidualOverW = std::max(Out.WorstResidualOverW, std::isnan(ResidualOverW) ? HUGE_VAL : ResidualOverW);
			Out.Violations += ResidualOverW <= OwnFieldRelative ? 0 : 1;
		}
	}
	return Out;
}
}
