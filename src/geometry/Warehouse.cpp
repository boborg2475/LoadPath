#include "loadpath/geometry/Warehouse.h"

#include "loadpath/geometry/BoxContacts.h"

#include <optional>

namespace loadpath::geometry
{
namespace
{
bool InOpening(const WarehouseSpec& Spec, int W, int Course, double S0, double S1)
{
	for (const Opening& O : Spec.Openings)
	{
		if (O.Wall == W && Course >= O.SillCourse && Course < O.LintelCourse && S0 >= O.StartBrick - 1e-9 && S1 <= O.StartBrick + O.WidthBricks + 1e-9)
		{
			return true;
		}
	}
	return false;
}

double Volume(const Box& B) { return 8.0 * B.HalfExtentsM.X * B.HalfExtentsM.Y * B.HalfExtentsM.Z; }
}

std::size_t Warehouse::BodyCount() const { return LiveBodies; }

std::size_t Warehouse::InterfaceCount() const { return LiveInterfaces; }

Warehouse BuildWarehouse(Structure& Model, const WarehouseSpec& Spec)
{
	Warehouse Out;
	const double L = Spec.BrickM.X, T = Spec.BrickM.Y, H = Spec.BrickM.Z, G = Spec.GroundZM, Gap = Spec.CornerGapM;
	const double Lx = Spec.LengthBricks * L, Wy = Spec.WidthBricks * L;
	const Result<BodyId> GroundAdded = Model.AddBody(BodyDesc{0.0, Vec3{0.5 * Lx, 0.5 * Wy, G - 0.5 * H}, true});
	Out.bComplete = GroundAdded.Ok();
	Out.Ground = GroundAdded.Value;

	const Vec3 Origins[4] = {Vec3{0.0, 0.5 * T, 0.0}, Vec3{-Gap - 0.5 * T, 0.0, 0.0}, Vec3{0.0, Wy - 0.5 * T, 0.0}, Vec3{Lx + Gap + 0.5 * T, 0.0, 0.0}};
	for (int W = 0; W < 4; ++W)
	{
		WallSpec WS;
		WS.Courses = Spec.Courses;
		WS.BricksPerCourse = W % 2 == 0 ? Spec.LengthBricks : Spec.WidthBricks;
		WS.BrickM = Spec.BrickM;
		WS.DensityKgPerM3 = Spec.DensityKgPerM3;
		WS.Bed = Spec.Bed;
		WS.Perpend = Spec.Perpend;
		WS.GroundZM = G;
		WS.OriginM = Origins[W];
		WS.bAlongY = W % 2 == 1;
		WS.Ground = Out.Ground;
		Out.Walls[W] = BuildRunningBondWall(Model, WS);
		Out.bComplete = Out.bComplete && Out.Walls[W].bComplete;
	}

	for (int W = 0; W < 4; ++W)
	{
		const int M = W % 2 == 0 ? Spec.LengthBricks : Spec.WidthBricks;
		for (int C = 0; C < static_cast<int>(Out.Walls[W].Courses.size()); ++C)
		{
			std::vector<BodyId>& Course = Out.Walls[W].Courses[C];
			std::vector<BodyId> Kept;
			const bool bOffset = C % 2 == 1;
			for (int K = 0; K < static_cast<int>(Course.size()); ++K)
			{
				const double S0 = bOffset ? (K == 0 ? 0.0 : K - 0.5) : K;
				const double S1 = bOffset ? (K == M ? static_cast<double>(M) : K + 0.5) : K + 1.0;
				if (InOpening(Spec, W, C, S0, S1))
				{
					Model.RemoveBody(Course[K]);
				}
				else
				{
					Kept.push_back(Course[K]);
				}
			}
			Course = Kept;
		}
	}

	const auto Join = [&](BodyId A, const Box& BoxA, BodyId B, const Box& BoxB, ProfileId Profile, std::vector<InterfaceId>& Into) {
		const std::optional<InterfaceDesc> Desc = FaceContact(A, BoxA, B, BoxB, Profile);
		const Result<InterfaceId> Added = Desc ? Model.AddInterface(*Desc) : Result<InterfaceId>{};
		if (Desc && Added.Ok())
		{
			Into.push_back(Added.Value);
		}
		else
		{
			Out.bComplete = false;
		}
	};
	const auto Add = [&](const Box& B, double DensityKgPerM3, std::vector<BodyId>& Into) {
		const Result<BodyId> Added = Model.AddBody(BodyDesc{DensityKgPerM3 * Volume(B), B.CentreM, false});
		Out.bComplete = Out.bComplete && Added.Ok();
		Into.push_back(Added.Value);
	};

	const double P = Spec.PostSideM, Hp = Spec.Courses * H, Span = (Lx - P) / Spec.Bays;
	const double YSide[2] = {T + P, Wy - T - P};
	std::vector<Box> PostBoxes, BeamBoxes;
	for (int Side = 0; Side < 2; ++Side)
	{
		for (int K = 0; K <= Spec.Bays; ++K)
		{
			const Box B{Vec3{0.5 * P + K * Span, YSide[Side], G + 0.5 * Hp}, Vec3{0.5 * P, 0.5 * P, 0.5 * Hp}};
			Add(B, Spec.TimberDensityKgPerM3, Out.Posts);
			PostBoxes.push_back(B);
			const Box Under{Vec3{B.CentreM.X, B.CentreM.Y, G - 0.5 * H}, Vec3{0.5 * P, 0.5 * P, 0.5 * H}};
			Join(Out.Ground, Under, Out.Posts.back(), B, Spec.Bearing, Out.Bearings);
		}
	}
	for (int Side = 0; Side < 2; ++Side)
	{
		for (int J = 0; J < Spec.Bays; ++J)
		{
			const Box B{Vec3{0.5 * P + (J + 0.5) * Span, YSide[Side], G + Hp + 0.5 * Spec.BeamDepthM}, Vec3{0.5 * Span, 0.5 * Spec.BeamWidthM, 0.5 * Spec.BeamDepthM}};
			Add(B, Spec.TimberDensityKgPerM3, Out.Beams);
			BeamBoxes.push_back(B);
			const std::size_t Left = static_cast<std::size_t>(Side * (Spec.Bays + 1) + J);
			Join(Out.Posts[Left], PostBoxes[Left], Out.Beams.back(), B, Spec.Bearing, Out.Bearings);
			Join(Out.Posts[Left + 1], PostBoxes[Left + 1], Out.Beams.back(), B, Spec.Bearing, Out.Bearings);
		}
	}
	const double PlankLengthM = YSide[1] - YSide[0] + Spec.BeamWidthM;
	for (int J = 0; J < Spec.Bays; ++J)
	{
		for (int I = 0; I < Spec.PlanksPerBay; ++I)
		{
			const Box B{Vec3{0.5 * P + J * Span + (I + 0.5) * Span / Spec.PlanksPerBay, 0.5 * (YSide[0] + YSide[1]), G + Hp + Spec.BeamDepthM + 0.5 * Spec.PlankThicknessM},
				Vec3{0.5 * Spec.PlankWidthM, 0.5 * PlankLengthM, 0.5 * Spec.PlankThicknessM}};
			Add(B, Spec.TimberDensityKgPerM3, Out.Planks);
			const std::size_t Left = static_cast<std::size_t>(J), Right = static_cast<std::size_t>(Spec.Bays + J);
			Join(Out.Beams[Left], BeamBoxes[Left], Out.Planks.back(), B, Spec.Plank, Out.PlankJoints);
			Join(Out.Beams[Right], BeamBoxes[Right], Out.Planks.back(), B, Spec.Plank, Out.PlankJoints);
		}
	}
	Out.LiveBodies = Model.IsLive(Out.Ground) ? 1 : 0;
	const auto CountBodies = [&](const std::vector<BodyId>& Ids) {
		for (const BodyId& Id : Ids)
		{
			Out.LiveBodies += Model.IsLive(Id) ? 1 : 0;
		}
	};
	const auto CountInterfaces = [&](const std::vector<InterfaceId>& Ids) {
		for (const InterfaceId& Id : Ids)
		{
			Out.LiveInterfaces += Model.IsLive(Id) ? 1 : 0;
		}
	};
	for (const Wall& W : Out.Walls)
	{
		for (const std::vector<BodyId>& Course : W.Courses)
		{
			CountBodies(Course);
		}
		CountInterfaces(W.Beds);
		CountInterfaces(W.Perpends);
	}
	CountBodies(Out.Posts);
	CountBodies(Out.Beams);
	CountBodies(Out.Planks);
	CountInterfaces(Out.Bearings);
	CountInterfaces(Out.PlankJoints);
	return Out;
}
}
