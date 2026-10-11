#pragma once

#include "StaticsTestSupport.h"

#include <cstddef>
#include <optional>
#include <vector>

namespace loadpath::test
{
inline constexpr double RockDensityKgPerM3 = 2000.0;

inline double BrickDensityKgPerM3()
{
	return BrickMassKg / (BrickLengthM * BrickWidthM * BrickHeightM);
}

struct BlockOnGround
{
	Structure Model;
	BodyId Block;
	InterfaceId Bed;
	Box Shape;
};

inline BlockOnGround MakeBlockOnGround(double WidthXM, double DepthYM, double HeightZM, double Friction, double DensityKgPerM3 = RockDensityKgPerM3)
{
	BlockOnGround F;
	const ProfileId P = AddProfileOk(F.Model, DryJoint(Friction));
	const BodyId Ground = AddGroundPlane(F.Model);
	F.Shape = BoxOfDensity(Vec3{0.0, 0.0, HeightZM / 2.0}, Vec3{WidthXM / 2.0, DepthYM / 2.0, HeightZM / 2.0}, DensityKgPerM3);
	F.Block = AddBox(F.Model, F.Shape);
	F.Bed = AddInterfaceOk(F.Model, OnGroundPlane(Ground, F.Block, F.Shape, P));
	return F;
}

inline Vec3 DownhillEdgePointM(const Box& Shape)
{
	return Vec3{Shape.CentreM.X + Shape.HalfExtentsM.X, Shape.CentreM.Y, Shape.CentreM.Z - Shape.HalfExtentsM.Z};
}

struct WallCourse
{
	BodyId Body;
	Box Shape;
};

struct CoursedWall
{
	Structure Model;
	std::vector<WallCourse> Courses;
	std::vector<InterfaceId> JointsBottomUp;
};

inline CoursedWall MakeCoursedWall(int CourseCount, double ThicknessXM, double LengthYM, double HeightZM, double Friction)
{
	CoursedWall W;
	const ProfileId P = AddProfileOk(W.Model, DryJoint(Friction));
	const BodyId Ground = AddGroundPlane(W.Model);
	const double CourseHeightM = HeightZM / CourseCount;
	for (int K = 0; K < CourseCount; ++K)
	{
		const Box Shape = BoxOfDensity(Vec3{0.0, 0.0, (K + 0.5) * CourseHeightM}, Vec3{ThicknessXM / 2.0, LengthYM / 2.0, CourseHeightM / 2.0}, BrickDensityKgPerM3());
		W.Courses.push_back(WallCourse{AddBox(W.Model, Shape), Shape});
	}
	W.JointsBottomUp.push_back(AddInterfaceOk(W.Model, OnGroundPlane(Ground, W.Courses[0].Body, W.Courses[0].Shape, P)));
	for (std::size_t K = 1; K < W.Courses.size(); ++K)
	{
		W.JointsBottomUp.push_back(AddInterfaceOk(W.Model, *StackedFaces(W.Courses[K - 1].Body, W.Courses[K - 1].Shape, W.Courses[K].Body, W.Courses[K].Shape, P)));
	}
	return W;
}

inline constexpr double BlockLengthM = 1.0;
inline constexpr double BlockHeightM = 0.2;
inline constexpr double BlockDepthM = 0.5;
inline constexpr double BlockMassKg = 1.0;
inline constexpr double TableStackFriction = 0.5;

struct PlacedBlock
{
	double CentreXM;
	int Row;
};

inline Box BlockShape(const PlacedBlock& Block)
{
	return Box{Vec3{Block.CentreXM, 0.0, (Block.Row + 0.5) * BlockHeightM}, Vec3{BlockLengthM / 2.0, BlockDepthM / 2.0, BlockHeightM / 2.0}, BlockMassKg};
}

struct Stack
{
	Structure Model;
	std::vector<BodyId> Blocks;
};

inline Stack BuildOnTable(const std::vector<PlacedBlock>& Placed, double TableEdgeXM)
{
	Stack S;
	const ProfileId P = AddProfileOk(S.Model, DryJoint(TableStackFriction));
	const Box Table{Vec3{TableEdgeXM - 5.0, 0.0, -0.5}, Vec3{5.0, 5.0, 0.5}, 0.0};
	const BodyId TableId = AddBox(S.Model, Table, true);
	for (const PlacedBlock& Block : Placed)
	{
		S.Blocks.push_back(AddBox(S.Model, BlockShape(Block)));
	}
	for (std::size_t Upper = 0; Upper < Placed.size(); ++Upper)
	{
		if (Placed[Upper].Row == 0)
		{
			AddInterfaceOk(S.Model, *StackedFaces(TableId, Table, S.Blocks[Upper], BlockShape(Placed[Upper]), P));
			continue;
		}
		for (std::size_t Lower = 0; Lower < Placed.size(); ++Lower)
		{
			if (Placed[Lower].Row + 1 != Placed[Upper].Row)
			{
				continue;
			}
			const std::optional<InterfaceDesc> Contact = StackedFaces(S.Blocks[Lower], BlockShape(Placed[Lower]), S.Blocks[Upper], BlockShape(Placed[Upper]), P);
			if (Contact)
			{
				AddInterfaceOk(S.Model, *Contact);
			}
		}
	}
	return S;
}

inline std::vector<double> HarmonicRightEdgesFromTopM(int Count, double OffsetScale)
{
	std::vector<double> RightEdges(static_cast<std::size_t>(Count));
	double Edge = 0.0;
	for (int K = Count; K >= 1; --K)
	{
		Edge += OffsetScale * BlockLengthM / (2.0 * K);
		RightEdges[static_cast<std::size_t>(K - 1)] = Edge;
	}
	return RightEdges;
}

inline std::vector<PlacedBlock> HarmonicStack(int Count, double OffsetScale)
{
	const std::vector<double> RightEdges = HarmonicRightEdgesFromTopM(Count, OffsetScale);
	std::vector<PlacedBlock> Placed;
	for (int K = 1; K <= Count; ++K)
	{
		Placed.push_back(PlacedBlock{RightEdges[static_cast<std::size_t>(K - 1)] - BlockLengthM / 2.0, Count - K});
	}
	return Placed;
}

inline std::vector<PlacedBlock> BrickRows(const std::vector<int>& CountsBottomUp)
{
	std::vector<PlacedBlock> Placed;
	for (std::size_t Row = 0; Row < CountsBottomUp.size(); ++Row)
	{
		const int Count = CountsBottomUp[Row];
		for (int J = 0; J < Count; ++J)
		{
			Placed.push_back(PlacedBlock{(J - (Count - 1) / 2.0) * BlockLengthM, static_cast<int>(Row)});
		}
	}
	return Placed;
}
}
