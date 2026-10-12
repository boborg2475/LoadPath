#include "loadpath/geometry/RunningBondWall.h"

#include "loadpath/geometry/BoxContacts.h"

#include <cstddef>
#include <optional>

namespace loadpath::geometry
{
namespace
{
Box Place(const WallSpec& Spec, double S, double HalfS, double Z, double HalfZ)
{
	const double HalfT = 0.5 * Spec.BrickM.Y;
	if (Spec.bAlongY)
	{
		return Box{Vec3{Spec.OriginM.X, Spec.OriginM.Y + S, Spec.OriginM.Z + Z}, Vec3{HalfT, HalfS, HalfZ}};
	}
	return Box{Vec3{Spec.OriginM.X + S, Spec.OriginM.Y, Spec.OriginM.Z + Z}, Vec3{HalfS, HalfT, HalfZ}};
}
}

Wall BuildRunningBondWall(Structure& Model, const WallSpec& Spec)
{
	Wall Out;
	const Vec3 Brick = Spec.BrickM;
	Out.LengthM = Spec.BricksPerCourse * Brick.X;
	Out.HeightM = Spec.Courses * Brick.Z;
	const double FullKg = Spec.DensityKgPerM3 * Brick.X * Brick.Y * Brick.Z;

	const auto Keep = [&Out](const auto& Added, auto& Into) {
		if (Added.Ok())
		{
			Into.push_back(Added.Value);
		}
		else
		{
			Out.bComplete = false;
		}
	};
	if (Spec.Ground == BodyId::Invalid())
	{
		const Box GroundBox = Place(Spec, 0.5 * Out.LengthM, 0.5 * Out.LengthM, Spec.GroundZM - 0.5 * Brick.Z, 0.5 * Brick.Z);
		const Result<BodyId> GroundAdded = Model.AddBody(BodyDesc{0.0, GroundBox.CentreM, true});
		Out.bComplete = GroundAdded.Ok();
		Out.Ground = GroundAdded.Value;
	}
	else
	{
		Out.bComplete = Model.IsLive(Spec.Ground);
		Out.Ground = Spec.Ground;
	}

	std::vector<std::vector<Box>> Boxes;
	for (int C = 0; C < Spec.Courses; ++C)
	{
		const double Z = Spec.GroundZM + (C + 0.5) * Brick.Z;
		const bool bOffset = Spec.bRunningBond && C % 2 == 1;
		std::vector<double> Lengths;
		if (bOffset)
		{
			Lengths.push_back(0.5 * Brick.X);
		}
		for (int K = 0; K < Spec.BricksPerCourse - (bOffset ? 1 : 0); ++K)
		{
			Lengths.push_back(Brick.X);
		}
		if (bOffset)
		{
			Lengths.push_back(0.5 * Brick.X);
		}
		std::vector<BodyId> Course;
		std::vector<Box> CourseBoxes;
		double X = 0.0;
		for (const double L : Lengths)
		{
			const Box B = Place(Spec, X + 0.5 * L, 0.5 * L, Z, 0.5 * Brick.Z);
			X += L;
			const Result<BodyId> Added = Model.AddBody(BodyDesc{FullKg * L / Brick.X, B.CentreM, false});
			Out.bComplete = Out.bComplete && Added.Ok();
			const BodyId Id = Added.Value;
			if (C == 0)
			{
				const Box Under{Vec3{B.CentreM.X, B.CentreM.Y, B.CentreM.Z - Brick.Z}, B.HalfExtentsM};
				if (const std::optional<InterfaceDesc> Bed = FaceContact(Out.Ground, Under, Id, B, Spec.Bed))
				{
					Keep(Model.AddInterface(*Bed), Out.Beds);
				}
			}
			else
			{
				for (std::size_t J = 0; J < Boxes[C - 1].size(); ++J)
				{
					if (const std::optional<InterfaceDesc> Bed = FaceContact(Out.Courses[C - 1][J], Boxes[C - 1][J], Id, B, Spec.Bed))
					{
						Keep(Model.AddInterface(*Bed), Out.Beds);
					}
				}
			}
			if (!Course.empty())
			{
				if (const std::optional<InterfaceDesc> Perpend = FaceContact(Course.back(), CourseBoxes.back(), Id, B, Spec.Perpend))
				{
					Keep(Model.AddInterface(*Perpend), Out.Perpends);
				}
			}
			Course.push_back(Id);
			CourseBoxes.push_back(B);
		}
		Out.Courses.push_back(Course);
		Boxes.push_back(CourseBoxes);
	}
	return Out;
}
}
