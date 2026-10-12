#include "loadpath/geometry/RunningBondWall.h"

#include "loadpath/geometry/BoxContacts.h"

#include <cstddef>
#include <optional>

namespace loadpath::geometry
{
Wall BuildRunningBondWall(Structure& Model, const WallSpec& Spec)
{
	Wall Out;
	const Vec3 Brick = Spec.BrickM;
	Out.LengthM = Spec.BricksPerCourse * Brick.X;
	Out.HeightM = Spec.Courses * Brick.Z;
	const double FullKg = Spec.DensityKgPerM3 * Brick.X * Brick.Y * Brick.Z;

	const Box GroundBox{Vec3{0.5 * Out.LengthM, 0.0, Spec.GroundZM - 0.5 * Brick.Z}, Vec3{0.5 * Out.LengthM, 0.5 * Brick.Y, 0.5 * Brick.Z}};
	Out.Ground = Model.AddBody(BodyDesc{0.0, GroundBox.CentreM, true}).Value;

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
			const Box B{Vec3{X + 0.5 * L, 0.0, Z}, Vec3{0.5 * L, 0.5 * Brick.Y, 0.5 * Brick.Z}};
			X += L;
			const BodyId Id = Model.AddBody(BodyDesc{FullKg * L / Brick.X, B.CentreM, false}).Value;
			if (C == 0)
			{
				const Box Under{Vec3{B.CentreM.X, 0.0, Spec.GroundZM - 0.5 * Brick.Z}, B.HalfExtentsM};
				if (const std::optional<InterfaceDesc> Bed = FaceContact(Out.Ground, Under, Id, B, Spec.Bed))
				{
					Out.Beds.push_back(Model.AddInterface(*Bed).Value);
				}
			}
			else
			{
				for (std::size_t J = 0; J < Boxes[C - 1].size(); ++J)
				{
					if (const std::optional<InterfaceDesc> Bed = FaceContact(Out.Courses[C - 1][J], Boxes[C - 1][J], Id, B, Spec.Bed))
					{
						Out.Beds.push_back(Model.AddInterface(*Bed).Value);
					}
				}
			}
			if (!Course.empty())
			{
				if (const std::optional<InterfaceDesc> Perpend = FaceContact(Course.back(), CourseBoxes.back(), Id, B, Spec.Perpend))
				{
					Out.Perpends.push_back(Model.AddInterface(*Perpend).Value);
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
