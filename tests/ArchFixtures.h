#pragma once

#include "StaticsTestSupport.h"

#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace loadpath::test
{
inline constexpr double ArchRadiusM = 1.0;
inline constexpr double ArchDepthM = 1.0;
inline constexpr double ArchDensityKgPerM3 = 2000.0;
inline constexpr int ArchJointStepDeg = 3;
inline constexpr double ArchHaunchJointDeg = 54.484;

// Every 3 degrees from -90 to 90, plus joints at +-54.484 degrees where the continuous least-thickness haunch hinge sits.
inline std::vector<double> ArchJointAnglesDeg()
{
	std::vector<double> Angles;
	for (int Step = 0; Step <= 180 / ArchJointStepDeg; ++Step)
	{
		const double AngleDeg = -90.0 + ArchJointStepDeg * Step;
		if (AngleDeg == -54.0)
		{
			Angles.push_back(-ArchHaunchJointDeg);
		}
		Angles.push_back(AngleDeg);
		if (AngleDeg == 54.0)
		{
			Angles.push_back(ArchHaunchJointDeg);
		}
	}
	return Angles;
}

struct SemicircularArch
{
	Structure Model;
	std::vector<BodyId> Voussoirs;
	// Joint K sits at JointAnglesDeg[K] from the crown; the first and last are the springings.
	std::vector<InterfaceId> Joints;
	std::vector<double> JointAnglesDeg;
};

// The true centroid of an annular sector of half-angle HalfAngleRad: on the bisector at (2/3)(sin d / d)(re^3 - ri^3)/(re^2 - ri^2).
inline double AnnularSectorCentroidRadiusM(double InnerM, double OuterM, double HalfAngleRad)
{
	return (2.0 / 3.0) * (std::sin(HalfAngleRad) / HalfAngleRad) * (OuterM * OuterM * OuterM - InnerM * InnerM * InnerM) / (OuterM * OuterM - InnerM * InnerM);
}

inline InterfaceDesc ArchJoint(BodyId A, BodyId B, double AngleRad, double ThicknessM, ProfileId Profile)
{
	InterfaceDesc Desc;
	Desc.BodyA = A;
	Desc.BodyB = B;
	Desc.CentreM = Vec3{ArchRadiusM * std::sin(AngleRad), 0.0, ArchRadiusM * std::cos(AngleRad)};
	Desc.NormalAToB = Vec3{std::cos(AngleRad), 0.0, -std::sin(AngleRad)};
	Desc.AxisU = Vec3{std::sin(AngleRad), 0.0, std::cos(AngleRad)};
	Desc.AxisV = Vec3{0.0, -1.0, 0.0};
	Desc.HalfExtentUM = ThicknessM / 2.0;
	Desc.HalfExtentVM = ArchDepthM / 2.0;
	Desc.Profile = Profile;
	return Desc;
}

// A semicircle of centreline radius 1 m centred on the origin, crown at (0, 0, 1), radial joints at ArchJointAnglesDeg, thickness Eta times the radius.
inline SemicircularArch MakeSemicircularArch(double Eta, const StrengthProfile& Profile)
{
	SemicircularArch Arch;
	Arch.JointAnglesDeg = ArchJointAnglesDeg();
	const std::size_t VoussoirCount = Arch.JointAnglesDeg.size() - 1;
	const double ThicknessM = Eta * ArchRadiusM;
	const double InnerM = ArchRadiusM - ThicknessM / 2.0;
	const double OuterM = ArchRadiusM + ThicknessM / 2.0;
	const ProfileId P = AddProfileOk(Arch.Model, Profile);
	const BodyId LeftGround = AddBox(Arch.Model, Box{Vec3{-ArchRadiusM, 0.0, -0.5}, Vec3{0.5, 0.5, 0.5}, 0.0}, true);
	const BodyId RightGround = AddBox(Arch.Model, Box{Vec3{ArchRadiusM, 0.0, -0.5}, Vec3{0.5, 0.5, 0.5}, 0.0}, true);
	for (std::size_t Index = 0; Index < VoussoirCount; ++Index)
	{
		const double HalfAngleRad = 0.5 * (Arch.JointAnglesDeg[Index + 1] - Arch.JointAnglesDeg[Index]) * RadPerDegree;
		const double BisectorRad = 0.5 * (Arch.JointAnglesDeg[Index] + Arch.JointAnglesDeg[Index + 1]) * RadPerDegree;
		const double CentroidRadiusM = AnnularSectorCentroidRadiusM(InnerM, OuterM, HalfAngleRad);
		BodyDesc Desc;
		Desc.MassKg = ArchDensityKgPerM3 * ArchDepthM * HalfAngleRad * (OuterM * OuterM - InnerM * InnerM);
		Desc.CentroidM = Vec3{CentroidRadiusM * std::sin(BisectorRad), 0.0, CentroidRadiusM * std::cos(BisectorRad)};
		const Result<BodyId> Added = Arch.Model.AddBody(Desc);
		CHECK(Added.Ok());
		Arch.Voussoirs.push_back(Added.Value);
	}
	for (std::size_t Joint = 0; Joint <= VoussoirCount; ++Joint)
	{
		const BodyId A = Joint == 0 ? LeftGround : Arch.Voussoirs[Joint - 1];
		const BodyId B = Joint == VoussoirCount ? RightGround : Arch.Voussoirs[Joint];
		Arch.Joints.push_back(AddInterfaceOk(Arch.Model, ArchJoint(A, B, Arch.JointAnglesDeg[Joint] * RadPerDegree, ThicknessM, P)));
	}
	return Arch;
}

inline constexpr double PileBlockM = 0.2;
inline constexpr int PileCourseCount = 6;
inline constexpr double PileUnitWeightNPerM3 = 20000.0;

struct HollowPile
{
	Structure Model;
	std::vector<BodyId> Blocks;
	std::vector<Box> Shapes;
};

struct PlanDomino
{
	int X0;
	int Y0;
	int X1;
	int Y1;
};

// The Orduna and Lourenco pile reconstructed from its published dimensions: a 0.8 x 0.6 x 1.2 m ring of 0.2 x 0.2 x 0.4 m blocks, two mirrored tilings alternating by course.
inline HollowPile MakeHollowPile(double Friction)
{
	const PlanDomino TilingOne[] = {{0, 0, 0, 1}, {3, 0, 3, 1}, {1, 0, 2, 0}, {0, 2, 1, 2}, {2, 2, 3, 2}};
	const PlanDomino TilingTwo[] = {{0, 1, 0, 2}, {3, 1, 3, 2}, {1, 2, 2, 2}, {0, 0, 1, 0}, {2, 0, 3, 0}};
	const double DensityKgPerM3 = PileUnitWeightNPerM3 / GravityMPerS2;
	HollowPile Pile;
	const ProfileId P = AddProfileOk(Pile.Model, DryJoint(Friction));
	const BodyId Ground = AddGroundPlane(Pile.Model);
	std::vector<int> CourseOf;
	for (int Course = 0; Course < PileCourseCount; ++Course)
	{
		const PlanDomino* Tiling = Course % 2 == 0 ? TilingOne : TilingTwo;
		for (int Index = 0; Index < 5; ++Index)
		{
			const PlanDomino& D = Tiling[Index];
			const Vec3 CentreM{PileBlockM * (0.5 * (D.X0 + D.X1) + 0.5), PileBlockM * (0.5 * (D.Y0 + D.Y1) + 0.5), PileBlockM * (Course + 0.5)};
			const Vec3 HalfM{PileBlockM * (D.X1 - D.X0 + 1) / 2.0, PileBlockM * (D.Y1 - D.Y0 + 1) / 2.0, PileBlockM / 2.0};
			const Box Shape = BoxOfDensity(CentreM, HalfM, DensityKgPerM3);
			Pile.Blocks.push_back(AddBox(Pile.Model, Shape));
			Pile.Shapes.push_back(Shape);
			CourseOf.push_back(Course);
		}
	}
	for (std::size_t Upper = 0; Upper < Pile.Blocks.size(); ++Upper)
	{
		if (CourseOf[Upper] == 0)
		{
			AddInterfaceOk(Pile.Model, OnGroundPlane(Ground, Pile.Blocks[Upper], Pile.Shapes[Upper], P));
			continue;
		}
		for (std::size_t Lower = 0; Lower < Pile.Blocks.size(); ++Lower)
		{
			if (CourseOf[Lower] + 1 != CourseOf[Upper])
			{
				continue;
			}
			const std::optional<InterfaceDesc> Contact = StackedFaces(Pile.Blocks[Lower], Pile.Shapes[Lower], Pile.Blocks[Upper], Pile.Shapes[Upper], P);
			if (Contact)
			{
				AddInterfaceOk(Pile.Model, *Contact);
			}
		}
	}
	return Pile;
}
}
