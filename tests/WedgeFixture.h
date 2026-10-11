#pragma once

#include "AnchorFixtures.h"
#include "StaticsTestSupport.h"

#include <algorithm>
#include <cmath>

namespace loadpath::test
{
inline Vec3 PlaneUpwardNormal(double DipDeg, double DipDirectionDeg)
{
	const double Dip = DipDeg * RadPerDegree;
	const double Direction = DipDirectionDeg * RadPerDegree;
	return Vec3{std::sin(Dip) * std::sin(Direction), std::sin(Dip) * std::cos(Direction), std::cos(Dip)};
}

struct WedgeGeometry
{
	Vec3 NormalA;
	Vec3 NormalB;
	Vec3 LineDown;
	Vec3 SideA;
	Vec3 SideB;
};

inline WedgeGeometry ItascaWedgeGeometry()
{
	WedgeGeometry G;
	G.NormalA = PlaneUpwardNormal(40.0, 130.0);
	G.NormalB = PlaneUpwardNormal(60.0, 220.0);
	G.LineDown = Unit(Cross(G.NormalA, G.NormalB));
	if (G.LineDown.Z > 0.0)
	{
		G.LineDown = Times(-1.0, G.LineDown);
	}
	G.SideA = Unit(Cross(G.NormalA, G.LineDown));
	if (Dot(G.NormalB, G.SideA) < 0.0)
	{
		G.SideA = Times(-1.0, G.SideA);
	}
	G.SideB = Unit(Cross(G.NormalB, G.LineDown));
	if (Dot(G.NormalA, G.SideB) < 0.0)
	{
		G.SideB = Times(-1.0, G.SideB);
	}
	return G;
}

inline double ClassicalWedgeFrictionAngleRad(const WedgeGeometry& G)
{
	const Vec3 Up{0.0, 0.0, 1.0};
	const Vec3 UpAcrossLine = Minus(Up, Times(Dot(Up, G.LineDown), G.LineDown));
	const double Cosine = Dot(G.NormalA, G.NormalB);
	const double OnA = Dot(G.NormalA, UpAcrossLine);
	const double OnB = Dot(G.NormalB, UpAcrossLine);
	const double ReactionA = (OnA - Cosine * OnB) / (1.0 - Cosine * Cosine);
	const double ReactionB = (OnB - Cosine * OnA) / (1.0 - Cosine * Cosine);
	CHECK(ReactionA > 0.0);
	CHECK(ReactionB > 0.0);
	return std::atan(std::abs(Dot(Up, G.LineDown)) / (ReactionA + ReactionB));
}

struct Wedge
{
	Structure Model;
	BodyId Body;
	Vec3 CentroidM;
};

inline InterfaceDesc WedgeFace(BodyId Ground, BodyId WedgeBody, const Vec3& Normal, const Vec3& LineDown, const Vec3& CentreM, double LengthM, double HeightM, ProfileId Profile)
{
	InterfaceDesc Desc;
	Desc.BodyA = Ground;
	Desc.BodyB = WedgeBody;
	Desc.NormalAToB = Normal;
	Desc.CentreM = CentreM;
	Desc.AxisU = LineDown;
	Desc.AxisV = Cross(Normal, LineDown);
	Desc.HalfExtentUM = LengthM / 2.0;
	Desc.HalfExtentVM = HeightM / 4.0;
	Desc.Profile = Profile;
	return Desc;
}

inline Wedge MakeItascaWedge(double FrictionAngleRad)
{
	const WedgeGeometry G = ItascaWedgeGeometry();
	const double LengthM = 10.0;
	const double HeightM = 5.0;
	const Vec3 Apex{};
	const Vec3 Toe = Times(LengthM, G.LineDown);
	const Vec3 TipA = Plus(Times(LengthM / 2.0, G.LineDown), Times(HeightM, G.SideA));
	const Vec3 TipB = Plus(Times(LengthM / 2.0, G.LineDown), Times(HeightM, G.SideB));
	const double VolumeM3 = std::abs(Dot(Toe, Cross(TipA, TipB))) / 6.0;

	Wedge W;
	W.CentroidM = Times(0.25, Plus(Plus(Apex, Toe), Plus(TipA, TipB)));
	const ProfileId P = AddProfileOk(W.Model, DryJoint(std::tan(FrictionAngleRad)));
	const Vec3 FaceCentreA = Times(1.0 / 3.0, Plus(Plus(Apex, Toe), TipA));
	const Vec3 FaceCentreB = Times(1.0 / 3.0, Plus(Plus(Apex, Toe), TipB));
	const BodyId GroundA = AddBox(W.Model, Box{Minus(FaceCentreA, G.NormalA), Vec3{1.0, 1.0, 1.0}, 0.0}, true);
	const BodyId GroundB = AddBox(W.Model, Box{Minus(FaceCentreB, G.NormalB), Vec3{1.0, 1.0, 1.0}, 0.0}, true);
	W.Body = AddBox(W.Model, Box{W.CentroidM, Vec3{1.0, 1.0, 1.0}, RockDensityKgPerM3 * VolumeM3});
	AddInterfaceOk(W.Model, WedgeFace(GroundA, W.Body, G.NormalA, G.LineDown, FaceCentreA, LengthM, HeightM, P));
	AddInterfaceOk(W.Model, WedgeFace(GroundB, W.Body, G.NormalB, G.LineDown, FaceCentreB, LengthM, HeightM, P));
	return W;
}

// Associative friction lets the wedge leave both planes along any direction within 90 - phi of both normals, so it slides once such a direction reaches the horizontal.
inline double AssociativeWedgeFrictionAngleRad(const WedgeGeometry& G)
{
	const Vec3 HorizontalA{G.NormalA.X, G.NormalA.Y, 0.0};
	const Vec3 HorizontalB{G.NormalB.X, G.NormalB.Y, 0.0};
	const Vec3 Difference = Minus(HorizontalA, HorizontalB);
	Vec3 Equal = Unit(Vec3{-Difference.Y, Difference.X, 0.0});
	if (Dot(Equal, HorizontalA) < 0.0)
	{
		Equal = Times(-1.0, Equal);
	}
	double Best = std::min(Dot(Equal, HorizontalA), Dot(Equal, HorizontalB));
	for (const Vec3& Candidate : {Unit(HorizontalA), Unit(HorizontalB)})
	{
		Best = std::max(Best, std::min(Dot(Candidate, HorizontalA), Dot(Candidate, HorizontalB)));
	}
	return std::asin(Best);
}
}
