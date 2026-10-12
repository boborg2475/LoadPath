#include "loadpath/geometry/BoxContacts.h"

#include <algorithm>
#include <cmath>

namespace loadpath::geometry
{
namespace
{
double Component(const Vec3& V, int Axis)
{
	return Axis == 0 ? V.X : (Axis == 1 ? V.Y : V.Z);
}

Vec3 UnitAxis(int Axis)
{
	return Vec3{Axis == 0 ? 1.0 : 0.0, Axis == 1 ? 1.0 : 0.0, Axis == 2 ? 1.0 : 0.0};
}

void SetComponent(Vec3& V, int Axis, double Value)
{
	(Axis == 0 ? V.X : (Axis == 1 ? V.Y : V.Z)) = Value;
}
}

std::optional<InterfaceDesc> FaceContact(BodyId A, const Box& BoxA, BodyId B, const Box& BoxB, ProfileId Profile, double ToleranceM)
{
	for (const Vec3& V : {BoxA.CentreM, BoxA.HalfExtentsM, BoxB.CentreM, BoxB.HalfExtentsM})
	{
		if (!std::isfinite(V.X) || !std::isfinite(V.Y) || !std::isfinite(V.Z))
		{
			return std::nullopt;
		}
	}
	if (!std::isfinite(ToleranceM))
	{
		return std::nullopt;
	}
	double Gap[3];
	double Overlap[3];
	double Mid[3];
	for (int Axis = 0; Axis < 3; ++Axis)
	{
		const double CA = Component(BoxA.CentreM, Axis);
		const double CB = Component(BoxB.CentreM, Axis);
		const double HA = Component(BoxA.HalfExtentsM, Axis);
		const double HB = Component(BoxB.HalfExtentsM, Axis);
		const double Low = std::max(CA - HA, CB - HB);
		const double High = std::min(CA + HA, CB + HB);
		Overlap[Axis] = High - Low;
		Mid[Axis] = 0.5 * (Low + High);
		Gap[Axis] = std::abs(CB - CA) - (HA + HB);
	}
	int ContactAxis = -1;
	for (int Axis = 0; Axis < 3; ++Axis)
	{
		if (!(std::abs(Gap[Axis]) <= ToleranceM))
		{
			continue;
		}
		const int U = (Axis + 1) % 3;
		const int V = (Axis + 2) % 3;
		if (Overlap[U] > ToleranceM && Overlap[V] > ToleranceM)
		{
			if (ContactAxis >= 0)
			{
				return std::nullopt;
			}
			ContactAxis = Axis;
		}
	}
	if (ContactAxis < 0)
	{
		return std::nullopt;
	}
	const int U = (ContactAxis + 1) % 3;
	const int V = (ContactAxis + 2) % 3;
	const bool bAFirst = Component(BoxA.CentreM, ContactAxis) < Component(BoxB.CentreM, ContactAxis);
	const Box& Lower = bAFirst ? BoxA : BoxB;
	const Box& Upper = bAFirst ? BoxB : BoxA;
	InterfaceDesc Desc;
	Desc.BodyA = bAFirst ? A : B;
	Desc.BodyB = bAFirst ? B : A;
	Desc.NormalAToB = UnitAxis(ContactAxis);
	const double Plane = 0.5 * ((Component(Lower.CentreM, ContactAxis) + Component(Lower.HalfExtentsM, ContactAxis)) + (Component(Upper.CentreM, ContactAxis) - Component(Upper.HalfExtentsM, ContactAxis)));
	SetComponent(Desc.CentreM, ContactAxis, Plane);
	SetComponent(Desc.CentreM, U, Mid[U]);
	SetComponent(Desc.CentreM, V, Mid[V]);
	Desc.AxisU = UnitAxis(U);
	Desc.AxisV = UnitAxis(V);
	Desc.HalfExtentUM = 0.5 * Overlap[U];
	Desc.HalfExtentVM = 0.5 * Overlap[V];
	Desc.Profile = Profile;
	return Desc;
}
}
