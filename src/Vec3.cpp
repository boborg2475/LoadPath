#include "loadpath/Vec3.h"

#include <cmath>

namespace loadpath
{
double Dot(const Vec3& A, const Vec3& B)
{
	return A.X * B.X + A.Y * B.Y + A.Z * B.Z;
}

Vec3 Cross(const Vec3& A, const Vec3& B)
{
	return Vec3{A.Y * B.Z - A.Z * B.Y, A.Z * B.X - A.X * B.Z, A.X * B.Y - A.Y * B.X};
}

double Length(const Vec3& V)
{
	return std::sqrt(Dot(V, V));
}

bool IsFinite(const Vec3& V)
{
	return IsFinite(V.X) && IsFinite(V.Y) && IsFinite(V.Z);
}
}
