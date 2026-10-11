#pragma once

#include "loadpath/Units.h"

namespace loadpath
{
struct Vec3
{
	double X = 0;
	double Y = 0;
	double Z = 0;
};

double Dot(const Vec3& A, const Vec3& B);
Vec3 Cross(const Vec3& A, const Vec3& B);
double Length(const Vec3& V);
bool IsFinite(const Vec3& V);
}
