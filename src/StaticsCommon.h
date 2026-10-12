#pragma once

#include "EigenInclude.h"

#include "loadpath/CollapseCertificate.h"
#include "loadpath/Handles.h"
#include "loadpath/Interface.h"
#include "loadpath/Structure.h"
#include "loadpath/Vec3.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <vector>

namespace loadpath::statics
{
inline constexpr int DofPerBody = 6;
inline constexpr std::size_t CornerCount = 4;

inline Vec3 Add(const Vec3& A, const Vec3& B)
{
	return Vec3{A.X + B.X, A.Y + B.Y, A.Z + B.Z};
}

inline Vec3 Sub(const Vec3& A, const Vec3& B)
{
	return Vec3{A.X - B.X, A.Y - B.Y, A.Z - B.Z};
}

inline Vec3 Scale(double S, const Vec3& V)
{
	return Vec3{S * V.X, S * V.Y, S * V.Z};
}

// Corner order (+u,+v), (-u,+v), (-u,-v), (+u,-v), as in InterfaceReading::Corners.
std::array<Vec3, CornerCount> CornerPoints(const InterfaceRecord& Record);

// Offsets are -1 for a grounded side, so its rows are never touched.
struct JointGeometry
{
	InterfaceId Id;
	Vec3 Normal;
	Vec3 CentreM;
	std::array<Vec3, CornerCount> CornerM;
	int OffsetA = -1;
	int OffsetB = -1;
	Vec3 CentroidAM;
	Vec3 CentroidBM;
};

JointGeometry MakeJointGeometry(const Structure& Model, InterfaceId Id, const std::vector<int>& OffsetOfSlot);

using loadpath::ComponentPartition;

ComponentPartition PartitionFreeBodies(const Structure& Model);

// -1 when both bodies are grounded.
int ComponentOf(const ComponentPartition& Partition, const InterfaceRecord& Record);

// Shared so the combined authority rejects exactly what the certificate rejects.
bool AreCertificateOptionsValid(const CertificateOptions& Options);

// Row-major 3x6: the velocity of a point at ArmM from its body's (translation, rotation).
std::array<double, 18> CornerJacobianRows(const Vec3& ArmM);

Eigen::Matrix<double, 3, 6> CornerJacobian(const Vec3& ArmM);

void AddForceAt(Eigen::VectorXd& Load, int Offset, const Vec3& CentroidM, const Vec3& PointM, const Vec3& ForceN);

// Rotation rows are divided by this so verdicts are scale-invariant; 1 m when no corner touches a free body.
template <class Joint>
double CharacteristicLengthM(const std::vector<Joint>& Joints)
{
	double LengthM = 0.0;
	for (const Joint& J : Joints)
	{
		for (std::size_t C = 0; C < CornerCount; ++C)
		{
			if (J.OffsetA >= 0)
			{
				LengthM = std::max(LengthM, Length(Sub(J.CornerM[C], J.CentroidAM)));
			}
			if (J.OffsetB >= 0)
			{
				LengthM = std::max(LengthM, Length(Sub(J.CornerM[C], J.CentroidBM)));
			}
		}
	}
	return LengthM > 0.0 ? LengthM : 1.0;
}
}
