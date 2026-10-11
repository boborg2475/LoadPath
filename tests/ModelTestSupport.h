#pragma once

#include "loadpath/Structure.h"

#include <algorithm>
#include <limits>
#include <ostream>
#include <string>
#include <vector>

namespace loadpath
{
inline const char* ModelErrorName(ModelError Error)
{
	switch (Error)
	{
	case ModelError::None: return "None";
	case ModelError::NonFiniteInput: return "NonFiniteInput";
	case ModelError::NegativeMass: return "NegativeMass";
	case ModelError::DegenerateNormal: return "DegenerateNormal";
	case ModelError::AxesNotOrthonormal: return "AxesNotOrthonormal";
	case ModelError::NonPositiveExtent: return "NonPositiveExtent";
	case ModelError::UnknownBody: return "UnknownBody";
	case ModelError::UnknownProfile: return "UnknownProfile";
	case ModelError::SameBody: return "SameBody";
	case ModelError::BodyRemoved: return "BodyRemoved";
	case ModelError::NegativeStrength: return "NegativeStrength";
	case ModelError::NonPositiveStiffness: return "NonPositiveStiffness";
	}
	return "ModelError(?)";
}

inline const char* InterfaceStateName(InterfaceState State)
{
	switch (State)
	{
	case InterfaceState::Intact: return "Intact";
	case InterfaceState::Cracked: return "Cracked";
	case InterfaceState::Severed: return "Severed";
	case InterfaceState::Removed: return "Removed";
	}
	return "InterfaceState(?)";
}

inline std::ostream& operator<<(std::ostream& Out, ModelError Error) { return Out << ModelErrorName(Error); }
inline std::ostream& operator<<(std::ostream& Out, InterfaceState State) { return Out << InterfaceStateName(State); }
inline std::ostream& operator<<(std::ostream& Out, const BodyId& Id) { return Out << "BodyId{" << Id.Index << ", " << Id.Generation << "}"; }
inline std::ostream& operator<<(std::ostream& Out, const InterfaceId& Id) { return Out << "InterfaceId{" << Id.Index << ", " << Id.Generation << "}"; }
inline std::ostream& operator<<(std::ostream& Out, const ProfileId& Id) { return Out << "ProfileId{" << Id.Index << ", " << Id.Generation << "}"; }
}

namespace loadpath::test
{
inline constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
inline constexpr double Infinity = std::numeric_limits<double>::infinity();

inline constexpr double BrickLengthM = 0.215;
inline constexpr double BrickWidthM = 0.1025;
inline constexpr double BrickHeightM = 0.065;
inline constexpr double BrickMassKg = 2.72;

template <class Id>
bool SameHandle(const Id& A, const Id& B)
{
	return A.Index == B.Index && A.Generation == B.Generation;
}

template <class Id>
std::string Describe(std::vector<Id> Ids)
{
	std::string Text;
	for (const Id& Each : Ids)
	{
		Text += "{" + std::to_string(Each.Index) + "," + std::to_string(Each.Generation) + "}";
	}
	return Text;
}

template <class Id>
std::string DescribeAscending(std::vector<Id> Ids)
{
	std::sort(Ids.begin(), Ids.end(), [](const Id& A, const Id& B) { return A.Index < B.Index; });
	return Describe(Ids);
}

inline StrengthProfile DistinctFieldProfile()
{
	StrengthProfile Profile;
	Profile.CompressiveStrengthPa = 7.0e6;
	Profile.ShearCohesionPa = 6.0e5;
	Profile.TensileStrengthPa = 5.0e5;
	Profile.FrictionCoefficient = 0.4;
	Profile.MaxShearStrengthPa = 3.0e6;
	Profile.NormalStiffnessPaPerM = 2.0e10;
	Profile.TangentialStiffnessPaPerM = 1.0e10;
	return Profile;
}

inline BodyDesc BrickAt(const Vec3& CentroidM, bool bGrounded = false)
{
	BodyDesc Desc;
	Desc.MassKg = BrickMassKg;
	Desc.CentroidM = CentroidM;
	Desc.bGrounded = bGrounded;
	return Desc;
}

inline Vec3 StackedBrickCentroidM(int Course)
{
	return Vec3{0.0, 0.0, (Course + 0.5) * BrickHeightM};
}

inline InterfaceDesc BedJoint(BodyId Lower, BodyId Upper, int UpperCourse, ProfileId Profile)
{
	InterfaceDesc Desc;
	Desc.BodyA = Lower;
	Desc.BodyB = Upper;
	Desc.NormalAToB = Vec3{0.0, 0.0, 1.0};
	Desc.CentreM = Vec3{0.0, 0.0, UpperCourse * BrickHeightM};
	Desc.AxisU = Vec3{1.0, 0.0, 0.0};
	Desc.AxisV = Vec3{0.0, 1.0, 0.0};
	Desc.HalfExtentUM = BrickLengthM / 2.0;
	Desc.HalfExtentVM = BrickWidthM / 2.0;
	Desc.Profile = Profile;
	return Desc;
}

inline InterfaceDesc HeadJointAlongX(BodyId Left, BodyId Right, int Course, ProfileId Profile)
{
	InterfaceDesc Desc;
	Desc.BodyA = Left;
	Desc.BodyB = Right;
	Desc.NormalAToB = Vec3{1.0, 0.0, 0.0};
	Desc.CentreM = Vec3{BrickLengthM / 2.0, 0.0, (Course + 0.5) * BrickHeightM};
	Desc.AxisU = Vec3{0.0, 1.0, 0.0};
	Desc.AxisV = Vec3{0.0, 0.0, 1.0};
	Desc.HalfExtentUM = BrickWidthM / 2.0;
	Desc.HalfExtentVM = BrickHeightM / 2.0;
	Desc.Profile = Profile;
	return Desc;
}
}
