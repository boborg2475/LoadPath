#pragma once

#include "ModelTestSupport.h"

#include "loadpath/StandingAuthority.h"

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <ostream>
#include <span>
#include <vector>

namespace loadpath
{
inline const char* StandingStateName(StandingState State)
{
	switch (State)
	{
	case StandingState::Stands: return "Stands";
	case StandingState::Overstressed: return "Overstressed";
	case StandingState::Mechanism: return "Mechanism";
	case StandingState::Undecided: return "Undecided";
	case StandingState::InvalidInput: return "InvalidInput";
	}
	return "StandingState(?)";
}

inline const char* GoverningAxisName(GoverningAxis Axis)
{
	switch (Axis)
	{
	case GoverningAxis::None: return "None";
	case GoverningAxis::Compression: return "Compression";
	case GoverningAxis::Tension: return "Tension";
	case GoverningAxis::Shear: return "Shear";
	}
	return "GoverningAxis(?)";
}

inline std::ostream& operator<<(std::ostream& Out, StandingState State) { return Out << StandingStateName(State); }
inline std::ostream& operator<<(std::ostream& Out, GoverningAxis Axis) { return Out << GoverningAxisName(Axis); }
inline std::ostream& operator<<(std::ostream& Out, const Vec3& V) { return Out << "(" << V.X << ", " << V.Y << ", " << V.Z << ")"; }
}

namespace loadpath::test
{
inline constexpr double Pi = 3.14159265358979323846;
inline constexpr double RadPerDegree = Pi / 180.0;

inline Vec3 Plus(const Vec3& A, const Vec3& B) { return Vec3{A.X + B.X, A.Y + B.Y, A.Z + B.Z}; }
inline Vec3 Minus(const Vec3& A, const Vec3& B) { return Vec3{A.X - B.X, A.Y - B.Y, A.Z - B.Z}; }
inline Vec3 Times(double S, const Vec3& V) { return Vec3{S * V.X, S * V.Y, S * V.Z}; }
inline Vec3 Unit(const Vec3& V) { return Times(1.0 / Length(V), V); }

struct Box
{
	Vec3 CentreM;
	Vec3 HalfExtentsM;
	double MassKg = 0;
};

inline Box BoxOfDensity(const Vec3& CentreM, const Vec3& HalfExtentsM, double DensityKgPerM3)
{
	return Box{CentreM, HalfExtentsM, DensityKgPerM3 * 8.0 * HalfExtentsM.X * HalfExtentsM.Y * HalfExtentsM.Z};
}

inline BodyId AddBox(Structure& Model, const Box& B, bool bGrounded = false)
{
	BodyDesc Desc;
	Desc.MassKg = B.MassKg;
	Desc.CentroidM = B.CentreM;
	Desc.bGrounded = bGrounded;
	const Result<BodyId> Added = Model.AddBody(Desc);
	CHECK(Added.Ok());
	return Added.Value;
}

// A grounded, massless body standing for the unbounded plane z = PlaneZM.
inline BodyId AddGroundPlane(Structure& Model, double PlaneZM = 0.0)
{
	BodyDesc Desc;
	Desc.CentroidM = Vec3{0.0, 0.0, PlaneZM};
	Desc.bGrounded = true;
	const Result<BodyId> Added = Model.AddBody(Desc);
	CHECK(Added.Ok());
	return Added.Value;
}

inline ProfileId AddProfileOk(Structure& Model, const StrengthProfile& Profile)
{
	const Result<ProfileId> Added = Model.AddProfile(Profile);
	CHECK(Added.Ok());
	return Added.Value;
}

inline InterfaceId AddInterfaceOk(Structure& Model, const InterfaceDesc& Desc)
{
	const Result<InterfaceId> Added = Model.AddInterface(Desc);
	INFO("AddInterface refused: " << Added.Error);
	CHECK(Added.Ok());
	return Added.Value;
}

// Interval intersection, min(highs) - max(lows), never reach - distance (TRAPS); MidM receives the overlap centre.
inline double OverlapM(double CentreA, double HalfA, double CentreB, double HalfB, double& MidM)
{
	const double Low = std::max(CentreA - HalfA, CentreB - HalfB);
	const double High = std::min(CentreA + HalfA, CentreB + HalfB);
	MidM = 0.5 * (Low + High);
	return High - Low;
}

inline std::optional<InterfaceDesc> FacesInContact(BodyId Lower, const Vec3& LowerCentreM, const Vec3& LowerHalfM, BodyId Upper, const Box& UpperBox, ProfileId Profile)
{
	double MidX = 0.0;
	double MidY = 0.0;
	const double SpanX = OverlapM(LowerCentreM.X, LowerHalfM.X, UpperBox.CentreM.X, UpperBox.HalfExtentsM.X, MidX);
	const double SpanY = OverlapM(LowerCentreM.Y, LowerHalfM.Y, UpperBox.CentreM.Y, UpperBox.HalfExtentsM.Y, MidY);
	if (!(SpanX > 0.0) || !(SpanY > 0.0))
	{
		return std::nullopt;
	}
	InterfaceDesc Desc;
	Desc.BodyA = Lower;
	Desc.BodyB = Upper;
	Desc.NormalAToB = Vec3{0.0, 0.0, 1.0};
	Desc.CentreM = Vec3{MidX, MidY, UpperBox.CentreM.Z - UpperBox.HalfExtentsM.Z};
	Desc.AxisU = Vec3{1.0, 0.0, 0.0};
	Desc.AxisV = Vec3{0.0, 1.0, 0.0};
	Desc.HalfExtentUM = 0.5 * SpanX;
	Desc.HalfExtentVM = 0.5 * SpanY;
	Desc.Profile = Profile;
	return Desc;
}

// The contact between Lower's top face and Upper's bottom face; nullopt when the faces do not overlap.
inline std::optional<InterfaceDesc> StackedFaces(BodyId Lower, const Box& LowerBox, BodyId Upper, const Box& UpperBox, ProfileId Profile)
{
	return FacesInContact(Lower, LowerBox.CentreM, LowerBox.HalfExtentsM, Upper, UpperBox, Profile);
}

// The contact between an unbounded ground plane and a box's bottom face, which is the whole face.
inline InterfaceDesc OnGroundPlane(BodyId Ground, BodyId Upper, const Box& UpperBox, ProfileId Profile)
{
	return *FacesInContact(Ground, UpperBox.CentreM, Vec3{Infinity, Infinity, 0.0}, Upper, UpperBox, Profile);
}

// Dry joint: no cohesion or tension, and a compressive strength high enough that crushing never governs.
inline StrengthProfile DryJoint(double FrictionCoefficient)
{
	StrengthProfile Profile;
	Profile.CompressiveStrengthPa = 1.0e9;
	Profile.ShearCohesionPa = 0.0;
	Profile.TensileStrengthPa = 0.0;
	Profile.FrictionCoefficient = FrictionCoefficient;
	Profile.NormalStiffnessPaPerM = 1.0e10;
	Profile.TangentialStiffnessPaPerM = 1.0e10;
	return Profile;
}

// Tilts the support by rotating gravity, never the geometry: downhill is +X.
inline StandingOptions TiltedPlane(double TiltRad)
{
	StandingOptions Options;
	Options.Gravity = Vec3{GravityMPerS2 * std::sin(TiltRad), 0.0, -GravityMPerS2 * std::cos(TiltRad)};
	return Options;
}

// A mass-proportional lateral load factor in +X: gravity becomes (Lambda g, 0, -g).
inline StandingOptions LateralLoad(double Lambda)
{
	StandingOptions Options;
	Options.Gravity = Vec3{Lambda * GravityMPerS2, 0.0, -GravityMPerS2};
	return Options;
}

struct ThresholdBracket
{
	double LastStanding = NaN;
	double FirstFailing = NaN;
	bool bBracketed = false;
};

// Bisects between a value that stands and one that does not, in either order; unbracketed when the ends do not differ.
template <class StandsAt>
ThresholdBracket Threshold(StandsAt&& Stands, double StandingEnd, double FailingEnd, double Tolerance)
{
	ThresholdBracket Bracket;
	if (!Stands(StandingEnd) || Stands(FailingEnd))
	{
		return Bracket;
	}
	double Standing = StandingEnd;
	double Failing = FailingEnd;
	for (int Step = 0; Step < 200 && !(std::abs(Failing - Standing) <= Tolerance); ++Step)
	{
		const double Mid = 0.5 * (Standing + Failing);
		if (Stands(Mid))
		{
			Standing = Mid;
		}
		else
		{
			Failing = Mid;
		}
	}
	Bracket.LastStanding = Standing;
	Bracket.FirstFailing = Failing;
	Bracket.bBracketed = true;
	return Bracket;
}

inline StandingResult SolveWith(const Structure& Model, const StandingOptions& Options, std::span<const BodyLoad> Loads = {})
{
	return MakeRbsmStandingAuthority()->Solve(Model, Loads, Options);
}

inline const InterfaceReading* FindReading(const StandingResult& Result, InterfaceId Id)
{
	for (const InterfaceReading& Reading : Result.Readings)
	{
		if (SameHandle(Reading.Id, Id))
		{
			return &Reading;
		}
	}
	return nullptr;
}

inline const MechanismMotion* FindMotion(const StandingResult& Result, BodyId Body)
{
	for (const MechanismMotion& Motion : Result.Mechanism)
	{
		if (SameHandle(Motion.Body, Body))
		{
			return &Motion;
		}
	}
	return nullptr;
}

inline bool HasBody(const std::vector<BodyId>& Bodies, BodyId Body)
{
	for (const BodyId& Each : Bodies)
	{
		if (SameHandle(Each, Body))
		{
			return true;
		}
	}
	return false;
}

inline void CheckRelative(double Got, double Want, double Relative, const char* What)
{
	INFO(doctest::String(What) << ": got " << Got << ", want " << Want);
	CHECK(std::abs(Got - Want) <= Relative * std::abs(Want));
}

// A tip must move the centroid along the in-plane load, so the motion does positive work with it.
inline void CheckRotationAboutEdge(const MechanismMotion& Motion, const Vec3& CentroidM, const Vec3& EdgePointM, const Vec3& EdgeAxis, const Vec3& LoadDirection, double Relative)
{
	INFO("translation " << Motion.TranslationM << ", rotation " << Motion.RotationRad);
	const double RotationSize = Length(Motion.RotationRad);
	CHECK(RotationSize > 0.0);
	const Vec3 Axis = Unit(EdgeAxis);
	const Vec3 OffAxis = Minus(Motion.RotationRad, Times(Dot(Motion.RotationRad, Axis), Axis));
	CHECK(Length(OffAxis) <= Relative * RotationSize);
	const Vec3 AboutEdge = Cross(Motion.RotationRad, Minus(CentroidM, EdgePointM));
	CHECK(Length(Minus(Motion.TranslationM, AboutEdge)) <= Relative * Length(AboutEdge));
	CHECK(Dot(Motion.TranslationM, Unit(LoadDirection)) > 0.0);
}

// A slide must translate along the in-plane load, never against it.
inline void CheckTranslationAlong(const MechanismMotion& Motion, const Vec3& LoadDirection, double Relative)
{
	INFO("translation " << Motion.TranslationM << ", rotation " << Motion.RotationRad);
	const double TranslationSize = Length(Motion.TranslationM);
	CHECK(TranslationSize > 0.0);
	const Vec3 Along = Unit(LoadDirection);
	const Vec3 OffLine = Minus(Motion.TranslationM, Times(Dot(Motion.TranslationM, Along), Along));
	CHECK(Length(OffLine) <= Relative * TranslationSize);
	CHECK(Length(Motion.RotationRad) <= Relative * TranslationSize);
	CHECK(Dot(Motion.TranslationM, Along) > 0.0);
}
}
