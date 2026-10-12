#pragma once

#include "StaticsTestSupport.h"

#include "loadpath/CollapseCertificate.h"

#include "doctest.h"

#include <cmath>
#include <ostream>
#include <span>
#include <vector>

namespace loadpath
{
inline const char* CertificateStateName(CertificateState State)
{
	switch (State)
	{
	case CertificateState::Stands: return "Stands";
	case CertificateState::Collapses: return "Collapses";
	case CertificateState::Undecided: return "Undecided";
	case CertificateState::InvalidInput: return "InvalidInput";
	}
	return "CertificateState(?)";
}

inline std::ostream& operator<<(std::ostream& Out, CertificateState State) { return Out << CertificateStateName(State); }
}

namespace loadpath::test
{
inline CertificateOptions CertificateUnderGravity(const Vec3& GravityMPerS2Vector)
{
	CertificateOptions Options;
	Options.Gravity = GravityMPerS2Vector;
	return Options;
}

// Tilts the support by rotating gravity, never the geometry: downhill is +X.
inline CertificateOptions CertificateTilt(double TiltRad)
{
	return CertificateUnderGravity(Vec3{GravityMPerS2 * std::sin(TiltRad), 0.0, -GravityMPerS2 * std::cos(TiltRad)});
}

// Mass-proportional lateral load factors: gravity becomes (LambdaX g, LambdaY g, -g).
inline CertificateOptions CertificateLateral(double LambdaX, double LambdaY = 0.0)
{
	return CertificateUnderGravity(Vec3{LambdaX * GravityMPerS2, LambdaY * GravityMPerS2, -GravityMPerS2});
}

inline CertificateResult DecideWith(const Structure& Model, const CertificateOptions& Options, std::span<const BodyLoad> Loads = {})
{
	return MakeLimitAnalysisCertificate()->Decide(Model, Loads, Options, {});
}

inline bool CertificateStands(const Structure& Model, const CertificateOptions& Options)
{
	return DecideWith(Model, Options).State == CertificateState::Stands;
}

inline const MechanismMotion* FindMotion(const CertificateResult& Result, BodyId Body)
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

inline bool HasInterface(const std::vector<InterfaceId>& Interfaces, InterfaceId Id)
{
	for (const InterfaceId& Each : Interfaces)
	{
		if (SameHandle(Each, Id))
		{
			return true;
		}
	}
	return false;
}

// The circumscribed polygon's radius over the inscribed one's for a friction circle cut into Generators sides.
inline double CircumscribedOverInscribed(int Generators)
{
	return 1.0 / std::cos(Pi / Generators);
}

// Associative friction dilates while it slides, so only the in-plane part follows the load and the normal part may only open the joint.
inline void CheckAssociativeSlideAlong(const MechanismMotion& Motion, const Vec3& LoadDirection, const Vec3& JointNormal, double Relative)
{
	const Vec3 Normal = Unit(JointNormal);
	const double OpeningM = Dot(Motion.TranslationM, Normal);
	INFO("translation " << Motion.TranslationM << ", opening " << OpeningM);
	const MechanismMotion InPlane{Motion.Body, Minus(Motion.TranslationM, Times(OpeningM, Normal)), Motion.RotationRad};
	CheckTranslationAlong(InPlane, LoadDirection, Relative);
	CHECK(OpeningM >= -Relative * Length(Motion.TranslationM));
}
}
