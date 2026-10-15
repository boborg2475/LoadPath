#pragma once

#include "loadpath/Cascade.h"
#include "loadpath/CollapseCertificate.h"
#include "loadpath/ComponentPartition.h"
#include "loadpath/StandingAuthority.h"
#include "loadpath/StructuralAuthority.h"

#include <bit>
#include <cstddef>
#include <cstdint>
#include <vector>

// Bit-for-bit equality of results, NaN equal only to the same bit pattern; Lineage, StaleLoads and AuthorityResult::Options are left out (a copy takes a fresh lineage; stale-load tests compare against a solve without the load and check StaleLoads themselves).
namespace loadpath::test::bitwise
{
inline bool Same(double A, double B) { return std::bit_cast<std::uint64_t>(A) == std::bit_cast<std::uint64_t>(B); }
inline bool Same(const Vec3& A, const Vec3& B) { return Same(A.X, B.X) && Same(A.Y, B.Y) && Same(A.Z, B.Z); }
inline bool Same(const BodyId& A, const BodyId& B) { return A.Index == B.Index && A.Generation == B.Generation; }
inline bool Same(const InterfaceId& A, const InterfaceId& B) { return A.Index == B.Index && A.Generation == B.Generation; }
inline bool Same(int A, int B) { return A == B; }
inline bool Same(std::size_t A, std::size_t B) { return A == B; }
inline bool Same(const CornerReading& A, const CornerReading& B);
inline bool Same(const InterfaceReading& A, const InterfaceReading& B);
inline bool Same(const MechanismMotion& A, const MechanismMotion& B);
inline bool Same(const StandingComponent& A, const StandingComponent& B);
inline bool Same(const CertificateComponent& A, const CertificateComponent& B);
inline bool Same(const BodyLoad& A, const BodyLoad& B);
inline bool Same(const ComponentReport& A, const ComponentReport& B);
inline bool Same(const FailedInterface& A, const FailedInterface& B);
inline bool Same(const ReleasedCluster& A, const ReleasedCluster& B);

template <class T>
bool Same(const std::vector<T>& A, const std::vector<T>& B)
{
	if (A.size() != B.size())
	{
		return false;
	}
	for (std::size_t I = 0; I < A.size(); ++I)
	{
		if (!Same(A[I], B[I]))
		{
			return false;
		}
	}
	return true;
}

inline bool Same(const CornerReading& A, const CornerReading& B)
{
	return Same(A.PointM, B.PointM) && Same(A.NormalForceN, B.NormalForceN) && Same(A.ShearForceN, B.ShearForceN) && A.bOpen == B.bOpen && A.bSlipping == B.bSlipping;
}

inline bool Same(const InterfaceReading& A, const InterfaceReading& B)
{
	for (std::size_t I = 0; I < A.Corners.size(); ++I)
	{
		if (!Same(A.Corners[I], B.Corners[I]))
		{
			return false;
		}
	}
	return Same(A.Id, B.Id) && Same(A.Utilisation, B.Utilisation) && A.Axis == B.Axis && Same(A.CompressionUtilisation, B.CompressionUtilisation) && Same(A.TensionUtilisation, B.TensionUtilisation) && Same(A.ShearUtilisation, B.ShearUtilisation) && Same(A.NormalForceN, B.NormalForceN) && Same(A.ShearForceN, B.ShearForceN) && Same(A.MomentNm, B.MomentNm);
}

inline bool Same(const MechanismMotion& A, const MechanismMotion& B) { return Same(A.Body, B.Body) && Same(A.TranslationM, B.TranslationM) && Same(A.RotationRad, B.RotationRad); }

inline bool Same(const StandingComponent& A, const StandingComponent& B)
{
	return Same(A.Bodies, B.Bodies) && A.State == B.State && A.ActiveSetIterations == B.ActiveSetIterations && Same(A.ReadingSlots, B.ReadingSlots);
}

inline bool Same(const StandingResult& A, const StandingResult& B)
{
	return A.State == B.State && Same(A.Readings, B.Readings) && Same(A.Mechanism, B.Mechanism) && Same(A.UnconstrainedBodies, B.UnconstrainedBodies) && Same(A.Components, B.Components) && A.ActiveSetIterations == B.ActiveSetIterations && Same(A.InvalidInterface, B.InvalidInterface);
}

inline bool Same(const CertificateComponent& A, const CertificateComponent& B)
{
	return Same(A.Bodies, B.Bodies) && A.State == B.State && Same(A.LoadFactorLower, B.LoadFactorLower) && Same(A.LoadFactorUpper, B.LoadFactorUpper) && A.Iterations == B.Iterations && Same(A.Mechanism, B.Mechanism) && Same(A.UnconstrainedBodies, B.UnconstrainedBodies) && Same(A.HingingInterfaces, B.HingingInterfaces);
}

inline bool Same(const CertificateResult& A, const CertificateResult& B)
{
	return A.State == B.State && Same(A.LoadFactorLower, B.LoadFactorLower) && Same(A.LoadFactorUpper, B.LoadFactorUpper) && Same(A.Mechanism, B.Mechanism) && Same(A.UnconstrainedBodies, B.UnconstrainedBodies) && Same(A.HingingInterfaces, B.HingingInterfaces) && A.Iterations == B.Iterations && Same(A.InvalidInterface, B.InvalidInterface) && Same(A.Components, B.Components);
}

inline bool Same(const BodyLoad& A, const BodyLoad& B) { return Same(A.Body, B.Body) && Same(A.ForceN, B.ForceN) && Same(A.PointM, B.PointM); }

inline bool Same(const ComponentReport& A, const ComponentReport& B)
{
	return Same(A.Bodies, B.Bodies) && A.Verdict == B.Verdict && A.Elastic == B.Elastic && A.Kinematic == B.Kinematic && A.bConsultedCertificate == B.bConsultedCertificate && A.bDisagreed == B.bDisagreed && Same(A.LoadFactorLower, B.LoadFactorLower) && Same(A.LoadFactorUpper, B.LoadFactorUpper) && Same(A.Mechanism, B.Mechanism) && Same(A.Release, B.Release) && Same(A.Hinging, B.Hinging) && A.bReused == B.bReused && Same(A.Loads, B.Loads);
}

inline bool Same(const AuthorityResult& A, const AuthorityResult& B)
{
	return A.Worst == B.Worst && Same(A.Readings, B.Readings) && Same(A.Components, B.Components) && A.Disagreements == B.Disagreements && Same(A.InvalidInterface, B.InvalidInterface) && A.Revision == B.Revision;
}

inline bool Same(const FailedInterface& A, const FailedInterface& B) { return Same(A.Id, B.Id) && A.Pass == B.Pass && A.Axis == B.Axis && Same(A.Ratio, B.Ratio) && A.bSevered == B.bSevered; }

inline bool Same(const ReleasedCluster& A, const ReleasedCluster& B)
{
	return A.Pass == B.Pass && Same(A.Bodies, B.Bodies) && Same(A.Motion, B.Motion) && Same(A.Severed, B.Severed) && Same(A.LoadFactorLower, B.LoadFactorLower) && Same(A.LoadFactorUpper, B.LoadFactorUpper);
}

inline bool Same(const CascadeVerdict& A, const CascadeVerdict& B)
{
	return A.State == B.State && A.FirstPass == B.FirstPass && A.LastPass == B.LastPass && A.Passes == B.Passes && Same(A.Failures, B.Failures) && Same(A.Clusters, B.Clusters) && Same(A.Readings, B.Readings) && Same(A.Components, B.Components) && Same(A.InvalidInterface, B.InvalidInterface);
}

inline bool Same(const ComponentPartition& A, const ComponentPartition& B) { return Same(A.ComponentOfSlot, B.ComponentOfSlot) && Same(A.OffsetOfSlot, B.OffsetOfSlot) && Same(A.FreeBodies, B.FreeBodies); }
}
