#include "StaticsTestSupport.h"

#include "EigenInclude.h"

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4702)
#endif
#include <Eigen/LU>
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#include "doctest.h"

#include <cmath>
#include <string>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double BlockHalfM = 0.1;
constexpr double SlabHalfZM = 0.1;
constexpr double PitchM = 0.4;
constexpr double DensityKgPerM3 = 2000.0;
constexpr double LiftOverWeight = 0.75;

struct BoundaryCase
{
	int Blocks = 0;
	int Tipping = 0;
	int Iterations = 0;
};

struct Corner
{
	int BodyA = -1;
	int BodyB = -1;
	Eigen::Vector3d PointM;
};

struct Fixture
{
	Structure Model;
	std::vector<BodyId> Blocks;
	std::vector<BodyLoad> Loads;
	std::vector<Eigen::Vector3d> CentroidsM;
	std::vector<Corner> ClosedCorners;
};

StrengthProfile Bonded()
{
	StrengthProfile Profile = DryJoint(0.7);
	Profile.ShearCohesionPa = 1.0e6;
	Profile.TensileStrengthPa = 1.0e6;
	return Profile;
}

Fixture Build(const BoundaryCase& C)
{
	Fixture F;
	const BodyId Ground = AddGroundPlane(F.Model);
	const ProfileId Dry = AddProfileOk(F.Model, DryJoint(0.7));
	const ProfileId Glue = AddProfileOk(F.Model, Bonded());
	const double SlabHalfXM = 0.5 * PitchM * C.Blocks;
	const Box Slab = BoxOfDensity(Vec3{SlabHalfXM, 0.0, SlabHalfZM}, Vec3{SlabHalfXM, 0.3, SlabHalfZM}, DensityKgPerM3);
	const BodyId SlabId = AddBox(F.Model, Slab);
	AddInterfaceOk(F.Model, OnGroundPlane(Ground, SlabId, Slab, Glue));
	F.CentroidsM.push_back(Eigen::Vector3d(Slab.CentreM.X, Slab.CentreM.Y, Slab.CentreM.Z));
	for (double X : {0.0, 2.0 * SlabHalfXM})
	{
		for (double Y : {-0.3, 0.3})
		{
			F.ClosedCorners.push_back(Corner{-1, 0, Eigen::Vector3d(X, Y, 0.0)});
		}
	}
	for (int K = 0; K < C.Blocks; ++K)
	{
		const Box Block = BoxOfDensity(Vec3{PitchM * (K + 0.5), 0.0, 2.0 * SlabHalfZM + BlockHalfM}, Vec3{BlockHalfM, BlockHalfM, BlockHalfM}, DensityKgPerM3);
		const BodyId Id = AddBox(F.Model, Block);
		const std::optional<InterfaceDesc> Seat = StackedFaces(SlabId, Slab, Id, Block, Dry);
		REQUIRE(Seat.has_value());
		AddInterfaceOk(F.Model, *Seat);
		F.Blocks.push_back(Id);
		F.CentroidsM.push_back(Eigen::Vector3d(Block.CentreM.X, Block.CentreM.Y, Block.CentreM.Z));
		const bool bTips = K < C.Tipping;
		for (double DX : {-BlockHalfM, BlockHalfM})
		{
			if (bTips && DX > 0.0)
			{
				continue;
			}
			for (double DY : {-BlockHalfM, BlockHalfM})
			{
				F.ClosedCorners.push_back(Corner{0, K + 1, Eigen::Vector3d(Block.CentreM.X + DX, DY, 2.0 * SlabHalfZM)});
			}
		}
		if (bTips)
		{
			const double LiftN = LiftOverWeight * Block.MassKg * GravityMPerS2;
			F.Loads.push_back(BodyLoad{Id, Vec3{0.0, 0.0, LiftN}, Vec3{Block.CentreM.X + BlockHalfM, 0.0, Block.CentreM.Z}});
		}
	}
	return F;
}

Eigen::Matrix<double, 3, 6> PointJacobian(const Eigen::Vector3d& ArmM)
{
	Eigen::Matrix<double, 3, 6> G;
	G.leftCols<3>().setIdentity();
	G.rightCols<3>() << 0.0, ArmM.z(), -ArmM.y(), -ArmM.z(), 0.0, ArmM.x(), ArmM.y(), -ArmM.x(), 0.0;
	return G;
}

Eigen::Index ConstraintRank(const Fixture& F)
{
	const Eigen::Index Dof = 6 * static_cast<Eigen::Index>(F.CentroidsM.size());
	Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3 * static_cast<Eigen::Index>(F.ClosedCorners.size()), Dof);
	for (std::size_t I = 0; I < F.ClosedCorners.size(); ++I)
	{
		const Corner& C = F.ClosedCorners[I];
		const Eigen::Index Row = 3 * static_cast<Eigen::Index>(I);
		J.block<3, 6>(Row, 6 * C.BodyB) += PointJacobian(C.PointM - F.CentroidsM[static_cast<std::size_t>(C.BodyB)]);
		if (C.BodyA >= 0)
		{
			J.block<3, 6>(Row, 6 * C.BodyA) -= PointJacobian(C.PointM - F.CentroidsM[static_cast<std::size_t>(C.BodyA)]);
		}
	}
	Eigen::FullPivLU<Eigen::MatrixXd> Lu(J);
	Lu.setThreshold(1e-12);
	return Lu.rank();
}

void CheckCase(const BoundaryCase& C)
{
	const Fixture F = Build(C);
	const int Dof = 6 * (C.Blocks + 1);
	const int Rank = Dof - C.Tipping;
	const StandingResult R = SolveWith(F.Model, StandingOptions{}, F.Loads);
	INFO(C.Blocks << " blocks on a slab, " << C.Tipping << " tipping, rank " << Rank << " of " << Dof << ": " << R.State << " after " << R.ActiveSetIterations);
	CHECK(ConstraintRank(F) == Rank);
	CHECK(R.State == StandingState::Mechanism);
	CHECK(R.ActiveSetIterations == C.Iterations);
	CHECK(R.Mechanism.size() == static_cast<std::size_t>(C.Tipping));
	CHECK(R.UnconstrainedBodies.size() == static_cast<std::size_t>(C.Tipping));
	if (R.Mechanism.size() != static_cast<std::size_t>(C.Tipping) || R.UnconstrainedBodies.size() != static_cast<std::size_t>(C.Tipping))
	{
		return;
	}
	for (int K = 0; K < C.Tipping; ++K)
	{
		const MechanismMotion& M = R.Mechanism[static_cast<std::size_t>(K)];
		INFO("block " << K << ": translation " << M.TranslationM << ", rotation " << M.RotationRad);
		CHECK(SameHandle(M.Body, F.Blocks[static_cast<std::size_t>(K)]));
		CHECK(SameHandle(R.UnconstrainedBodies[static_cast<std::size_t>(K)], F.Blocks[static_cast<std::size_t>(K)]));
		CHECK(std::abs(M.TranslationM.X + BlockHalfM) <= 1e-9);
		CHECK(std::abs(M.TranslationM.Y) <= 1e-9);
		CHECK(std::abs(M.TranslationM.Z - BlockHalfM) <= 1e-9);
		CHECK(std::abs(M.RotationRad.X) <= 1e-9);
		CHECK(std::abs(M.RotationRad.Y + 1.0) <= 1e-9);
		CHECK(std::abs(M.RotationRad.Z) <= 1e-9);
	}
}
}

TEST_CASE("Statics.Factor.ARankOnThePanelBoundaryReadsItsNullSpace")
{
	const std::vector<BoundaryCase> Cases{
		{9, 2, 2},
		{10, 2, 2},
		{11, 2, 2},
		{21, 4, 2},
		{22, 4, 2},
	};
	for (const BoundaryCase& C : Cases)
	{
		CheckCase(C);
	}
}
