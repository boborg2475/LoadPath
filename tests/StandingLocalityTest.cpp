#include "StaticsTestSupport.h"

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double StoneDensityKgPerM3 = 2000.0;
constexpr double CubeFriction = 0.7;
constexpr double LocalityRelative = 1.0e-12;
constexpr std::uint32_t MasterSeed = 20261011u;
constexpr int DrawCount = 50;

struct Scene
{
	Structure Model;
	InterfaceId CubeBed;
	double CubeWeightN = 0;
};

StrengthProfile NeverReleasingBond()
{
	StrengthProfile Profile = DryJoint(CubeFriction);
	Profile.TensileStrengthPa = 1.0e7;
	Profile.ShearCohesionPa = 1.0e7;
	return Profile;
}

Box BoxOnGround(double CentreXM, const Vec3& EdgesM)
{
	return BoxOfDensity(Vec3{CentreXM, 0.0, EdgesM.Z / 2.0}, Times(0.5, EdgesM), StoneDensityKgPerM3);
}

Scene MakeScene(const Box& Cube, const std::vector<Box>& Slabs)
{
	Scene S;
	const ProfileId CubeProfile = AddProfileOk(S.Model, DryJoint(CubeFriction));
	const ProfileId SlabProfile = AddProfileOk(S.Model, NeverReleasingBond());
	const BodyId Ground = AddGroundPlane(S.Model);
	const BodyId CubeId = AddBox(S.Model, Cube);
	S.CubeBed = AddInterfaceOk(S.Model, OnGroundPlane(Ground, CubeId, Cube, CubeProfile));
	S.CubeWeightN = Cube.MassKg * GravityMPerS2;
	for (const Box& Slab : Slabs)
	{
		const BodyId SlabId = AddBox(S.Model, Slab);
		AddInterfaceOk(S.Model, OnGroundPlane(Ground, SlabId, Slab, SlabProfile));
	}
	return S;
}

void CheckClose(double Got, double Want, double ToleranceAbs, const char* What)
{
	INFO(doctest::String(What) << ": with slabs " << Got << ", alone " << Want);
	CHECK(std::abs(Got - Want) <= ToleranceAbs);
}

void CheckCloseVec3(const Vec3& Got, const Vec3& Want, double ToleranceAbs, const char* What)
{
	CheckClose(Got.X, Want.X, ToleranceAbs, What);
	CheckClose(Got.Y, Want.Y, ToleranceAbs, What);
	CheckClose(Got.Z, Want.Z, ToleranceAbs, What);
}

void CheckSameCubeVerdict(const StandingResult& Alone, const StandingResult& WithSlabs, InterfaceId Bed, double CubeWeightN)
{
	INFO("alone " << Alone.State << " after " << Alone.ActiveSetIterations << " passes, with slabs " << WithSlabs.State << " after " << WithSlabs.ActiveSetIterations);
	CHECK(WithSlabs.State == Alone.State);
	const InterfaceReading* A = FindReading(Alone, Bed);
	const InterfaceReading* B = FindReading(WithSlabs, Bed);
	CHECK(A != nullptr);
	CHECK(B != nullptr);
	if (A == nullptr || B == nullptr)
	{
		return;
	}
	const double ForceToleranceN = LocalityRelative * CubeWeightN;
	CheckClose(B->Utilisation, A->Utilisation, LocalityRelative * std::abs(A->Utilisation), "bed utilisation");
	CheckClose(B->NormalForceN, A->NormalForceN, ForceToleranceN, "bed normal force");
	CheckCloseVec3(B->ShearForceN, A->ShearForceN, ForceToleranceN, "bed shear force");
	for (std::size_t C = 0; C < 4; ++C)
	{
		INFO("corner " << C);
		CHECK(B->Corners[C].bOpen == A->Corners[C].bOpen);
		CHECK(B->Corners[C].bSlipping == A->Corners[C].bSlipping);
		CheckClose(B->Corners[C].NormalForceN, A->Corners[C].NormalForceN, ForceToleranceN, "corner normal force");
		CheckCloseVec3(B->Corners[C].ShearForceN, A->Corners[C].ShearForceN, ForceToleranceN, "corner shear force");
	}
}

double Uniform(std::mt19937& Rng, double Low, double High)
{
	return Low + (High - Low) * (static_cast<double>(Rng()) / 4294967296.0);
}
}

TEST_CASE("Statics.Solver.VerdictsAreLocalToTheConnectedComponent")
{
	SUBCASE("1 cm cube beside a 10 m slab")
	{
		const Box Cube = BoxOnGround(0.0, Vec3{0.01, 0.01, 0.01});
		const Box Slab = BoxOnGround(100.0, Vec3{10.0, 10.0, 1.0});
		const StandingOptions Options = LateralLoad(0.3);
		const Scene Alone = MakeScene(Cube, {});
		const Scene WithSlab = MakeScene(Cube, {Slab});
		const StandingResult AloneResult = SolveWith(Alone.Model, Options);
		const StandingResult WithSlabResult = SolveWith(WithSlab.Model, Options);
		CHECK(AloneResult.State == StandingState::Stands);
		CheckSameCubeVerdict(AloneResult, WithSlabResult, WithSlab.CubeBed, Alone.CubeWeightN);
	}
	SUBCASE("seeded small boxes beside large unconnected slabs")
	{
		int StandsAlone = 0;
		int MechanismAlone = 0;
		for (int Draw = 0; Draw < DrawCount; ++Draw)
		{
			const std::uint32_t Seed = MasterSeed + static_cast<std::uint32_t>(Draw);
			INFO("seed " << Seed);
			std::mt19937 Rng(Seed);
			const Box Cube = BoxOnGround(0.0, Vec3{Uniform(Rng, 0.005, 0.05), Uniform(Rng, 0.005, 0.05), Uniform(Rng, 0.005, 0.05)});
			const double Lambda = Uniform(Rng, 0.0, 0.5);
			const int SlabCount = 1 + static_cast<int>(Rng() % 3u);
			std::vector<Box> Slabs;
			for (int K = 0; K < SlabCount; ++K)
			{
				const double EdgeXM = Uniform(Rng, 1.0, 20.0);
				const double EdgeYM = Uniform(Rng, 1.0, 20.0);
				const double EdgeZM = Uniform(Rng, 1.0, std::min(EdgeXM, EdgeYM));
				Slabs.push_back(BoxOnGround(100.0 * (K + 1), Vec3{EdgeXM, EdgeYM, EdgeZM}));
			}
			INFO("cube edges " << Times(2.0, Cube.HalfExtentsM) << ", lambda " << Lambda << ", slabs " << SlabCount);
			const Scene Alone = MakeScene(Cube, {});
			const Scene WithSlabs = MakeScene(Cube, Slabs);
			const StandingResult AloneResult = SolveWith(Alone.Model, LateralLoad(Lambda));
			const StandingResult WithSlabsResult = SolveWith(WithSlabs.Model, LateralLoad(Lambda));
			StandsAlone += AloneResult.State == StandingState::Stands ? 1 : 0;
			MechanismAlone += AloneResult.State == StandingState::Mechanism ? 1 : 0;
			CheckSameCubeVerdict(AloneResult, WithSlabsResult, WithSlabs.CubeBed, Alone.CubeWeightN);
		}
		INFO("stands alone " << StandsAlone << ", mechanism alone " << MechanismAlone);
		CHECK(StandsAlone > 0);
		CHECK(MechanismAlone > 0);
	}
}
