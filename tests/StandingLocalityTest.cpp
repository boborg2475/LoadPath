#include "CatalogueTestSupport.h"
#include "StaticsTestSupport.h"
#include "SlowInDebug.h"

#include "loadpath/Catalogue.h"
#include "loadpath/StructuralAuthority.h"
#include "loadpath/geometry/RunningBondWall.h"

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <random>
#include <string>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double StoneDensityKgPerM3 = 2000.0;
constexpr double CubeFriction = 0.7;
constexpr double LocalityRelative = 1.0e-12;
constexpr double ReadingRelative = 1.0e-9;
constexpr std::uint32_t MasterSeed = 20261011u;
constexpr int DrawCount = 50;
#if defined(NDEBUG)
constexpr int CombinedDrawCount = 50;
#else
constexpr int CombinedDrawCount = 15;
#endif

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

namespace
{
struct MixedScene
{
	Structure Model;
	std::vector<BodyId> Free;
	std::vector<BodyLoad> Loads;
	std::vector<std::vector<BodyId>> SiteBodies;
	std::vector<std::vector<InterfaceId>> SiteInterfaces;
	std::vector<double> SiteWeightN;
	std::vector<double> SiteSizeM;
};

MixedScene MakeMixedScene(std::mt19937& Rng)
{
	MixedScene S;
	const BodyId Ground = AddGroundPlane(S.Model);
	const int SiteCount = 2 + static_cast<int>(Rng() % 3u);
	for (int K = 0; K < SiteCount; ++K)
	{
		const Vec3 EdgesM{Uniform(Rng, 0.05, 0.5), Uniform(Rng, 0.05, 0.5), Uniform(Rng, 0.05, 0.5)};
		const int Height = 1 + static_cast<int>(Rng() % 3u);
		StrengthProfile Profile = DryJoint(Uniform(Rng, 0.3, 0.9));
		if (Rng() % 2u == 0u)
		{
			Profile.TensileStrengthPa = 1.0e4;
			Profile.ShearCohesionPa = 1.0e4;
			Profile.CompressiveStrengthPa = Uniform(Rng, 0.5, 2.0) * StoneDensityKgPerM3 * GravityMPerS2 * EdgesM.Z * Height;
		}
		const ProfileId P = AddProfileOk(S.Model, Profile);
		S.SiteBodies.emplace_back();
		S.SiteInterfaces.emplace_back();
		S.SiteWeightN.push_back(0.0);
		S.SiteSizeM.push_back(std::max({EdgesM.X, EdgesM.Y, EdgesM.Z * Height}));
		BodyId Below{};
		Box BelowShape{};
		for (int Level = 0; Level < Height; ++Level)
		{
			const Box Shape = BoxOfDensity(Vec3{10.0 * K, 0.0, (Level + 0.5) * EdgesM.Z}, Times(0.5, EdgesM), StoneDensityKgPerM3);
			const BodyId Id = AddBox(S.Model, Shape);
			const InterfaceId Joint = Level == 0 ? AddInterfaceOk(S.Model, OnGroundPlane(Ground, Id, Shape, P)) : AddInterfaceOk(S.Model, *StackedFaces(Below, BelowShape, Id, Shape, P));
			S.Free.push_back(Id);
			S.SiteBodies.back().push_back(Id);
			S.SiteInterfaces.back().push_back(Joint);
			S.SiteWeightN.back() += Shape.MassKg * GravityMPerS2;
			if (Rng() % 2u == 0u)
			{
				const double PushN = Uniform(Rng, 0.0, 1.0) * Shape.MassKg * GravityMPerS2;
				S.Loads.push_back(BodyLoad{Id, Vec3{PushN, 0.0, 0.0}, Shape.CentreM});
			}
			Below = Id;
			BelowShape = Shape;
		}
	}
	return S;
}

const InterfaceReading* FindIn(const std::vector<InterfaceReading>& Readings, InterfaceId Id)
{
	for (const InterfaceReading& Reading : Readings)
	{
		if (SameHandle(Reading.Id, Id))
		{
			return &Reading;
		}
	}
	return nullptr;
}

bool Matches(double Got, double Want, double ScaleAbs)
{
	if (std::isnan(Got) || std::isnan(Want))
	{
		return std::isnan(Got) && std::isnan(Want);
	}
	if (std::isinf(Got) || std::isinf(Want))
	{
		return Got == Want;
	}
	return std::abs(Got - Want) <= ReadingRelative * std::max({std::abs(Got), std::abs(Want), ScaleAbs});
}

void CheckMatch(double Got, double Want, double ScaleAbs, const char* What)
{
	INFO(doctest::String(What) << ": whole " << Got << ", component alone " << Want);
	CHECK(Matches(Got, Want, ScaleAbs));
}

void CheckMatchVec3(const Vec3& Got, const Vec3& Want, double ScaleAbs, const char* What)
{
	CheckMatch(Got.X, Want.X, ScaleAbs, What);
	CheckMatch(Got.Y, Want.Y, ScaleAbs, What);
	CheckMatch(Got.Z, Want.Z, ScaleAbs, What);
}

void CheckSameReadings(const std::vector<InterfaceReading>& Whole, const std::vector<InterfaceReading>& Alone, const std::vector<InterfaceId>& Interfaces, double WeightN, double SizeM)
{
	CHECK(Alone.size() == Interfaces.size());
	for (const InterfaceId Id : Interfaces)
	{
		const InterfaceReading* W = FindIn(Whole, Id);
		const InterfaceReading* A = FindIn(Alone, Id);
		INFO("interface index " << Id.Index);
		CHECK(W != nullptr);
		CHECK(A != nullptr);
		if (W == nullptr || A == nullptr)
		{
			continue;
		}
		CheckMatch(W->Utilisation, A->Utilisation, 1.0, "utilisation");
		CheckMatch(W->NormalForceN, A->NormalForceN, WeightN, "normal force");
		CheckMatchVec3(W->ShearForceN, A->ShearForceN, WeightN, "shear force");
		CheckMatchVec3(W->MomentNm, A->MomentNm, WeightN * SizeM, "moment");
		for (std::size_t C = 0; C < 4; ++C)
		{
			INFO("corner " << C);
			CheckMatch(W->Corners[C].NormalForceN, A->Corners[C].NormalForceN, WeightN, "corner normal force");
			CheckMatchVec3(W->Corners[C].ShearForceN, A->Corners[C].ShearForceN, WeightN, "corner shear force");
		}
	}
}

int SiteOf(const MixedScene& S, BodyId Body)
{
	for (std::size_t Site = 0; Site < S.SiteBodies.size(); ++Site)
	{
		if (HasBody(S.SiteBodies[Site], Body))
		{
			return static_cast<int>(Site);
		}
	}
	return -1;
}

bool SameBodyList(const std::vector<BodyId>& A, const std::vector<BodyId>& B)
{
	if (A.size() != B.size())
	{
		return false;
	}
	for (std::size_t I = 0; I < A.size(); ++I)
	{
		if (!SameHandle(A[I], B[I]))
		{
			return false;
		}
	}
	return true;
}
}

TEST_CASE("Statics.Combined.ComponentVerdictsAreLocalAcrossSeededStructures")
{
	const std::unique_ptr<IStructuralAuthority> Authority = MakeElasticAuthority(MakeRbsmStandingAuthority());
	int Overstressed = 0;
	int Undecided = 0;
	int MultiBody = 0;
	int Collapsing = 0;
	for (int Draw = 0; Draw < CombinedDrawCount; ++Draw)
	{
		const std::uint32_t Seed = MasterSeed + 1000u + static_cast<std::uint32_t>(Draw);
		INFO("seed " << Seed);
		std::mt19937 Rng(Seed);
		const MixedScene Whole = MakeMixedScene(Rng);
		const double TiltRad = Uniform(Rng, 0.0, 35.0) * RadPerDegree;
		AuthorityOptions Options;
		Options.Standing = TiltedPlane(TiltRad);
		Options.Standing.MaxActiveSetIterations = 1 + static_cast<int>(Rng() % 3u);
		INFO("tilt " << TiltRad / RadPerDegree << " deg, pass cap " << Options.Standing.MaxActiveSetIterations << ", bodies " << Whole.Free.size() << ", loads " << Whole.Loads.size());
		const AuthorityResult All = Authority->Judge(Whole.Model, Whole.Loads, Options, nullptr);
		CHECK(All.Components.size() == Whole.SiteBodies.size());
		for (const ComponentReport& Report : All.Components)
		{
			Structure Alone = Whole.Model;
			for (const BodyId Other : Whole.Free)
			{
				if (!HasBody(Report.Bodies, Other))
				{
					CHECK(Alone.RemoveBody(Other));
				}
			}
			std::vector<BodyLoad> Loads;
			for (const BodyLoad& Load : Whole.Loads)
			{
				if (HasBody(Report.Bodies, Load.Body))
				{
					Loads.push_back(Load);
				}
			}
			const AuthorityResult Single = Authority->Judge(Alone, Loads, Options, nullptr);
			CHECK(Single.Components.size() == 1u);
			if (Single.Components.size() != 1u)
			{
				continue;
			}
			const ComponentReport& Own = Single.Components.front();
			const int Site = SiteOf(Whole, Report.Bodies.front());
			CHECK(Site >= 0);
			if (Site >= 0)
			{
				const std::size_t Index = static_cast<std::size_t>(Site);
				CHECK(SameBodyList(Report.Bodies, Whole.SiteBodies[Index]));
				CheckSameReadings(All.Readings, Single.Readings, Whole.SiteInterfaces[Index], Whole.SiteWeightN[Index], Whole.SiteSizeM[Index]);
			}
			INFO("component " << Describe(Report.Bodies) << ": whole verdict " << static_cast<int>(Report.Verdict) << " elastic " << Report.Elastic << "; alone verdict " << static_cast<int>(Own.Verdict) << " elastic " << Own.Elastic);
			CHECK(SameBodyList(Own.Bodies, Report.Bodies));
			CHECK(Own.Verdict == Report.Verdict);
			CHECK(Own.Elastic == Report.Elastic);
			CHECK(SameBodyList(Own.Release, Report.Release));
			Overstressed += Own.Elastic == StandingState::Overstressed ? 1 : 0;
			Undecided += Own.Elastic == StandingState::Undecided ? 1 : 0;
			MultiBody += Report.Bodies.size() > 1u ? 1 : 0;
			Collapsing += Own.Verdict == ComponentVerdict::Collapses ? 1 : 0;
		}
	}
	MESSAGE("multi-body " << MultiBody << ", collapses " << Collapsing);
	INFO("overstressed " << Overstressed << ", undecided " << Undecided << ", multi-body " << MultiBody << ", collapses " << Collapsing);
	CHECK(MultiBody > 0);
	CHECK(Collapsing > 0);
	CHECK(Overstressed > 0);
	CHECK(Undecided > 0);
}
