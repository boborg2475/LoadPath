#include "BrickFixtures.h"
#include "CascadeTestSupport.h"
#include "CatalogueTestSupport.h"
#include "EventRegionTestSupport.h"
#include "StaticsTestSupport.h"

#include "loadpath/Catalogue.h"
#include "loadpath/geometry/RunningBondWall.h"

#include "doctest.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <string>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
struct FloatingPair
{
	BodyId Lower, Upper;
	InterfaceId Joint;
};

FloatingPair AddFloatingPair(Structure& Model, ProfileId Profile, double CentreXM)
{
	FloatingPair P;
	const Box LowerShape = BrickShape(CentreXM, 1);
	const Box UpperShape = BrickShape(CentreXM, 2);
	P.Lower = AddBox(Model, LowerShape);
	P.Upper = AddBox(Model, UpperShape);
	P.Joint = AddInterfaceOk(Model, *StackedFaces(P.Lower, LowerShape, P.Upper, UpperShape, Profile));
	return P;
}

std::vector<BodyId> AddGroundedStack(Structure& Model, ProfileId Profile, BodyId Ground, int Count)
{
	std::vector<BodyId> Bricks;
	for (int Course = 0; Course < Count; ++Course)
	{
		Bricks.push_back(AddBox(Model, BrickShape(0.0, Course)));
	}
	AddInterfaceOk(Model, OnGroundPlane(Ground, Bricks[0], BrickShape(0.0, 0), Profile));
	for (int Course = 1; Course < Count; ++Course)
	{
		AddInterfaceOk(Model, *StackedFaces(Bricks[static_cast<std::size_t>(Course - 1)], BrickShape(0.0, Course - 1), Bricks[static_cast<std::size_t>(Course)], BrickShape(0.0, Course), Profile));
	}
	return Bricks;
}

void Characterise(const StandingResult& R)
{
	MESSAGE("state " << R.State << ", iterations " << R.ActiveSetIterations << ", unconstrained " << R.UnconstrainedBodies.size() << ", mechanism bodies " << R.Mechanism.size() << ", readings " << R.Readings.size() << ", components " << R.Components.size());
	for (const StandingComponent& C : R.Components)
	{
		MESSAGE("component of " << C.Bodies.size() << " bodies: " << C.State << ", iterations " << C.ActiveSetIterations);
	}
	for (std::size_t K = 0; K < R.Mechanism.size() && K < 3; ++K)
	{
		MESSAGE("motion body " << R.Mechanism[K].Body.Index << ": translation " << R.Mechanism[K].TranslationM << ", rotation " << R.Mechanism[K].RotationRad);
	}
	for (std::size_t K = 0; K < R.Readings.size() && K < 3; ++K)
	{
		const InterfaceReading& Reading = R.Readings[K];
		MESSAGE("reading " << K << ": utilisation " << Reading.Utilisation << " " << Reading.Axis << ", normal " << Reading.NormalForceN << ", shear " << Reading.ShearForceN << ", moment " << Reading.MomentNm << ", corner normals " << Reading.Corners[0].NormalForceN << " " << Reading.Corners[1].NormalForceN << " " << Reading.Corners[2].NormalForceN << " " << Reading.Corners[3].NormalForceN);
	}
}

void CheckRigidFall(const StandingResult& R, const std::vector<BodyId>& Bodies, const Vec3& Fall)
{
	CHECK(R.State == StandingState::Mechanism);
	CHECK(R.ActiveSetIterations == 0);
	CHECK(SameIdList(R.UnconstrainedBodies, Bodies));
	REQUIRE(R.Mechanism.size() == Bodies.size());
	for (const MechanismMotion& M : R.Mechanism)
	{
		INFO("body " << M.Body.Index << ": translation " << M.TranslationM << ", rotation " << M.RotationRad);
		CHECK(NearVec(M.TranslationM, Fall, 1.0e-12, 1.0e-12));
		CHECK(NearVec(M.RotationRad, Vec3{0.0, 0.0, 0.0}, 0.0, 1.0e-12));
	}
}

std::vector<BodyId> LiveFree(const Structure& Model, const geometry::Wall& W)
{
	std::vector<BodyId> Out;
	for (const std::vector<BodyId>& Course : W.Courses)
	{
		for (const BodyId& Id : Course)
		{
			if (Model.IsLive(Id))
			{
				Out.push_back(Id);
			}
		}
	}
	std::sort(Out.begin(), Out.end(), [](const BodyId& A, const BodyId& B) { return A.Index < B.Index; });
	return Out;
}

geometry::Wall BuildFloatingWall(Structure& Model, const StrengthProfile& Bed, const StrengthProfile& Perpend, int Courses, int BricksPerCourse)
{
	geometry::WallSpec Spec;
	Spec.Bed = AddProfileOk(Model, Bed);
	Spec.Perpend = AddProfileOk(Model, Perpend);
	Spec.Courses = Courses;
	Spec.BricksPerCourse = BricksPerCourse;
	geometry::Wall W = geometry::BuildRunningBondWall(Model, Spec);
	REQUIRE(W.bComplete);
	for (const BodyId& Id : W.Courses[0])
	{
		REQUIRE(Model.RemoveBody(Id));
	}
	return W;
}
void CheckUncertifiedReadings(const StandingResult& R, std::size_t Expected)
{
	CHECK(R.Readings.size() == Expected);
	for (const InterfaceReading& Reading : R.Readings)
	{
		INFO("reading " << Reading.Id.Index);
		CHECK(Reading.Utilisation == 0.0);
		CHECK(Reading.Axis == GoverningAxis::None);
		CHECK(Reading.NormalForceN == 0.0);
		CHECK(NearVec(Reading.ShearForceN, Vec3{}, 0.0, 0.0));
		CHECK(NearVec(Reading.MomentNm, Vec3{}, 0.0, 0.0));
		for (const CornerReading& Corner : Reading.Corners)
		{
			CHECK(Corner.NormalForceN == 0.0);
		}
	}
}

void CheckCornerPointsMatchAGroundedSolve(const StandingResult& R, InterfaceId Joint)
{
	Structure Grounded;
	const ProfileId Profile = AddProfileOk(Grounded, DryJoint(0.7));
	const BodyId Ground = AddGroundPlane(Grounded);
	const BodyId Base = AddBox(Grounded, BrickShape(0.0, 0));
	AddInterfaceOk(Grounded, OnGroundPlane(Ground, Base, BrickShape(0.0, 0), Profile));
	const BodyId Lower = AddBox(Grounded, BrickShape(0.0, 1));
	const BodyId Upper = AddBox(Grounded, BrickShape(0.0, 2));
	AddInterfaceOk(Grounded, *StackedFaces(Base, BrickShape(0.0, 0), Lower, BrickShape(0.0, 1), Profile));
	const InterfaceId GroundedJoint = AddInterfaceOk(Grounded, *StackedFaces(Lower, BrickShape(0.0, 1), Upper, BrickShape(0.0, 2), Profile));
	const StandingResult G = SolveWith(Grounded, StandingOptions{});
	REQUIRE(G.State == StandingState::Stands);
	const InterfaceReading* Floating = FindReading(R, Joint);
	const InterfaceReading* Reference = FindReading(G, GroundedJoint);
	REQUIRE(Floating != nullptr);
	REQUIRE(Reference != nullptr);
	for (std::size_t K = 0; K < Floating->Corners.size(); ++K)
	{
		INFO("corner " << K << ": " << Floating->Corners[K].PointM << " against " << Reference->Corners[K].PointM);
		CHECK(NearVec(Floating->Corners[K].PointM, Reference->Corners[K].PointM, 0.0, 0.0));
	}
}

AuthorityResult JudgeWith(const Structure& Model, const AuthorityOptions& Options, std::span<const BodyLoad> Loads = {})
{
	return MakeAuthority()->Judge(Model, Loads, Options, nullptr);
}

CascadeVerdict CascadeWith(Structure& Model, const AuthorityOptions& Options, std::span<const BodyLoad> Loads = {})
{
	CascadeOptions C;
	C.Authority = Options;
	return MakeCascade(MakeAuthority())->Run(Model, Loads, C, nullptr);
}
}

TEST_CASE("Statics.Ungrounded.AFloatingPairIsAMechanismWithoutIterating")
{
	Structure Model;
	const ProfileId Profile = AddProfileOk(Model, DryJoint(0.7));
	AddGroundPlane(Model);
	const FloatingPair P = AddFloatingPair(Model, Profile, 0.0);
	SUBCASE("gravity only")
	{
		const StandingResult R = SolveWith(Model, StandingOptions{});
		Characterise(R);
		CheckRigidFall(R, {P.Lower, P.Upper}, Vec3{0.0, 0.0, -1.0});
		CheckUncertifiedReadings(R, 1);
		CheckCornerPointsMatchAGroundedSolve(R, P.Joint);
	}
	SUBCASE("a sideways push on the upper brick takes the full solve")
	{
		const BodyLoad Loads[] = {BodyLoad{P.Upper, Vec3{30.0, 0.0, 0.0}, BrickShape(0.0, 2).CentreM}};
		const StandingResult R = SolveWith(Model, StandingOptions{}, Loads);
		Characterise(R);
		CHECK(R.State == StandingState::Mechanism);
		CHECK(R.ActiveSetIterations >= 1);
		CHECK(R.UnconstrainedBodies.size() == 2u);
	}
	SUBCASE("a zero-force load is no load")
	{
		const BodyLoad Loads[] = {BodyLoad{P.Upper, Vec3{0.0, 0.0, 0.0}, BrickShape(0.0, 2).CentreM}};
		const StandingResult R = SolveWith(Model, StandingOptions{}, Loads);
		Characterise(R);
		CheckRigidFall(R, {P.Lower, P.Upper}, Vec3{0.0, 0.0, -1.0});
		CheckUncertifiedReadings(R, 1);
	}
}

TEST_CASE("Statics.Ungrounded.TheFloatingRemainderOfTheWallReadsTheSame")
{
	Structure Model;
	const geometry::Wall W = BuildFloatingWall(Model, DryJoint(0.7), DryJoint(0.7), 10, 8);
	const std::vector<BodyId> Floating = LiveFree(Model, W);
	CHECK(Floating.size() == 77u);
	const StandingResult R = SolveWith(Model, StandingOptions{});
	Characterise(R);
	CheckRigidFall(R, Floating, Vec3{0.0, 0.0, -1.0});
}

TEST_CASE("Statics.Ungrounded.AGroundedComponentBesideAFloatingOneIsUnchanged")
{
	Structure Alone;
	const ProfileId AloneProfile = AddProfileOk(Alone, DryJoint(0.7));
	const BodyId AloneGround = AddGroundPlane(Alone);
	AddGroundedStack(Alone, AloneProfile, AloneGround, 3);
	Structure Both;
	const ProfileId BothProfile = AddProfileOk(Both, DryJoint(0.7));
	const BodyId BothGround = AddGroundPlane(Both);
	const std::vector<BodyId> Stack = AddGroundedStack(Both, BothProfile, BothGround, 3);
	const FloatingPair P = AddFloatingPair(Both, BothProfile, 2.0);
	const BodyLoad Push[] = {BodyLoad{Stack[2], Vec3{5.0, 0.0, 0.0}, BrickShape(0.0, 2).CentreM}};
	const StandingResult A = SolveWith(Alone, StandingOptions{}, Push);
	const StandingResult R = SolveWith(Both, StandingOptions{}, Push);
	Characterise(A);
	Characterise(R);
	REQUIRE(R.Components.size() == 2u);
	REQUIRE(A.Components.size() == 1u);
	const StandingComponent& Grounded = R.Components[0];
	CHECK(SameIdList(Grounded.Bodies, Stack));
	CHECK(Grounded.State == A.Components[0].State);
	CHECK(Grounded.ActiveSetIterations == A.Components[0].ActiveSetIterations);
	REQUIRE(Grounded.ReadingSlots.size() == A.Readings.size());
	for (std::size_t K = 0; K < A.Readings.size(); ++K)
	{
		INFO("reading " << K);
		CHECK(SameReading(R.Readings[Grounded.ReadingSlots[K]], A.Readings[K], 0.0, 0.0));
	}
	CHECK(SameIdList(R.Components[1].Bodies, {P.Lower, P.Upper}));
	CHECK(R.Components[1].State == StandingState::Mechanism);
	CHECK(SameIdList(R.UnconstrainedBodies, {P.Lower, P.Upper}));
	REQUIRE(R.Mechanism.size() == 2u);
	for (const MechanismMotion& M : R.Mechanism)
	{
		INFO("body " << M.Body.Index << ": translation " << M.TranslationM << ", rotation " << M.RotationRad);
		CHECK(NearVec(M.TranslationM, Vec3{0.0, 0.0, -1.0}, 1.0e-12, 1.0e-12));
		CHECK(NearVec(M.RotationRad, Vec3{0.0, 0.0, 0.0}, 0.0, 1.0e-12));
	}
}

TEST_CASE("Slow.Statics.Ungrounded.TheFloatingWallRemainderSolvesInsideItsBudget")
{
	const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
	REQUIRE(Loaded.Ok());
	Structure Model;
	const geometry::Wall W = BuildFloatingWall(Model, JointOrFail(Loaded.Value, "clay_gp_mortar_bed_ec6").Profile, JointOrFail(Loaded.Value, "clay_gp_mortar_perpend_ec6").Profile, 32, 40);
	const std::vector<BodyId> Floating = LiveFree(Model, W);
	const std::chrono::steady_clock::time_point Start = std::chrono::steady_clock::now();
	const StandingResult R = SolveWith(Model, StandingOptions{});
	const double Ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Start).count();
	MESSAGE("floating remainder of " << Floating.size() << " bricks solved in " << Ms << " ms");
	Characterise(R);
	CHECK(R.State == StandingState::Mechanism);
	CHECK(SameIdList(R.UnconstrainedBodies, Floating));
	CHECK(Ms < 50.0);
}

TEST_CASE("Statics.Ungrounded.AHeldFloatingBrickStandsAsBefore")
{
	SUBCASE("a lone brick held at its centroid")
	{
		Structure Model;
		AddGroundPlane(Model);
		const BodyId Brick = AddBox(Model, BrickShape(0.0, 3));
		const double Weight = Model.Body(Brick).MassKg * GravityMPerS2;
		const BodyLoad Hold[] = {BodyLoad{Brick, Vec3{0.0, 0.0, Weight}, BrickShape(0.0, 3).CentreM}};
		const StandingResult R = SolveWith(Model, StandingOptions{}, Hold);
		Characterise(R);
		CHECK(R.State == StandingState::Stands);
		const AuthorityResult A = JudgeWith(Model, AuthorityOptions{}, Hold);
		CHECK(A.Worst == ComponentVerdict::Stands);
		CHECK(A.Disagreements == 0u);
	}
	SUBCASE("a dry pair held at the lower brick")
	{
		Structure Model;
		const ProfileId Profile = AddProfileOk(Model, DryJoint(0.7));
		AddGroundPlane(Model);
		const FloatingPair P = AddFloatingPair(Model, Profile, 0.0);
		const double LowerWeight = Model.Body(P.Lower).MassKg * GravityMPerS2;
		const double UpperWeight = Model.Body(P.Upper).MassKg * GravityMPerS2;
		const BodyLoad Hold[] = {BodyLoad{P.Lower, Vec3{0.0, 0.0, LowerWeight + UpperWeight}, BrickShape(0.0, 1).CentreM}};
		const StandingResult R = SolveWith(Model, StandingOptions{}, Hold);
		Characterise(R);
		CHECK(R.State == StandingState::Stands);
		const InterfaceReading* Reading = FindReading(R, P.Joint);
		REQUIRE(Reading != nullptr);
		INFO("normal " << Reading->NormalForceN << " against the upper weight " << UpperWeight);
		CHECK(Reading->Axis == GoverningAxis::Compression);
		CHECK(std::abs(Reading->NormalForceN - UpperWeight) <= 1.0e-9);
		const AuthorityResult A = JudgeWith(Model, AuthorityOptions{}, Hold);
		CHECK(A.Worst == ComponentVerdict::Stands);
		CHECK(A.Disagreements == 0u);
	}
}

TEST_CASE("Statics.Ungrounded.ANonFiniteFloatingPairIsInvalidInput")
{
	Structure Model;
	StrengthProfile Stiff = DryJoint(0.7);
	Stiff.NormalStiffnessPaPerM = 1.0e308;
	Stiff.TangentialStiffnessPaPerM = 1.0e308;
	const ProfileId Profile = AddProfileOk(Model, Stiff);
	AddGroundPlane(Model);
	const Box LowerShape = BoxOfDensity(Vec3{0.0, 0.0, 15.0}, Vec3{5.0, 5.0, 5.0}, 2000.0);
	const Box UpperShape = BoxOfDensity(Vec3{0.0, 0.0, 25.0}, Vec3{5.0, 5.0, 5.0}, 2000.0);
	const BodyId Lower = AddBox(Model, LowerShape);
	const BodyId Upper = AddBox(Model, UpperShape);
	AddInterfaceOk(Model, *StackedFaces(Lower, LowerShape, Upper, UpperShape, Profile));
	const StandingResult R = SolveWith(Model, StandingOptions{});
	Characterise(R);
	CHECK(R.State == StandingState::InvalidInput);
	CHECK(JudgeWith(Model, AuthorityOptions{}).Worst == ComponentVerdict::InvalidInput);
	const CascadeVerdict V = CascadeWith(Model, AuthorityOptions{});
	CHECK(V.State == CascadeState::InvalidInput);
	CHECK(V.Clusters.empty());
}

TEST_CASE("Statics.Ungrounded.AWeightlessFloatingPairSolvesAsBefore")
{
	Structure Model;
	const ProfileId Profile = AddProfileOk(Model, DryJoint(0.7));
	AddGroundPlane(Model);
	AddFloatingPair(Model, Profile, 0.0);
	const AuthorityOptions Weightless = AuthorityUnderGravity(Vec3{0.0, 0.0, 0.0});
	const StandingResult R = SolveWith(Model, Weightless.Standing);
	Characterise(R);
	CHECK(R.State == StandingState::Stands);
	CHECK(R.ActiveSetIterations == 2);
	CHECK(R.Mechanism.empty());
	const AuthorityResult A = JudgeWith(Model, Weightless);
	CHECK(A.Worst == ComponentVerdict::Stands);
	CHECK(A.Disagreements == 0u);
}

TEST_CASE("Statics.Ungrounded.TheFallFollowsATiltedGravity")
{
	Structure Model;
	const ProfileId Profile = AddProfileOk(Model, DryJoint(0.7));
	AddGroundPlane(Model);
	const FloatingPair P = AddFloatingPair(Model, Profile, 0.0);
	const StandingResult R = SolveWith(Model, AuthorityTilted(0.5).Standing);
	Characterise(R);
	CheckRigidFall(R, {P.Lower, P.Upper}, Vec3{std::tan(0.5), 0.0, -1.0});
}

namespace
{
BodyId AddStiffFloatingPair(Structure& Model, double StiffnessPaPerM)
{
	StrengthProfile Stiff = DryJoint(0.7);
	Stiff.NormalStiffnessPaPerM = StiffnessPaPerM;
	Stiff.TangentialStiffnessPaPerM = StiffnessPaPerM;
	const ProfileId Profile = AddProfileOk(Model, Stiff);
	AddGroundPlane(Model);
	const Box LowerShape = BoxOfDensity(Vec3{0.0, 0.0, 15.0}, Vec3{5.0, 5.0, 5.0}, 2000.0);
	const Box UpperShape = BoxOfDensity(Vec3{0.0, 0.0, 25.0}, Vec3{5.0, 5.0, 5.0}, 2000.0);
	const BodyId Lower = AddBox(Model, LowerShape);
	const BodyId Upper = AddBox(Model, UpperShape);
	AddInterfaceOk(Model, *StackedFaces(Lower, LowerShape, Upper, UpperShape, Profile));
	return Upper;
}
}

namespace
{
BodyId AddScaledFloatingWall(Structure& Model, ProfileId Profile)
{
	geometry::WallSpec Spec;
	Spec.Bed = Profile;
	Spec.Perpend = Profile;
	Spec.Courses = 4;
	Spec.BricksPerCourse = 3;
	Spec.BrickM = Vec3{0.215 * 40.0, 0.1025 * 40.0, 0.065 * 40.0};
	geometry::Wall W = geometry::BuildRunningBondWall(Model, Spec);
	REQUIRE(W.bComplete);
	for (const BodyId& Id : W.Courses[0])
	{
		REQUIRE(Model.RemoveBody(Id));
	}
	return W.Courses.back().back();
}

BodyId AddBigPairWith(Structure& Model, ProfileId Profile)
{
	AddGroundPlane(Model);
	const Box LowerShape = BoxOfDensity(Vec3{0.0, 0.0, 15.0}, Vec3{5.0, 5.0, 5.0}, 2000.0);
	const Box UpperShape = BoxOfDensity(Vec3{0.0, 0.0, 25.0}, Vec3{5.0, 5.0, 5.0}, 2000.0);
	const BodyId Lower = AddBox(Model, LowerShape);
	const BodyId Upper = AddBox(Model, UpperShape);
	AddInterfaceOk(Model, *StackedFaces(Lower, LowerShape, Upper, UpperShape, Profile));
	return Upper;
}

struct OverflowFixture
{
	const char* Name;
	BodyId (*Build)(Structure&, ProfileId);
};

StrengthProfile StiffJoint(double NormalPaPerM, double TangentialPaPerM)
{
	StrengthProfile P = DryJoint(0.7);
	P.NormalStiffnessPaPerM = NormalPaPerM;
	P.TangentialStiffnessPaPerM = TangentialPaPerM;
	return P;
}
}

TEST_CASE("Statics.Ungrounded.AnOverflowingFloatingPairIsInvalidInputAsOnTheFullPath")
{
	const OverflowFixture Fixtures[] = {OverflowFixture{"big pair", AddBigPairWith}, OverflowFixture{"4 x 3 wall scaled 40", AddScaledFloatingWall}};
	const int Points = 100;
	const double LowExponent = 300.0;
	const double HighExponent = std::log10(std::numeric_limits<double>::max());
	int BigPairFinite = 0;
	int BigPairInvalid = 0;
	for (const OverflowFixture& F : Fixtures)
	{
		for (int Mode = 0; Mode < 3; ++Mode)
		{
			for (const int DenseLimit : {600, 0})
			{
				int Agree = 0;
				int Invalid = 0;
				double LastFinite = 0.0;
				double FirstInvalid = 0.0;
				for (int I = 0; I < Points; ++I)
				{
					const double K = I == Points - 1 ? std::numeric_limits<double>::max() : std::pow(10.0, LowExponent + (HighExponent - LowExponent) * I / (Points - 1));
					const double Kn = Mode == 2 ? 1.0e9 : K;
					const double Kt = Mode == 1 ? 1.0e9 : K;
					StandingOptions Options;
					Options.DenseDofLimit = DenseLimit;
					Structure Unloaded;
					F.Build(Unloaded, AddProfileOk(Unloaded, StiffJoint(Kn, Kt)));
					const StandingResult RU = SolveWith(Unloaded, Options);
					Structure Loaded;
					const BodyId Pushed = F.Build(Loaded, AddProfileOk(Loaded, StiffJoint(Kn, Kt)));
					const BodyLoad Loads[] = {BodyLoad{Pushed, Vec3{1.0, 0.0, 0.0}, Loaded.Body(Pushed).CentroidM}};
					const StandingResult RL = SolveWith(Loaded, Options, Loads);
					const bool UnloadedInvalid = RU.State == StandingState::InvalidInput;
					const bool LoadedInvalid = RL.State == StandingState::InvalidInput;
					INFO(std::string(F.Name) << ", mode " << Mode << ", dense limit " << DenseLimit << ", k " << K << ": unloaded " << RU.State << ", loaded " << RL.State);
					CHECK(UnloadedInvalid == LoadedInvalid);
					Agree += UnloadedInvalid == LoadedInvalid ? 1 : 0;
					if (UnloadedInvalid)
					{
						++Invalid;
						if (FirstInvalid == 0.0)
						{
							FirstInvalid = K;
						}
					}
					else
					{
						LastFinite = K;
					}
					if (F.Build == AddBigPairWith && Mode == 0)
					{
						(UnloadedInvalid ? BigPairInvalid : BigPairFinite) += 1;
					}
				}
				MESSAGE(std::string(F.Name) << ", mode " << Mode << ", dense limit " << DenseLimit << ": " << Agree << " of " << Points << " agree, " << Invalid << " InvalidInput, last finite " << LastFinite << ", first InvalidInput " << FirstInvalid);
			}
		}
	}
	CHECK(BigPairFinite > 0);
	CHECK(BigPairInvalid > 0);
	{
		INFO("unloaded at 1e305");
		Structure Model;
		AddStiffFloatingPair(Model, 1.0e305);
		const StandingResult R = SolveWith(Model, StandingOptions{});
		CHECK(R.State == StandingState::InvalidInput);
		CHECK(JudgeWith(Model, AuthorityOptions{}).Worst == ComponentVerdict::InvalidInput);
		const CascadeVerdict V = CascadeWith(Model, AuthorityOptions{});
		CHECK(V.State == CascadeState::InvalidInput);
		CHECK(V.Clusters.empty());
	}
	{
		INFO("a 1 N push on the upper box at 1e305");
		Structure Model;
		const BodyId Upper = AddStiffFloatingPair(Model, 1.0e305);
		const BodyLoad Loads[] = {BodyLoad{Upper, Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 0.0, 25.0}}};
		const StandingResult R = SolveWith(Model, StandingOptions{}, Loads);
		CHECK(R.State == StandingState::InvalidInput);
		CHECK(JudgeWith(Model, AuthorityOptions{}, Loads).Worst == ComponentVerdict::InvalidInput);
		const CascadeVerdict V = CascadeWith(Model, AuthorityOptions{}, Loads);
		CHECK(V.State == CascadeState::InvalidInput);
		CHECK(V.Clusters.empty());
	}
}

TEST_CASE("Statics.Ungrounded.ATinyGravityStillFallsAlongIt")
{
	struct Case
	{
		Vec3 Gravity;
		Vec3 Fall;
	};
	const Case Cases[] = {
		Case{Vec3{0.0, 0.0, -1.0e-310}, Vec3{0.0, 0.0, -1.0}},
		Case{Vec3{1.0e-310, 0.0, -2.0e-310}, Vec3{0.5, 0.0, -1.0}},
		Case{Vec3{0.0, 0.0, -1.0e-163}, Vec3{0.0, 0.0, -1.0}},
		Case{Vec3{0.0, 0.0, -1.0e-161}, Vec3{0.0, 0.0, -1.0}},
	};
	for (const Case& C : Cases)
	{
		INFO("gravity (" << C.Gravity.X << ", " << C.Gravity.Y << ", " << C.Gravity.Z << ")");
		Structure Model;
		const ProfileId Profile = AddProfileOk(Model, DryJoint(0.7));
		AddGroundPlane(Model);
		const FloatingPair P = AddFloatingPair(Model, Profile, 0.0);
		const AuthorityOptions Tiny = AuthorityUnderGravity(C.Gravity);
		const StandingResult R = SolveWith(Model, Tiny.Standing);
		Characterise(R);
		CheckRigidFall(R, {P.Lower, P.Upper}, C.Fall);
		const AuthorityResult A = JudgeWith(Model, Tiny);
		MESSAGE("judge worst " << static_cast<int>(A.Worst) << ", disagreements " << A.Disagreements);
		CHECK(A.Worst == ComponentVerdict::Collapses);
		CHECK(A.Disagreements == 0);
	}
}
