#include "BrickFixtures.h"
#include "CascadeTestSupport.h"
#include "CatalogueTestSupport.h"
#include "SlowInDebug.h"
#include "StaticAdmissibilityOracle.h"

#include "loadpath/Cascade.h"
#include "loadpath/Catalogue.h"
#include "loadpath/StructureFile.h"
#include "loadpath/geometry/RunningBondWall.h"

#include "doctest.h"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <sstream>
#include <string>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr int FuzzMaxPasses = 16;
constexpr double FuzzCrackBand = 0.5;
constexpr int FuzzDraws = oracle::bOptimisedBuild ? 40 : 8;
constexpr std::uint64_t FuzzSeedBase = 0x5c0ffee5ull;
constexpr int LimeFuzzDraws = oracle::bOptimisedBuild ? 40 : 8;
constexpr std::uint64_t LimeFuzzSeedBase = 0x11e5eed5ull;
constexpr int EccentricFuzzDraws = oracle::bOptimisedBuild ? 6 : 3;
constexpr std::uint64_t EccentricFuzzSeedBase = 0xecc0c0deull;

class SplitMix
{
public:
	explicit SplitMix(std::uint64_t Seed) : State(Seed) {}

	std::uint64_t Next()
	{
		std::uint64_t Z = (State += 0x9e3779b97f4a7c15ull);
		Z = (Z ^ (Z >> 30)) * 0xbf58476d1ce4e5b9ull;
		Z = (Z ^ (Z >> 27)) * 0x94d049bb133111ebull;
		return Z ^ (Z >> 31);
	}

	double Uniform(double Low, double High) { return Low + (High - Low) * (static_cast<double>(Next() >> 11) * 0x1.0p-53); }

	int Between(int Low, int High) { return Low + static_cast<int>(Next() % static_cast<std::uint64_t>(High - Low + 1)); }

private:
	std::uint64_t State;
};

struct FuzzDraw
{
	Structure Model;
	std::vector<InterfaceId> AllInterfaces;
	std::vector<BodyLoad> Loads;
	AuthorityOptions Options;
	std::string Description;
};

const JointRow& CatalogueJoint(const char* Name)
{
	static const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
	REQUIRE(Loaded.Ok());
	return JointOrFail(Loaded.Value, Name);
}

FuzzDraw MakeDraw(std::uint64_t Seed, bool bLime)
{
	FuzzDraw D;
	SplitMix Rng(Seed);
	std::ostringstream Out;
	const int Family = Rng.Between(0, 2);
	const double Friction = Rng.Uniform(0.4, 0.9);
	const double Weakening = std::pow(10.0, Rng.Uniform(-4.0, -2.0));
	const double Crushing = std::pow(10.0, Rng.Uniform(-4.0, -2.5));
	StrengthProfile Bed = DryJoint(Friction);
	StrengthProfile Perpend = DryJoint(Friction);
	if (Family > 0)
	{
		Bed = CatalogueJoint("clay_gp_mortar_bed_ec6").Profile;
		Perpend = CatalogueJoint("clay_gp_mortar_perpend_ec6").Profile;
		Out << "bonded ec6";
	}
	else if (bLime)
	{
		Bed = CatalogueJoint("lime_mortar_handmade_brick_upc_mean").Profile;
		Perpend = Bed;
		Out << "lime";
	}
	else
	{
		Out << "dry mu " << Friction;
	}
	if (Family == 2)
	{
		for (StrengthProfile* Profile : {&Bed, &Perpend})
		{
			Profile->TensileStrengthPa *= Weakening;
			Profile->ShearCohesionPa *= Weakening;
			Profile->CompressiveStrengthPa *= Crushing;
			Profile->MaxShearStrengthPa *= Weakening;
		}
		Out << " weakened x" << Weakening << ", crushing x" << Crushing;
	}
	if (Family == 2 || Rng.Between(0, 1) == 0)
	{
		const int Count = Rng.Between(2, 5);
		Out << ", stack of " << Count << " offsets";
		const ProfileId P = AddProfileOk(D.Model, Bed);
		const BodyId Ground = AddGroundPlane(D.Model);
		double X = 0.0;
		BodyId Lower{};
		Box LowerShape{};
		for (int K = 0; K < Count; ++K)
		{
			if (K > 0)
			{
				const double Shift = Rng.Uniform(-0.45, 0.45) * BrickLengthM;
				X += Shift;
				Out << " " << Shift;
			}
			const Box Shape = BrickShape(X, K);
			const BodyId Brick = AddBox(D.Model, Shape);
			if (K == 0)
			{
				AddInterfaceOk(D.Model, OnGroundPlane(Ground, Brick, Shape, P));
			}
			else
			{
				AddInterfaceOk(D.Model, *StackedFaces(Lower, LowerShape, Brick, Shape, P));
			}
			Lower = Brick;
			LowerShape = Shape;
		}
	}
	else
	{
		geometry::WallSpec Spec;
		Spec.Courses = Rng.Between(3, 6);
		Spec.BricksPerCourse = Rng.Between(3, 5);
		Spec.Bed = AddProfileOk(D.Model, Bed);
		Spec.Perpend = AddProfileOk(D.Model, Perpend);
		const geometry::Wall W = geometry::BuildRunningBondWall(D.Model, Spec);
		Out << ", wall " << Spec.Courses << " x " << Spec.BricksPerCourse << " removed";
		const int Removals = Rng.Between(0, 3);
		for (int R = 0; R < Removals; ++R)
		{
			const std::vector<BodyId>& Course = W.Courses[static_cast<std::size_t>(Rng.Between(0, Spec.Courses - 1))];
			const BodyId Victim = Course[static_cast<std::size_t>(Rng.Between(0, static_cast<int>(Course.size()) - 1))];
			if (D.Model.RemoveBody(Victim))
			{
				Out << " " << Victim.Index;
			}
		}
	}
	D.AllInterfaces = D.Model.LiveInterfaces();
	const bool bTilt = Rng.Between(0, 1) == 0;
	const bool bAlongY = Rng.Between(0, 1) == 1;
	Vec3 Gravity{0.0, 0.0, -GravityMPerS2};
	double Horizontal = 0.0;
	if (bTilt)
	{
		const double TiltDeg = Rng.Uniform(0.0, 30.0);
		Out << ", tilt " << TiltDeg << " deg";
		Horizontal = GravityMPerS2 * std::sin(TiltDeg * RadPerDegree);
		Gravity.Z = -GravityMPerS2 * std::cos(TiltDeg * RadPerDegree);
	}
	else
	{
		const double Lambda = Rng.Uniform(0.0, 1.0);
		Out << ", lateral " << Lambda;
		Horizontal = Lambda * GravityMPerS2;
	}
	Out << (bAlongY ? " along y" : " along x");
	(bAlongY ? Gravity.Y : Gravity.X) = Horizontal;
	D.Options = AuthorityUnderGravity(Gravity);
	if (Rng.Between(0, 5) == 0)
	{
		const BodyId Loose = AddBox(D.Model, BrickShape(0.0, 40));
		Out << ", loose brick " << Loose.Index;
	}
	D.Description = Out.str();
	return D;
}

CascadeOptions FuzzOptions(const FuzzDraw& D, int MaxPasses)
{
	CascadeOptions Options;
	Options.Authority = D.Options;
	Options.CrackBandFraction = FuzzCrackBand;
	Options.MaxPasses = MaxPasses;
	return Options;
}

bool SameDouble(double A, double B) { return A == B || (std::isnan(A) && std::isnan(B)); }

bool SameVec(const Vec3& A, const Vec3& B) { return SameDouble(A.X, B.X) && SameDouble(A.Y, B.Y) && SameDouble(A.Z, B.Z); }

bool SameFailure(const FailedInterface& A, const FailedInterface& B)
{
	return SameHandle(A.Id, B.Id) && A.Pass == B.Pass && A.Axis == B.Axis && SameDouble(A.Ratio, B.Ratio) && A.bSevered == B.bSevered;
}

bool SameCluster(const ReleasedCluster& A, const ReleasedCluster& B)
{
	return A.Pass == B.Pass && SameIdList(A.Bodies, B.Bodies) && SameIdList(A.Severed, B.Severed) && SameHandle(A.Motion.Body, B.Motion.Body) &&
		SameVec(A.Motion.TranslationM, B.Motion.TranslationM) && SameVec(A.Motion.RotationRad, B.Motion.RotationRad) &&
		SameDouble(A.LoadFactorLower, B.LoadFactorLower) && SameDouble(A.LoadFactorUpper, B.LoadFactorUpper);
}

const ComponentReport* ComponentHolding(const AuthorityResult& Result, BodyId Body)
{
	for (const ComponentReport& Component : Result.Components)
	{
		if (HasBody(Component.Bodies, Body))
		{
			return &Component;
		}
	}
	return nullptr;
}

bool HasFailure(const CascadeVerdict& V, InterfaceId Id, std::uint32_t Pass, bool bSevered)
{
	for (const FailedInterface& F : V.Failures)
	{
		if (SameHandle(F.Id, Id) && F.Pass == Pass && F.bSevered == bSevered)
		{
			return true;
		}
	}
	return false;
}

CascadeState ExpectedEnd(ComponentVerdict Worst)
{
	switch (Worst)
	{
	case ComponentVerdict::Stands: return CascadeState::Stands;
	case ComponentVerdict::Overstressed: return CascadeState::Overstressed;
	case ComponentVerdict::InvalidInput: return CascadeState::InvalidInput;
	case ComponentVerdict::Collapses:
	case ComponentVerdict::Undecided: return CascadeState::Undecided;
	}
	return CascadeState::Undecided;
}

void CheckNoCohesiveCornerSlips(const Structure& Model, const std::vector<InterfaceReading>& Readings, const std::string& Judge)
{
	for (const InterfaceReading& Reading : Readings)
	{
		if (!Model.IsLive(Reading.Id) || !(Model.EffectiveProfile(Reading.Id).ShearCohesionPa > 0.0))
		{
			continue;
		}
		for (std::size_t C = 0; C < Reading.Corners.size(); ++C)
		{
			INFO(Judge << ": interface " << Reading.Id.Index << " corner " << C << " has a cohesive effective profile");
			CHECK_FALSE(Reading.Corners[C].bSlipping);
		}
	}
}

void CheckReleasesCertified(const Structure& Before, const FuzzDraw& D, const CascadeVerdict& V)
{
	if (V.Clusters.empty())
	{
		return;
	}
	const AuthorityResult Judged = MakeAuthority()->Judge(Before, D.Loads, D.Options, nullptr);
	INFO("judge before the pass: " << DescribeAuthority(Judged));
	CheckNoCohesiveCornerSlips(Before, Judged.Readings, "judge before the pass");
	for (const ReleasedCluster& Cluster : V.Clusters)
	{
		INFO("cluster pass " << Cluster.Pass << " bodies " << Describe(Cluster.Bodies));
		const ComponentReport* Component = ComponentHolding(Judged, Cluster.Bodies.front());
		REQUIRE(Component != nullptr);
		CHECK(Component->Verdict == ComponentVerdict::Collapses);
		for (const BodyId Body : Cluster.Bodies)
		{
			INFO("body " << Body.Index);
			CHECK(HasBody(Component->Release, Body));
		}
	}
	const auto ClusterOf = [&V](BodyId Body)
	{
		for (std::size_t C = 0; C < V.Clusters.size(); ++C)
		{
			if (HasBody(V.Clusters[C].Bodies, Body))
			{
				return static_cast<int>(C);
			}
		}
		return -1;
	};
	for (const ComponentReport& Component : Judged.Components)
	{
		for (const InterfaceId Hinge : Component.Hinging)
		{
			const InterfaceRecord& R = Before.Interface(Hinge);
			const int A = ClusterOf(R.BodyA);
			const int B = ClusterOf(R.BodyB);
			INFO("hinging interface " << Hinge.Index << " between clusters " << A << " and " << B);
			if (A >= 0 && B >= 0)
			{
				CHECK(A != B);
			}
		}
	}
}

void CheckStampsMatchRecords(const Structure& Before, const Structure& After, const FuzzDraw& D, const CascadeVerdict& V)
{
	CHECK(V.FirstPass == Before.HighestPass() + 1);
	CHECK(V.Passes >= 0);
	CHECK(V.LastPass + 1 == V.FirstPass + static_cast<std::uint32_t>(V.Passes));
	if (V.Passes > 0)
	{
		CHECK(After.HighestPass() == V.LastPass);
	}
	else
	{
		CHECK(After.HighestPass() == Before.HighestPass());
	}
	for (const FailedInterface& F : V.Failures)
	{
		const InterfaceRecord& R = After.Interface(F.Id);
		INFO("failure " << F.Id.Index << " pass " << F.Pass << (F.bSevered ? " severed" : " cracked") << "; record " << doctest::String(InterfaceStateName(R.State)) << " crack " << R.CrackPass << " sever " << R.SeverPass);
		CHECK(F.Pass >= V.FirstPass);
		CHECK(F.Pass <= V.LastPass);
		CHECK(R.State == (F.bSevered ? InterfaceState::Severed : InterfaceState::Cracked));
		CHECK((F.bSevered ? R.SeverPass : R.CrackPass) == F.Pass);
	}
	for (const InterfaceId Id : D.AllInterfaces)
	{
		const InterfaceRecord& Was = Before.Interface(Id);
		const InterfaceRecord& Now = After.Interface(Id);
		INFO("interface " << Id.Index << ": crack " << Was.CrackPass << " -> " << Now.CrackPass << ", sever " << Was.SeverPass << " -> " << Now.SeverPass);
		if (Now.CrackPass != Was.CrackPass)
		{
			CHECK(Now.CrackPass >= V.FirstPass);
			CHECK(Now.CrackPass <= V.LastPass);
			CHECK(HasFailure(V, Id, Now.CrackPass, false));
		}
		if (Now.SeverPass != Was.SeverPass)
		{
			CHECK(Now.SeverPass >= V.FirstPass);
			CHECK(Now.SeverPass <= V.LastPass);
			CHECK(HasFailure(V, Id, Now.SeverPass, true));
		}
	}
	for (const ReleasedCluster& Cluster : V.Clusters)
	{
		for (const BodyId Body : Cluster.Bodies)
		{
			CHECK_FALSE(After.IsLive(Body));
		}
		for (const InterfaceId Id : Cluster.Severed)
		{
			CHECK(After.Interface(Id).State == InterfaceState::Severed);
			CHECK(After.Interface(Id).SeverPass == Cluster.Pass);
		}
	}
}

void CheckRemainderJudged(const Structure& End, const FuzzDraw& D, const CascadeVerdict& Last)
{
	INFO("final: " << DescribeVerdict(Last));
	CHECK(Last.Passes == 0);
	const AuthorityResult Fresh = MakeAuthority()->Judge(End, D.Loads, D.Options, nullptr);
	INFO("fresh judge: " << DescribeAuthority(Fresh));
	CheckNoCohesiveCornerSlips(End, Fresh.Readings, "fresh judge");
	CHECK(Last.State == ExpectedEnd(Fresh.Worst));
	REQUIRE(Last.Components.size() == Fresh.Components.size());
	for (std::size_t C = 0; C < Fresh.Components.size(); ++C)
	{
		CHECK(Last.Components[C].Verdict == Fresh.Components[C].Verdict);
		CHECK(SameIdList(Last.Components[C].Bodies, Fresh.Components[C].Bodies));
		if (Last.State == CascadeState::Stands || Last.State == CascadeState::Overstressed)
		{
			CHECK(Last.Components[C].Verdict != ComponentVerdict::Collapses);
		}
		if (Last.Components[C].Verdict == ComponentVerdict::Collapses)
		{
			CHECK(Last.Components[C].Release.empty());
		}
	}
}

void CheckFileRoundTrip(const Structure& End)
{
	LoadedStructure Original;
	Original.Model = End;
	const std::string Text = WriteStructure(Original);
	const LoadResult Loaded = LoadStructure(Text);
	INFO("line " << Loaded.Error.Line << ": " << Loaded.Error.Message);
	REQUIRE(Loaded.Ok());
	const Structure& Back = Loaded.Value.Model;
	CHECK(Back.HighestPass() == End.HighestPass());
	const std::vector<BodyId> Bodies = End.LiveBodies();
	const std::vector<BodyId> BackBodies = Back.LiveBodies();
	REQUIRE(BackBodies.size() == Bodies.size());
	for (std::size_t B = 0; B < Bodies.size(); ++B)
	{
		CHECK(SameDouble(Back.Body(BackBodies[B]).MassKg, End.Body(Bodies[B]).MassKg));
		CHECK(SameVec(Back.Body(BackBodies[B]).CentroidM, End.Body(Bodies[B]).CentroidM));
		CHECK(Back.Body(BackBodies[B]).bGrounded == End.Body(Bodies[B]).bGrounded);
	}
	const std::vector<InterfaceId> Joints = End.LiveInterfaces();
	const std::vector<InterfaceId> BackJoints = Back.LiveInterfaces();
	REQUIRE(BackJoints.size() == Joints.size());
	for (std::size_t J = 0; J < Joints.size(); ++J)
	{
		const InterfaceRecord& Want = End.Interface(Joints[J]);
		const InterfaceRecord& Got = Back.Interface(BackJoints[J]);
		INFO("interface " << J << ": " << doctest::String(InterfaceStateName(Got.State)) << " vs " << doctest::String(InterfaceStateName(Want.State)));
		CHECK(Got.State == Want.State);
		CHECK(Got.CrackPass == Want.CrackPass);
		CHECK(Got.SeverPass == Want.SeverPass);
	}
}

struct Coverage
{
	int Stands = 0, Overstressed = 0, Undecided = 0, Capped = 0, Other = 0;
	int Cracks = 0, Severs = 0, CompressionSevers = 0, Clusters = 0, MultiBodyClusters = 0, DrawsWithCrack = 0, DrawsWithRelease = 0;
};

FuzzDraw MakeEccentricDraw(std::uint64_t Seed)
{
	FuzzDraw D;
	SplitMix Rng(Seed);
	const double Eccentricity = Rng.Uniform(0.05, 0.3);
	const double PointLoadOverWeight = Rng.Uniform(2.0, 12.0);
	const double SignX = Rng.Between(0, 1) == 0 ? -1.0 : 1.0;
	const double SignY = Rng.Between(0, 1) == 0 ? -1.0 : 1.0;
	const double WeightN = BrickMassKg * GravityMPerS2;
	const double VerticalN = (1.0 + PointLoadOverWeight) * WeightN;
	const double PlasticShare = (1.0 + Eccentricity) / 4.0;
	const double ElasticShare = (1.0 + 2.0 * Eccentricity) / 4.0;
	const double CornerShare = PlasticShare + Rng.Uniform(0.15, 0.85) * (ElasticShare - PlasticShare);
	StrengthProfile Bed = DryJoint(Rng.Uniform(0.5, 0.9));
	Bed.CompressiveStrengthPa = 4.0 * CornerShare * VerticalN / (BrickLengthM * BrickWidthM);
	const ProfileId P = AddProfileOk(D.Model, Bed);
	const BodyId Ground = AddGroundPlane(D.Model);
	const Box Shape = BrickShape(0.0, 0);
	const BodyId Brick = AddBox(D.Model, Shape);
	AddInterfaceOk(D.Model, OnGroundPlane(Ground, Brick, Shape, P));
	const double LeverScale = Eccentricity * (1.0 + PointLoadOverWeight) / PointLoadOverWeight;
	D.Loads.push_back(BodyLoad{Brick, Vec3{0.0, 0.0, -PointLoadOverWeight * WeightN}, Vec3{Shape.CentreM.X + SignX * LeverScale * 0.5 * BrickLengthM, Shape.CentreM.Y + SignY * LeverScale * 0.5 * BrickWidthM, Shape.CentreM.Z}});
	D.AllInterfaces = D.Model.LiveInterfaces();
	std::ostringstream Out;
	Out << "eccentric crushing e " << Eccentricity << ", load " << PointLoadOverWeight << " W, corner share " << CornerShare << " in (" << PlasticShare << ", " << ElasticShare << ")";
	D.Description = Out.str();
	return D;
}

enum class FuzzFamily
{
	Masonry,
	Lime,
	EccentricCrushing
};

FuzzDraw MakeFamilyDraw(std::uint64_t Seed, FuzzFamily Family)
{
	return Family == FuzzFamily::EccentricCrushing ? MakeEccentricDraw(Seed) : MakeDraw(Seed, Family == FuzzFamily::Lime);
}

void RunDraw(std::uint64_t Seed, FuzzFamily Family, Coverage& Seen)
{
	const auto DrawStart = std::chrono::steady_clock::now();
	FuzzDraw D = MakeFamilyDraw(Seed, Family);
	INFO("seed " << Seed << ": " << D.Description);
	const std::size_t LiveAtEntry = D.Model.LiveInterfaceCount();
	const std::unique_ptr<ICascade> Cascade = MakeCascade(MakeAuthority());
	std::vector<FailedInterface> Failures;
	std::vector<ReleasedCluster> Clusters;
	CascadeVerdict Last;
	int Passes = 0;
	while (true)
	{
		const Structure Before = D.Model;
		Last = Cascade->Run(D.Model, D.Loads, FuzzOptions(D, 1), nullptr);
		INFO("pass run from " << Before.HighestPass() << ": " << DescribeVerdict(Last));
		CheckReleasesCertified(Before, D, Last);
		CheckNoCohesiveCornerSlips(D.Model, Last.Readings, "pass verdict");
		CheckStampsMatchRecords(Before, D.Model, D, Last);
		Failures.insert(Failures.end(), Last.Failures.begin(), Last.Failures.end());
		Clusters.insert(Clusters.end(), Last.Clusters.begin(), Last.Clusters.end());
		Passes += Last.Passes;
		if (Last.Passes == 0 || Passes >= FuzzMaxPasses)
		{
			break;
		}
	}
	CHECK(static_cast<std::size_t>(Passes) <= LiveAtEntry);
	const CascadeState End = Last.Passes == 0 ? Last.State : CascadeState::PassCapReached;
	if (Last.Passes == 0)
	{
		CheckRemainderJudged(D.Model, D, Last);
	}
	CheckFileRoundTrip(D.Model);

	FuzzDraw Again = MakeFamilyDraw(Seed, Family);
	const CascadeVerdict Whole = MakeCascade(MakeAuthority())->Run(Again.Model, Again.Loads, FuzzOptions(Again, FuzzMaxPasses), nullptr);
	INFO("whole run: " << DescribeVerdict(Whole));
	CheckNoCohesiveCornerSlips(Again.Model, Whole.Readings, "whole run");
	CHECK(Whole.State == End);
	CHECK(Whole.Passes == Passes);
	REQUIRE(Whole.Failures.size() == Failures.size());
	for (std::size_t F = 0; F < Failures.size(); ++F)
	{
		CHECK(SameFailure(Whole.Failures[F], Failures[F]));
	}
	REQUIRE(Whole.Clusters.size() == Clusters.size());
	for (std::size_t C = 0; C < Clusters.size(); ++C)
	{
		CHECK(SameCluster(Whole.Clusters[C], Clusters[C]));
	}

	switch (End)
	{
	case CascadeState::Stands: ++Seen.Stands; break;
	case CascadeState::Overstressed: ++Seen.Overstressed; break;
	case CascadeState::Undecided: ++Seen.Undecided; break;
	case CascadeState::PassCapReached: ++Seen.Capped; break;
	case CascadeState::InvalidInput: ++Seen.Other; break;
	}
	int Cracks = 0;
	for (const FailedInterface& F : Failures)
	{
		(F.bSevered ? Seen.Severs : Cracks) += 1;
		Seen.CompressionSevers += F.bSevered && F.Axis == GoverningAxis::Compression ? 1 : 0;
	}
	Seen.Cracks += Cracks;
	Seen.DrawsWithCrack += Cracks > 0 ? 1 : 0;
	Seen.DrawsWithRelease += Clusters.empty() ? 0 : 1;
	for (const ReleasedCluster& C : Clusters)
	{
		++Seen.Clusters;
		Seen.MultiBodyClusters += C.Bodies.size() > 1 ? 1 : 0;
	}
	MESSAGE("seed " << Seed << ": " << D.Description << " -> " << std::string(CascadeStateName(End)) << ", passes " << Passes << ", cracks " << Cracks << ", clusters " << Clusters.size() << ", " << std::chrono::duration<double>(std::chrono::steady_clock::now() - DrawStart).count() << " s");
}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Cascade.Fuzz.SeededStructuresKeepTheCascadeInvariants")
{
	Coverage Seen;
	const auto Start = std::chrono::steady_clock::now();
	for (int Draw = 0; Draw < FuzzDraws; ++Draw)
	{
		RunDraw(FuzzSeedBase + static_cast<std::uint64_t>(Draw), FuzzFamily::Masonry, Seen);
	}
	for (int Draw = 0; Draw < EccentricFuzzDraws; ++Draw)
	{
		RunDraw(EccentricFuzzSeedBase + static_cast<std::uint64_t>(Draw), FuzzFamily::EccentricCrushing, Seen);
	}
	const double Seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - Start).count();
	MESSAGE("cascade fuzz: " << FuzzDraws << " + " << EccentricFuzzDraws << " eccentric draws in " << Seconds << " s; ends Stands " << Seen.Stands << ", Overstressed " << Seen.Overstressed << ", Undecided " << Seen.Undecided << ", PassCapReached " << Seen.Capped << ", other " << Seen.Other << "; cracks " << Seen.Cracks << " in " << Seen.DrawsWithCrack << " draws, severs " << Seen.Severs << " (" << Seen.CompressionSevers << " crushed), clusters " << Seen.Clusters << " (" << Seen.MultiBodyClusters << " multi-body) in " << Seen.DrawsWithRelease << " draws");
	CHECK(Seen.Cracks >= 1);
	CHECK(Seen.MultiBodyClusters >= 1);
	CHECK(Seen.CompressionSevers >= 1);
	CHECK(Seen.Overstressed >= 1);
	CHECK(Seen.Stands >= 1);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Cascade.Fuzz.LimeStructuresKeepTheCascadeInvariants")
{
	Coverage Seen;
	int LimeDraws = 0;
	for (int Draw = 0; Draw < LimeFuzzDraws; ++Draw)
	{
		const std::uint64_t Seed = LimeFuzzSeedBase + static_cast<std::uint64_t>(Draw);
		LimeDraws += MakeDraw(Seed, true).Description.rfind("lime", 0) == 0 ? 1 : 0;
		RunDraw(Seed, FuzzFamily::Lime, Seen);
	}
	MESSAGE("lime cascade fuzz: " << LimeFuzzDraws << " draws, " << LimeDraws << " with lime joints; ends Stands " << Seen.Stands << ", Overstressed " << Seen.Overstressed << ", Undecided " << Seen.Undecided << ", PassCapReached " << Seen.Capped);
	CHECK(LimeDraws >= 1);
}

TEST_CASE("Statics.Lime.TheSeedThatFiveSelectionPassesLoseEndsStands")
{
	const std::uint64_t Seed = LimeFuzzSeedBase + 34u;
	CHECK(Seed == 300281591u);
	FuzzDraw D = MakeDraw(Seed, true);
	INFO("seed " << Seed << ": " << D.Description);
	const CascadeVerdict Whole = MakeCascade(MakeAuthority())->Run(D.Model, {}, FuzzOptions(D, FuzzMaxPasses), nullptr);
	INFO("whole run: " << DescribeVerdict(Whole));
	CHECK(Whole.State == CascadeState::Stands);
}
