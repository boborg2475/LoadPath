#include "CascadeTestSupport.h"

#include "loadpath/Cascade.h"
#include "loadpath/StructuralAuthority.h"

#include "doctest.h"

#include <cmath>
#include <cstddef>
#include <span>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr double PierHalfWidthM = 1.4;
constexpr double PierBlockHeightM = 2.0;
constexpr double PierDensityKgPerM3 = 1800.0;
constexpr int PierBlockCount = 3;
constexpr double PaviaBaseStressPa = 1.85e6;
constexpr double PierFriction = 0.6;

double PierAreaM2()
{
	return (2.0 * PierHalfWidthM) * (2.0 * PierHalfWidthM);
}

double PierBlockWeightN()
{
	return PierDensityKgPerM3 * PierAreaM2() * PierBlockHeightM * GravityMPerS2;
}

double BelfryN()
{
	return PaviaBaseStressPa * PierAreaM2() - PierBlockCount * PierBlockWeightN();
}

struct Pier
{
	Structure Model;
	std::vector<BodyId> BlocksBottomUp;
	std::vector<InterfaceId> JointsBottomUp;
	std::vector<BodyLoad> Loads;
};

Pier MakePier(double CompressiveStrengthPa)
{
	Pier P;
	StrengthProfile Profile = DryJoint(PierFriction);
	Profile.CompressiveStrengthPa = CompressiveStrengthPa;
	const ProfileId Joint = AddProfileOk(P.Model, Profile);
	const BodyId Ground = AddGroundPlane(P.Model);
	const Vec3 Half{PierHalfWidthM, PierHalfWidthM, 0.5 * PierBlockHeightM};
	Box Below{};
	for (int K = 0; K < PierBlockCount; ++K)
	{
		const Box Block = BoxOfDensity(Vec3{0.0, 0.0, (K + 0.5) * PierBlockHeightM}, Half, PierDensityKgPerM3);
		const BodyId Id = AddBox(P.Model, Block);
		P.JointsBottomUp.push_back(K == 0 ? AddInterfaceOk(P.Model, OnGroundPlane(Ground, Id, Block, Joint)) : AddInterfaceOk(P.Model, *StackedFaces(P.BlocksBottomUp.back(), Below, Id, Block, Joint)));
		P.BlocksBottomUp.push_back(Id);
		Below = Block;
	}
	P.Loads.push_back(BodyLoad{P.BlocksBottomUp.back(), Vec3{0.0, 0.0, -BelfryN()}, Vec3{0.0, 0.0, PierBlockCount * PierBlockHeightM}});
	return P;
}

const InterfaceReading* ReadingOf(const std::vector<InterfaceReading>& Readings, InterfaceId Id)
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

CascadeVerdict RunCascadeOn(Structure& Model, const CascadeOptions& Options, std::span<const BodyLoad> Loads)
{
	return MakeCascade(MakeAuthority())->Run(Model, Loads, Options, nullptr);
}
}

TEST_CASE("Statics.Anchor.APierAboveItsCrushingStrengthCollapsesAndBelowItStands")
{
	const double TotalAboveBaseN = PierBlockCount * PierBlockWeightN() + BelfryN();
	CHECK(std::abs(TotalAboveBaseN / PierAreaM2() - PaviaBaseStressPa) <= 1.0e-9 * PaviaBaseStressPa);
	CHECK(BelfryN() > 0.0);

	SUBCASE("prism strength 1.5 MPa: the base crushes and the whole pier is released")
	{
		const double FcPa = 1.5e6;
		Pier P = MakePier(FcPa);
		const AuthorityResult Judged = Judge(P.Model, AuthorityOptions{}, P.Loads);
		INFO(DescribeAuthority(Judged) << "; readings" << DescribeReadings(Judged.Readings));
		for (int K = 0; K < PierBlockCount; ++K)
		{
			INFO("joint " << K << " from the base");
			const InterfaceReading* Reading = ReadingOf(Judged.Readings, P.JointsBottomUp[static_cast<std::size_t>(K)]);
			CHECK(Reading != nullptr);
			if (!(Reading != nullptr))
			{
				return;
			}
			const double CarriedN = (PierBlockCount - K) * PierBlockWeightN() + BelfryN();
			CHECK(Reading->Axis == GoverningAxis::Compression);
			CheckRelative(Reading->CompressionUtilisation, CarriedN / (FcPa * PierAreaM2()), 1.0e-6, "compression utilisation");
			CHECK(Reading->TensionUtilisation <= 1.0);
			CHECK(Reading->ShearUtilisation <= 1.0);
		}
		CHECK(Judged.Components.size() == 1u);
		if (!(Judged.Components.size() == 1u))
		{
			return;
		}
		const ComponentReport& Only = Judged.Components.front();
		CHECK(Only.Elastic == StandingState::Overstressed);
		CHECK(Only.Verdict == ComponentVerdict::Collapses);
		CHECK(SameIdList(Only.Release, P.BlocksBottomUp));

		const CascadeVerdict V = RunCascadeOn(P.Model, CascadeOptions{}, P.Loads);
		INFO(DescribeVerdict(V));
		CHECK(V.State == CascadeState::Stands);
		CHECK(V.Passes == 1);
		CHECK(V.Failures.size() == 1u);
		if (!(V.Failures.size() == 1u))
		{
			return;
		}
		CHECK(SameHandle(V.Failures[0].Id, P.JointsBottomUp[0]));
		CHECK(V.Failures[0].Pass == 1u);
		CHECK(V.Failures[0].bSevered);
		CHECK(V.Failures[0].Axis == GoverningAxis::Compression);
		CHECK(V.Clusters.size() == 1u);
		if (!(V.Clusters.size() == 1u))
		{
			return;
		}
		CHECK(SameIdList(V.Clusters[0].Bodies, P.BlocksBottomUp));
		CHECK(SameIdList(V.Clusters[0].Severed, {P.JointsBottomUp[0]}));
		CheckStamps(P.Model, P.JointsBottomUp[0], InterfaceState::Severed, 0, 1);
	}

	SUBCASE("prism strength 3.5 MPa: no joint is overstressed and the pier stands")
	{
		Pier P = MakePier(3.5e6);
		const AuthorityResult Judged = Judge(P.Model, AuthorityOptions{}, P.Loads);
		INFO(DescribeAuthority(Judged));
		CHECK(Judged.Worst == ComponentVerdict::Stands);
		CHECK(Judged.Components.size() == 1u);
		if (!(Judged.Components.size() == 1u))
		{
			return;
		}
		CHECK(Judged.Components.front().Elastic == StandingState::Stands);
		const CascadeVerdict V = RunCascadeOn(P.Model, CascadeOptions{}, P.Loads);
		INFO(DescribeVerdict(V));
		CHECK(V.State == CascadeState::Stands);
		CHECK(V.Failures.empty());
		CHECK(V.Clusters.empty());
	}
}

TEST_CASE("Statics.Combined.ACornerCrushedOnlyElasticallyIsConsultedAndStaysOverstressed")
{
	const double Eccentricity = 0.25;
	const double CornerShare = 0.34;
	const double PointLoadOverWeight = 9.0;
	const double WeightN = BrickMassKg * GravityMPerS2;
	const double VerticalN = (1.0 + PointLoadOverWeight) * WeightN;
	const double AreaM2 = BrickLengthM * BrickWidthM;
	const double ElasticPeakCornerN = VerticalN / 4.0 * (1.0 + 2.0 * Eccentricity);
	const double CornerCapacityN = CornerShare * VerticalN;
	const double PlasticLoadFactor = 4.0 * CornerCapacityN / (VerticalN * (1.0 + Eccentricity));
	CHECK(ElasticPeakCornerN > CornerCapacityN);
	CHECK(PlasticLoadFactor > 1.0);
	CHECK(Eccentricity < 1.0 / 3.0);
	StrengthProfile Profile = DryJoint(0.7);
	Profile.CompressiveStrengthPa = 4.0 * CornerCapacityN / AreaM2;
	BrickOnGround F = MakeBrickOnGround(Profile);
	const double LeverScale = Eccentricity * (1.0 + PointLoadOverWeight) / PointLoadOverWeight;
	const Vec3 Centre = BrickShape(0.0).CentreM;
	const std::vector<BodyLoad> Loads{BodyLoad{F.Brick, Vec3{0.0, 0.0, -PointLoadOverWeight * WeightN}, Vec3{Centre.X + LeverScale * 0.5 * BrickLengthM, Centre.Y + LeverScale * 0.5 * BrickWidthM, Centre.Z}}};
	const AuthorityResult Judged = Judge(F.Model, AuthorityOptions{}, Loads);
	INFO(DescribeAuthority(Judged) << "; readings" << DescribeReadings(Judged.Readings));
	const InterfaceReading* Bed = ReadingOf(Judged.Readings, F.Bed);
	CHECK(Bed != nullptr);
	if (!(Bed != nullptr))
	{
		return;
	}
	CHECK(Bed->Axis == GoverningAxis::Compression);
	CheckRelative(Bed->CompressionUtilisation, ElasticPeakCornerN / CornerCapacityN, 1.0e-6, "peak corner utilisation");
	CHECK(Judged.Components.size() == 1u);
	if (!(Judged.Components.size() == 1u))
	{
		return;
	}
	const ComponentReport& Only = Judged.Components.front();
	CHECK(Only.Elastic == StandingState::Overstressed);
	CHECK(Only.Verdict == ComponentVerdict::Overstressed);
	CHECK(Only.Release.empty());
	const CascadeVerdict V = RunCascadeOn(F.Model, CascadeOptions{}, Loads);
	INFO(DescribeVerdict(V));
	CHECK(V.State == CascadeState::Overstressed);
	CHECK(V.Failures.empty());
	CHECK(V.Clusters.empty());
	CHECK(F.Model.IsLive(F.Brick));
	CheckStamps(F.Model, F.Bed, InterfaceState::Intact, 0, 0);
}

TEST_CASE("Statics.Combined.ACrushedJointBesideAShearedBondIsNotConsulted")
{
	const double WeightN = BrickMassKg * GravityMPerS2;
	const double AreaM2 = BrickLengthM * BrickWidthM;
	const double PushOverWeight = 0.3;
	StrengthProfile CrushingBed = BondedJoint(1.0e7, 1.0e7, 0.7);
	CrushingBed.CompressiveStrengthPa = 0.5 * 2.0 * WeightN / AreaM2;
	const StrengthProfile WeakBond = BondedJoint(1.0e7, 100.0, 0.1);
	Structure Model;
	const ProfileId BedProfile = AddProfileOk(Model, CrushingBed);
	const ProfileId JointProfile = AddProfileOk(Model, WeakBond);
	const BodyId Ground = AddGroundPlane(Model);
	const BodyId Lower = AddBox(Model, BrickShape(0.0, 0));
	const BodyId Upper = AddBox(Model, BrickShape(0.0, 1));
	const InterfaceId Bed = AddInterfaceOk(Model, OnGroundPlane(Ground, Lower, BrickShape(0.0, 0), BedProfile));
	const InterfaceId Joint = AddInterfaceOk(Model, *StackedFaces(Lower, BrickShape(0.0, 0), Upper, BrickShape(0.0, 1), JointProfile));
	const std::vector<BodyLoad> Loads{BodyLoad{Upper, Vec3{PushOverWeight * WeightN, 0.0, 0.0}, BrickShape(0.0, 1).CentreM}};
	const AuthorityResult Judged = Judge(Model, AuthorityOptions{}, Loads);
	INFO(DescribeAuthority(Judged) << "; readings" << DescribeReadings(Judged.Readings));
	const InterfaceReading* BedReading = ReadingOf(Judged.Readings, Bed);
	const InterfaceReading* JointReading = ReadingOf(Judged.Readings, Joint);
	CHECK(BedReading != nullptr);
	if (!(BedReading != nullptr))
	{
		return;
	}
	CHECK(JointReading != nullptr);
	if (!(JointReading != nullptr))
	{
		return;
	}
	CHECK(BedReading->Axis == GoverningAxis::Compression);
	CHECK(BedReading->CompressionUtilisation > 1.0);
	CHECK(BedReading->TensionUtilisation <= 1.0);
	CHECK(BedReading->ShearUtilisation <= 1.0);
	CHECK(JointReading->Axis == GoverningAxis::Shear);
	CHECK(JointReading->ShearUtilisation > 1.0);
	CHECK(Judged.Components.size() == 1u);
	if (!(Judged.Components.size() == 1u))
	{
		return;
	}
	const ComponentReport& Only = Judged.Components.front();
	CHECK(Only.Elastic == StandingState::Overstressed);
	CHECK(Only.Verdict == ComponentVerdict::Overstressed);
	CHECK(Only.Release.empty());
	CascadeOptions Options;
	Options.MaxPasses = 1;
	const CascadeVerdict V = RunCascadeOn(Model, Options, Loads);
	INFO(DescribeVerdict(V));
	CHECK_FALSE(V.Failures.empty());
	if (V.Failures.empty())
	{
		return;
	}
	CHECK(SameHandle(V.Failures[0].Id, Joint));
	CHECK(V.Failures[0].Pass == 1u);
	CHECK_FALSE(V.Failures[0].bSevered);
	CHECK(V.Failures[0].Axis == GoverningAxis::Shear);
	CHECK(V.Failures.size() == 1u);
	CHECK(V.Clusters.empty());
	CheckStamps(Model, Bed, InterfaceState::Intact, 0, 0);
}

TEST_CASE("Statics.Combined.AZeroCrushingStrengthIsNotConsulted")
{
	StrengthProfile Profile = DryJoint(0.7);
	Profile.CompressiveStrengthPa = 0.0;
	BrickOnGround F = MakeBrickOnGround(Profile);
	const AuthorityResult Judged = Judge(F.Model, AuthorityOptions{});
	INFO(DescribeAuthority(Judged) << "; readings" << DescribeReadings(Judged.Readings));
	const InterfaceReading* Bed = ReadingOf(Judged.Readings, F.Bed);
	CHECK(Bed != nullptr);
	if (!(Bed != nullptr))
	{
		return;
	}
	CHECK(std::isinf(Bed->CompressionUtilisation));
	CHECK(Bed->CompressionUtilisation > 0.0);
	CHECK(Judged.Components.size() == 1u);
	if (!(Judged.Components.size() == 1u))
	{
		return;
	}
	const ComponentReport& Only = Judged.Components.front();
	CHECK(Only.Elastic == StandingState::Overstressed);
	CHECK(Only.Verdict == ComponentVerdict::Overstressed);
	CHECK(Only.Release.empty());
	CHECK(Only.Mechanism.empty());
	const CascadeVerdict V = RunCascadeOn(F.Model, CascadeOptions{}, {});
	INFO(DescribeVerdict(V));
	CHECK(V.Clusters.empty());
	CHECK(F.Model.IsLive(F.Brick));
	CheckStamps(F.Model, F.Bed, InterfaceState::Intact, 0, 0);
}

TEST_CASE("Statics.Combined.APierCrushedOnlyByRoundingIsNeverReleased")
{
	const double HalfWidthM = 0.7;
	const double HalfHeightM = 0.5;
	const double DensityKgPerM3 = 2000.0;
	const double AreaM2 = 4.0 * HalfWidthM * HalfWidthM;
	const double BlockWeightN = DensityKgPerM3 * 8.0 * HalfWidthM * HalfWidthM * HalfHeightM * GravityMPerS2;
	for (int Blocks = 1; Blocks <= 3; ++Blocks)
	{
		for (const int K : {1, 4, 16, 40})
		{
			for (const int Sign : {-1, 1})
			{
				const double SigmaPa = Blocks * BlockWeightN / AreaM2;
				const double FcPa = SigmaPa * (1.0 + Sign * K * 0x1.0p-50);
				INFO(Blocks << " blocks, fc = sigma (1 " << (Sign < 0 ? "-" : "+") << " " << K << " 2^-50)");
				StrengthProfile Profile = DryJoint(0.6);
				Profile.CompressiveStrengthPa = FcPa;
				Structure Model;
				const ProfileId Joint = AddProfileOk(Model, Profile);
				const BodyId Ground = AddGroundPlane(Model);
				std::vector<BodyId> Ids;
				Box Below{};
				for (int B = 0; B < Blocks; ++B)
				{
					const Box Block = BoxOfDensity(Vec3{0.0, 0.0, (2 * B + 1) * HalfHeightM}, Vec3{HalfWidthM, HalfWidthM, HalfHeightM}, DensityKgPerM3);
					const BodyId Id = AddBox(Model, Block);
					if (B == 0)
					{
						AddInterfaceOk(Model, OnGroundPlane(Ground, Id, Block, Joint));
					}
					else
					{
						AddInterfaceOk(Model, *StackedFaces(Ids.back(), Below, Id, Block, Joint));
					}
					Ids.push_back(Id);
					Below = Block;
				}
				const AuthorityResult Judged = Judge(Model, AuthorityOptions{});
				INFO(DescribeAuthority(Judged));
				for (const ComponentReport& Report : Judged.Components)
				{
					CHECK(Report.Verdict != ComponentVerdict::Collapses);
					CHECK(Report.Release.empty());
				}
				const CascadeVerdict V = RunCascadeOn(Model, CascadeOptions{}, {});
				INFO(DescribeVerdict(V));
				CHECK(V.Clusters.empty());
				for (const BodyId Id : Ids)
				{
					CHECK(Model.IsLive(Id));
				}
			}
		}
	}
}
