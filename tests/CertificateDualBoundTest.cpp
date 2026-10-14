#include "BrickFixtures.h"
#include "CatalogueTestSupport.h"
#include "CertificateProbe.h"
#include "CertificateTestSupport.h"
#include "FactorTestSupport.h"
#include "SlowInDebug.h"
#include "StaticAdmissibilityOracle.h"
#include "StaticsTestSupport.h"

#include "loadpath/Catalogue.h"
#include "loadpath/CollapseCertificate.h"

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using namespace loadpath;
using namespace loadpath::test;

namespace
{
constexpr std::uint32_t BoxSeed = 4242u;
constexpr int BoxDrawCount = oracle::bOptimisedBuild ? 300 : 30;
constexpr double BoxRelative = 1.0e-9;
constexpr double BoxAbsolute = 1.0e-9;
constexpr double BoundRelative = 1.0e-10;
constexpr double TightnessRelative = 1.0e-6;
constexpr int GeneratorChoices[] = {4, 8, 16};

struct BoundCase
{
	std::string Name;
	Structure Model;
	CertificateOptions Options;
	double Friction = 0.0;
};

StrengthProfile CappedProfile(double Friction, double CompressivePa, double MaxShearPa)
{
	StrengthProfile Profile = DryJoint(Friction);
	Profile.CompressiveStrengthPa = CompressivePa;
	Profile.MaxShearStrengthPa = MaxShearPa;
	return Profile;
}

BoundCase MuFiveBlock()
{
	BoundCase C;
	C.Name = "mu 5 block 1 x 1 x 0.1 at lateral 4";
	const Box Shape = BoxOfDensity(Vec3{0.0, 0.0, 0.05}, Vec3{0.5, 0.5, 0.05}, 2000.0);
	const double WeightN = Shape.MassKg * GravityMPerS2;
	const double AreaM2 = 1.0;
	const ProfileId P = AddProfileOk(C.Model, CappedProfile(5.0, 2.0 * WeightN / AreaM2, 1.2 * WeightN / (AreaM2 / 4.0)));
	const BodyId Ground = AddGroundPlane(C.Model);
	const BodyId Block = AddBox(C.Model, Shape);
	AddInterfaceOk(C.Model, OnGroundPlane(Ground, Block, Shape, P));
	C.Options = CertificateLateral(4.0);
	C.Friction = 5.0;
	return C;
}

BoundCase SeededCase(int Draw)
{
	BoundCase C;
	const std::uint32_t Seed = BoxSeed + static_cast<std::uint32_t>(Draw);
	std::mt19937 Rng(Seed);
	const int Bricks = 1 + static_cast<int>(Rng() % 3u);
	C.Friction = oracle::Uniform(Rng, 0.0, 6.0);
	const int Generators = GeneratorChoices[Rng() % 3u];
	const double CapOverWeight = oracle::Uniform(Rng, 0.1, 6.0);
	const double CrushOverWeight = oracle::Uniform(Rng, 1.2, 4.0);
	const double LambdaX = oracle::Uniform(Rng, 0.0, 4.0);
	const double LambdaY = oracle::Uniform(Rng, 0.0, 2.0);
	const double WeightN = Bricks * BrickMassKg * GravityMPerS2;
	const double AreaM2 = BrickLengthM * BrickWidthM;
	const ProfileId P = AddProfileOk(C.Model, CappedProfile(C.Friction, CrushOverWeight * WeightN / AreaM2, CapOverWeight * WeightN / AreaM2));
	const BodyId Ground = AddGroundPlane(C.Model);
	std::vector<Box> Shapes;
	std::vector<BodyId> Bodies;
	double CentreXM = 0.0;
	for (int Course = 0; Course < Bricks; ++Course)
	{
		if (Course > 0)
		{
			CentreXM += oracle::Uniform(Rng, -0.04, 0.04);
		}
		Shapes.push_back(BrickShape(CentreXM, Course));
		Bodies.push_back(AddBox(C.Model, Shapes.back()));
		if (Course == 0)
		{
			AddInterfaceOk(C.Model, OnGroundPlane(Ground, Bodies.back(), Shapes.back(), P));
		}
		else
		{
			AddInterfaceOk(C.Model, *StackedFaces(Bodies[Bodies.size() - 2], Shapes[Shapes.size() - 2], Bodies.back(), Shapes.back(), P));
		}
	}
	C.Options = CertificateLateral(LambdaX, LambdaY);
	C.Options.ConeGenerators = Generators;
	std::ostringstream Name;
	Name << "seed " << Seed << ": " << Bricks << " capped bricks, mu " << C.Friction << ", G " << Generators << ", cap " << CapOverWeight << " W/A, crushing " << CrushOverWeight << " W/A, lateral (" << LambdaX << ", " << LambdaY << ")";
	C.Name = Name.str();
	return C;
}

BoundCase BoltedBlock()
{
	BoundCase C;
	C.Name = "bolted 0.1 x 0.1 x 0.065 block at lateral 0.5";
	const CatalogueResult Loaded = LoadCatalogue(ReadCatalogueFile());
	REQUIRE(Loaded.Ok());
	const FastenerRow& Bolt = FastenerOrFail(Loaded.Value, "ec5_bolt_m16_double_shear");
	const StrengthProfile Profile = FastenerProfile(Bolt, 1, 0.01);
	const Box Shape = BoxOfDensity(Vec3{0.0, 0.0, 0.0325}, Vec3{0.05, 0.05, 0.0325}, 2000.0);
	const ProfileId P = AddProfileOk(C.Model, Profile);
	const BodyId Ground = AddGroundPlane(C.Model);
	const BodyId Block = AddBox(C.Model, Shape);
	AddInterfaceOk(C.Model, OnGroundPlane(Ground, Block, Shape, P));
	C.Options = CertificateLateral(0.5);
	C.Friction = Profile.FrictionCoefficient;
	return C;
}

std::vector<BoundCase> BoundCases()
{
	std::vector<BoundCase> Cases;
	Cases.push_back(MuFiveBlock());
	for (int Draw = 0; Draw < BoxDrawCount; ++Draw)
	{
		Cases.push_back(SeededCase(Draw));
	}
	Cases.push_back({"dry 6 x 5 wall at lateral 0.3", RunningBondWall(DryJoint(0.6), DryJoint(0.6), 6, 5), CertificateLateral(0.3), 0.6});
	Cases.push_back({"ec6 4 x 4 wall at lateral 0.4", Ec6Wall(4, 4), CertificateLateral(0.4), 0.0});
	Cases.push_back(BoltedBlock());
	return Cases;
}

BoundCase CappedBrick(double Friction, double CapOverWeight, double Lambda)
{
	BoundCase C;
	std::ostringstream Name;
	Name << "brick mu " << Friction << ", cap " << CapOverWeight << " W/A, lateral " << Lambda;
	C.Name = Name.str();
	StrengthProfile Profile = DryJoint(Friction);
	Profile.MaxShearStrengthPa = CapOverWeight * BrickMassKg * GravityMPerS2 / (BrickLengthM * BrickWidthM);
	C.Model = MakeBrickOnGround(Profile).Model;
	C.Options = CertificateLateral(Lambda);
	C.Friction = Friction;
	return C;
}

BoundCase CappedStack(double Lambda)
{
	BoundCase C;
	std::ostringstream Name;
	Name << "3 capped bricks mu 5, cap 0.5 W/A, lateral " << Lambda;
	C.Name = Name.str();
	const double WeightN = 3.0 * BrickMassKg * GravityMPerS2;
	const double AreaM2 = BrickLengthM * BrickWidthM;
	const ProfileId P = AddProfileOk(C.Model, CappedProfile(5.0, 3.0 * WeightN / AreaM2, 0.5 * WeightN / AreaM2));
	const BodyId Ground = AddGroundPlane(C.Model);
	std::vector<Box> Shapes;
	std::vector<BodyId> Bodies;
	for (int Course = 0; Course < 3; ++Course)
	{
		Shapes.push_back(BrickShape(0.02 * Course, Course));
		Bodies.push_back(AddBox(C.Model, Shapes.back()));
		if (Course == 0)
		{
			AddInterfaceOk(C.Model, OnGroundPlane(Ground, Bodies.back(), Shapes.back(), P));
		}
		else
		{
			AddInterfaceOk(C.Model, *StackedFaces(Bodies[Bodies.size() - 2], Shapes[Shapes.size() - 2], Bodies.back(), Shapes.back(), P));
		}
	}
	C.Options = CertificateLateral(Lambda);
	C.Friction = 5.0;
	return C;
}

std::vector<BoundCase> TightnessCases()
{
	std::vector<BoundCase> Cases = BoundCases();
	for (const double Lambda : {0.5, 1.0, 0.3 / 0.9999, 2.0})
	{
		Cases.push_back(CappedBrick(5.0, 0.3, Lambda));
	}
	for (const double Lambda : {0.2, 0.5, 1.0})
	{
		Cases.push_back(CappedBrick(0.6, 0.3, Lambda));
	}
	for (const double Lambda : {0.3, 1.0})
	{
		Cases.push_back(CappedStack(Lambda));
	}
	return Cases;
}

struct Probed
{
	std::string Name;
	double Friction = 0.0;
	detail::ProgramProbe Probe;
};

std::vector<Probed> ProbeEveryCase(const std::vector<BoundCase>& Cases)
{
	std::vector<Probed> Out;
	for (const BoundCase& C : Cases)
	{
		const std::size_t Components = C.Model.Components().FreeBodies.size();
		for (std::size_t Component = 0; Component < Components; ++Component)
		{
			for (const bool bInscribed : {true, false})
			{
				std::ostringstream Name;
				Name << C.Name << ", component " << Component << (bInscribed ? ", inscribed" : ", circumscribed");
				Out.push_back({Name.str(), C.Friction, detail::ProbeProgram(C.Model, {}, C.Options, Component, bInscribed)});
			}
		}
	}
	return Out;
}
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.DualBound.EveryConvergedPrimalLiesInsideItsBox")
{
	int Converged = 0;
	int Breaches = 0;
	std::ostringstream Breached;
	for (const Probed& P : ProbeEveryCase(BoundCases()))
	{
		if (!P.Probe.bConverged)
		{
			continue;
		}
		++Converged;
		REQUIRE(P.Probe.Primal.size() == P.Probe.Boxes.size());
		double WorstRatio = 0.0;
		std::size_t WorstColumn = 0;
		bool bBreach = false;
		for (std::size_t I = 0; I < P.Probe.Primal.size(); ++I)
		{
			const double Bound = P.Probe.Boxes[I];
			const double X = P.Probe.Primal[I];
			if (!(X <= Bound * (1.0 + BoxRelative) + BoxAbsolute))
			{
				bBreach = true;
				const double Ratio = X / Bound;
				if (!(Ratio <= WorstRatio))
				{
					WorstRatio = Ratio;
					WorstColumn = I;
				}
			}
		}
		INFO(std::setprecision(12) << P.Name << ": column " << WorstColumn << " of " << P.Probe.Primal.size() << " at " << WorstRatio << " times its box, t " << P.Probe.LoadFactor);
		CHECK_FALSE(bBreach);
		if (bBreach)
		{
			++Breaches;
			Breached << " mu " << std::setprecision(4) << P.Friction;
		}
	}
	MESSAGE(Converged << " converged probes, " << Breaches << " breach a box:" << Breached.str());
	CHECK(Converged > 0);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.DualBound.BoundsEveryConvergedLoadFactor")
{
	int Converged = 0;
	for (const Probed& P : ProbeEveryCase(BoundCases()))
	{
		if (!P.Probe.bConverged)
		{
			continue;
		}
		++Converged;
		const double T = P.Probe.LoadFactor;
		INFO(std::setprecision(17) << P.Name << ": t " << T << ", U " << P.Probe.DualBound << ", U - t " << P.Probe.DualBound - T);
		CHECK(P.Probe.DualBound >= T - BoundRelative * (1.0 + std::abs(T)));
	}
	MESSAGE(Converged << " converged probes");
	CHECK(Converged > 0);
}

TEST_CASE("Certificate.DualBound.AnOverflowedDualReadsUnbounded")
{
	Structure Model;
	const ProfileId P = AddProfileOk(Model, DryJoint(0.0));
	const BodyId Ground = AddGroundPlane(Model);
	const Box Shape = BoxOfDensity(Vec3{0.0, 0.0, 0.5}, Vec3{0.5, 0.5, 0.5}, 2000.0);
	const BodyId Cube = AddBox(Model, Shape);
	AddInterfaceOk(Model, OnGroundPlane(Ground, Cube, Shape, P));
	const CertificateOptions Options;
	const detail::ProgramProbe Probe = detail::ProbeProgram(Model, {}, Options, 0, true);
	REQUIRE(Probe.bConverged);
	REQUIRE(Probe.Duals.size() >= 6u);
	std::vector<double> Duals(Probe.Duals.size(), 0.0);
	Duals[1] = -1.7e308;
	Duals[3] = -1.7e308;
	Duals[2] = 2.0;
	const double U = detail::ProbeDualBound(Model, {}, Options, 0, true, Duals);
	INFO(std::setprecision(17) << "cube mu 0, t " << Probe.LoadFactor << ", U at the overflowed dual " << U);
	CHECK((U == std::numeric_limits<double>::infinity() || U >= Probe.LoadFactor));
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.DualBound.EveryGroundedCollapsesHasItsOuterDualBoundBelowOne")
{
	int Collapses = 0;
	int Components = 0;
	for (const BoundCase& Base : BoundCases())
	{
		const CertificateOptions& Options = Base.Options;
		const CertificateResult R = DecideWith(Base.Model, Options);
		const ComponentPartition& Partition = Base.Model.Components();
		REQUIRE(R.Components.size() == Partition.FreeBodies.size());
		for (std::size_t K = 0; K < R.Components.size(); ++K)
		{
			++Components;
			if (R.Components[K].State != CertificateState::Collapses)
			{
				continue;
			}
			bool bGrounded = false;
			for (const BodyId Id : Partition.FreeBodies[K])
			{
				for (const InterfaceId Each : Base.Model.InterfacesOf(Id))
				{
					const InterfaceRecord& Record = Base.Model.Interface(Each);
					if (Base.Model.IsLive(Each) && (Base.Model.Body(Record.BodyA).bGrounded || Base.Model.Body(Record.BodyB).bGrounded))
					{
						bGrounded = true;
					}
				}
			}
			if (!bGrounded)
			{
				continue;
			}
			++Collapses;
			const detail::ProgramProbe Outer = detail::ProbeProgram(Base.Model, {}, Options, K, false);
			INFO(std::setprecision(17) << Base.Name << ", component " << K << ": t_out " << Outer.LoadFactor << ", U_out " << Outer.DualBound << ", converged " << Outer.bConverged);
			CHECK(Outer.DualBound < 1.0);
		}
	}
	MESSAGE(Components << " components, " << Collapses << " grounded Collapses");
	CHECK(Collapses > 0);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.DualBound.ConvergedBoundsSitWithinTheirTolerance")
{
	int Converged = 0;
	int Loose = 0;
	double WorstRelative = 0.0;
	for (const Probed& P : ProbeEveryCase(TightnessCases()))
	{
		if (!P.Probe.bConverged)
		{
			continue;
		}
		++Converged;
		const double T = P.Probe.LoadFactor;
		const double Relative = (P.Probe.DualBound - T) / (1.0 + std::abs(T));
		WorstRelative = std::max(WorstRelative, Relative);
		const bool bTight = P.Probe.DualBound <= T + TightnessRelative * (1.0 + std::abs(T));
		INFO(std::setprecision(17) << P.Name << ": t " << T << ", U " << P.Probe.DualBound << ", (U - t) / (1 + |t|) " << Relative);
		CHECK(bTight);
		if (!bTight)
		{
			++Loose;
		}
	}
	MESSAGE(Converged << " converged probes, " << Loose << " loose, worst (U - t) / (1 + |t|) " << std::setprecision(4) << WorstRelative);
	CHECK(Converged > 0);
}
