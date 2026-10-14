#include "BrickFixtures.h"
#include "CatalogueTestSupport.h"
#include "CertificateProbe.h"
#include "CertificateTestSupport.h"
#include "FactorTestSupport.h"
#include "LpOracle.h"
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
constexpr int DofPerBodyRows = 6;

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

namespace
{
constexpr double OracleFloorRelative = 1.0e-12;
constexpr double OracleCeilingRelative = 1.0e-6;
constexpr int PerturbationsPerExport = 20;
constexpr std::uint32_t PerturbationSeed = 9090u;
constexpr int PerturbationKinds = 5;
constexpr std::size_t OracleEntryLimit = 500000;
constexpr int DefaultFacetPivotLimit = 20000;
constexpr int DefaultFacetBlandAfter = 2000;

struct OracleTally
{
	int Exports = 0;
	int Trusted = 0;
	int Untrusted = 0;
	int TooLarge = 0;
	int Perturbed = 0;
	double WorstProbe = std::numeric_limits<double>::infinity();
	double WorstAtOptimumLow = std::numeric_limits<double>::infinity();
	double WorstAtOptimumHigh = -std::numeric_limits<double>::infinity();
	double WorstPerturbed = std::numeric_limits<double>::infinity();
	double LoosestRandomFacets = -std::numeric_limits<double>::infinity();
	double LoosestRegrouped = -std::numeric_limits<double>::infinity();
	std::ostringstream Reasons;
};

double Relative(double U, double T)
{
	return (U - T) / (1.0 + std::abs(T));
}

std::vector<double> Perturbed(const std::vector<double>& Base, int Kind, std::mt19937& Rng, std::size_t EquilibriumRows, std::size_t FirstFacetRow)
{
	std::vector<double> Y = Base;
	double Largest = 0.0;
	for (const double V : Base)
	{
		Largest = std::max(Largest, std::abs(V));
	}
	const double Scale = 1.0 + Largest;
	const double Noise = std::pow(10.0, oracle::Uniform(Rng, -12.0, 0.0));
	switch (Kind)
	{
	case 0:
		for (double& V : Y)
		{
			V += Noise * Scale * oracle::Uniform(Rng, -1.0, 1.0);
		}
		break;
	case 1:
		for (double& V : Y)
		{
			V *= 1.0 + Noise * oracle::Uniform(Rng, -1.0, 1.0);
		}
		break;
	case 2:
		for (double& V : Y)
		{
			if (oracle::Uniform(Rng, 0.0, 1.0) < 0.1)
			{
				V = 0.0;
			}
		}
		break;
	case 3:
	{
		const double Factor = std::pow(10.0, oracle::Uniform(Rng, -3.0, 3.0));
		for (std::size_t I = 0; I < EquilibriumRows && I < Y.size(); ++I)
		{
			Y[I] *= Factor;
		}
		break;
	}
	default:
		for (std::size_t I = FirstFacetRow; I < Y.size(); ++I)
		{
			Y[I] = Scale * oracle::Uniform(Rng, -1.0, 1.0);
		}
		break;
	}
	return Y;
}

std::size_t FirstFacetRow(const detail::ExportedProgram& E, std::size_t EquilibriumRows)
{
	for (std::size_t R = EquilibriumRows; R < static_cast<std::size_t>(E.Rows); ++R)
	{
		if (E.A[static_cast<std::size_t>(E.LoadFactorColumn) * static_cast<std::size_t>(E.Rows) + R] != 0.0)
		{
			return R + 1;
		}
	}
	return static_cast<std::size_t>(E.Rows);
}

lp::OracleResult SolveExport(const detail::ExportedProgram& E)
{
	std::vector<double> Cost(static_cast<std::size_t>(std::max(0, E.Columns)), 0.0);
	if (E.LoadFactorColumn >= 0 && E.LoadFactorColumn < E.Columns)
	{
		Cost[static_cast<std::size_t>(E.LoadFactorColumn)] = 1.0;
	}
	return lp::SolveMaxLp(E.Rows, E.Columns, E.A, E.B, Cost);
}

void BracketTheOptimum(const std::vector<BoundCase>& Cases, OracleTally& Tally)
{
	for (const BoundCase& C : Cases)
	{
		const ComponentPartition& Partition = C.Model.Components();
		for (std::size_t K = 0; K < Partition.FreeBodies.size(); ++K)
		{
			for (const bool bInscribed : {true, false})
			{
				const std::uint32_t Seed = PerturbationSeed + static_cast<std::uint32_t>(Tally.Exports);
				++Tally.Exports;
				std::ostringstream Name;
				Name << C.Name << ", component " << K << (bInscribed ? ", inscribed" : ", circumscribed") << ", perturbation seed " << Seed;
				const detail::ExportedProgram E = detail::ExportProgram(C.Model, {}, C.Options, K, bInscribed);
				INFO(Name.str());
				REQUIRE(E.Rows > 0);
				REQUIRE(E.Columns > 0);
				REQUIRE(E.A.size() == static_cast<std::size_t>(E.Rows) * static_cast<std::size_t>(E.Columns));
				REQUIRE(E.B.size() == static_cast<std::size_t>(E.Rows));
				REQUIRE(E.Boxes.size() == static_cast<std::size_t>(E.Columns));
				REQUIRE(E.LoadFactorColumn >= 0);
				REQUIRE(E.LoadFactorColumn < E.Columns);
				if (static_cast<std::size_t>(E.Rows) * static_cast<std::size_t>(E.Columns) > OracleEntryLimit)
				{
					++Tally.TooLarge;
					Tally.Reasons << "\n  too large for the dense oracle (" << E.Rows << " x " << E.Columns << "): " << Name.str();
					continue;
				}
				const lp::OracleResult O = SolveExport(E);
				if (!O.bTrusted)
				{
					++Tally.Untrusted;
					Tally.Reasons << "\n  untrusted (" << O.Why << "): " << Name.str();
					continue;
				}
				++Tally.Trusted;
				const double T = O.Optimum;
				const double Floor = T - OracleFloorRelative * (1.0 + std::abs(T));
				const double Ceiling = T + OracleCeilingRelative * (1.0 + std::abs(T));
				const detail::ProgramProbe Probe = detail::ProbeProgram(C.Model, {}, C.Options, K, bInscribed);
				const double AtOptimum = detail::ProbeDualBound(C.Model, {}, C.Options, K, bInscribed, O.Y);
				Tally.WorstProbe = std::min(Tally.WorstProbe, Relative(Probe.DualBound, T));
				Tally.WorstAtOptimumLow = std::min(Tally.WorstAtOptimumLow, Relative(AtOptimum, T));
				Tally.WorstAtOptimumHigh = std::max(Tally.WorstAtOptimumHigh, Relative(AtOptimum, T));
				INFO(std::setprecision(17) << "t* " << T << ", probe U " << Probe.DualBound << " (converged " << Probe.bConverged << ", t " << Probe.LoadFactor << "), U at y* " << AtOptimum);
				CHECK(Probe.DualBound >= Floor);
				CHECK(AtOptimum >= Floor);
				CHECK(AtOptimum <= Ceiling);
				const std::size_t EquilibriumRows = static_cast<std::size_t>(DofPerBodyRows) * Partition.FreeBodies[K].size();
				const std::size_t Facets = FirstFacetRow(E, EquilibriumRows);
				std::vector<double> GroupsZeroed = O.Y;
				std::fill(GroupsZeroed.begin() + static_cast<std::ptrdiff_t>(EquilibriumRows), GroupsZeroed.begin() + static_cast<std::ptrdiff_t>(Facets), 0.0);
				const double Regrouped = detail::ProbeDualBound(C.Model, {}, C.Options, K, bInscribed, GroupsZeroed);
				Tally.LoosestRegrouped = std::max(Tally.LoosestRegrouped, Relative(Regrouped, T));
				INFO(std::setprecision(17) << "U at y* with its group and cap duals zeroed " << Regrouped);
				CHECK(Regrouped >= Floor);
				CHECK(Regrouped <= Ceiling);
				const bool bProbeDuals = Probe.Duals.size() == O.Y.size();
				std::mt19937 Rng(Seed);
				for (int Draw = 0; Draw < PerturbationsPerExport; ++Draw)
				{
					const bool bFromProbe = bProbeDuals && Draw % 2 == 1;
					const int Kind = (Draw / 2) % PerturbationKinds;
					const double U = detail::ProbeDualBound(C.Model, {}, C.Options, K, bInscribed, Perturbed(bFromProbe ? Probe.Duals : O.Y, Kind, Rng, EquilibriumRows, Facets));
					++Tally.Perturbed;
					Tally.WorstPerturbed = std::min(Tally.WorstPerturbed, Relative(U, T));
					if (Kind == PerturbationKinds - 1 && !bFromProbe)
					{
						Tally.LoosestRandomFacets = std::max(Tally.LoosestRandomFacets, Relative(U, T));
					}
					INFO(std::setprecision(17) << "draw " << Draw << " kind " << Kind << std::string(bFromProbe ? " from the probe's duals" : " from y*") << ": U " << U);
					CHECK(U >= Floor);
				}
			}
		}
	}
}

void ReportTally(const OracleTally& Tally)
{
	MESSAGE(std::setprecision(4) << Tally.Exports << " exports, " << Tally.Trusted << " trusted, " << Tally.Untrusted << " untrusted, " << Tally.TooLarge << " too large, " << Tally.Perturbed << " perturbed duals; worst (U - t*) / (1 + |t*|): probe " << Tally.WorstProbe << ", at y* [" << Tally.WorstAtOptimumLow << ", " << Tally.WorstAtOptimumHigh << "], perturbed " << Tally.WorstPerturbed << "; loosest at random facet duals from y* " << Tally.LoosestRandomFacets << ", at y* with group and cap duals zeroed " << Tally.LoosestRegrouped << Tally.Reasons.str());
}

std::vector<BoundCase> SmallOracleCases()
{
	std::vector<BoundCase> Cases;
	Cases.push_back(MuFiveBlock());
	for (int Draw = 0; Draw < 6; ++Draw)
	{
		Cases.push_back(SeededCase(Draw));
	}
	for (const double Lambda : {0.5, 1.0, 2.0})
	{
		Cases.push_back(CappedBrick(5.0, 0.3, Lambda));
	}
	Cases.push_back(CappedBrick(0.6, 0.3, 0.5));
	Cases.push_back(CappedStack(0.3));
	return Cases;
}

std::vector<BoundCase> CapSiteCases()
{
	std::vector<BoundCase> Cases;
	Cases.push_back(MuFiveBlock());
	for (const double Lambda : {0.5, 1.0, 0.3 / 0.9999, 2.0})
	{
		Cases.push_back(CappedBrick(5.0, 0.3, Lambda));
	}
	Cases.push_back(CappedBrick(0.6, 0.3, 0.5));
	Cases.push_back(CappedStack(1.0));
	return Cases;
}

std::vector<double> ColumnMajor(const std::vector<std::vector<double>>& RowsOfA)
{
	const std::size_t Rows = RowsOfA.size();
	const std::size_t Columns = Rows > 0 ? RowsOfA[0].size() : 0;
	std::vector<double> A(Rows * Columns);
	for (std::size_t R = 0; R < Rows; ++R)
	{
		for (std::size_t Q = 0; Q < Columns; ++Q)
		{
			A[Q * Rows + R] = RowsOfA[R][Q];
		}
	}
	return A;
}

std::vector<double> BealeDualCost()
{
	return {0.0, 0.0, 1.0};
}

std::vector<double> BealeDualMatrix()
{
	return ColumnMajor({{-0.25, -0.5, 0.0}, {8.0, 12.0, 0.0}, {1.0, 0.5, -1.0}, {-9.0, -3.0, 0.0}});
}

std::vector<double> BealeDualBounds()
{
	return {-0.75, 20.0, -0.5, 6.0};
}
}

TEST_CASE("Slow.Certificate.DualBound.TheBoundBracketsTheOptimum")
{
	OracleTally Tally;
	BracketTheOptimum(TightnessCases(), Tally);
	ReportTally(Tally);
	CHECK(Tally.Trusted > 0);
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.DualBound.TheBoundBracketsTheOptimumOnSmallPrograms")
{
	OracleTally Tally;
	BracketTheOptimum(SmallOracleCases(), Tally);
	ReportTally(Tally);
	CHECK(Tally.Trusted == Tally.Exports);
}

TEST_CASE("Certificate.DualBound.FacetSimplexSolvesASmallLp")
{
	const std::vector<double> A = ColumnMajor({{-1.0, -1.0, 0.0}, {0.0, -1.0, -1.0}});
	std::vector<double> X;
	REQUIRE(detail::FacetSimplexForTest({1.0, 2.0, 3.0}, A, {-2.0, -3.0}, DefaultFacetPivotLimit, DefaultFacetBlandAfter, X));
	REQUIRE(X.size() == 3u);
	INFO(std::setprecision(17) << "x (" << X[0] << ", " << X[1] << ", " << X[2] << ")");
	CHECK(std::abs(X[0]) <= 1.0e-12);
	CHECK(std::abs(X[1] - 3.0) <= 1.0e-12);
	CHECK(std::abs(X[2]) <= 1.0e-12);
}

TEST_CASE("Certificate.DualBound.FacetSimplexRejectsARowWithNoAdmissibleEntry")
{
	std::vector<double> X;
	CHECK_FALSE(detail::FacetSimplexForTest({1.0, 1.0}, ColumnMajor({{1.0, 1.0}}), {-1.0}, DefaultFacetPivotLimit, DefaultFacetBlandAfter, X));
}

TEST_CASE("Certificate.DualBound.FacetSimplexTerminatesOnBealesDualThroughBland")
{
	std::vector<double> X;
	REQUIRE(detail::FacetSimplexForTest(BealeDualCost(), BealeDualMatrix(), BealeDualBounds(), 50, 0, X));
	REQUIRE(X.size() == 3u);
	INFO(std::setprecision(17) << "u (" << X[0] << ", " << X[1] << ", " << X[2] << ")");
	CHECK(std::abs(X[0]) <= 1.0e-12);
	CHECK(std::abs(X[1] - 1.5) <= 1.0e-12);
	CHECK(std::abs(X[2] - 1.25) <= 1.0e-12);
	std::vector<double> Limited;
	CHECK_FALSE(detail::FacetSimplexForTest(BealeDualCost(), BealeDualMatrix(), BealeDualBounds(), 0, 0, Limited));
	std::vector<double> Dantzig;
	CHECK_FALSE(detail::FacetSimplexForTest(BealeDualCost(), BealeDualMatrix(), BealeDualBounds(), 1000, 1000, Dantzig));
}

TEST_CASE(LOADPATH_SLOW_IN_DEBUG "Certificate.DualBound.AFailedFacetLpKeepsAValidBound")
{
	int Trusted = 0;
	int Changed = 0;
	for (const BoundCase& C : CapSiteCases())
	{
		for (const bool bInscribed : {true, false})
		{
			const lp::OracleResult O = SolveExport(detail::ExportProgram(C.Model, {}, C.Options, 0, bInscribed));
			const std::string Label = C.Name + (bInscribed ? ", inscribed: " : ", circumscribed: ") + O.Why;
			INFO(Label);
			REQUIRE(O.bTrusted);
			++Trusted;
			const double T = O.Optimum;
			const detail::ProgramProbe Probe = detail::ProbeProgram(C.Model, {}, C.Options, 0, bInscribed);
			const double Polished = detail::ProbeDualBound(C.Model, {}, C.Options, 0, bInscribed, Probe.Duals);
			const double Kept = detail::ProbeDualBoundWithFacetPivotLimit(C.Model, {}, C.Options, 0, bInscribed, Probe.Duals, 0);
			INFO(std::setprecision(17) << "t* " << T << ", U polished " << Polished << ", U with the facet LP failed " << Kept);
			CHECK(std::isfinite(Kept));
			CHECK(Kept >= T - OracleFloorRelative * (1.0 + std::abs(T)));
			Changed += Kept != Polished ? 1 : 0;
		}
	}
	MESSAGE(Trusted << " trusted cap-site exports, " << Changed << " where the failed facet LP moved the bound");
	CHECK(Changed > 0);
}
