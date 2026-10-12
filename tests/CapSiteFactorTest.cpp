#include "CapSiteFactor.h"
#include "EigenInclude.h"

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

using namespace loadpath;

namespace
{
constexpr std::uint32_t MasterSeed = 20261012u;
constexpr int SolveCases = 200;
constexpr int FailureCases = 50;
constexpr int FailureKinds = 5;
constexpr double BackwardRelative = 1.0e-12;
constexpr double ForwardRelative = 1.0e-10;
constexpr double ForwardPerCondition = 1.0e-14;
constexpr double SchurRelative = 1.0e-10;
constexpr double TwoPi = 6.28318530717958647692;

struct CapData
{
	Eigen::VectorXd GroupSlackD;
	Eigen::VectorXd FacetSlackD;
	Eigen::MatrixXd Facets;
	Eigen::MatrixXd Directions;
	std::vector<Eigen::Index> DirectionGroup;
	Eigen::VectorXd DirectionD;
};

double LogUniform(std::mt19937& Rng, double Low, double High)
{
	std::uniform_real_distribution<double> U(std::log(Low), std::log(High));
	return std::exp(U(Rng));
}

Eigen::Vector3d RandomUnit(std::mt19937& Rng)
{
	std::normal_distribution<double> N(0.0, 1.0);
	const Eigen::Vector3d V(N(Rng), N(Rng), N(Rng));
	return V / V.norm();
}

CapData RandomCap(std::mt19937& Rng)
{
	std::uniform_int_distribution<int> GroupCount(1, 3);
	std::uniform_int_distribution<int> FacetCount(4, 16);
	std::uniform_int_distribution<int> DirectionCount(8, 16);
	std::uniform_real_distribution<double> Unit01(0.0, 1.0);
	CapData Cap;
	const int Groups = GroupCount(Rng);
	const int Facets = FacetCount(Rng);
	Cap.GroupSlackD.resize(Groups);
	for (int G = 0; G < Groups; ++G)
	{
		Cap.GroupSlackD(G) = LogUniform(Rng, 1.0e-6, 1.0e6);
	}
	Cap.FacetSlackD.resize(Facets);
	Cap.Facets.resize(Facets, 3);
	const Eigen::Vector3d AxisU = RandomUnit(Rng);
	const Eigen::Vector3d Other = RandomUnit(Rng);
	const Eigen::Vector3d AxisV = (Other - Other.dot(AxisU) * AxisU).normalized();
	const double Phase = TwoPi * Unit01(Rng);
	const bool bOutOfPlane = Unit01(Rng) < 0.3;
	for (int K = 0; K < Facets; ++K)
	{
		Cap.FacetSlackD(K) = LogUniform(Rng, 1.0e-6, 1.0e6);
		const double AngleRad = Phase + TwoPi * K / Facets;
		Eigen::Vector3d Facet = std::cos(AngleRad) * AxisU + std::sin(AngleRad) * AxisV;
		if (bOutOfPlane && K % 3 == 0)
		{
			Facet = RandomUnit(Rng);
		}
		Cap.Facets.row(K) = Facet.transpose();
	}
	std::vector<Eigen::Vector3d> Directions;
	for (int G = 0; G < Groups; ++G)
	{
		const int Count = DirectionCount(Rng);
		for (int J = 0; J < Count; ++J)
		{
			Directions.push_back(RandomUnit(Rng));
			Cap.DirectionGroup.push_back(G);
		}
	}
	Cap.Directions.resize(static_cast<Eigen::Index>(Directions.size()), 3);
	Cap.DirectionD.resize(static_cast<Eigen::Index>(Directions.size()));
	for (std::size_t J = 0; J < Directions.size(); ++J)
	{
		Cap.Directions.row(static_cast<Eigen::Index>(J)) = Directions[J].transpose();
		Cap.DirectionD(static_cast<Eigen::Index>(J)) = LogUniform(Rng, 1.0e-6, 1.0e6);
	}
	return Cap;
}

struct DenseBlock
{
	Eigen::MatrixXd Mww;
	Eigen::MatrixXd F;
	Eigen::Matrix3d K3;
};

DenseBlock BuildDense(const CapData& Cap)
{
	const Eigen::Index GroupRows = Cap.GroupSlackD.size();
	const Eigen::Index Width = GroupRows + Cap.FacetSlackD.size();
	DenseBlock B;
	B.Mww = Eigen::MatrixXd::Zero(Width, Width);
	B.F = Eigen::MatrixXd::Zero(3, Width);
	B.K3 = Eigen::Matrix3d::Zero();
	for (Eigen::Index G = 0; G < GroupRows; ++G)
	{
		B.Mww(G, G) += Cap.GroupSlackD(G);
	}
	for (Eigen::Index J = 0; J < Cap.Directions.rows(); ++J)
	{
		const Eigen::Vector3d Dir = Cap.Directions.row(J).transpose();
		Eigen::VectorXd A = Eigen::VectorXd::Zero(Width);
		A(Cap.DirectionGroup[static_cast<std::size_t>(J)]) = 1.0;
		for (Eigen::Index K = 0; K < Cap.Facets.rows(); ++K)
		{
			A(GroupRows + K) = Cap.Facets.row(K).dot(Cap.Directions.row(J));
		}
		const double D = Cap.DirectionD(J);
		B.K3 += D * Dir * Dir.transpose();
		B.F += D * Dir * A.transpose();
		B.Mww += D * A * A.transpose();
	}
	for (Eigen::Index K = 0; K < Cap.Facets.rows(); ++K)
	{
		B.Mww(GroupRows + K, GroupRows + K) += Cap.FacetSlackD(K);
	}
	return B;
}

bool DensePositive(const Eigen::LLT<Eigen::MatrixXd>& Factor)
{
	if (Factor.info() != Eigen::Success)
	{
		return false;
	}
	const Eigen::VectorXd Pivots = Factor.matrixLLT().diagonal();
	for (Eigen::Index I = 0; I < Pivots.size(); ++I)
	{
		if (!(Pivots(I) > 0.0) || !std::isfinite(Pivots(I)))
		{
			return false;
		}
	}
	return true;
}

CapSiteFactor BuildStructured(const CapData& Cap)
{
	CapSiteFactor Factor;
	Factor.Build(Cap.GroupSlackD, Cap.FacetSlackD, Cap.Facets, Cap.Directions, Cap.DirectionGroup, Cap.DirectionD);
	return Factor;
}

double Condition(const Eigen::MatrixXd& M, const Eigen::LLT<Eigen::MatrixXd>& Factor)
{
	const Eigen::MatrixXd Inverse = Factor.solve(Eigen::MatrixXd::Identity(M.rows(), M.cols()));
	return M.cwiseAbs().rowwise().sum().maxCoeff() * Inverse.cwiseAbs().rowwise().sum().maxCoeff();
}

double BackwardError(const Eigen::MatrixXd& M, const Eigen::MatrixXd& X, const Eigen::MatrixXd& V)
{
	return (M * X - V).norm() / (M.norm() * X.norm() + V.norm());
}

Eigen::MatrixXd SolveOrNaN(const CapSiteFactor& Factor, const Eigen::MatrixXd& V)
{
	if (!Factor.bPositive)
	{
		return Eigen::MatrixXd::Constant(V.rows(), V.cols(), std::numeric_limits<double>::quiet_NaN());
	}
	return Factor.Solve(V);
}
}

TEST_CASE("Certificate.CapSite.StructuredSolveMatchesTheDenseSchurBlock")
{
	double WorstBackward = 0.0;
	double WorstForward = 0.0;
	double WorstSchur = 0.0;
	double WorstCondition = 0.0;
	int Failures = 0;
	for (int Case = 0; Case < SolveCases; ++Case)
	{
		const std::uint32_t Seed = MasterSeed + static_cast<std::uint32_t>(Case);
		std::mt19937 Rng(Seed);
		const CapData Cap = RandomCap(Rng);
		const DenseBlock Dense = BuildDense(Cap);
		const Eigen::LLT<Eigen::MatrixXd> DenseFactor(Dense.Mww);
		REQUIRE(DensePositive(DenseFactor));
		const CapSiteFactor Structured = BuildStructured(Cap);
		const double Cond = Condition(Dense.Mww, DenseFactor);
		WorstCondition = std::max(WorstCondition, Cond);
		const double ForwardTolerance = std::max(ForwardRelative, ForwardPerCondition * Cond);
		std::normal_distribution<double> N(0.0, 1.0);
		Eigen::VectorXd R(Dense.Mww.rows());
		for (Eigen::Index I = 0; I < R.size(); ++I)
		{
			R(I) = N(Rng);
		}
		const Eigen::MatrixXd FT = Dense.F.transpose();
		const Eigen::MatrixXd DenseFT = DenseFactor.solve(FT);
		const Eigen::MatrixXd DenseR = DenseFactor.solve(R);
		const Eigen::MatrixXd GotFT = SolveOrNaN(Structured, FT);
		const Eigen::MatrixXd GotR = SolveOrNaN(Structured, R);
		const Eigen::Matrix3d DenseK3 = Dense.K3 - Dense.F * DenseFT;
		const Eigen::Matrix3d GotK3 = Dense.K3 - Dense.F * GotFT;
		const double Backward = std::max(BackwardError(Dense.Mww, GotFT, FT), BackwardError(Dense.Mww, GotR, R));
		const double Forward = std::max((GotFT - DenseFT).norm() / DenseFT.norm(), (GotR - DenseR).norm() / DenseR.norm());
		const double Schur = (GotK3 - DenseK3).norm() / Dense.K3.norm();
		const bool bOk = Structured.bPositive && Backward <= BackwardRelative && Forward <= ForwardTolerance && Schur <= SchurRelative;
		if (!bOk)
		{
			++Failures;
			if (Failures <= 5)
			{
				MESSAGE("seed " << Seed << ": groups " << Cap.GroupSlackD.size() << ", facets " << Cap.FacetSlackD.size() << ", directions " << Cap.Directions.rows() << ", positive " << Structured.bPositive << ", condition " << Cond << ", backward " << Backward << ", forward " << Forward << " (tolerance " << ForwardTolerance << "), schur " << Schur);
			}
		}
		if (std::isfinite(Backward) && std::isfinite(Forward) && std::isfinite(Schur))
		{
			WorstBackward = std::max(WorstBackward, Backward);
			WorstForward = std::max(WorstForward, Forward / ForwardTolerance);
			WorstSchur = std::max(WorstSchur, Schur);
		}
	}
	MESSAGE(SolveCases << " cases, " << Failures << " fail; worst condition " << WorstCondition << ", worst backward " << WorstBackward << ", worst forward over its tolerance " << WorstForward << ", worst schur " << WorstSchur);
	CHECK(Failures == 0);
}

TEST_CASE("Certificate.CapSite.FailureIsReportedWhereTheDenseFactorFailed")
{
	int Mismatches = 0;
	int DenseFailures = 0;
	int DensePositives = 0;
	for (int Case = 0; Case < FailureCases; ++Case)
	{
		const std::uint32_t Seed = MasterSeed + 1000u + static_cast<std::uint32_t>(Case);
		std::mt19937 Rng(Seed);
		CapData Cap = RandomCap(Rng);
		const int Kind = Case % FailureKinds;
		const Eigen::Index Facet = static_cast<Eigen::Index>(Seed % static_cast<std::uint32_t>(Cap.FacetSlackD.size()));
		if (Kind == 1)
		{
			const DenseBlock Before = BuildDense(Cap);
			const Eigen::Index Row = Cap.GroupSlackD.size() + Facet;
			Cap.FacetSlackD(Facet) = -2.0 * Before.Mww(Row, Row);
		}
		if (Kind == 2)
		{
			const Eigen::Index Empty = Cap.GroupSlackD.size();
			Cap.GroupSlackD.conservativeResize(Empty + 1);
			Cap.GroupSlackD(Empty) = 0.0;
		}
		if (Kind == 3)
		{
			Cap.FacetSlackD(Facet) = 0.0;
			Cap.Facets.row(Facet) = Eigen::RowVector3d(0.0, 0.0, 1.0);
			Cap.Directions.col(2).setZero();
		}
		if (Kind == 4)
		{
			Cap.FacetSlackD(Facet) = std::numeric_limits<double>::quiet_NaN();
		}
		const Eigen::LLT<Eigen::MatrixXd> DenseFactor(BuildDense(Cap).Mww);
		const bool bDense = DensePositive(DenseFactor);
		const CapSiteFactor Structured = BuildStructured(Cap);
		INFO("seed " << Seed << ", kind " << Kind << ": dense positive " << bDense << ", structured positive " << Structured.bPositive);
		CHECK(Structured.bPositive == bDense);
		Mismatches += Structured.bPositive == bDense ? 0 : 1;
		DenseFailures += bDense ? 0 : 1;
		DensePositives += bDense ? 1 : 0;
	}
	MESSAGE(FailureCases << " cases: dense fails " << DenseFailures << ", dense positive " << DensePositives << ", mismatches " << Mismatches);
	CHECK(DenseFailures == (FailureKinds - 1) * FailureCases / FailureKinds);
	CHECK(DensePositives == FailureCases / FailureKinds);
}
