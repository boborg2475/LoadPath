#pragma once

#include "EigenInclude.h"

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4702)
#endif
#include <Eigen/LU>
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <vector>

// A test-tier dense two-phase simplex for max c^T x, A x = b, x >= 0, sharing no code with src/.
namespace loadpath::test::lp
{
inline constexpr double TrustRelative = 1.0e-9;
inline constexpr double PivotEntry = 1.0e-9;
inline constexpr double CostTolerance = 1.0e-11;
inline constexpr int StallPivots = 50;
inline constexpr int RefinementPasses = 3;
inline constexpr double FloorRelative = 1.0e-12;
inline constexpr double RhsPerturbation = 1.0e-10;
inline constexpr unsigned PerturbationSeed = 20261014u;

struct OracleResult
{
	bool bTrusted = false;
	double Optimum = 0.0;
	std::vector<double> X;
	// The certificate's sign convention: Y = -u for the textbook dual u (min b^T u, A^T u >= c), so A^T Y + c <= 0 at the optimum.
	std::vector<double> Y;
	int Pivots = 0;
	bool bUsedBland = false;
	std::string Why;
};

namespace detail
{
using RowMajor = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

struct Tableau
{
	RowMajor T;
	Eigen::VectorXd Rhs;
	Eigen::VectorXd Phase1;
	double Phase1Value = 0.0;
	Eigen::VectorXd Phase2;
	double Phase2Value = 0.0;
	std::vector<Eigen::Index> Basis;
	std::vector<bool> Dropped;
	int Pivots = 0;
	bool bUsedBland = false;
};

inline void Pivot(Tableau& Tab, Eigen::Index Row, Eigen::Index Col)
{
	const double P = Tab.T(Row, Col);
	Tab.T.row(Row) /= P;
	Tab.Rhs(Row) /= P;
	for (Eigen::Index I = 0; I < Tab.T.rows(); ++I)
	{
		const double F = Tab.T(I, Col);
		if (I != Row && F != 0.0)
		{
			Tab.T.row(I) -= F * Tab.T.row(Row);
			Tab.Rhs(I) -= F * Tab.Rhs(Row);
			Tab.T(I, Col) = 0.0;
		}
	}
	const double F1 = Tab.Phase1(Col);
	if (F1 != 0.0)
	{
		Tab.Phase1 -= F1 * Tab.T.row(Row).transpose();
		Tab.Phase1Value -= F1 * Tab.Rhs(Row);
	}
	const double F2 = Tab.Phase2(Col);
	if (F2 != 0.0)
	{
		Tab.Phase2 -= F2 * Tab.T.row(Row).transpose();
		Tab.Phase2Value -= F2 * Tab.Rhs(Row);
	}
	Tab.Basis[static_cast<std::size_t>(Row)] = Col;
	++Tab.Pivots;
}

inline std::string Scientific(double V)
{
	std::ostringstream Out;
	Out << std::scientific << std::setprecision(3) << V;
	return Out.str();
}

inline Eigen::Index BasisKey(const Tableau& Tab, Eigen::Index Row)
{
	const Eigen::Index B = Tab.Basis[static_cast<std::size_t>(Row)];
	return B >= 0 ? B : Tab.T.cols() + Row;
}

inline bool RunPhase(Tableau& Tab, bool bPhase1, int PivotCap, std::string& Why)
{
	const Eigen::Index M = Tab.T.rows();
	const Eigen::Index N = Tab.T.cols();
	bool bBland = false;
	double Best = std::numeric_limits<double>::infinity();
	int SinceImprovement = 0;
	for (;;)
	{
		if (Tab.Pivots >= PivotCap)
		{
			Why = "pivot cap";
			return false;
		}
		const Eigen::VectorXd& Cost = bPhase1 ? Tab.Phase1 : Tab.Phase2;
		const double Value = -(bPhase1 ? Tab.Phase1Value : Tab.Phase2Value);
		if (Value < Best - 1.0e-14 * (1.0 + std::abs(Best)))
		{
			Best = Value;
			SinceImprovement = 0;
			bBland = false;
		}
		else if (++SinceImprovement >= StallPivots)
		{
			bBland = true;
			Tab.bUsedBland = true;
		}
		const double Scale = 1.0 + Cost.cwiseAbs().maxCoeff();
		const double Eligible = -CostTolerance * Scale;
		Eigen::Index Col = -1;
		if (bBland)
		{
			for (Eigen::Index J = 0; J < N && Col < 0; ++J)
			{
				Col = Cost(J) < Eligible ? J : -1;
			}
		}
		else
		{
			const Eigen::RowVectorXd Norms = Tab.T.cwiseAbs2().colwise().sum();
			double Steepest = 0.0;
			for (Eigen::Index J = 0; J < N; ++J)
			{
				const double Edge = Cost(J) < Eligible ? Cost(J) * Cost(J) / (1.0 + Norms(J)) : 0.0;
				if (Edge > Steepest)
				{
					Steepest = Edge;
					Col = J;
				}
			}
		}
		if (Col < 0)
		{
			return true;
		}
		Eigen::Index Row = -1;
		double Ratio = std::numeric_limits<double>::infinity();
		for (Eigen::Index I = 0; I < M; ++I)
		{
			const double Entry = Tab.T(I, Col);
			if (Tab.Dropped[static_cast<std::size_t>(I)] || !(Entry > PivotEntry))
			{
				continue;
			}
			const double R = std::max(0.0, Tab.Rhs(I)) / Entry;
			const bool bTie = Row >= 0 && std::abs(R - Ratio) <= 1.0e-12 * (1.0 + Ratio);
			const bool bBetterTie = bTie && (bBland ? BasisKey(Tab, I) < BasisKey(Tab, Row) : Entry > Tab.T(Row, Col));
			if ((!bTie && R < Ratio) || bBetterTie)
			{
				Ratio = R;
				Row = I;
			}
		}
		if (Row < 0)
		{
			Why = "unbounded";
			return false;
		}
		Pivot(Tab, Row, Col);
	}
}
}

// A is dense column-major, Rows x Columns; phase 2 pivots on a seeded perturbation of b, and a result is trusted only when the final basis, re-solved by full-pivot LU against the exact b, is primal and dual feasible to 1e-9 relative.
inline OracleResult SolveMaxLp(int Rows, int Columns, const std::vector<double>& A, const std::vector<double>& B, const std::vector<double>& C)
{
	using namespace detail;
	OracleResult Out;
	const Eigen::Index M = Rows;
	const Eigen::Index N = Columns;
	if (M <= 0 || N <= 0 || A.size() != static_cast<std::size_t>(M * N) || B.size() != static_cast<std::size_t>(M) || C.size() != static_cast<std::size_t>(N))
	{
		Out.Why = "bad shape";
		return Out;
	}
	const Eigen::Map<const Eigen::MatrixXd> Original(A.data(), M, N);
	const Eigen::Map<const Eigen::VectorXd> Bv(B.data(), M);
	const Eigen::Map<const Eigen::VectorXd> Cv(C.data(), N);
	if (!Original.allFinite() || !Bv.allFinite() || !Cv.allFinite())
	{
		Out.Why = "non-finite input";
		return Out;
	}
	Tableau Tab;
	Tab.T = Original;
	Tab.Rhs = Bv;
	Tab.Basis.assign(static_cast<std::size_t>(M), -1);
	Tab.Dropped.assign(static_cast<std::size_t>(M), false);
	for (Eigen::Index I = 0; I < M; ++I)
	{
		const double Largest = Tab.T.row(I).cwiseAbs().maxCoeff();
		if (!(Largest > 0.0))
		{
			if (Tab.Rhs(I) != 0.0)
			{
				Out.Why = "zero row with non-zero bound";
				return Out;
			}
			Tab.Dropped[static_cast<std::size_t>(I)] = true;
			continue;
		}
		const double S = (Tab.Rhs(I) < 0.0 ? -1.0 : 1.0) / Largest;
		Tab.T.row(I) *= S;
		Tab.Rhs(I) *= S;
	}
	for (Eigen::Index J = 0; J < N; ++J)
	{
		Eigen::Index Only = -1;
		int NonZeros = 0;
		for (Eigen::Index I = 0; I < M; ++I)
		{
			if (Tab.T(I, J) != 0.0)
			{
				Only = I;
				++NonZeros;
			}
		}
		if (NonZeros == 1 && Tab.T(Only, J) > 0.0 && Tab.Basis[static_cast<std::size_t>(Only)] < 0 && !Tab.Dropped[static_cast<std::size_t>(Only)])
		{
			const double P = Tab.T(Only, J);
			Tab.T.row(Only) /= P;
			Tab.Rhs(Only) /= P;
			Tab.Basis[static_cast<std::size_t>(Only)] = J;
		}
	}
	Tab.Phase1 = Eigen::VectorXd::Zero(N);
	for (Eigen::Index I = 0; I < M; ++I)
	{
		if (Tab.Basis[static_cast<std::size_t>(I)] < 0 && !Tab.Dropped[static_cast<std::size_t>(I)])
		{
			Tab.Phase1 -= Tab.T.row(I).transpose();
			Tab.Phase1Value -= Tab.Rhs(I);
		}
	}
	Tab.Phase2 = -Cv;
	for (Eigen::Index I = 0; I < M; ++I)
	{
		const Eigen::Index J = Tab.Basis[static_cast<std::size_t>(I)];
		if (J >= 0 && Tab.Phase2(J) != 0.0)
		{
			const double F = Tab.Phase2(J);
			Tab.Phase2 -= F * Tab.T.row(I).transpose();
			Tab.Phase2Value -= F * Tab.Rhs(I);
		}
	}
	const int PivotCap = static_cast<int>(20 * (M + N));
	if (-Tab.Phase1Value > 0.0 && !RunPhase(Tab, true, PivotCap, Out.Why))
	{
		Out.Why = "phase 1: " + Out.Why;
		return Out;
	}
	if (-Tab.Phase1Value > TrustRelative * (1.0 + Tab.Rhs.cwiseAbs().maxCoeff()))
	{
		Out.Why = "infeasible";
		return Out;
	}
	for (Eigen::Index I = 0; I < M; ++I)
	{
		if (Tab.Basis[static_cast<std::size_t>(I)] >= 0 || Tab.Dropped[static_cast<std::size_t>(I)])
		{
			continue;
		}
		Eigen::Index Col = -1;
		double Largest = 1.0e-7;
		for (Eigen::Index J = 0; J < N; ++J)
		{
			if (std::abs(Tab.T(I, J)) > Largest)
			{
				Largest = std::abs(Tab.T(I, J));
				Col = J;
			}
		}
		if (Col < 0)
		{
			Tab.Dropped[static_cast<std::size_t>(I)] = true;
			continue;
		}
		Pivot(Tab, I, Col);
	}
	std::mt19937 Rng(PerturbationSeed);
	for (Eigen::Index I = 0; I < M; ++I)
	{
		if (!Tab.Dropped[static_cast<std::size_t>(I)])
		{
			Tab.Rhs(I) += RhsPerturbation * (1.0 + std::abs(Tab.Rhs(I))) * std::uniform_real_distribution<double>(0.5, 1.0)(Rng);
		}
	}
	Tab.Phase1.setZero();
	if (!RunPhase(Tab, false, PivotCap, Out.Why))
	{
		Out.Why = "phase 2: " + Out.Why;
		return Out;
	}
	Out.Pivots = Tab.Pivots;
	Out.bUsedBland = Tab.bUsedBland;
	std::vector<Eigen::Index> KeptRows;
	std::vector<Eigen::Index> BasisColumns;
	for (Eigen::Index I = 0; I < M; ++I)
	{
		if (!Tab.Dropped[static_cast<std::size_t>(I)])
		{
			KeptRows.push_back(I);
			BasisColumns.push_back(Tab.Basis[static_cast<std::size_t>(I)]);
		}
	}
	const Eigen::Index K = static_cast<Eigen::Index>(KeptRows.size());
	Eigen::MatrixXd Bm(K, K);
	Eigen::VectorXd Bk(K);
	Eigen::VectorXd Cb(K);
	for (Eigen::Index R = 0; R < K; ++R)
	{
		Bk(R) = Bv(KeptRows[static_cast<std::size_t>(R)]);
		Cb(R) = Cv(BasisColumns[static_cast<std::size_t>(R)]);
		for (Eigen::Index Q = 0; Q < K; ++Q)
		{
			Bm(R, Q) = Original(KeptRows[static_cast<std::size_t>(R)], BasisColumns[static_cast<std::size_t>(Q)]);
		}
	}
	const Eigen::FullPivLU<Eigen::MatrixXd> Lu(Bm);
	if (Lu.rank() != K)
	{
		Out.Why = "singular final basis";
		return Out;
	}
	Eigen::VectorXd Xb = Lu.solve(Bk);
	Eigen::MatrixXd BmT = Bm.transpose();
	const Eigen::FullPivLU<Eigen::MatrixXd> LuT(BmT);
	Eigen::VectorXd Uk = LuT.solve(Cb);
	for (int Pass = 0; Pass < RefinementPasses; ++Pass)
	{
		Xb += Lu.solve(Eigen::VectorXd(Bk - Bm * Xb));
		Uk += LuT.solve(Eigen::VectorXd(Cb - BmT * Uk));
	}
	Eigen::VectorXd X = Eigen::VectorXd::Zero(N);
	for (Eigen::Index R = 0; R < K; ++R)
	{
		X(BasisColumns[static_cast<std::size_t>(R)]) = Xb(R);
	}
	Eigen::VectorXd U = Eigen::VectorXd::Zero(M);
	for (Eigen::Index R = 0; R < K; ++R)
	{
		U(KeptRows[static_cast<std::size_t>(R)]) = Uk(R);
	}
	if (!X.allFinite() || !U.allFinite())
	{
		Out.Why = "non-finite re-solve";
		return Out;
	}
	const double XScale = 1.0 + X.cwiseAbs().maxCoeff();
	if (X.minCoeff() < -TrustRelative * XScale)
	{
		Out.Why = "primal infeasible after re-solve";
		return Out;
	}
	const Eigen::VectorXd Residual = Original * X - Bv;
	const Eigen::VectorXd ResidualScale = Bv.cwiseAbs() + Original.cwiseAbs() * X.cwiseAbs();
	const double ResidualFloor = FloorRelative * ResidualScale.maxCoeff();
	for (Eigen::Index I = 0; I < M; ++I)
	{
		if (std::abs(Residual(I)) > TrustRelative * ResidualScale(I) + ResidualFloor)
		{
			Out.Why = "residual at row " + std::to_string(I) + ": " + Scientific(Residual(I)) + " against scale " + Scientific(ResidualScale(I));
			return Out;
		}
	}
	const Eigen::VectorXd Reduced = Cv - Original.transpose() * U;
	const Eigen::VectorXd ReducedScale = Cv.cwiseAbs() + Original.cwiseAbs().transpose() * U.cwiseAbs();
	const double ReducedFloor = FloorRelative * ReducedScale.maxCoeff();
	for (Eigen::Index J = 0; J < N; ++J)
	{
		if (Reduced(J) > TrustRelative * ReducedScale(J) + ReducedFloor)
		{
			Out.Why = "dual infeasible at column " + std::to_string(J) + ": reduced cost " + Scientific(Reduced(J)) + " against scale " + Scientific(ReducedScale(J));
			return Out;
		}
	}
	Out.bTrusted = true;
	Out.Optimum = Cv.dot(X);
	Out.X.assign(X.data(), X.data() + N);
	Out.Y.resize(static_cast<std::size_t>(M));
	for (Eigen::Index I = 0; I < M; ++I)
	{
		Out.Y[static_cast<std::size_t>(I)] = -U(I);
	}
	return Out;
}
}
