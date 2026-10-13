#include "ModifiedCholesky.h"

#include <algorithm>
#include <cmath>

namespace loadpath
{
namespace
{
Eigen::Index Unblocked(Eigen::Ref<Eigen::MatrixXd> Mat, double Floor)
{
	const Eigen::Index Size = Mat.rows();
	for (Eigen::Index K = 0; K < Size; ++K)
	{
		const Eigen::Index Rs = Size - K - 1;
		auto A21 = Mat.block(K + 1, K, Rs, 1);
		auto A10 = Mat.block(K, 0, 1, K);
		auto A20 = Mat.block(K + 1, 0, Rs, K);
		double X = Mat(K, K);
		if (K > 0)
		{
			X -= A10.squaredNorm();
		}
		if (Floor > 0.0 && X <= Floor)
		{
			X = ModifiedLlt::RemovedPivot;
		}
		if (!(X > 0.0))
		{
			return K;
		}
		Mat(K, K) = X = std::sqrt(X);
		if (K > 0 && Rs > 0)
		{
			A21.noalias() -= A20 * A10.adjoint();
		}
		if (Rs > 0)
		{
			A21 /= X;
		}
	}
	return -1;
}

Eigen::Index Blocked(Eigen::MatrixXd& M, double Floor)
{
	const Eigen::Index Size = M.rows();
	if (Size < 32)
	{
		return Unblocked(M, Floor);
	}
	Eigen::Index BlockSize = Size / 8;
	BlockSize = (BlockSize / 16) * 16;
	BlockSize = std::min(std::max(BlockSize, Eigen::Index(8)), Eigen::Index(128));
	for (Eigen::Index K = 0; K < Size; K += BlockSize)
	{
		const Eigen::Index Bs = std::min(BlockSize, Size - K);
		const Eigen::Index Rs = Size - K - Bs;
		auto A11 = M.block(K, K, Bs, Bs);
		auto A21 = M.block(K + Bs, K, Rs, Bs);
		auto A22 = M.block(K + Bs, K + Bs, Rs, Rs);
		const Eigen::Index Ret = Unblocked(A11, Floor);
		if (Ret >= 0)
		{
			return K + Ret;
		}
		if (Rs > 0)
		{
			A11.adjoint().triangularView<Eigen::Upper>().solveInPlace<Eigen::OnTheRight>(A21);
			A22.selfadjointView<Eigen::Lower>().rankUpdate(A21, -1.0);
		}
	}
	return -1;
}
}

void ModifiedLlt::ComputeModified(const Eigen::MatrixXd& A, double RelativeFloor)
{
	const Eigen::Index Size = A.rows();
	m_matrix = A;
	m_l1_norm = 0.0;
	for (Eigen::Index Col = 0; Col < Size; ++Col)
	{
		const double Sum = m_matrix.col(Col).tail(Size - Col).lpNorm<1>() + m_matrix.row(Col).head(Col).lpNorm<1>();
		m_l1_norm = std::max(m_l1_norm, Sum);
	}
	const double Floor = Size > 0 ? RelativeFloor * A.diagonal().maxCoeff() : 0.0;
	m_isInitialized = true;
	m_info = Blocked(m_matrix, Floor) == -1 ? Eigen::Success : Eigen::NumericalIssue;
}
}
