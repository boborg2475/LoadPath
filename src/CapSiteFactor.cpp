#include "CapSiteFactor.h"

#include <cmath>

namespace loadpath
{
void CapSiteFactor::Build(const Eigen::VectorXd& GroupSlackD, const Eigen::VectorXd& FacetSlackD, const Eigen::MatrixXd& Facets, const Eigen::MatrixXd& Directions, const std::vector<Eigen::Index>& DirectionGroup, const Eigen::VectorXd& DirectionD)
{
	bPositive = false;
	if (!GroupSlackD.allFinite() || !FacetSlackD.allFinite() || !Facets.allFinite() || !Directions.allFinite() || !DirectionD.allFinite())
	{
		return;
	}
	const Eigen::Index Groups = GroupSlackD.size();
	const Eigen::Index FacetCount = FacetSlackD.size();
	Total.setZero(Groups);
	GroupSum.setZero(3, Groups);
	Pivot = GroupSlackD;
	for (Eigen::Index J = 0; J < Directions.rows(); ++J)
	{
		const Eigen::Index W = DirectionGroup[static_cast<std::size_t>(J)];
		Total(W) += DirectionD(J);
		Pivot(W) += DirectionD(J);
		GroupSum.col(W).noalias() += DirectionD(J) * Directions.row(J).transpose();
	}
	for (Eigen::Index W = 0; W < Groups; ++W)
	{
		if (!(Pivot(W) > 0.0) || !std::isfinite(Pivot(W)))
		{
			return;
		}
	}
	Mean.setZero(3, Groups);
	for (Eigen::Index W = 0; W < Groups; ++W)
	{
		if (Total(W) > 0.0)
		{
			Mean.col(W) = GroupSum.col(W) / Total(W);
		}
	}
	Eigen::Matrix3d Core = Eigen::Matrix3d::Zero();
	for (Eigen::Index J = 0; J < Directions.rows(); ++J)
	{
		const Eigen::Index W = DirectionGroup[static_cast<std::size_t>(J)];
		const Eigen::Vector3d Spread = Directions.row(J).transpose() - Mean.col(W);
		Core.noalias() += DirectionD(J) * Spread * Spread.transpose();
	}
	for (Eigen::Index W = 0; W < Groups; ++W)
	{
		const Eigen::Vector3d M = Mean.col(W);
		Core.noalias() += (Total(W) * GroupSlackD(W) / Pivot(W)) * M * M.transpose();
	}
	InversePivot = Pivot.cwiseInverse();
	Cross.noalias() = GroupSum.transpose().lazyProduct(Facets.transpose());
	Projected.noalias() = Facets.lazyProduct(Core);
	Block.noalias() = Projected.lazyProduct(Facets.transpose());
	Block.diagonal() += FacetSlackD;
	FacetFactor.compute(Block);
	if (FacetFactor.info() != Eigen::Success)
	{
		return;
	}
	const auto Diagonal = FacetFactor.matrixLLT().diagonal();
	for (Eigen::Index K = 0; K < FacetCount; ++K)
	{
		if (!(Diagonal(K) > 0.0) || !std::isfinite(Diagonal(K)))
		{
			return;
		}
	}
	bPositive = true;
}

void CapSiteFactor::SolveInto(const Eigen::MatrixXd& V, Eigen::MatrixXd& X) const
{
	const Eigen::Index Groups = Pivot.size();
	const Eigen::Index FacetCount = V.rows() - Groups;
	X.resize(V.rows(), V.cols());
	X.topRows(Groups) = InversePivot.asDiagonal() * V.topRows(Groups);
	X.bottomRows(FacetCount) = V.bottomRows(FacetCount);
	X.bottomRows(FacetCount).noalias() -= Cross.transpose().lazyProduct(X.topRows(Groups));
	FacetFactor.solveInPlace(X.bottomRows(FacetCount));
	X.topRows(Groups) = V.topRows(Groups);
	X.topRows(Groups).noalias() -= Cross.lazyProduct(X.bottomRows(FacetCount));
	X.topRows(Groups) = InversePivot.asDiagonal() * X.topRows(Groups);
}

Eigen::MatrixXd CapSiteFactor::Solve(const Eigen::MatrixXd& V) const
{
	Eigen::MatrixXd X;
	SolveInto(V, X);
	return X;
}
}
