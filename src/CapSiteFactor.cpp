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
	Eigen::VectorXd Total = Eigen::VectorXd::Zero(Groups);
	GroupSum = Eigen::MatrixXd::Zero(3, Groups);
	Eigen::MatrixXd& Sum = GroupSum;
	Pivot = GroupSlackD;
	for (Eigen::Index J = 0; J < Directions.rows(); ++J)
	{
		const Eigen::Index W = DirectionGroup[static_cast<std::size_t>(J)];
		Total(W) += DirectionD(J);
		Pivot(W) += DirectionD(J);
		Sum.col(W).noalias() += DirectionD(J) * Directions.row(J).transpose();
	}
	for (Eigen::Index W = 0; W < Groups; ++W)
	{
		if (!(Pivot(W) > 0.0) || !std::isfinite(Pivot(W)))
		{
			return;
		}
	}
	Eigen::MatrixXd Mean = Eigen::MatrixXd::Zero(3, Groups);
	for (Eigen::Index W = 0; W < Groups; ++W)
	{
		if (Total(W) > 0.0)
		{
			Mean.col(W) = Sum.col(W) / Total(W);
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
		Core.noalias() += (Total(W) * GroupSlackD(W) / Pivot(W)) * Mean.col(W) * Mean.col(W).transpose();
	}
	InversePivot = Pivot.cwiseInverse();
	Cross = Sum.transpose().lazyProduct(Facets.transpose());
	const Eigen::MatrixXd Projected = Facets * Core;
	Eigen::MatrixXd Block = Projected.lazyProduct(Facets.transpose());
	Block.diagonal() += FacetSlackD;
	FacetFactor.compute(Block);
	if (FacetFactor.info() != Eigen::Success)
	{
		return;
	}
	const Eigen::VectorXd Diagonal = FacetFactor.matrixLLT().diagonal();
	for (Eigen::Index K = 0; K < FacetCount; ++K)
	{
		if (!(Diagonal(K) > 0.0) || !std::isfinite(Diagonal(K)))
		{
			return;
		}
	}
	bPositive = true;
}

Eigen::MatrixXd CapSiteFactor::Solve(const Eigen::MatrixXd& V) const
{
	const Eigen::Index Groups = Pivot.size();
	const Eigen::Index FacetCount = V.rows() - Groups;
	Eigen::MatrixXd X(V.rows(), V.cols());
	X.topRows(Groups) = InversePivot.asDiagonal() * V.topRows(Groups);
	X.bottomRows(FacetCount) = V.bottomRows(FacetCount);
	X.bottomRows(FacetCount).noalias() -= Cross.transpose().lazyProduct(X.topRows(Groups));
	FacetFactor.solveInPlace(X.bottomRows(FacetCount));
	X.topRows(Groups) = V.topRows(Groups);
	X.topRows(Groups).noalias() -= Cross.lazyProduct(X.bottomRows(FacetCount));
	X.topRows(Groups) = InversePivot.asDiagonal() * X.topRows(Groups);
	return X;
}
}
