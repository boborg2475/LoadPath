#pragma once

#include "EigenInclude.h"

#include <vector>

namespace loadpath
{
// The cap-site block Mww factored by eliminating the diagonal group rows first, then a dense LLT of the facet Schur block; buffers are reused across builds.
struct CapSiteFactor
{
	bool bPositive = false;
	void Build(const Eigen::VectorXd& GroupSlackD, const Eigen::VectorXd& FacetSlackD, const Eigen::MatrixXd& Facets, const Eigen::MatrixXd& Directions, const std::vector<Eigen::Index>& DirectionGroup, const Eigen::VectorXd& DirectionD);
	Eigen::MatrixXd Solve(const Eigen::MatrixXd& V) const;
	void SolveInto(const Eigen::MatrixXd& V, Eigen::MatrixXd& X) const;

	Eigen::VectorXd Pivot;
	Eigen::VectorXd InversePivot;
	Eigen::VectorXd Total;
	Eigen::MatrixXd GroupSum;
	Eigen::MatrixXd Mean;
	Eigen::MatrixXd Cross;
	Eigen::MatrixXd Projected;
	Eigen::MatrixXd Block;
	Eigen::LLT<Eigen::MatrixXd> FacetFactor;
};
}
