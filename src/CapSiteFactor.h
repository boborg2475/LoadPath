#pragma once

#include "EigenInclude.h"

#include <vector>

namespace loadpath
{
// The cap-site block Mww factored by eliminating the diagonal group rows first, then a dense LLT of the facet Schur block.
struct CapSiteFactor
{
	bool bPositive = false;
	void Build(const Eigen::VectorXd& GroupSlackD, const Eigen::VectorXd& FacetSlackD, const Eigen::MatrixXd& Facets, const Eigen::MatrixXd& Directions, const std::vector<Eigen::Index>& DirectionGroup, const Eigen::VectorXd& DirectionD);
	Eigen::MatrixXd Solve(const Eigen::MatrixXd& V) const;

	Eigen::VectorXd Pivot;
	Eigen::VectorXd InversePivot;
	Eigen::MatrixXd GroupSum;
	Eigen::MatrixXd Cross;
	Eigen::LLT<Eigen::MatrixXd> FacetFactor;
};
}
