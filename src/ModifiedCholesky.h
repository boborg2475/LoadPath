#pragma once

#include "EigenInclude.h"

namespace loadpath
{
// Eigen's blocked lower LLT, except that a pivot at or below RelativeFloor times the largest diagonal is set to RemovedPivot, which removes that direction instead of failing.
// A non-positive RelativeFloor gives the plain factorisation.
class ModifiedLlt : public Eigen::LLT<Eigen::MatrixXd>
{
public:
	static constexpr double RemovedPivot = 1.0e128;
	void ComputeModified(const Eigen::MatrixXd& A, double RelativeFloor);
};
}
