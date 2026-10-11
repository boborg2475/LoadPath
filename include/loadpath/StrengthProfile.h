#pragma once

#include <limits>

namespace loadpath
{
struct StrengthProfile
{
	double CompressiveStrengthPa = 0;
	double ShearCohesionPa = 0;
	double TensileStrengthPa = 0;
	double FrictionCoefficient = 0;
	// Unset means plain Mohr-Coulomb with no cap.
	double MaxShearStrengthPa = std::numeric_limits<double>::infinity();
	double NormalStiffnessPaPerM = 0;
	double TangentialStiffnessPaPerM = 0;
};
}
