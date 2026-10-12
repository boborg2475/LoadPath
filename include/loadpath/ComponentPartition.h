#pragma once

#include "loadpath/Handles.h"

#include <vector>

namespace loadpath
{
// Free bodies joined by a live interface share a component; grounded bodies join none and read -1 by slot.
struct ComponentPartition
{
	std::vector<int> ComponentOfSlot;
	std::vector<int> OffsetOfSlot;
	// Components ascend by lowest body slot, and each lists its bodies in ascending slot order.
	std::vector<std::vector<BodyId>> FreeBodies;
};
}
