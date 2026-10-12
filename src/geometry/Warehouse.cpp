#include "loadpath/geometry/Warehouse.h"

namespace loadpath::geometry
{
std::size_t Warehouse::BodyCount() const { return 0; }

std::size_t Warehouse::InterfaceCount() const { return 0; }

Warehouse BuildWarehouse(Structure&, const WarehouseSpec&) { return Warehouse{}; }
}
