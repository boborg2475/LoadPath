#include "doctest.h"

// Proves the harness runs under the no-exceptions, no-RTTI contract. Nothing physical.
TEST_CASE("Harness.RunsUnderTheCompilerContract")
{
	CHECK(1 + 1 == 2);
}
