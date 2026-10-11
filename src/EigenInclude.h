#pragma once

// Eigen's no-exceptions EIGEN_TRY/EIGEN_CATCH leaves a dead return that MSVC reports as C4702 from the backend, which /external:W0 does not silence.
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4702)
#endif
#include <Eigen/Core>
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
