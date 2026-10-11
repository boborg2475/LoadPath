#pragma once

// Cases that take seconds in Debug but under a second in Release run under Slow. in Debug only.
#if defined(NDEBUG)
#define LOADPATH_SLOW_IN_DEBUG ""
#else
#define LOADPATH_SLOW_IN_DEBUG "Slow."
#endif
