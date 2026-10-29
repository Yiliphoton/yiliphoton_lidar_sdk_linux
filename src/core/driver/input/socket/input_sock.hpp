
#pragma once

#ifdef _WIN32

#include "core/driver/input/socket/input_sock_win.hpp"

#else  //__linux__

#include "core/driver/input/socket/input_sock_unix.hpp"

#endif

