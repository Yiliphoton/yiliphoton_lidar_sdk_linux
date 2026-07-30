
#pragma once

#include <iostream>

#define YLP_ERROR   std::cout << "\033[1m\033[31m"  // bold red
#define YLP_WARNING std::cout << "\033[1m\033[33m"  // bold yellow
#define YLP_INFO    std::cout << "\033[1m\033[32m"  // bold green
#define YLP_INFOL   std::cout << "\033[32m"         // green
#define YLP_DEBUG   std::cout << "\033[1m\033[36m"  // bold cyan
#define YLP_REND    "\033[0m" << std::endl

#define YLP_TITLE   std::cout << "\033[1m\033[35m"  // bold magenta
#define YLP_MSG     std::cout << "\033[1m\033[37m"  // bold white

