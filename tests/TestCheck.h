#pragma once

#include <cstdlib>
#include <iostream>

// Test assertion that is never compiled out.
// Plain assert() vanishes under NDEBUG, so a Release test build would pass
// without checking anything. CHECK stays active in every configuration.
#define CHECK(expr)                                                        \
    do {                                                                   \
        if (!(expr)) {                                                     \
            std::cerr << "\nCHECK failed: " #expr "\n  at " << __FILE__    \
                      << ":" << __LINE__ << "\n";                          \
            std::exit(1);                                                  \
        }                                                                  \
    } while (0)
