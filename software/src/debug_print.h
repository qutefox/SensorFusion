#pragma once

#include <stdio.h>
#include <string.h>

#define ENABLE_DEBUG_PRINT 1 // comment out for release build

// #ifdef ENABLE_DEBUG_PRINT
// #define debug_print(fmt, ...) printf(fmt, ##__VA_ARGS__)
// #else
// #define debug_print(fmt, ...) do {} while (0) // should be optimized away
// #endif

#if ENABLE_DEBUG_PRINT
#define debug_print(...) fprintf(stdout, __VA_ARGS__)
#else
#define debug_print(...)
#endif