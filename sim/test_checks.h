#pragma once
#include <stdio.h>

extern int g_tests, g_fails, g_xfails;

#define CHECK(cond, msg) do { g_tests++; if (!(cond)) { printf("  FAIL: %s\n", msg); g_fails++; } } while (0)

#define CHECK_SPEC(cond, cite, msg) do { g_tests++; \
    if (!(cond)) { printf("  FAIL: %s  [%s]\n", msg, cite); g_fails++; } } while (0)

/* Specified but not implemented: expected false (XFAIL). Turning true fails the suite (XPASS). */
#define KNOWN_DISCREPANCY(cond, cite, msg) do { g_tests++; \
    if (cond) { printf("  XPASS: %s  [%s] -- promote to CHECK_SPEC\n", msg, cite); g_fails++; } \
    else { printf("  XFAIL: %s  [%s]\n", msg, cite); g_xfails++; } } while (0)
