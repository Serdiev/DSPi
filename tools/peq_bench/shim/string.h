/*
 * Freestanding <string.h> shim for tools/peq_bench.
 */
#ifndef PEQ_BENCH_SHIM_STRING_H
#define PEQ_BENCH_SHIM_STRING_H

#include <stddef.h>

void *memset(void *dst, int c, size_t n);
void *memcpy(void *dst, const void *src, size_t n);
int memcmp(const void *a, const void *b, size_t n);

#endif /* PEQ_BENCH_SHIM_STRING_H */