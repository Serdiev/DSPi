/*
 * Freestanding <math.h> shim for tools/peq_bench.
 *
 * The PEQ DSP sources are compiled for the host with -nostdlibinc so that the
 * benchmark can link without a system libc.  Only the handful of single
 * precision math entry points the coefficient design code actually calls are
 * declared here; host_math.c implements them.
 */
#ifndef PEQ_BENCH_SHIM_MATH_H
#define PEQ_BENCH_SHIM_MATH_H

float fabsf(float x);
float sqrtf(float x);
float sinf(float x);
float cosf(float x);
float tanf(float x);
float powf(float x, float y);
float fminf(float x, float y);
float fmaxf(float x, float y);

#endif /* PEQ_BENCH_SHIM_MATH_H */