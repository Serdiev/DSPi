#ifndef DSP_SVF_H
#define DSP_SVF_H

#include "config.h"

#if PICO_RP2350
static inline void dsp_svf_first_order(Filter * __restrict f, float * __restrict samples, uint32_t count) {
    // Load SVF coefficients
    float a1 = f->sva1, a2 = f->sva2;
    float m0 = f->svm0, m1 = f->svm1, m2 = f->svm2;
    float ic1eq = f->svic1eq, ic2eq = f->svic2eq;
    float *sp = samples;
    float v0, v1;
    uint32_t blk_count = count >> 2; // unroll loops by 4
    // Per-type specialization: eliminates zero-multiplies in inner loop
    switch (f->filter_type)
    {
        // One-pole TPT SVF: a1 = 1/(1+g), a2 = g/(1+g) (multiply-only).
        case FILTER_LOWPASS1:
            while(blk_count > 0) {
                v0 = sp[0];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[0] = v1;

                v0 = sp[1];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[1] = v1;

                v0 = sp[2];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[2] = v1;

                v0 = sp[3];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[3] = v1;

                sp += 4;
                blk_count--;
            }
            blk_count = count & 0x3;
            while(blk_count > 0) {
                v0 = *sp;
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                *sp++ = v1;
                blk_count--;
            }
        break;

        case FILTER_HIGHPASS1:
            while(blk_count > 0) {
                v0 = sp[0];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[0] = v0 - v1;

                v0 = sp[1];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[1] = v0 - v1;

                v0 = sp[2];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[2] = v0 - v1;

                v0 = sp[3];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[3] = v0 - v1;

                sp += 4;
                blk_count--;
            }
            blk_count = count & 0x3;
            while(blk_count > 0) {
                v0 = *sp;
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                *sp++ = v0 - v1;
                blk_count--;
            }
        break;

        case FILTER_ALLPASS1:
            while(blk_count > 0) {

                v0 = sp[0];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[0] = v1 + v1 - v0;

                v0 = sp[1];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[1] = v1 + v1 - v0;

                v0 = sp[2];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[2] = v1 + v1 - v0;

                v0 = sp[3];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[3] = v1 + v1 - v0;

                sp += 4;
                blk_count--;
            }
            blk_count = count & 0x3;
            while(blk_count > 0) {
                float v0, v1;
                v0 = *sp;
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                *sp++ = v1 + v1 - v0;
                blk_count--;
            }
        break;

        case FILTER_HIGHSHELF1:
            while(blk_count > 0) {
                v0 = sp[0];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[0] = v0 + m2 * (v0 - v1);

                v0 = sp[1];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[1] = v0 + m2 * (v0 - v1);

                v0 = sp[2];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[2] = v0 + m2 * (v0 - v1);

                v0 = sp[3];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[3] = v0 + m2 * (v0 - v1);

                sp += 4;
                blk_count--;
            }
            blk_count = count & 0x3;
            while(blk_count > 0) {
                float v0, v1;
                v0 = *sp;
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                *sp++ = v0 + m2 * (v0 - v1);
                blk_count--;
            }
        break;

        case FILTER_LOWSHELF1:
            while(blk_count > 0) {
                v0 = sp[0];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[0] = v0 + m1 * v1;

                v0 = sp[1];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[1] = v0 + m1 * v1;

                v0 = sp[2];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[2] = v0 + m1 * v1;

                v0 = sp[3];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[3] = v0 + m1 * v1;

                sp += 4;
                blk_count--;
            }
            blk_count = count & 0x3;
            while(blk_count > 0) {
                v0 = *sp;
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                *sp++ = v0 + m1 * v1;
                blk_count--;
            }
        break;

        default:
            while(blk_count > 0) {
                v0 = sp[0];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[0] = v0 * m0 + m1 * v1 + m2 * (v0 - v1);

                v0 = sp[1];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[1] = v0 * m0 + m1 * v1 + m2 * (v0 - v1);

                v0 = sp[2];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[2] = v0 * m0 + m1 * v1 + m2 * (v0 - v1);

                v0 = sp[3];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[3] = v0 * m0 + m1 * v1 + m2 * (v0 - v1);

                sp += 4;
                blk_count--;
            }
            blk_count = count & 0x3;
            while(blk_count > 0) {
                float v0, v1;
                v0 = *sp;
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                *sp++ = v0 * m0 + m1 * v1 + m2 * (v0 - v1);
                blk_count--;
            }
    }

    f->svic1eq = ic1eq;
}

/*
 * Second-order TPT state-variable section.
 *
 * Coefficient convention (see dsp_pipeline.c / crossover.c): the state
 * integrator coefficient sva1/sva2 are as usual, but the two coefficients that
 * only exist to serve the per-sample recurrence are pre-folded at design time:
 *
 *   f->g    = 2*g            the state update runs ic2' = ic2 + 2*g*v1
 *   f->svm1 = m1 + g*m2      v2 is never materialised: the output mix reads
 *   f->svm2 = m2             the integrator state ic2 directly
 *
 * so the loop is
 *
 *   v3   = x - ic2
 *   v1   = sva1*ic1 + sva2*v3
 *   y    = m0*x + svm1*v1 + svm2*ic2
 *   ic1' = 2*v1 - ic1
 *   ic2' = ic2 + f->g*v1
 *
 * which is the same transfer function as the textbook form (the fold is
 * algebraically exact) with two fewer arithmetic operations per sample.
 */
static inline void dsp_svf_second_order(Filter * __restrict f, float * __restrict samples, uint32_t count) {
    // Load SVF coefficients
    float a1 = f->sva1, a2 = f->sva2;
    float m0 = f->svm0, m1 = f->svm1, m2 = f->svm2;
    float ic1eq = f->svic1eq, ic2eq = f->svic2eq;
    float g = f->g;                       // pre-doubled: ic2' = ic2 + g*v1
    float *sp = samples;
    float v0, v1, v2, v3;
    uint32_t blk_count = count >> 2; // unroll loops by 4

    // Per-type specialization: eliminates zero-multiplies in inner loop
    switch (f->filter_type) {
        case FILTER_LOWPASS:
            // y = svm1*v1 + ic2   (m0 = 0, svm1 = g, svm2 = 1);
            // the state update still needs the doubled g.
            while(blk_count > 0) {
                v0 = sp[0];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + m1 * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = ic2eq + g * v1;
                sp[0] = v2;

                v0 = sp[1];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + m1 * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = ic2eq + g * v1;
                sp[1] = v2;

                v0 = sp[2];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + m1 * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = ic2eq + g * v1;
                sp[2] = v2;

                v0 = sp[3];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + m1 * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = ic2eq + g * v1;
                sp[3] = v2;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                v0 = *sp;
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + m1 * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = ic2eq + g * v1;
                *sp++ = v2;
                blk_count--;
            }
            break;
        case FILTER_HIGHPASS:
            // y = svm1*v1 + (x - ic2)   (svm1 = -(k+g), svm2 = -1)
            while(blk_count > 0) {
                v0 = sp[0];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2;
                sp[0] = m1 * v1 + v3;

                v0 = sp[1];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2;
                sp[1] = m1 * v1 + v3;

                v0 = sp[2];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2;
                sp[2] = m1 * v1 + v3;

                v0 = sp[3];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2;
                sp[3] = m1 * v1 + v3;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                v0 = *sp;
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2;
                *sp++ = m1 * v1 + v3;
                blk_count--;
            }
            break;
        case FILTER_PEAKING:
        case FILTER_NOTCH:
        case FILTER_ALLPASS:
            // y = x + svm1*v1   (svm2 = 0, so no ic2 term)
            while(blk_count > 0) {
                v0 = sp[0];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2;
                sp[0] = v0 + m1 * v1;

                v0 = sp[1];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2;
                sp[1] = v0 + m1 * v1;

                v0 = sp[2];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2;
                sp[2] = v0 + m1 * v1;

                v0 = sp[3];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2;
                sp[3] = v0 + m1 * v1;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                v0 = *sp;
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2;
                *sp++ = v0 + m1 * v1;
                blk_count--;
            }
            break;
        default: // FILTER_LOWSHELF, FILTER_HIGHSHELF, FILTER_LINKWITZ_TRANSFORM
            while(blk_count > 0) {
                v0 = sp[0];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                sp[0] = m0 * v0 + m1 * v1 + m2 * ic2eq;
                ic2eq = v2;

                v0 = sp[1];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                sp[1] = m0 * v0 + m1 * v1 + m2 * ic2eq;
                ic2eq = v2;

                v0 = sp[2];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                sp[2] = m0 * v0 + m1 * v1 + m2 * ic2eq;
                ic2eq = v2;

                v0 = sp[3];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                sp[3] = m0 * v0 + m1 * v1 + m2 * ic2eq;
                ic2eq = v2;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                v0 = *sp;
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                float y = m0 * v0 + m1 * v1 + m2 * ic2eq;
                ic2eq = v2;
                *sp++ = y;
                blk_count--;
            }
            break;
    }
    f->svic1eq = ic1eq;
    f->svic2eq = ic2eq;
}
#endif // PICO_RP2350

#endif // DSP_SVF_H
