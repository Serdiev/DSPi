/*
 * peq_bench.c — host-side PEQ throughput + equivalence harness.
 *
 * Compiles the real firmware PEQ design + processing path
 * (firmware/DSPi/dsp_pipeline.c + dsp_svf.h + dsp_biquad.h) for the host and
 * exports it through a freestanding DLL so that tools/peq_bench/run.py can
 * drive it from Python without a system libc.
 *
 * What it measures:
 *   - design cost  : dsp_recalculate_all_filters() over every channel/band
 *   - runtime cost : dsp_process_channel_block() over every channel
 *   - the resulting sample block, which run.py compares sample-by-sample
 *     against a build of a different revision.  That is what makes the
 *     "equivalence" checks in run.py meaningful.
 */

#include <stdint.h>
#include <stddef.h>

#include "math.h"
#include "config.h"
#include "dsp_pipeline.h"
#include "crossover.h"

#define EXPORT __declspec(dllexport)

#define BENCH_MAX_BLOCK 512

/* ---- deterministic PRNG (same sequence on every host) -------------------- */
static uint32_t rng_state;

static uint32_t rng_next(void) {
    uint32_t x = rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    rng_state = x;
    return x;
}

static float rng_unit(void) {
    return (float)(rng_next() >> 8) * (1.0f / 16777216.0f);
}

static float pow2f(float x) { return powf(2.0f, x); }

/* ---- exported state ------------------------------------------------------ */

static float  g_block[NUM_CHANNELS][BENCH_MAX_BLOCK];
static float  g_input[BENCH_MAX_BLOCK];
static double g_checksum;
static double g_design_cycles;

EXPORT uint32_t peq_bench_num_channels(void) { return (uint32_t)NUM_CHANNELS; }

EXPORT const float *peq_bench_block_ptr(uint32_t ch) { return g_block[ch]; }

EXPORT double peq_bench_design_cycles(void) { return g_design_cycles; }

EXPORT double peq_bench_checksum(void) { return g_checksum; }

EXPORT uint32_t peq_bench_max_bands(void) { return (uint32_t)MAX_BANDS; }

EXPORT uint32_t peq_bench_active_bands(void) {
    uint32_t n = 0;
    for (uint32_t ch = 0; ch < NUM_CHANNELS; ch++)
        for (uint32_t b = 0; b < MAX_BANDS; b++)
            if (!filters[ch][b].bypass) n++;
    return n;
}

/*
 * Fill every channel/band recipe with a deterministic mix that covers every
 * PEQ kernel: the SVF branch and the TDF2 branch, first and second order, the
 * gain types, the Linkwitz Transform, plus flat and user-bypassed bands.
 *
 * active_pct is the percentage of band slots that hold a real filter; the rest
 * are FLAT or user-bypassed.  That is the "many band slots, few filters" shape
 * a large PEQ bank has in practice.
 */
/* Force every active band to one type (-1 = use the default mix). */
static int g_type_override = -1;
EXPORT void peq_bench_set_type(int t) { g_type_override = t; }

/* Channels at or above this index carry digital silence (NUM_CHANNELS = none).
 * Models the common case of a stereo source on a device with many outputs,
 * where the unused outputs are a memset-to-zero buffer. */
static uint32_t g_silent_from = 0xFFFFFFFFu;
EXPORT void peq_bench_set_silent(uint32_t from_channel) { g_silent_from = from_channel; }

static void bench_fill_recipes(uint32_t bands, uint32_t active_pct, uint32_t seed) {
    static const uint8_t types[] = {
        FILTER_PEAKING,          FILTER_PEAKING,      FILTER_PEAKING,
        FILTER_LOWSHELF,         FILTER_HIGHSHELF,
        FILTER_LOWPASS,          FILTER_HIGHPASS,
        FILTER_NOTCH,            FILTER_ALLPASS,      FILTER_ALLPASS,
        FILTER_ALLPASS1,         FILTER_LOWSHELF1,    FILTER_HIGHSHELF1,
        FILTER_LOWPASS1,         FILTER_HIGHPASS1,
        FILTER_LINKWITZ_TRANSFORM,
    };
    const uint32_t n_types = (uint32_t)(sizeof(types) / sizeof(types[0]));
    const float log_lo = 4.32193f;   /* log2(20)   */
    const float log_hi = 14.28771f;  /* log2(20000) */

    rng_state = seed ? seed : 0x1234567u;
    for (uint32_t ch = 0; ch < NUM_CHANNELS; ch++) {
        for (uint32_t b = 0; b < bands; b++) {
            EqParamPacket *p = &filter_recipes[ch][b];
            p->channel = (uint8_t)ch;
            p->band = (uint8_t)b;
            p->type = FILTER_FLAT;
            p->bypass = 0;
            p->freq = 1000.0f;
            p->Q = 0.707f;
            p->gain_db = 0.0f;
            peq_qp_x512[ch][b] = 0;

            uint32_t roll = rng_next() % 100u;
            if (roll >= active_pct) {
                /* Half of the unused slots are user-bypassed rather than flat. */
                if (roll >= active_pct + (100u - active_pct) / 2u) p->bypass = 1;
                continue;
            }

            uint8_t t = types[rng_next() % n_types];
            if (g_type_override >= 0) t = (uint8_t)g_type_override;
            p->type = t;
            /* Log-uniform 20 Hz .. 20 kHz: straddles the Fs/7.5 SVF cutoff. */
            p->freq = pow2f(log_lo + rng_unit() * (log_hi - log_lo));
            p->Q = 0.3f + rng_unit() * 4.0f;
            p->gain_db = -12.0f + rng_unit() * 24.0f;
            if (t == FILTER_LINKWITZ_TRANSFORM) {
                p->Q = 0.8f + rng_unit() * 0.8f;                 /* Q0  */
                p->gain_db = p->freq * (0.3f + rng_unit() * 0.5f); /* fp in Hz */
                peq_qp_x512[ch][b] = (uint16_t)(300u + rng_next() % 400u);
            }
        }
    }
}

static void bench_excite(uint32_t block) {
    /* Bounded, deterministic broadband excitation: a full-scale-safe sawtooth
     * whose period is coprime with the block length, so every band sees a
     * spectrum across the whole band. */
    for (uint32_t i = 0; i < block; i++) {
        float s = (float)((i * 37u + 11u) % 257u) * (2.0f / 257.0f) - 1.0f;
        g_input[i] = s * 0.5f;
    }
}

/*
 * Design + run the full PEQ bank.  Returns the rdtsc cycles spent inside the
 * audio-path block processing; the design cost is available separately via
 * peq_bench_design_cycles().
 */
EXPORT double peq_bench_run(uint32_t bands, uint32_t block, uint32_t iters,
                            uint32_t active_pct, uint32_t seed) {
    const float fs = 48000.0f;
    if (block > BENCH_MAX_BLOCK) block = BENCH_MAX_BLOCK;
    if (bands > MAX_BANDS) bands = MAX_BANDS;

    dsp_init_default_filters();
    bench_fill_recipes(bands, active_pct, seed);
    bench_excite(block);
    for (uint32_t ch = 0; ch < NUM_CHANNELS; ch++) {
        channel_band_counts[ch] = (uint8_t)bands;
        for (uint32_t i = 0; i < block; i++) {
            g_block[ch][i] = (ch >= g_silent_from) ? 0.0f : g_input[i];
        }
    }

    uint64_t d0 = __builtin_ia32_rdtsc();
    dsp_recalculate_all_filters(fs);
    uint64_t d1 = __builtin_ia32_rdtsc();
    g_design_cycles = (double)(d1 - d0);

    for (uint32_t ch = 0; ch < NUM_CHANNELS; ch++)
        for (uint32_t i = 0; i < block; i++)
            g_block[ch][i] = (ch >= g_silent_from) ? 0.0f : g_input[i];

    uint64_t t0 = __builtin_ia32_rdtsc();
    for (uint32_t it = 0; it < iters; it++) {
        for (uint32_t ch = 0; ch < NUM_CHANNELS; ch++) {
            if (channel_bypassed[ch]) continue;
            // The pipeline's digital-silence guard; only present in revisions
            // that have it, which is exactly what this benchmark compares.
#if PEQ_BENCH_SILENT_GUARD
            if (dsp_peq_block_is_silent(filters[ch], g_block[ch], block, (uint8_t)ch))
                continue;
#endif
            dsp_process_channel_block(filters[ch], g_block[ch], block, (uint8_t)ch);
        }
    }
    uint64_t t1 = __builtin_ia32_rdtsc();

    double sum = 0.0;
    for (uint32_t ch = 0; ch < NUM_CHANNELS; ch++)
        for (uint32_t i = 0; i < block; i++) sum += (double)g_block[ch][i];
    g_checksum = sum;

    return (double)(t1 - t0);
}

/* Coefficient-design cost only (control path), averaged over `iters` calls. */
EXPORT double peq_bench_design_cost(uint32_t bands, uint32_t iters,
                                    uint32_t active_pct, uint32_t seed) {
    const float fs = 48000.0f;
    if (bands > MAX_BANDS) bands = MAX_BANDS;

    dsp_init_default_filters();
    bench_fill_recipes(bands, active_pct, seed);
    for (uint32_t ch = 0; ch < NUM_CHANNELS; ch++)
        channel_band_counts[ch] = (uint8_t)bands;

    double acc = 0.0;
    for (uint32_t i = 0; i < iters; i++) {
        uint64_t d0 = __builtin_ia32_rdtsc();
        dsp_recalculate_all_filters(fs);
        uint64_t d1 = __builtin_ia32_rdtsc();
        acc += (double)(d1 - d0);
    }
    return acc / (double)(iters ? iters : 1);
}

/*
 * Design/realisation cross-check for ONE design: run the band through the
 * kernel the firmware dispatches to and compare it against an independent
 * state-variable reference built from the sva1/sva2/svm0/svm1/svm2 design
 * coefficients that dsp_compute_coefficients() also stores.  Returns the worst
 * absolute difference over the probe block.
 */
static float g_probe_worst;
static uint32_t g_probe_type;
static float g_probe_freq, g_probe_q, g_probe_gain;

/*
 * Reference state-variable section, written in the TEXTBOOK (un-folded) form:
 *
 *   v1 = sva1*ic1 + sva2*(x - ic2);  v2 = ic2 + g*v1
 *   y  = m0*x + m1*v1 + m2*v2
 *   ic1' = 2*v1 - ic1;  ic2' = 2*v2 - ic2
 *
 * dsp_compute_coefficients() stores pre-folded coefficients (f->g = 2*g and
 * f->svm1 = m1 + g*m2), so the reference recovers the textbook form from them.
 * Comparing it against the kernel is what proves the folding is a pure
 * algebraic identity rather than a change of response.
 *
 * Revisions that predate the folding store the textbook coefficients directly;
 * PEQ_BENCH_FOLDED_SVF (set by run.py from the tree under test) selects which.
 */
static float svf_reference(const Filter *f, float x, float *ic1, float *ic2) {
#if PEQ_BENCH_FOLDED_SVF
    const float g = f->g * 0.5f;
    const float m1 = f->svm1 - g * f->svm2;
#else
    const float g = f->g;
    const float m1 = f->svm1;
#endif
    float v1 = f->sva1 * (*ic1) + f->sva2 * (x - *ic2);
    float v2 = *ic2 + g * v1;
    float y = f->svm0 * x + m1 * v1 + f->svm2 * v2;
    *ic1 = v1 + v1 - *ic1;
    *ic2 = v2 + v2 - *ic2;
    return y;
}

EXPORT float peq_bench_probe(uint32_t type, float freq, float q, float gain) {
    const float fs = 48000.0f;
    static float buf[BENCH_MAX_BLOCK];

    dsp_init_default_filters();
    bench_excite(BENCH_MAX_BLOCK);
    channel_band_counts[0] = 1;
    for (uint32_t ch = 1; ch < NUM_CHANNELS; ch++) channel_band_counts[ch] = 0;

    EqParamPacket p = {0, 0, (uint8_t)type, 0, freq, q, gain};
    peq_qp_x512[0][0] = 0;
    Filter *f = &filters[0][0];
    dsp_compute_coefficients(&p, f, fs);

    g_probe_worst = 0.0f;
    g_probe_type = type;
    g_probe_freq = p.freq;
    g_probe_q = p.Q;
    g_probe_gain = p.gain_db;
    if (f->bypass) return -1.0f;   /* bypassed: nothing to compare */

    float ic1 = 0, ic2 = 0;
    for (uint32_t n = 0; n < BENCH_MAX_BLOCK; n++) buf[n] = g_input[n];
    dsp_process_channel_block(filters[0], buf, BENCH_MAX_BLOCK, 0);
    float worst = 0.0f, peak = 0.0f;
    for (uint32_t n = 0; n < BENCH_MAX_BLOCK; n++) {
        float ya = svf_reference(f, g_input[n], &ic1, &ic2);
        float d = ya > buf[n] ? ya - buf[n] : buf[n] - ya;
        if (d > worst) worst = d;
        float ay = ya < 0.0f ? -ya : ya;
        if (ay > peak) peak = ay;
    }
    g_probe_worst = peak > 0.0f ? worst / peak : worst;
    return g_probe_worst;
}

/* Incremental-recalc correctness: design everything, snapshot the coefficients,
 * change exactly one band, recalc, and count how many sections moved.  Only the
 * edited band may differ.  Returns the number of unexpected changes. */
EXPORT uint32_t peq_bench_recalc_check(uint32_t bands) {
    const float fs = 48000.0f;
    static float before[NUM_CHANNELS][MAX_BANDS][5];
    if (bands > MAX_BANDS) bands = MAX_BANDS;

    dsp_init_default_filters();
    bench_fill_recipes(bands, 100, 7);
    for (uint32_t ch = 0; ch < NUM_CHANNELS; ch++)
        channel_band_counts[ch] = (uint8_t)bands;
    dsp_recalculate_all_filters(fs);

    for (uint32_t ch = 0; ch < NUM_CHANNELS; ch++)
        for (uint32_t b = 0; b < bands; b++) {
            before[ch][b][0] = filters[ch][b].b0;
            before[ch][b][1] = filters[ch][b].b1;
            before[ch][b][2] = filters[ch][b].b2;
            before[ch][b][3] = filters[ch][b].a1;
            before[ch][b][4] = filters[ch][b].a2;
        }

    const uint32_t ech = 3, eb = 2;
    filter_recipes[ech][eb].gain_db += 3.0f;
    dsp_recalculate_all_filters(fs);

    uint32_t unexpected = 0;
    for (uint32_t ch = 0; ch < NUM_CHANNELS; ch++)
        for (uint32_t b = 0; b < bands; b++) {
            int changed = (before[ch][b][0] != filters[ch][b].b0) ||
                          (before[ch][b][1] != filters[ch][b].b1) ||
                          (before[ch][b][2] != filters[ch][b].b2) ||
                          (before[ch][b][3] != filters[ch][b].a1) ||
                          (before[ch][b][4] != filters[ch][b].a2);
            if (changed && !(ch == ech && b == eb)) unexpected++;
        }
    return unexpected;
}

EXPORT uint32_t peq_bench_probe_type(void) { return g_probe_type; }
EXPORT float peq_bench_probe_freq(void) { return g_probe_freq; }
EXPORT float peq_bench_probe_q(void) { return g_probe_q; }
EXPORT float peq_bench_probe_gain(void) { return g_probe_gain; }
/* Coeff / state dump so the driver can inspect a single design. */
EXPORT float peq_bench_probe_coeff(uint32_t i) {
    const Filter *f = &filters[0][0];
    switch (i) {
        case 0: return f->b0;  case 1: return f->b1;  case 2: return f->b2;
        case 3: return f->a1;  case 4: return f->a2;
        case 5: return f->sva1; case 6: return f->sva2; case 7: return f->g;
        case 8: return f->svm0; case 9: return f->svm1; case 10: return f->svm2;
        default: return 0.0f;
    }
}

/* Cold redesign cost: every call changes the sample rate, which invalidates
 * every section — the preset-load / sample-rate-change path. */
EXPORT double peq_bench_cold_design_cost(uint32_t bands, uint32_t iters,
                                         uint32_t active_pct, uint32_t seed) {
    static const float rates[2] = {48000.0f, 44100.0f};
    if (bands > MAX_BANDS) bands = MAX_BANDS;

    dsp_init_default_filters();
    bench_fill_recipes(bands, active_pct, seed);
    for (uint32_t ch = 0; ch < NUM_CHANNELS; ch++)
        channel_band_counts[ch] = (uint8_t)bands;

    double acc = 0.0;
    for (uint32_t i = 0; i < iters; i++) {
        float fs = rates[i & 1u];
        uint64_t d0 = __builtin_ia32_rdtsc();
        dsp_recalculate_all_filters(fs);
        uint64_t d1 = __builtin_ia32_rdtsc();
        acc += (double)(d1 - d0);
    }
    return acc / (double)(iters ? iters : 1);
}

int __stdcall DllMain(void *h, unsigned long long r, void *v) {
    (void)h; (void)r; (void)v;
    return 1;
}