/*
 * Host math + runtime shims for tools/peq_bench.
 *
 * The benchmark links with -nostdlib, so the few libm/libc entry points that
 * the firmware coefficient-design path calls are implemented here.  Accuracy
 * only has to be good enough for the filters to behave like filters: the
 * equivalence tests compare baseline against candidate, both built against this
 * same implementation, so any consistent implementation cancels out.
 */

#include <stdint.h>
#include <stddef.h>

float _fltused = 0.0f;

/* The MSVC x64 ABI expects these to exist; they are unused here. */
void __chkstk(void) {}

void *memset(void *dst, int c, size_t n) {
    unsigned char *d = (unsigned char *)dst;
    for (size_t i = 0; i < n; i++) d[i] = (unsigned char)c;
    return dst;
}

void *memcpy(void *dst, const void *src, size_t n) {
    unsigned char *d = (unsigned char *)dst;
    const unsigned char *s = (const unsigned char *)src;
    for (size_t i = 0; i < n; i++) d[i] = s[i];
    return dst;
}

int memcmp(const void *a, const void *b, size_t n) {
    const unsigned char *x = (const unsigned char *)a;
    const unsigned char *y = (const unsigned char *)b;
    for (size_t i = 0; i < n; i++)
        if (x[i] != y[i]) return (int)x[i] - (int)y[i];
    return 0;
}

float fabsf(float x) { return __builtin_fabsf(x); }
float sqrtf(float x) { return __builtin_sqrtf(x); }
float fminf(float x, float y) { return x < y ? x : y; }
float fmaxf(float x, float y) { return x > y ? x : y; }

/* --- double precision trig, range reduced with a 2-part pi/2 --------------- */

#define PIO2_HI 1.5707963267948966
#define PIO2_LO 6.123233995736766e-17

/* sin(y)/y and cos(y) for |y| <= pi/4 */
static double d_sinc(double y) {
    double y2 = y * y;
    double p = -7.647163731819816e-13;
    p = p * y2 + 1.6059043836821613e-10;
    p = p * y2 + -2.5052108385441720e-08;
    p = p * y2 + 2.7557319223985893e-06;
    p = p * y2 + -1.9841269841269841e-04;
    p = p * y2 + 8.3333333333333333e-03;
    p = p * y2 + -1.6666666666666666e-01;
    p = p * y2 + 1.0;
    return y * p;
}

static double d_cosp(double y) {
    double y2 = y * y;
    double p = -2.7557319223985893e-07;   /* -y^10/10! */
    p = p * y2 + 2.4801587301587302e-05;  /* +y^8/8!  */
    p = p * y2 + -1.3888888888888889e-03; /* -y^6/6!  */
    p = p * y2 + 4.1666666666666664e-02;  /* +y^4/4!  */
    p = p * y2 + -5.0000000000000000e-01; /* -y^2/2!  */
    p = p * y2 + 1.0;
    return p;
}

static double d_sin(double x) {
    double r = x * (1.0 / PIO2_HI);
    long long k = (long long)((r >= 0.0) ? (r + 0.5) : (r - 0.5));
    double y = (x - (double)k * PIO2_HI) - (double)k * PIO2_LO;
    switch ((int)(k & 3)) {
        case 0:  return d_sinc(y);
        case 1:  return d_cosp(y);
        case 2:  return -d_sinc(y);
        default: return -d_cosp(y);
    }
}

static double d_cos(double x) { return d_sin(x + PIO2_HI + PIO2_LO); }

float sinf(float x) { return (float)d_sin((double)x); }
float cosf(float x) { return (float)d_cos((double)x); }

float tanf(float x) {
    /* Finite result for the coefficient ranges the firmware clamps to. */
    return (float)(d_sin((double)x) / d_cos((double)x));
}

/* --- pow(x, y) = exp2(y * log2(x)) for x > 0 ------------------------------ */

static double d_log2(double x) {
    /* frexp-style extraction: x = m * 2^e, m in [1, 2) */
    int e = 0;
    while (x >= 2.0) { x *= 0.5; e++; }
    while (x < 1.0)  { x *= 2.0; e--; }
    double z = (x - 1.0) / (x + 1.0);
    double z2 = z * z;
    /* ln(m) = 2 * (z + z^3/3 + z^5/5 + ...), |z| < 1/3 */
    double sum = 0.0, zp = z;
    for (int j = 1; j <= 19; j += 2) {
        sum += zp / (double)j;
        zp *= z2;
    }
    return (double)e + 2.0 * sum * 1.4426950408889634;
}

static double d_exp2(double x) {
    /* x = i + f, i integer, f in [-0.5, 0.5] */
    double y = (double)x;
    double i = (y >= 0.0) ? (double)(long long)(y + 0.5) : -(double)(long long)(-y + 0.5);
    double f = y - i;
    /* exp(f * ln2) for |f| <= 0.5, degree 11 Taylor series */
    double g = f * 0.69314718055994531;
    double sum = 1.0, term = 1.0;
    for (int k = 1; k <= 11; k++) {
        term *= g / (double)k;
        sum += term;
    }
    double r = 1.0;
    int n = (int)i;
    /* 2^n by exact squaring for |n| < 1024 */
    if (n < 0) { for (int k = 0; k > n; k--) r *= 0.5; }
    else       { for (int k = 0; k < n; k++) r *= 2.0;  }
    return sum * r;
}

float powf(float x, float y) {
    if (x <= 0.0f) return 0.0f;
    return (float)d_exp2((double)y * d_log2((double)x));
}

/* powf(2.0f, x) folds to these, so they have to exist too. */
float exp2f(float x) { return (float)d_exp2((double)x); }
float log2f(float x) { return (float)d_log2((double)x); }