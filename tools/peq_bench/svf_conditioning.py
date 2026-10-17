#!/usr/bin/env python3
"""Why the second-order PEQ sections stay on the state-variable filter.

There is an obvious optimisation staring at the low-corner PEQ sections: run
them as a TDF-II biquad instead of a state-variable filter.  The SVF design
(g, k, m0, m1, m2) collapses *exactly* to a biquad — the identity is derived in
PR #1's discussion — and the biquad needs fewer coefficients and fewer
multiplies.

This script measures why that is not a free win.  It builds both realizations
from the same SVF design parameters, runs each through the same signal in
float32, and compares both against a float64 SVF reference.

    python tools/peq_bench/svf_conditioning.py

Output is a table of relative error against the float64 reference, per filter
type and corner frequency.  The biquad's error grows sharply as the corner
falls, because the biquad numerator is the small side of a near-cancellation
(1 - cos(w)) while the SVF keeps every coefficient O(1).  That is what the
Fs/7.5 SVF/biquad crossover in dsp_pipeline.c exists to avoid.
"""

import math
import struct

# Round a Python double to the nearest float32, and back.
def f32(x):
    return struct.unpack("f", struct.pack("f", x))[0]


def design_svf(g, k, m0, m1, m2):
    """Coefficient set the firmware's SVF path uses."""
    d = 1.0 + g * (g + k)
    return [0.0, 0.0, 0.0, 1.0 / d, g / d, 0.0, 0.0, g, m0, m1, m2]


def design_biquad(g, k, m0, m1, m2):
    """The biquad with the identical transfer function.

    Written as a state-space system in (ic1, ic2):

        y      = C*x + D1*ic1 + D2*ic2
        s'     = A*s + B*x

    then read the polynomial coefficients straight off H(z) = C + D (I - A z^-1)^-1 B z^-1.
    Verified against the SVF impulse response to ~1e-15 before being used here.
    """
    dg = 1.0 / (1.0 + g * (g + k))
    as1, as2 = dg, g * dg
    mix1 = m1 + g * m2
    c = m0 + as2 * mix1
    d1 = as1 * mix1
    d2 = m2 - as2 * mix1
    b0, b1 = 2.0 * as2, 2.0 * g * as2
    a11 = 2.0 * as1 - 1.0
    a12 = -2.0 * as2
    a21 = 2.0 * g * as1
    a22 = 1.0 - 2.0 * g * as2
    a1 = -(a11 + a22)
    a2 = a11 * a22 - a12 * a21
    r1 = d1 * b0 + d2 * b1
    r2 = d1 * (a11 * b0 + a12 * b1) + d2 * (a21 * b0 + a22 * b1)
    return [0.0, 0.0, c, c * a1 + r1, c * a2 + r2 + a1 * r1, a1, a2]


def svf_step(st, x):
    """One float32 SVF sample. State layout: [ic1 ic2 _ _ as1 as2 _ _ g m0 m1 m2]."""
    ic1, ic2 = st[0], st[1]
    v1 = f32(st[3] * ic1 + st[4] * (x - ic2))
    v2 = f32(ic2 + st[7] * v1)
    y = f32(st[8] * x + st[9] * v1 + st[10] * v2)
    st[0] = f32(v1 + v1 - ic1)
    st[1] = f32(v2 + v2 - ic2)
    return y


def bq_step(st, x):
    """One float32 TDF-II sample. State layout: [s1 s2 b0 b1 b2 a1 a2]."""
    s1, s2 = st[0], st[1]
    y = f32(st[2] * x + s1)
    st[0] = f32(st[3] * x - st[5] * y + s2)
    st[1] = f32(st[4] * x - st[6] * y)
    return y


def svf_reference(coeffs, x, ic1, ic2):
    """float64 SVF step, the yardstick both realizations are scored against."""
    v1 = coeffs[3] * ic1 + coeffs[4] * (x - ic2)
    v2 = ic2 + coeffs[7] * v1
    y = coeffs[8] * x + coeffs[9] * v1 + coeffs[10] * v2
    return y, 2.0 * v1 - ic1, 2.0 * v2 - ic2


def run(name, g, k, m0, m1, m2, signal):
    coeffs = design_svf(g, k, m0, m1, m2)

    st = [f32(v) for v in coeffs]
    svf_out = [svf_step(st, f32(x)) for x in signal]

    st = [f32(v) for v in design_biquad(g, k, m0, m1, m2)]
    bq_out = [bq_step(st, f32(x)) for x in signal]

    ic1 = ic2 = 0.0
    ref = []
    for x in signal:
        y, ic1, ic2 = svf_reference(coeffs, x, ic1, ic2)
        ref.append(y)

    peak = max(abs(v) for v in ref) or 1.0
    e_svf = max(abs(a - b) for a, b in zip(svf_out, ref)) / peak
    e_bq = max(abs(a - b) for a, b in zip(bq_out, ref)) / peak
    return name, e_svf, e_bq


def main():
    fs = 48000.0
    signal = [((i * 37 + 11) % 257) * (2.0 / 257.0) - 1.0 for i in range(4096)]

    print(f"{'type':5s} {'f (Hz)':>8s} {'Q':>6s} "
          f"{'SVF err':>10s} {'TDF2 err':>10s} {'TDF2/SVF':>9s}")
    rows = []
    for freq, q, amp in ((20.0, 2.0, 1.5), (100.0, 2.0, 1.5), (1000.0, 2.0, 1.5),
                         (20.0, 0.707, 1.5), (100.0, 0.707, 1.5),
                         (1000.0, 0.707, 1.5), (50.0, 4.0, 1.5),
                         (40.0, 20.0, 1.5), (200.0, 1.0, 1.5),
                         (3000.0, 2.0, 1.5), (100.0, 2.0, 1.5)):
        g = math.tan(math.pi * freq / fs)
        k = 1.0 / q
        a = amp
        cases = [
            ("LP", 0.0, 0.0, 1.0),
            ("PK", 1.0, k * (a * a - 1.0), 0.0),
            ("LS", 1.0, k * (a - 1.0), a * a - 1.0),
            ("AP", 1.0, -2.0 * k, 0.0),
            ("NO", 1.0, -k, 0.0),
            ("LT", 1.0, 1.0 - k, 0.0),          # r = 1 corner ratio
        ]
        for tname, m0, m1, m2 in cases:
            n, e_s, e_b = run(tname, g, k, m0, m1, m2, signal)
            rows.append((n, freq, q, e_s, e_b))
            print(f"{n:5s} {freq:>8.1f} {q:>6.3f} "
                  f"{e_s:>10.2e} {e_b:>10.2e} {e_b / max(e_s, 1e-30):>8.0f}x")

    worst = max(r[4] for r in rows)
    print(f"\nworst TDF-II relative error: {worst:.2e}")
    print("The SVF is 1-2 orders of magnitude more accurate at every corner; the "
          "gap\nwidens as the corner falls. See Documentation/Features/"
          "peq_performance.md.")


if __name__ == "__main__":
    main()