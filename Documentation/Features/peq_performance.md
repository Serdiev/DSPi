# PEQ Performance — Findings, Measurements and Rejected Ideas

*Last updated: 2026-08-17*
*RP2350 (Pico 2) primary, RP2040 noted where relevant*
*Companion to `peq_filters.md` (host protocol) — this document is about the
audio path and its cost, not the wire format.*

This records an investigation into how much PEQ performance headroom exists on
this hardware, what was taken, and — more importantly — what was tried and
rejected on measurement. The rejected list is the valuable part: several of
those ideas look obviously correct and would be re-attempted by the next
person who reads the SVF code.

The measurement rig is `tools/peq_bench`; see its `README.md`. Everything below
was produced by it or by `arm_asm.py` on that rig.

---

## 1. The headline finding: the audio path was already near its floor

A second-order IIR section needs **four multiplies**. The RP2350 SVF path was
already spending four, already had per-type specialisation that removes the
zero-multiplies, already had 4x-unrolled inner loops, and already had
flush-to-zero enabled on both cores (`main.c`). Generated code for the
second-order band loops on Cortex-M33:

```
   mean instructions per band-sample:  10.07
  median instructions per band-sample:   9.25
   worst instructions per band-sample:  17.50   (three-coefficient mix)
```

So the **worst realistic total win available on the audio path was ~1.45x**,
and that is what was taken. Anyone expecting several-fold gains from smarter
filter arithmetic on this core should read section 4 before starting.

Corollary: the four multiplies are the floor, and 4 is what it already was.
Further gains had to come from *not doing work*, which is what most of this
series turned out to be.

### Why the RP2350 offers no SIMD escape hatch

A stereo-paired `float32x2` implementation — processing L and R through one
kernel with Helium — would be the obvious 2x. **The RP2350 is Cortex-M33, which
has no Helium.** Helium is M55/M85. `vcvt`/`vst4` style NEON is unavailable at
any -march. Cross-channel SIMD is not an option on this part, which is why the
per-section cost is a hard floor rather than a soft one.

---

## 2. What was taken

Five commits, each one change. Numbers are `tools/peq_bench/run.py` host cycles
unless stated; `rel err` is the output difference against `main` normalised by
the block's own peak.

| # | Change | Effect |
|---|--------|--------|
| 1 | Fold the SVF state and output-mix coefficients at design time (`Filter.g` holds `2*g`, `Filter.svm1` holds `m1 + g*m2`); `v2` is never materialised | 1.45x on the audio path; worst-case kernel 17.50 → 15.50 instructions/band-sample |
| 2 | Skip the cascade for digitally silent blocks (`dsp_peq_block_is_silent()`) | 1.00x when every channel carries signal; 2.4x–9.6x when they do not (section 3) |
| 3 | Per-band design shadow so an unchanged section is not redesigned | 26x lower `dsp_recalculate_all_filters()` cost when a bank is re-sent unchanged |
| 4 | Evaluate only the transcendentals a filter type uses | 1.27x faster cold redesign; output bit-identical |
| 5 | The same design shadow for the crossover bank the same entry point rebuilds | the crossover half of #3 |

Combined, with all 17 channels carrying signal (12-band stress bank):

```
 audio path:  1.4x      (1.42x - 1.48x across band counts and repeat runs)

 bands  active   rel err (reproduces exactly)
     4      68       1.98e-07
     8     136       2.56e-07
    12     204       6.35e-07
```

Flat in band count, because it is a per-section win, not a per-block one.
The audio-path figure is quoted as a range deliberately: the x86 proxy carries
a few percent of run-to-run variance, which is the same size as the differences
between band counts, so two decimal places would be false precision. `rel err`
reproduces exactly and is the number to check when re-running.

### The numerical cost of change #1, honestly

Only #1 perturbs the output at all; #2–#5 are bit-identical (`rel err 0.00e+00`).

Change #1 replaces

```
ic2' = 2*(ic2 + g*v1) - ic2        ← fl(2·fl(ic2 + fl(g·v1)) − ic2)
```

with

```
ic2' = ic2 + 2*g*v1               ← fl(ic2 + fl(2g·v1))
```

which is a **different function** of the same inputs. Absolute worst case, one
192-sample block, 17 channels, a 12-band stress bank (±12 dB, Q up to 4.3):

| bands | peak \|y\| | max \|Δy\| | relative | dBFS | ≈24-bit LSB |
|---|---|---|---|---|---|
| 4 | 1.207 | 2.38e-07 | 2.0e-07 | −132.5 | 2 |
| 8 | 1.399 | 3.58e-07 | 2.6e-07 | −128.9 | 3 |
| 12 | 0.704 | 4.47e-07 | 6.4e-07 | −127.0 | 4 |

Two facts that make this defensible rather than alarming:

* **It is bounded, not a drift.** Both builds are the same linear recursion, so
  the difference between their states obeys the same homogeneous recurrence and
  decays with the filter's own poles. Feeding the same signal through 100,000
  consecutive blocks: the relative divergence stays inside a −98…−124 dB band
  throughout, it does not grow.
* **It is float rounding, not a response change.** The transfer function is
  algebraically identical; only the summation order differs.

If a downstream consumer needs bit-exactness (A/B against a captured render,
diffing rendered responses), see the escape hatch in section 5.

---

## 3. Where the large multipliers actually come from

Section 1 says the per-section cost is capped at 1.45x. The large numbers in
this series are all "do not process what is not there":

```
live channels of 17:      17      2      4      6      8     10     12
speedup:                1.42x  10.7x   5.4x   3.9x   2.9x   2.4x   2.0x
```

(the two-live-channel figure moves ±5% between runs; the rest are stable to
about ±2%)

A stereo source on a 17-channel device leaves most outputs unrouted; the matrix
mixer `memset`s those to zero and they were then being multiplied through every
band of their PEQ bank. `dsp_peq_block_is_silent()` scans with an **early
exit**, so a block that carries audio costs exactly one comparison — the
all-signal row above is unchanged by its presence.

Same shape for the control path: the 26x on design cost is a preset load, an
A/B compare or a re-sync re-sending a byte-identical bank.

---

## 4. Rejected ideas

Each of these looks correct from reading the source. Each was implemented or
measured and thrown away. **Do not re-attempt them without new evidence.**

### 4.1 TDF-II biquad realization of the second-order sections — *the big one*

The SVF design `(g, k, m0, m1, m2)` collapses **exactly** to a biquad; the
identity was derived and verified to ~1e-15 against the SVF impulse response
(the derivation is in PR #1's history and in
`tools/peq_bench/svf_conditioning.py`). It is **1.6x faster** and needs fewer
coefficients.

It is numerically far worse at low corners. `tools/peq_bench/svf_conditioning.py`
runs both realizations from the same design in float32 against a float64 SVF
reference:

```
type    f (Hz)      Q    SVF err   TDF2 err  TDF2/SVF
LP        20.0  2.000   5.39e-06   1.02e-03      189x
PK        20.0  2.000   7.18e-08   3.92e-04     5463x
LS        20.0  2.000   9.10e-08   1.42e-04     1564x
NO      100.0  2.000   6.76e-08   1.73e-05      256x
PK      1000.0  2.000   1.12e-07   8.22e-07        7x
NO      3000.0  2.000   1.09e-07   2.02e-07        2x
```

Cause: the biquad numerator is the small side of a near-cancellation
(`1 − cos ω` at 20 Hz is ~1e-6) while every SVF coefficient stays O(1). The
floating-point precision lost to that cancellation is unrecoverable at runtime.
**This is exactly what the Fs/7.5 SVF/biquad crossover in
`dsp_compute_coefficients()` exists to avoid.** Above ~3 kHz the two are
equivalent, which is why the biquad path is used there today.

### 4.2 Sample-major cascade to remove store→load forwarding

The obvious argument: in band-outer order, band *b+1* loads the sample band *b*
just stored, and store-to-load forwarding costs several cycles on an in-order
core. **The argument is wrong.** Band *b* finishes all 192 samples before band
*b+1* starts, so band *b+1*'s first load is 191 iterations after the store
retired. There is no stall to remove. The alternative (sample-outer, state in
RAM) has the same forwarding edge, now between *samples*, and additionally
turns one state load/store per block into one per sample.

### 4.3 Kernel-symmetry classification

`dsp_biquad.h` already specializes on coefficient relations that look like
they need a runtime classifier — `b1 == a1` for peaking/all-pass/notch,
`b0 == b2` for low-pass/high-pass/notch. Verified over the full realistic
parameter grid, **the SVF-derived coefficients satisfy the same relations
exactly** (to 3e-15), because the SVF design was derived from the same
topology. No classifier needed.

### 4.4 Compacting the active-section bank

Per skipped band per 192-sample block, the `bypass` load, test and branch cost
**2 instructions** — against ~1900 for a live band. At 12 bands with a quarter
live that is 0.08%. Removing the per-band `use_svf`/`first_order`/`filter_type`
dispatch saves ~7 instructions per *active* band per block, 0.3%. Measured as a
non-event. It only starts to matter somewhere past 30 bands, and even then the
hot working set (12 bands × 60 B × 17 ch = 12 KB) fits L1 with room to spare.

### 4.5 Folding one more multiply out of the three-coefficient mix

`FILTER_LOWSHELF` and `FILTER_LINKWITZ_TRANSFORM` have `m0 == 1` and
`FILTER_HIGHSHELF` has `m0 + m2 == 1`, so the feedthrough term can be folded
away: one multiply fewer per sample on the three slowest kernels. Implemented,
measured, **dropped**: splitting the kernel switch on those cases changed
clang's code layout and regressed the *other* kernels (bank mean 9.82 → 9.95
instructions/band-sample, worst case 15.50 → 14.75) while the x86 proxy showed
1.00x per filter type. The headline number was not defensible, so it went. Worth
revisiting with GCC and hardware, where the layout effect may not appear — note
that those two figures came from the throwaway branch and cannot be re-run from
this tree; the per-type 1.00x is reproducible.

### 4.6 Moving the design-shadow check into `dsp_compute_coefficients()`

To also cover the single-band `REQ_SET_EQ_PARAM` path. Measured **0.35x on the
full-rebuild path** — two extra 16-byte `memcmp`s and a non-inlined call on every
section design — for a 1.4x gain on a rare event. The check stayed in
`dsp_recalculate_all_filters()` where the compiler can hoist it out of the
loop.

### 4.7 Things already fine

Worth recording so nobody spends time on them: **flush-to-zero is already
enabled** on both cores (`main.c`, `pdm_generator.c`); the inner loops are
already 4x unrolled; **the RP2350 has no SIMD** (section 1); the delay-line
and meter passes are not on the PEQ critical path.

---

## 5. The bit-exact escape hatch

One of change #1's three saved operations is available with **no numerical
difference at all**. `ic1' = 2*v1 - ic1` is two instructions only because of
the sign of the state. Store the first integrator negated (`ic1n = -ic1`, with
`sva1` negated to match) and it collapses to:

```
ic1n' = vfma(-2.0f, v1, ic1n)
```

Doubling is exact in binary floating point and round-to-nearest is
sign-symmetric, so `fl(2·v1 − ic1)` and `fl(−2·v1 + ic1n)` are the same value
with the same single rounding. `v1` is unaffected — the sign fold lands in the
stored coefficient.

Worth about **1 op in 13 ≈ 8%**, against the ~45% the `ic2` reassociation
buys. `ic2` has no bit-exact alternative: `2·v2 − ic2` needs `−ic2`, and
maintaining a negated copy just trades the subtraction for the negation.

If bit-exactness turns out to matter, the options are:

| | audio path | output |
|---|---|---|
| as shipped | **1.45x** | 2–4 LSB (−127 dBFS) |
| replace #1 with the negated-`ic1` variant | ~1.05–1.08x | **bit-identical** |
| drop #1, keep #2–#5 | 1.00x per section; 2.4–9.6x on realistic mixes (#2); 26x on the design path (#3/#5) | **bit-identical** |

---

## 6. How many bands can this part actually run?

Two ceilings, and they are nowhere near each other.

### 6.1 Addressable: 20

The band index is a 5-bit wire field and `XOVER_BAND_BASE` is 20, so
`config.h` already reserves band indices 10..19 for PEQ growth. Going from 10
active bands to 20 is a `WIRE_MAX_BANDS` change (12 → 20), a
`WIRE_FORMAT_VERSION` bump (28 → 29) and host-side doc updates. No DSP work.

RAM cost: 17 channels × 10 extra sections × ~98 B (`Filter` + `EqParamPacket` +
the design shadow from #3) ≈ **17 KB**.

### 6.2 CPU-bound: well under 20, and core 0 decides

Per live channel, per band, at 48 kHz: `48000 × c` cycles, where `c` is cycles
per band-sample on the M33. Never measured on hardware; from the instruction
counts above at roughly 1 IPC, **c is 10–16, call it 12**. As a fraction of one
150 MHz core per channel per band:

| c | 0.32% | 0.384% | 0.512% |
|---|---|---|---|
| | c=10 | c=12 | c=16 |

The device runs at 307.2 MHz when that succeeds and 150 MHz otherwise
(`main.c`), so halve everything for the fast case.

**Core 0 owns 11 of the 17 channels** — all inputs plus outputs 0, 1 and the PDM
sub — *and* does USB, the matrix, metering, the leveller, the upmixer and the
control plane. Core 1 only handles outputs 2..7. At `c = 12` and 11 live
channels, the PEQ stage alone is **4.2% × bands** of core 0:

| bands | core 0 PEQ | core 1 PEQ (6 ch) |
|---|---|---|
| 10 (today) | 42% | 23% |
| 14 | 59% | 33% |
| 20 | 84% | 46% |

Core 0's 42% at 10 bands is most of why it is the bottleneck, so **20 bands per
channel is not reachable on core 0 at 150 MHz** however clever the filter code
gets.

In band terms, with the series applied:

* every channel live (worst case, where #2 does nothing): 1.45x → **10 → 14–15
  bands**, or **16** if the bank is mostly second-order peaking filters (the
  1.56–1.64x per-section figure);
* a realistic setup where most channels are silent: #2 makes idle channels free,
  so only the 2–4 live channels matter → **20+ is affordable**, which happens
  to meet the addressable ceiling;
* at 307 MHz, 20 bands on core 0 becomes ~42%.

**Measure it rather than trusting the table.** `global_status.cpu0_load` and
`cpu1_load` are already published to the host. Set 10 bands, read the load,
add a band, read again — the slope is `c × live channels`, measured. Everything
above is derived from instruction counts, and `c` could be 30% either side of 12.

---

## 7. Open items

1. **Core 0 owns 11 of 17 channels.** This is a worse imbalance than anything
   fixed in this series, and rebalancing is measurable from the two load
   values already on the wire. Likely worth more than the 1.45x.
2. **RP2040 is the biggest untouched cost.** The Q28 assembly kernel uses a
   generic 5-multiply biquad for every type (peaking needs 4, notch 3) and
   reloads all five coefficients from memory every sample. Nothing here touched
   it: there is no way to execute, or even assemble-test, a change to that file
   from the tooling available. It is also the part most likely to be unable to
   run 12 bands at 48 kHz on all 7 channels at all.
3. **The design shadows add ~4.4 KB of RAM** (`EqParamPacket` + `uint16` per
   section, both banks). Not checked against the linker script budget.
4. **Section 4.5 deserves a second look on hardware** with GCC, where the
   code-layout regression that killed it here may not appear.
5. If band counts grow past 12, revisit section 4.4 — it is the first thing
   that becomes worth anything.

---

## 8. Reproducing any of this

```
python tools/peq_bench/run.py --base-ref main            # harness + series table
python tools/peq_bench/run.py --base-ref main --silent-from 4   # section 3 table
python tools/peq_bench/arm_asm.py --base-ref main       # Cortex-M33 instruction counts
python tools/peq_bench/svf_conditioning.py              # section 4.1 table
```

`run.py` exits non-zero if the block-equivalence check, the design cross-check
or the incremental-recalc check fails.