# PEQ benchmark / equivalence harness

Host-side measurement rig for the RP2350 PEQ (`dsp_pipeline.c` +
`dsp_svf.h` + `dsp_biquad.h`). It compiles the real firmware sources for the
host, runs them through a deterministic workload, and reports:

* throughput in host cycles for a given band count,
* a sample-for-sample comparison against a build of another revision,
* an SVF-vs-dispatched-kernel cross-check of every filter design,
* the coefficient-design (control-path) cost of `dsp_recalculate_all_filters()`,
* Cortex-M33 instructions per band-sample, from generated ARM assembly.

There is no ARM toolchain in CI and the firmware cannot run on a PC, so this is
how the PEQ changes in this series get their numbers.

## Why it is built like this

`clang -nostdlibinc` against `shim/math.h` + `shim/string.h` lets the firmware
DSP sources compile without a system libc; `clang -shared -nostdlib
-fuse-ld=lld` links them into a plain DLL that Python drives through
`ctypes`. Only two third-party tools are needed: `clang` and `python`.

`shim/host_math.c` implements the handful of libm entry points the coefficient
design calls (`sinf`, `cosf`, `tanf`, `powf`, ...) in double precision. Both
builds under comparison link the same implementation, so any inaccuracy in it
cancels out of the comparison; the absolute accuracy only has to be good enough
for the filters to behave like filters.

`shim/crossover_stub.c` satisfies the crossover entry points
`dsp_pipeline.c` calls for default-init and sample-rate rebuilds. The benchmark
does not exercise crossover filters.

## Usage

```
python tools/peq_bench/run.py                        # benchmark the working tree
python tools/peq_bench/run.py --base-ref main        # + compare against a git ref
python tools/peq_bench/run.py --bands 4,8,12,20 --iters 400 --reps 5
python tools/peq_bench/arm_asm.py                    # Cortex-M33 instructions/sample
```

`run.py` exits non-zero if the equivalence check or the design cross-check
fails, so it works as a regression test.

## Reading the numbers

* **Host cycles** are an x86-64 proxy. It is good for ranking implementations
  that differ in operation count or memory traffic; it is not a model of the
  RP2350's in-order Cortex-M33.
* **`arm_asm.py`** is the target-specific number: it counts the instructions in
  every unrolled sample loop of `dsp_process_channel_block()` in the assembly
  clang generates for `-mcpu=cortex-m33 -mfpu=fpv5-sp-d16`. clang does not
  contract multiply-add into `vfma.f32` for that core while the firmware's GCC
  build does, so the counts are slightly pessimistic — uniformly, for baseline
  and candidate, which is what the comparisons need.
* **Relative error** in the equivalence table is normalised by the output
  block's own peak. Two realizations of the same transfer function differ in
  the last bits, and those differences accumulate over a resonant cascade; the
  default tolerance (`--tol 1e-3`, about -60 dB) is far above the noise floor
  and far below anything audible.