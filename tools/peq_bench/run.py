#!/usr/bin/env python3
"""Build and drive the PEQ host benchmark (tools/peq_bench/peq_bench.c).

The firmware PEQ sources are compiled for the host in freestanding mode
(-nostdlibinc against shim/math.h + shim/string.h) and linked into a DLL with
lld, so no system libc or ARM toolchain is required.

    python tools/peq_bench/run.py                 # bench the working tree
    python tools/peq_bench/run.py --base-ref main # also compare against a git ref
    python tools/peq_bench/run.py --bands 4,8,12,20 --block 192 --iters 400
"""

from __future__ import annotations

import argparse
import ctypes
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
SHIM = os.path.join(HERE, "shim")


def _run(cmd, **kw):
    print("  $ " + " ".join(cmd), file=sys.stderr)
    subprocess.run(cmd, check=True, **kw)


def build_dll(fw_dir: str, out_dll: str, extra_defines=()) -> None:
    """Compile the PEQ path from `fw_dir` (a firmware/DSPi directory) into a DLL."""
    sources = [
        os.path.join(HERE, "peq_bench.c"),
        os.path.join(SHIM, "host_math.c"),
        os.path.join(fw_dir, "dsp_pipeline.c"),
    ]
    # The real crossover, so that dsp_recalculate_all_filters()'s cost includes
    # the crossover rebuild it triggers.  Falls back to the stub when the source
    # tree does not have it.
    xover = os.path.join(fw_dir, "crossover.c")
    sources.append(xover if os.path.exists(xover)
                   else os.path.join(SHIM, "crossover_stub.c"))
    for s in sources:
        if not os.path.exists(s):
            raise SystemExit(f"missing source: {s}")

    defines = list(extra_defines)
    # The harness has to build against every revision in the series, including
    # ones that predate a given feature; select the call-site behaviour from the
    # tree under test rather than failing to compile.
    hdr = open(os.path.join(fw_dir, "dsp_pipeline.h"), encoding="utf-8").read()
    defines.append("-DPEQ_BENCH_SILENT_GUARD=%d" %
                   ("dsp_peq_block_is_silent" in hdr))
    # dsp_svf.h documents the pre-folded coefficient convention; revisions
    # before that commit store g and m1 in textbook form.
    svf = open(os.path.join(fw_dir, "dsp_svf.h"), encoding="utf-8").read()
    defines.append("-DPEQ_BENCH_FOLDED_SVF=%d" % ("pre-doubled" in svf))

    cmd = [
        "clang", "-O2", "-fno-math-errno",
        "-nostdlibinc",
        "-I", SHIM, "-I", fw_dir,
        "-DPICO_RP2350=1",
        *defines,
        "-shared", "-nostdlib", "-fuse-ld=lld",
        "-Wl,/entry:DllMain",
        *sources,
        "-o", out_dll,
    ]
    _run(cmd, stdout=subprocess.DEVNULL)


class Bench:
    def __init__(self, dll_path: str):
        self.lib = ctypes.WinDLL(dll_path)
        self.lib.peq_bench_run.restype = ctypes.c_double
        self.lib.peq_bench_run.argtypes = [ctypes.c_uint] * 5
        self.lib.peq_bench_design_cost.restype = ctypes.c_double
        self.lib.peq_bench_design_cost.argtypes = [ctypes.c_uint] * 4
        self.lib.peq_bench_design_cycles.restype = ctypes.c_double
        self.lib.peq_bench_cold_design_cost.restype = ctypes.c_double
        self.lib.peq_bench_cold_design_cost.argtypes = [ctypes.c_uint] * 4
        self.lib.peq_bench_num_channels.restype = ctypes.c_uint
        self.lib.peq_bench_max_bands.restype = ctypes.c_uint
        self.lib.peq_bench_active_bands.restype = ctypes.c_uint
        self.lib.peq_bench_checksum.restype = ctypes.c_double
        self.lib.peq_bench_probe.restype = ctypes.c_float
        self.lib.peq_bench_probe.argtypes = [ctypes.c_uint, ctypes.c_float,
                                             ctypes.c_float, ctypes.c_float]
        self.lib.peq_bench_probe_freq.restype = ctypes.c_float
        self.lib.peq_bench_probe_q.restype = ctypes.c_float
        self.lib.peq_bench_probe_gain.restype = ctypes.c_float
        self.lib.peq_bench_probe_coeff.restype = ctypes.c_float
        self.lib.peq_bench_probe_coeff.argtypes = [ctypes.c_uint]
        self.lib.peq_bench_set_type.argtypes = [ctypes.c_int]
        if hasattr(self.lib, "peq_bench_recalc_check"):
            self.lib.peq_bench_recalc_check.restype = ctypes.c_uint
            self.lib.peq_bench_recalc_check.argtypes = [ctypes.c_uint]
        self.lib.peq_bench_block_ptr.restype = ctypes.POINTER(ctypes.c_float)
        self.lib.peq_bench_block_ptr.argtypes = [ctypes.c_uint]
        self.channels = int(self.lib.peq_bench_num_channels())
        self.max_bands = int(self.lib.peq_bench_max_bands())

    def run(self, bands, block, iters, active_pct=100, seed=1):
        cycles = float(self.lib.peq_bench_run(bands, block, iters, active_pct, seed))
        design = float(self.lib.peq_bench_design_cycles())
        active = int(self.lib.peq_bench_active_bands())
        return cycles, design, active

    def blocks(self, ch, block):
        ptr = self.lib.peq_bench_block_ptr(ch)
        return [ptr[i] for i in range(block)]


def measure(b: Bench, bands, block, iters, active_pct, reps=5):
    """Best-of-`reps` run cycles, to suppress scheduler noise."""
    best = None
    active = 0
    design = 0.0
    for _ in range(reps):
        cycles, design, active = b.run(bands, block, iters, active_pct)
        if best is None or cycles < best:
            best = cycles
    return best, design, active


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--base-ref", default=None,
                    help="git ref to compare against (e.g. main); exports a worktree")
    ap.add_argument("--bands", default="4,8,12,20")
    ap.add_argument("--block", type=int, default=192)
    ap.add_argument("--iters", type=int, default=300)
    ap.add_argument("--active-pct", type=int, default=100)
    ap.add_argument("--silent-from", type=int, default=None,
                    help="channel index from which every channel carries digital "
                         "silence (models unused outputs on a wide device)")
    ap.add_argument("--reps", type=int, default=5)
    ap.add_argument("--tol", type=float, default=1e-3,
                    help="max allowed relative output difference (float rounding "
                         "between two realizations of the same transfer function "
                         "accumulates over a resonant cascade)")
    ap.add_argument("--no-equiv", action="store_true",
                    help="skip the sample-for-sample equivalence check")
    ap.add_argument("--no-probe", action="store_true",
                    help="skip the SVF/TDF-II design cross-check")
    args = ap.parse_args()

    band_list = [int(x) for x in args.bands.split(",")]
    tmp = tempfile.mkdtemp(prefix="peqbench_")
    try:
        cand_dll = os.path.join(tmp, "cand.dll")
        build_dll(os.path.join(ROOT, "firmware", "DSPi"), cand_dll)
        cand = Bench(cand_dll)
        if args.silent_from is not None:
            cand.lib.peq_bench_set_silent(args.silent_from)

        base = None
        if args.base_ref:
            wt = os.path.join(tmp, "wt")
            _run(["git", "-C", ROOT, "worktree", "add", "--detach", wt, args.base_ref])
            try:
                base_dll = os.path.join(tmp, "base.dll")
                build_dll(os.path.join(wt, "firmware", "DSPi"), base_dll)
                base = Bench(base_dll)
                if args.silent_from is not None:
                    base.lib.peq_bench_set_silent(args.silent_from)
            finally:
                subprocess.run(["git", "-C", ROOT, "worktree", "remove", "--force", wt],
                               check=False)

        print(f"\nblock={args.block} iters={args.iters} "
              f"active={args.active_pct}%  channels={cand.channels}")
        hdr = f"{'bands':>6} {'active':>7} {'cand cyc':>12}"
        if base:
            hdr += f" {'base cyc':>12} {'speedup':>8} {'rel err':>10}"
        print(hdr)
        print("-" * len(hdr))

        ok = True
        for bands in band_list:
            if bands > cand.max_bands:
                print(f"{bands:>6}   (skipped: MAX_BANDS={cand.max_bands})")
                continue
            c_cyc, c_design, c_active = measure(cand, bands, args.block, args.iters,
                                                 args.active_pct, args.reps)
            row = f"{bands:>6} {c_active:>7} {c_cyc:>12.0f}"
            if base:
                b_cyc, b_design, b_active = measure(base, bands, args.block, args.iters,
                                                    args.active_pct, args.reps)
                # Equivalence: single block, identical excitation, compare
                # samples relative to the signal's own scale (a 12-band
                # cascade can legitimately change the level by tens of dB,
                # so an absolute threshold would be meaningless).
                if not args.no_equiv:
                    cand.run(bands, args.block, 1, args.active_pct)
                    base.run(bands, args.block, 1, args.active_pct)
                    scale = 0.0
                    diffs = []
                    for ch in range(cand.channels):
                        a = cand.blocks(ch, args.block)
                        b = base.blocks(ch, args.block)
                        scale = max(scale, max(abs(v) for v in b))
                        diffs.extend(abs(x - y) for x, y in zip(a, b))
                    worst = max(diffs) / scale if scale else 0.0
                    status = "" if worst <= args.tol else "  <-- MISMATCH"
                    if worst > args.tol:
                        ok = False
                else:
                    worst = float("nan")
                    status = ""
                row += (f" {b_cyc:>12.0f} {b_cyc / c_cyc if c_cyc else 0:>7.2f}x "
                        f"{worst:>10.2e}{status}")
            print(row)

        print("\ndesign cost (cycles per dsp_recalculate_all_filters):")
        for bands in band_list:
            if bands > cand.max_bands:
                continue
            dc = float(cand.lib.peq_bench_design_cost(bands, 200, args.active_pct, 1))
            line = f"  bands={bands:>3}: {dc:>10.0f}"
            if base:
                db = float(base.lib.peq_bench_design_cost(bands, 200, args.active_pct, 1))
                line += f"   base {db:>10.0f}   {db / dc if dc else 0:>5.2f}x"
            print(line)

        print("\ncold redesign cost (cycles, rate changed every call):")
        for bands in band_list:
            if bands > cand.max_bands:
                continue
            dc = float(cand.lib.peq_bench_cold_design_cost(bands, 200, args.active_pct, 1))
            line = "  bands=%3d: %10.0f" % (bands, dc)
            if base:
                db = float(base.lib.peq_bench_cold_design_cost(bands, 200, args.active_pct, 1))
                line += "   base %10.0f   %5.2fx" % (db, db / dc if dc else 0)
            print(line)

        if hasattr(cand.lib, "peq_bench_recalc_check"):
            bad = int(cand.lib.peq_bench_recalc_check(min(12, cand.max_bands)))
            print("\nincremental-recalc check: %d sections changed that should "
                  "not have" % bad)
            if bad:
                print("  <-- MISMATCH")
                ok = False

        if not args.no_probe and hasattr(cand.lib, "peq_bench_probe"):
            types = {0: "FLAT", 1: "PEAKING", 2: "LOWSHELF", 3: "HIGHSHELF",
                     4: "LOWPASS", 5: "HIGHPASS", 6: "NOTCH", 7: "ALLPASS",
                     8: "ALLPASS1", 9: "LOWSHELF1", 10: "HIGHSHELF1",
                     11: "LINKWITZ", 12: "LOWPASS1", 13: "HIGHPASS1"}
            first_order = {8, 9, 10, 12, 13}
            worst = (0.0, None)
            for t in types:
                if t in first_order:
                    continue
                for fr in (20., 60., 200., 800., 3000., 6000.):
                    for q in (0.1, 0.707, 2.0, 20.0):
                        for gn in (-12.0, -3.0, 0.5, 6.0, 12.0):
                            gain = fr * 0.5 if t == 11 else gn
                            d = float(cand.lib.peq_bench_probe(t, fr, q, gain))
                            if d > worst[0]:
                                c = [float(cand.lib.peq_bench_probe_coeff(i))
                                     for i in range(5)]
                                worst = (d, (types[t], fr, q, gain, c))
            print(f"\nSVF-vs-dispatched-kernel cross-check: worst relative |dy| = "
                  f"{worst[0]:.3e}")
            if worst[1]:
                n, fr, q, gn, c = worst[1]
                print(f"  worst case: {n} f={fr} Q={q} gain={gn}  "
                      f"b=[{c[0]:.6f} {c[1]:.6f} {c[2]:.6f}] a=[{c[3]:.6f} {c[4]:.6f}]")
            if worst[0] > args.tol:
                print("  <-- MISMATCH")
                ok = False

        return 0 if ok else 1
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())