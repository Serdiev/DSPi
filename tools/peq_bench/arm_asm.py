#!/usr/bin/env python3
"""Count Cortex-M33 instructions per PEQ band-sample from generated assembly.

The host benchmark in run.py measures wall-clock cycles on x86, which is only
an indirect proxy for the RP2350's in-order Cortex-M33.  This script closes the
loop: it compiles dsp_pipeline.c for the real target core with clang and counts
the instructions in every unrolled sample loop of dsp_process_channel_block(),
reporting instructions per sample per kernel.  That number is what the kernel
optimisations in this series are measured against.

    python tools/peq_bench/arm_asm.py [--fw firmware/DSPi]
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
SHIM = os.path.join(HERE, "shim")

LABEL = re.compile(r"^(\.LBB[\w_]+):")
INSTR = re.compile(r"^\s+([a-z][a-z0-9.]*)\b")
STORE = re.compile(r"^v?str\b")


def strip(line: str) -> str:
    for sep in ("@", "//", ";"):
        i = line.find(sep)
        if i >= 0:
            line = line[:i]
    return line.rstrip()


def gen_asm(fw_dir: str, out: str) -> None:
    cmd = [
        "clang", "--target=arm-none-eabi",
        "-mcpu=cortex-m33", "-mfloat-abi=hard", "-mfpu=fpv5-sp-d16",
        "-O2", "-ffp-contract=fast",
        "-nostdlibinc", "-I", SHIM, "-I", fw_dir,
        "-DPICO_RP2350=1",
        "-S", os.path.join(fw_dir, "dsp_pipeline.c"), "-o", out,
    ]
    print("  $ " + " ".join(cmd), file=sys.stderr)
    subprocess.run(cmd, check=True)


def function_body(lines, name):
    start = None
    for i, l in enumerate(lines):
        if strip(l).strip() == name + ":":
            start = i
            break
    if start is None:
        raise SystemExit(f"function {name} not found in assembly")
    out = []
    for l in lines[start + 1:]:
        out.append(l)
        if l.strip().startswith(".size") and name in l:
            break
    return out


def split_blocks(body):
    blocks = []
    cur = None
    for raw in body:
        line = strip(raw)
        m = LABEL.match(line)
        if m:
            cur = {"label": m.group(1), "instrs": []}
            blocks.append(cur)
            continue
        if cur is None:
            continue
        s = line.strip()
        if not s or s.startswith("."):
            continue
        m = INSTR.match(line)
        if m:
            cur["instrs"].append(m.group(1))
    return blocks


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--fw", default=os.path.join(ROOT, "firmware", "DSPi"))
    ap.add_argument("--func", default="dsp_process_channel_block")
    ap.add_argument("--base-ref", default=None,
                    help="git ref to compare the instruction counts against")
    args = ap.parse_args()

    with tempfile.TemporaryDirectory() as tmp:
        asm = os.path.join(tmp, "dsp_pipeline.s")
        gen_asm(args.fw, asm)
        blocks = split_blocks(function_body(open(asm, encoding="utf-8",
                                                 errors="replace").read().splitlines(),
                                              args.func))

    loops = [b for b in blocks
             if sum(1 for i in b["instrs"] if STORE.match(i)) >= 4]
    per = sorted(len(b["instrs"]) / sum(1 for i in b["instrs"] if STORE.match(i))
                for b in loops)
    stats = (sum(per) / len(per), per[-1], per[len(per) // 2])

    print(f"\n{args.func}: unrolled sample loops on Cortex-M33")
    if args.base_ref:
        base = measure_ref(args.base_ref, args.func)
        for label, b, c in (("mean", base[0], stats[0]), ("median", base[2], stats[2]),
                            ("worst", base[1], stats[1])):
            print(f"  {label:>6} instructions per band-sample: {b:6.2f} -> {c:6.2f}"
                  f"  ({(b - c) / b * 100.0:+.1f}%)")
        return 0

    print(f"{'block':>14} {'instrs':>7} {'stores':>7} {'per sample':>11}")
    print("-" * 42)
    for b in loops:
        stores = sum(1 for i in b["instrs"] if STORE.match(i))
        print(f"{b['label']:>14} {len(b['instrs']):>7} {stores:>7} "
              f"{len(b['instrs']) / stores:>11.2f}")
    print(f"{'mean':>14} {'':>7} {'':>7} {stats[0]:>11.2f}")
    return 0


def measure_ref(ref: str, func: str) -> tuple[float, float, float]:
    import shutil
    tmp = tempfile.mkdtemp(prefix="peqasm_")
    try:
        wt = os.path.join(tmp, "wt")
        subprocess.run(["git", "-C", ROOT, "worktree", "add", "--detach", wt, ref],
                       check=True)
        try:
            asm = os.path.join(tmp, "base.s")
            gen_asm(os.path.join(wt, "firmware", "DSPi"), asm)
            blocks = split_blocks(function_body(
                open(asm, encoding="utf-8", errors="replace").read().splitlines(), func))
        finally:
            subprocess.run(["git", "-C", ROOT, "worktree", "remove", "--force", wt],
                           check=False)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    loops = [b for b in blocks
             if sum(1 for i in b["instrs"] if STORE.match(i)) >= 4]
    per = sorted(len(b["instrs"]) / sum(1 for i in b["instrs"] if STORE.match(i))
                 for b in loops)
    return (sum(per) / len(per), per[-1], per[len(per) // 2])


if __name__ == "__main__":
    raise SystemExit(main())