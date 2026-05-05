"""Extract a clean upward step window from step_kf1.csv for use as a fit reference.

The raw CSV spans 51+ seconds with repeated 2.5-s step cycles. The first window
(0–2.5 s) is a startup transient from h≈0.78 m and is unsuitable for fitting.
We take the second complete cycle (t = 5.0–7.5 s): h_des goes 0.35→0.45 m,
which is the first clean upward step after the system has settled.

Output columns: time (re-zeroed to 0), q (measured angle), dq (angular velocity).
The evaluator only requires q; dq is included for the dq_weight term.
"""

from pathlib import Path

import numpy as np
import pandas as pd

STEP_START = 5.0   # seconds in the raw CSV where the clean step begins
STEP_END   = 7.5   # seconds in the raw CSV where the step cycle ends
H_DES_BEFORE = 0.35
H_DES_AFTER  = 0.45

_REPO_ROOT = Path(__file__).resolve().parents[1]
# real_data lives in the main project, not in a worktree sub-path
_DATA_ROOT = _REPO_ROOT / "real_data"
if not _DATA_ROOT.exists():
    # running from inside a git worktree: walk up to find the shared real_data dir
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "real_data"
        if candidate.exists():
            _DATA_ROOT = candidate
            break

RAW_CSV = _DATA_ROOT / "processed_csv" / "step_kf1.csv"
OUT_CSV = _DATA_ROOT / "step_reference.csv"


def main() -> None:
    df = pd.read_csv(RAW_CSV)

    window = df[(df["time"] >= STEP_START) & (df["time"] < STEP_END)].copy()
    if window.empty:
        raise RuntimeError(f"No data found between t={STEP_START} and t={STEP_END}")

    # Verify the step is in this window
    h_vals = window["h_des"].unique()
    if not (H_DES_BEFORE in h_vals and H_DES_AFTER in h_vals):
        raise RuntimeError(
            f"Expected h_des values {H_DES_BEFORE} and {H_DES_AFTER} in window; "
            f"found: {sorted(h_vals)}"
        )

    window["time"] = window["time"] - STEP_START
    window = window.reset_index(drop=True)

    ref = window[["time", "q", "dq"]].copy()

    q0 = float(ref["q"].iloc[0])
    q_final = float(ref["q"].iloc[-1])
    peak_q = float(ref["q"].max())
    print(f"Step window: t={STEP_START}–{STEP_END} s  (re-zeroed to 0–{STEP_END - STEP_START} s)")
    print(f"  q initial : {q0:.4f} rad  (h_des={H_DES_BEFORE} m)")
    print(f"  q final   : {q_final:.4f} rad  (h_des={H_DES_AFTER} m)")
    print(f"  q peak    : {peak_q:.4f} rad  (overshoot present: {peak_q > q_final})")
    print(f"  rows      : {len(ref)}")

    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    ref.to_csv(OUT_CSV, index=False)
    print(f"Saved → {OUT_CSV}")


if __name__ == "__main__":
    main()
