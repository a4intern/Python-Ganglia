#!/usr/bin/env python3
"""
Optuna-based hyperparameter search for ADRC and PID velocity controllers.

For each controller, runs `--trials` step-response evaluations across all
`--targets`, scores each with a composite ITAE metric, then calls the
benchmark script with the best-found parameters to generate a report.

Usage:
    .venv/bin/python3 scripts/optimize_controllers.py
    .venv/bin/python3 scripts/optimize_controllers.py --targets 100 200 --trials 40
    .venv/bin/python3 scripts/optimize_controllers.py --targets 100 --trials 15  # quick

Requires the mock (or real) server to be running first:
    python3 run_headless.py        # or  .venv/bin/python3 mock_sim.py
"""
import argparse
import asyncio
import json
import math
import pathlib
import statistics
import subprocess
import sys
import time

import optuna
import requests
import websockets

optuna.logging.set_verbosity(optuna.logging.WARNING)

BASE_URL   = "http://127.0.0.1:8000"
WS_TEL_URL = "ws://127.0.0.1:8000/ws/telemetry"
LOG_DIR    = pathlib.Path(__file__).parent.parent / "logs"
DB_PATH    = LOG_DIR / "optuna.db"

# Physical limits
V_MAX      = 24.0   # rail voltage
PID_COUNTS = 4000   # PID output count at full voltage


# ---------------------------------------------------------------------------
# Motor control helpers
# ---------------------------------------------------------------------------

def _post(path, body=None):
    return requests.post(f"{BASE_URL}{path}", json=body or {}, timeout=5).json()


def _get(path):
    return requests.get(f"{BASE_URL}{path}", timeout=5).json()


def motor_init(port: str = "Virtual Motor"):
    _post("/connect",     {"port": port, "device_id": 48})
    _post("/set_op_mode", {"mode": -2})
    # Start in ADRC mode; we'll switch per trial
    _post("/set_pid", {"mode": "velocity", "p": 0, "i": 0, "d": 0,
                       "gain_output": 1.0, "limit_i": 30000, "blend": 100})
    _post("/set_adrc", {"mode": "velocity", "wc": 3.0, "b0": 120.0, "ramp_time": 0.0})
    _post("/set_target", {"mode": "velocity", "value": 0,
                          "min_limit": -4000, "max_limit": 4000})
    _post("/start")
    time.sleep(0.5)


def _apply_adrc(wc: float, b0: float):
    _post("/set_pid", {"mode": "velocity", "p": 0, "i": 0, "d": 0,
                       "gain_output": 1.0, "limit_i": 30000, "blend": 100})
    _post("/set_adrc", {"mode": "velocity", "wc": wc, "b0": b0, "ramp_time": 0.0})


def _apply_pid(p: float, i: float, d: float):
    _post("/set_pid", {"mode": "velocity", "p": p, "i": i, "d": d,
                       "gain_output": 1.0, "limit_i": 5000, "blend": 0})


def _set_target(rpm: float):
    _post("/set_target", {"mode": "velocity", "value": int(rpm),
                          "min_limit": -4000, "max_limit": 4000})


# ---------------------------------------------------------------------------
# Telemetry capture
# ---------------------------------------------------------------------------

async def _capture_async(duration: float) -> list:
    samples, t0 = [], time.time()
    try:
        async with websockets.connect(WS_TEL_URL) as ws:
            while time.time() - t0 < duration:
                try:
                    pts = json.loads(await asyncio.wait_for(ws.recv(), 0.5))
                    for pt in pts:
                        if "velocity" in pt:
                            samples.append({
                                "t":        round(time.time() - t0, 4),
                                "velocity": pt["velocity"],
                            })
                except asyncio.TimeoutError:
                    continue
    except Exception:
        pass
    return samples


def capture(duration: float) -> list:
    return asyncio.run(_capture_async(duration))


# ---------------------------------------------------------------------------
# Objective (composite score, lower = better)
#
# M = ITAE/target²  +  e_ss/|target|  +  stdev/|target|
#       transient          dc error        noise
#
# The three terms are all dimensionless and roughly 0–1 for a well-tuned
# controller, making the composite easy to reason about.
# A large penalty (100) is added for motor stall (mean < 5% of target).
# ---------------------------------------------------------------------------

_STALL_PENALTY = 100.0


def _score_one(samples: list, target: float) -> float:
    """Composite score for a single step-test; lower is better."""
    if not samples or abs(target) < 1:
        return _STALL_PENALTY

    vels  = [s["velocity"] for s in samples]
    t_arr = [s["t"]        for s in samples]

    ss_start = int(len(vels) * 0.75)
    ss_vels  = vels[ss_start:] or vels
    ss_mean  = statistics.mean(ss_vels)
    ss_stdev = statistics.stdev(ss_vels) if len(ss_vels) > 1 else 0.0

    # Stall detection: motor never reached 5% of target
    if abs(ss_mean) < 0.05 * abs(target):
        return _STALL_PENALTY

    e_ss = abs(ss_mean - target)

    # ITAE (from t=0; the step is applied at t≈0)
    ITAE = sum(
        t_arr[i] * abs(target - vels[i]) * (t_arr[i+1] - t_arr[i])
        for i in range(len(samples) - 1)
    )

    return (
        ITAE      / (target ** 2)   # transient shape
        + e_ss    / abs(target)     # dc accuracy
        + ss_stdev / abs(target)    # noise / oscillation
    )


def run_trial(apply_fn, targets: list, settle_s: float, capture_s: float) -> float:
    """Apply controller, run step tests across all targets, return mean score."""
    apply_fn()
    scores = []
    for tgt in targets:
        _set_target(0)
        time.sleep(settle_s)
        _set_target(tgt)
        samples = capture(capture_s)
        scores.append(_score_one(samples, tgt))

    _set_target(0)
    return sum(scores) / len(scores)


# ---------------------------------------------------------------------------
# Optuna objectives
# ---------------------------------------------------------------------------

def make_adrc_objective(targets, settle_s, capture_s):
    def objective(trial: optuna.Trial) -> float:
        wc = trial.suggest_float("wc", 1.0, 25.0)
        # b0 must satisfy b0 > wc * max_target / V_MAX to avoid constant saturation.
        # We search log-uniformly over a wide range; Optuna learns the constraint.
        b0 = trial.suggest_float("b0", 10.0, 800.0, log=True)

        # Warn: if b0 is very small relative to wc/target, voltage will saturate.
        # Let the penalty from a stalled/wild trial guide Optuna away from this.
        return run_trial(lambda: _apply_adrc(wc, b0), targets, settle_s, capture_s)
    return objective


def make_pid_objective(targets, settle_s, capture_s):
    def objective(trial: optuna.Trial) -> float:
        p = trial.suggest_float("p", 1.0, 60.0)
        i = trial.suggest_float("i", 1e-4, 10.0, log=True)
        d = trial.suggest_float("d", 0.0,  0.5)
        return run_trial(lambda: _apply_pid(p, i, d), targets, settle_s, capture_s)
    return objective


# ---------------------------------------------------------------------------
# Progress callback
# ---------------------------------------------------------------------------

def _make_callback(label: str, n_trials: int):
    def callback(study: optuna.Study, trial: optuna.Trial):
        t = trial.number + 1
        score = trial.value if trial.value is not None else float("nan")
        best  = study.best_value
        best_p = study.best_params
        param_str = "  ".join(f"{k}={v:.3f}" for k, v in best_p.items())
        bar_done  = int(t / n_trials * 20)
        bar       = "█" * bar_done + "░" * (20 - bar_done)
        print(f"\r[{label}] [{bar}] {t:>3}/{n_trials}  "
              f"trial={score:.4f}  best={best:.4f}  {param_str}",
              end="", flush=True)
    return callback


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--targets", nargs="+", type=float, default=[50.0, 100.0, 200.0],
                        metavar="RPM",
                        help="Target velocities used for optimization (default: 50 100 200)")
    parser.add_argument("--trials", type=int, default=30,
                        help="Optuna trials per controller (default: 30)")
    parser.add_argument("--capture", type=float, default=8.0,
                        help="Telemetry capture duration per trial in s (default: 8)")
    parser.add_argument("--settle", type=float, default=2.0,
                        help="Settle time at 0 RPM between steps in s (default: 2)")
    parser.add_argument("--port", type=str, default="Virtual Motor",
                        help="Motor port (default: 'Virtual Motor')")
    parser.add_argument("--no-benchmark", action="store_true",
                        help="Skip the final benchmark report after optimization")
    parser.add_argument("--db", type=str, default=str(DB_PATH),
                        help="SQLite storage path for Optuna studies (persists across runs)")
    parser.add_argument("--skip-adrc", action="store_true", help="Optimize PID only")
    parser.add_argument("--skip-pid",  action="store_true", help="Optimize ADRC only")
    args = parser.parse_args()

    # Verify server
    try:
        _get("/api/state")
    except Exception:
        print("ERROR: Server not reachable at http://127.0.0.1:8000", file=sys.stderr)
        print("  Start it with:  .venv/bin/python3 mock_sim.py", file=sys.stderr)
        sys.exit(1)

    LOG_DIR.mkdir(exist_ok=True)
    storage = f"sqlite:///{args.db}"

    est_seconds = (
        (0 if args.skip_adrc else args.trials) +
        (0 if args.skip_pid  else args.trials)
    ) * len(args.targets) * (args.settle + args.capture)
    print(f"Optuna ADRC+PID optimizer")
    print(f"  Targets:  {args.targets} RPM")
    print(f"  Trials:   {args.trials} per controller")
    print(f"  Capture:  {args.capture}s + {args.settle}s settle per step")
    print(f"  Storage:  {args.db}")
    print(f"  Est. time: ~{est_seconds/60:.0f} min\n")

    print("Initialising motor...")
    motor_init(args.port)

    # -------------------------------------------------------------------
    # ADRC study
    # -------------------------------------------------------------------
    best_adrc = None
    if not args.skip_adrc:
        print(f"\n{'='*60}")
        print(f" Optimising ADRC  ({args.trials} trials)")
        print(f"  Search space: wc ∈ [1, 25]  b0 ∈ [10, 800] (log)")
        print(f"{'='*60}")
        adrc_study = optuna.create_study(
            study_name="adrc_velocity",
            direction="minimize",
            storage=storage,
            load_if_exists=True,
            sampler=optuna.samplers.TPESampler(seed=42),
        )
        adrc_study.optimize(
            make_adrc_objective(args.targets, args.settle, args.capture),
            n_trials=args.trials,
            callbacks=[_make_callback("ADRC", args.trials)],
            show_progress_bar=False,
        )
        print()  # newline after \r progress
        best_adrc = adrc_study.best_params
        best_adrc_score = adrc_study.best_value
        print(f"\n  Best ADRC:  wc={best_adrc['wc']:.3f}  b0={best_adrc['b0']:.2f}")
        print(f"  Score: {best_adrc_score:.5f}")

    # -------------------------------------------------------------------
    # PID study
    # -------------------------------------------------------------------
    best_pid = None
    if not args.skip_pid:
        print(f"\n{'='*60}")
        print(f" Optimising PID  ({args.trials} trials)")
        print(f"  Search space: P ∈ [1, 60]  I ∈ [1e-4, 10] (log)  D ∈ [0, 0.5]")
        print(f"{'='*60}")
        pid_study = optuna.create_study(
            study_name="pid_velocity",
            direction="minimize",
            storage=storage,
            load_if_exists=True,
            sampler=optuna.samplers.TPESampler(seed=42),
        )
        pid_study.optimize(
            make_pid_objective(args.targets, args.settle, args.capture),
            n_trials=args.trials,
            callbacks=[_make_callback("PID ", args.trials)],
            show_progress_bar=False,
        )
        print()
        best_pid = pid_study.best_params
        best_pid_score = pid_study.best_value
        print(f"\n  Best PID:  P={best_pid['p']:.3f}  I={best_pid['i']:.4f}  D={best_pid['d']:.4f}")
        print(f"  Score: {best_pid_score:.5f}")

    # -------------------------------------------------------------------
    # Restore clean state
    # -------------------------------------------------------------------
    _set_target(0)
    if best_adrc:
        _apply_adrc(best_adrc["wc"], best_adrc["b0"])

    # -------------------------------------------------------------------
    # Summary
    # -------------------------------------------------------------------
    print(f"\n{'='*60}")
    print(" Optimization complete")
    print(f"{'='*60}")
    if best_adrc:
        print(f"  ADRC: wc={best_adrc['wc']:.3f}  b0={best_adrc['b0']:.2f}  score={adrc_study.best_value:.5f}")
    if best_pid:
        print(f"  PID:  P={best_pid['p']:.3f}  I={best_pid['i']:.5f}  D={best_pid['d']:.4f}  score={pid_study.best_value:.5f}")
    print(f"\n  Study history persisted at: {args.db}")
    if best_adrc and best_pid:
        print(f"\n  Re-run benchmark manually:")
        print(f"  .venv/bin/python3 scripts/benchmark_adrc_vs_pid.py \\")
        print(f"    --adrc-wc {best_adrc['wc']:.3f} --adrc-b0 {best_adrc['b0']:.2f} \\")
        print(f"    --pid-p {best_pid['p']:.3f} --pid-i {best_pid['i']:.5f} --pid-d {best_pid['d']:.4f}")

    # -------------------------------------------------------------------
    # Auto-run benchmark
    # -------------------------------------------------------------------
    if not args.no_benchmark and best_adrc and best_pid:
        print(f"\nRunning final benchmark with optimised parameters…")
        python = sys.executable
        for venv in [".venv/bin/python3", "venv/bin/python3"]:
            if pathlib.Path(venv).exists():
                python = venv
                break
        cmd = [
            python, "scripts/benchmark_adrc_vs_pid.py",
            "--targets", *[str(t) for t in args.targets],
            "--duration", "12",
            "--adrc-wc", f"{best_adrc['wc']:.4f}",
            "--adrc-b0", f"{best_adrc['b0']:.4f}",
            "--pid-p",   f"{best_pid['p']:.4f}",
            "--pid-i",   f"{best_pid['i']:.6f}",
            "--pid-d",   f"{best_pid['d']:.4f}",
            "--note",    f"Optuna optimised ({args.trials} trials per controller)",
            "--port",    args.port,
        ]
        print(" ".join(cmd))
        subprocess.run(cmd)


if __name__ == "__main__":
    main()
