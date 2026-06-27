#!/usr/bin/env python3
"""
Headless ADRC tuner — starts server + agent with no browser required.

Usage:
    python3 run_headless.py                          # mock sim, no initial target
    python3 run_headless.py --target 50              # mock sim, start tuning at 50 RPM
    python3 run_headless.py --real /dev/ttyUSB0      # real hardware on given port
    python3 run_headless.py --real /dev/ttyUSB0 --target 100
"""
import argparse
import os
import pathlib
import signal
import subprocess
import sys
import threading
import time

import requests

BASE_URL = "http://127.0.0.1:8000"
LOG_DIR = pathlib.Path("logs")
TUNER_LOG = LOG_DIR / "tuner_subprocess.log"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _python_cmd() -> str:
    for venv in [".venv/bin/python3", "venv/bin/python3"]:
        if pathlib.Path(venv).exists():
            return venv
    return sys.executable


def _kill_port_8000():
    try:
        pids = subprocess.check_output(["lsof", "-ti:8000"]).decode().split()
        for pid in pids:
            os.kill(int(pid), signal.SIGKILL)
        time.sleep(0.3)
    except Exception:
        pass


def _wait_for_server(timeout: float = 20.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            if requests.get(f"{BASE_URL}/api/state", timeout=1).status_code == 200:
                return True
        except Exception:
            pass
        time.sleep(0.4)
    return False


def _tail_log(path: pathlib.Path, stop: threading.Event):
    """Stream a growing log file to stdout until stop is set."""
    path.parent.mkdir(exist_ok=True)
    path.touch()
    with open(path) as fh:
        fh.seek(0, 2)            # start from current end, not history
        while not stop.is_set():
            line = fh.readline()
            if line:
                print(f"[tuner] {line}", end="", flush=True)
            else:
                time.sleep(0.08)


def _capture_best_response():
    """On shutdown, find the best iteration in today's diagnostic log and run
    the benchmark script against the current server to capture a snapshot."""
    import json, glob, time as _time
    today = _time.strftime("%Y%m%d")
    log_dir = pathlib.Path("logs")
    log_path = log_dir / f"agent_diagnostic_{today}.jsonl"
    if not log_path.exists():
        print("[headless] No diagnostic log for today — skipping capture.")
        return

    best = None
    with open(log_path) as f:
        for line in f:
            try:
                d = json.loads(line)
                stats = d.get("stats", {})
                state = d.get("state", {})
                target = state.get("target", 0)
                error  = stats.get("tracking_error", 999)
                stdev  = stats.get("velocity_stdev", 999)
                if abs(target) < 1.0 and (error + stdev) < 0.05:
                    continue
                score = (error + stdev) / abs(target) if abs(target) > 0 else 999
                if best is None or score < best["score"]:
                    best = {"score": score, "wc": state.get("wc"), "b0": state.get("b0")}
            except Exception:
                continue

    if best:
        print(f"[headless] Best params from today: wc={best['wc']}, b0={best['b0']} "
              f"(score={best['score']:.4f})")
        print("[headless] Running benchmark capture…")
        import subprocess as _sp
        python = _python_cmd()
        _sp.run([python, "scripts/benchmark_adrc_vs_pid.py", "--targets", "50", "100"],
                timeout=120)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Headless ADRC tuner (no browser needed)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--real", metavar="PORT",
        help="Use real hardware on this serial port (e.g. /dev/ttyUSB0 or COM3). "
             "Omit to use the built-in mock motor simulation.",
    )
    parser.add_argument(
        "--target", type=float, default=None, metavar="RPM",
        help="Inject an initial target velocity (RPM) as an agent instruction.",
    )
    parser.add_argument(
        "--capture", action="store_true",
        help="On Ctrl+C, run benchmark_adrc_vs_pid.py to capture a response snapshot report.",
    )
    args = parser.parse_args()

    use_real = args.real is not None
    python = _python_cmd()

    print(f"[headless] Mode: {'real hardware (' + args.real + ')' if use_real else 'mock simulation'}")
    if args.target is not None:
        print(f"[headless] Initial target: {args.target:.0f} RPM")

    # ------------------------------------------------------------------
    # 1. Free port 8000 and start server
    # ------------------------------------------------------------------
    _kill_port_8000()

    if use_real:
        server_cmd = [python, "-m", "uvicorn", "main:app", "--host", "127.0.0.1", "--port", "8000"]
    else:
        server_cmd = [python, "mock_sim.py"]

    print(f"[headless] Starting server: {' '.join(server_cmd)}")
    server_proc = subprocess.Popen(
        server_cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    # ------------------------------------------------------------------
    # 2. Wait for server readiness
    # ------------------------------------------------------------------
    print("[headless] Waiting for server to be ready...")
    if not _wait_for_server(timeout=20):
        print("[headless] ERROR: Server did not become ready in time.", file=sys.stderr)
        server_proc.kill()
        sys.exit(1)
    print("[headless] Server ready.")

    # ------------------------------------------------------------------
    # 3. Connect to hardware (real only — mock auto-connects via tuner)
    # ------------------------------------------------------------------
    if use_real:
        print(f"[headless] Connecting to real motor on {args.real}...")
        try:
            r = requests.post(
                f"{BASE_URL}/connect",
                json={"port": args.real, "device_id": 48},
                timeout=5,
            )
            resp = r.json()
            if resp.get("status") != "connected":
                print(f"[headless] WARNING: Connect response: {resp}", file=sys.stderr)
            else:
                print(f"[headless] Connected: {resp['message']}")
        except Exception as e:
            print(f"[headless] ERROR: Could not connect to hardware: {e}", file=sys.stderr)
            server_proc.kill()
            sys.exit(1)

    # ------------------------------------------------------------------
    # 4. Start the agent tuner (via API so the server manages its lifecycle)
    # ------------------------------------------------------------------
    print("[headless] Starting agent tuner...")
    try:
        r = requests.post(f"{BASE_URL}/api/start_tuner", timeout=5)
        resp = r.json()
        print(f"[headless] Tuner started (PID {resp.get('pid', '?')}), log → {resp.get('log', TUNER_LOG)}")
    except Exception as e:
        print(f"[headless] ERROR: Could not start tuner: {e}", file=sys.stderr)
        server_proc.kill()
        sys.exit(1)

    # ------------------------------------------------------------------
    # 5. Inject initial target as an agent prompt (if requested)
    #    The tuner clears stale prompts on startup, so we wait briefly
    #    for the process to actually start before injecting.
    # ------------------------------------------------------------------
    if args.target is not None:
        time.sleep(1.5)
        instruction = f"Set target velocity to {args.target:.0f} RPM and begin tuning."
        try:
            requests.post(
                f"{BASE_URL}/api/agent_prompt",
                json={"prompt": instruction},
                timeout=2,
            )
            print(f"[headless] Injected agent prompt: \"{instruction}\"")
        except Exception as e:
            print(f"[headless] WARNING: Could not inject target prompt: {e}", file=sys.stderr)

    # ------------------------------------------------------------------
    # 6. Stream tuner logs to stdout
    # ------------------------------------------------------------------
    stop_event = threading.Event()
    log_thread = threading.Thread(
        target=_tail_log, args=(TUNER_LOG, stop_event), daemon=True
    )
    log_thread.start()
    print(f"[headless] Streaming logs from {TUNER_LOG}. Press Ctrl+C to stop.\n")

    # ------------------------------------------------------------------
    # 7. Shutdown handler
    # ------------------------------------------------------------------
    def _shutdown(sig, frame):
        print("\n[headless] Shutting down...")
        stop_event.set()
        try:
            requests.post(f"{BASE_URL}/api/stop_tuner", timeout=2)
        except Exception:
            pass
        if args.capture:
            _capture_best_response()
        server_proc.terminate()
        try:
            server_proc.wait(timeout=4)
        except subprocess.TimeoutExpired:
            server_proc.kill()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    # Keep alive; also restart tuner if server dies unexpectedly.
    while True:
        if server_proc.poll() is not None:
            print("[headless] ERROR: Server exited unexpectedly.", file=sys.stderr)
            stop_event.set()
            sys.exit(1)
        time.sleep(1)


if __name__ == "__main__":
    main()
