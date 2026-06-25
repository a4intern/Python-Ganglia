import asyncio
import websockets
import json
import statistics
import time
import os
import pathlib
import requests
from collections import deque
from dotenv import load_dotenv
from llm_backends import TuningResult, create_backend

LOG_DIR = pathlib.Path(__file__).parent.parent / "logs"
LOG_DIR.mkdir(exist_ok=True)

WS_URL = "ws://127.0.0.1:8000/ws/telemetry"
BASE_URL = "http://127.0.0.1:8000"

load_dotenv()
try:
    backend = create_backend()
    print(f"LLM backend: {backend.__class__.__name__}")
except Exception as e:
    print(f"Error initialising LLM backend: {e}")
    exit(1)

SYSTEM_PROMPT = """You are an expert control systems engineer autonomously tuning an ADRC (Active Disturbance Rejection Controller) for a brushless motor.

## ADRC Parameters
- **wc** (Observer Bandwidth): bounds [1.0, 20.0]. Higher = faster disturbance rejection but amplifies sensor noise and causes oscillation. Start around 5.0.
- **b0** (System Gain): bounds [1.0, 150.0]. Represents the motor's expected acceleration sensitivity. Too high → controller under-drives, motor stalls or barely moves. Too low → controller over-drives, oscillation or overshoot.
- **ramp_time**: bounds [0.0, 5.0]. Use 0.0 for step response testing. Only increase if you want smooth acceleration profiles.

## Tuning Protocol

Observe the current performance and adjust ONE parameter at a time:

| Symptom | Fix |
|---|---|
| Tracking error high (mean far from target) | Decrease b0 by 20–40% |
| Stall / barely moves | Decrease b0 by 30–50% |
| Oscillation high (stdev > 0.5 RPM) | Decrease wc by 20–30% |
| Error AND oscillation both high | Fix oscillation first (reduce wc), then tracking (reduce b0) |
| Error low but response is sluggish | Increase wc by 20% |
| After b0 decrease, motor now overshoots | Decrease wc slightly |

Max adjustment: ±30% per step. Never change both wc AND b0 in the same step.

## Target Velocity
Do NOT change the target_velocity unless a USER INSTRUCTION explicitly asks you to switch targets or perform a step test. Keep the current target and tune from there.

## Key Insight on b0 at Low RPM
At low RPM, motors almost always need very low b0 (target range: 1–10). If b0 > 20 and oscillation persists, it is almost certainly too high — halve it immediately. b0=50 at 5 RPM will always oscillate.

## User Instructions
If a USER INSTRUCTION is provided, you MUST honor it exactly, including any override of target_velocity or parameters. After fulfilling it, resume the standard protocol.
"""

async def measure_telemetry(duration=5.0):
    velocities = []
    currents = []
    horizon = []  # raw time-series: [{t, velocity, z1, z2, z3}, ...]
    current_state = {}
    t0 = time.time()

    try:
        async with websockets.connect(WS_URL) as ws:
            start_time = time.time()
            while time.time() - start_time < duration:
                try:
                    data = await asyncio.wait_for(ws.recv(), timeout=0.5)
                    pts = json.loads(data)
                    for pt in pts:
                        row = {"t": round(time.time() - t0, 3)}
                        if "velocity" in pt:
                            velocities.append(pt["velocity"])
                            row["velocity"] = pt["velocity"]
                        if "current" in pt:
                            currents.append(pt["current"])
                        row["z1"] = pt.get("z1", 0)
                        row["z2"] = pt.get("z2", 0)
                        row["z3"] = pt.get("z3", 0)
                        horizon.append(row)
                except asyncio.TimeoutError:
                    continue
    except Exception as e:
        print(f"WS Error: {e}")
        return None, None, None

    try:
        r = requests.get(f"{BASE_URL}/api/state", timeout=1)
        if r.status_code == 200:
            api_state = r.json()
            current_state["target"] = api_state.get("target_velocity", 0)
            current_state["wc"] = api_state.get("adrc_wc", 0)
            current_state["b0"] = api_state.get("adrc_b0", 0)
            current_state["ramp_time"] = 0.0
    except Exception:
        pass

    if not velocities:
        return None, None, None

    target = current_state.get("target", 0)
    vel_mean = statistics.mean(velocities)
    vel_stdev = statistics.stdev(velocities) if len(velocities) > 1 else 0.0

    stats = {
        "velocity_mean": vel_mean,
        "velocity_stdev": vel_stdev,
        "tracking_error": abs(vel_mean - target),
        "current_mean": statistics.mean(currents) if currents else 0.0,
        "current_stdev": statistics.stdev(currents) if len(currents) > 1 else 0.0,
        "sample_count": len(velocities),
    }
    return stats, current_state, horizon


def write_diagnostic(iteration: int, prompt: str, ai_response: dict, stats: dict, state: dict, horizon: list):
    log_file = LOG_DIR / f"agent_diagnostic_{time.strftime('%Y%m%d')}.jsonl"
    entry = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "iteration": iteration,
        "state": state,
        "stats": {k: round(v, 4) if isinstance(v, float) else v for k, v in stats.items()},
        "prompt_sent": prompt,
        "ai_response": ai_response,
        "horizon": horizon,
    }
    with open(log_file, "a") as f:
        f.write(json.dumps(entry) + "\n")
    print(f"[diag] wrote iteration {iteration} → {log_file.name}")

def log_to_ui(msg: str):
    print(msg)
    try:
        requests.post(f"{BASE_URL}/post_agent_log", json={"message": msg}, timeout=1)
    except Exception:
        pass

def set_adrc(wc, b0, ramp=0.0, target=None):
    wc = max(1.0, min(20.0, float(wc)))
    b0 = max(1.0, min(150.0, float(b0)))
    ramp = max(0.0, min(5.0, float(ramp)))
    log_to_ui(f"Applying: wc={wc:.2f}, b0={b0:.2f}, ramp={ramp:.2f}")
    requests.post(f"{BASE_URL}/set_adrc", json={
        "mode": "velocity", "wc": wc, "b0": b0, "ramp_time": ramp
    })
    if target is not None:
        target = float(target)
        requests.post(f"{BASE_URL}/set_target", json={
            "mode": "velocity", "value": int(target), "min_limit": -4000, "max_limit": 4000
        })
        log_to_ui(f"Setting target to: {target:.0f} RPM")

async def agent_loop():
    log_to_ui("Starting GenAI Agentic Tuner Loop...")
    history: deque = deque(maxlen=6)
    iteration = 0

    await asyncio.sleep(2.0)

    while True:
        iteration += 1
        log_to_ui("\n--- Settling (3s) then observing (5s) ---")
        await asyncio.sleep(3.0)
        stats, state, horizon = await measure_telemetry(5.0)

        if stats is None:
            log_to_ui("Failed to read telemetry. Retrying...")
            await asyncio.sleep(1)
            continue

        log_to_ui(f"State: target={state['target']} RPM, wc={state['wc']:.2f}, b0={state['b0']:.2f}")
        log_to_ui(f"Stats: mean={stats['velocity_mean']:.2f} RPM, stdev={stats['velocity_stdev']:.2f} RPM, error={stats['tracking_error']:.2f} RPM")

        history.append({
            "target": state["target"],
            "wc": state["wc"],
            "b0": state["b0"],
            "velocity_mean": stats["velocity_mean"],
            "velocity_stdev": stats["velocity_stdev"],
            "tracking_error": stats["tracking_error"],
        })

        history_lines = ""
        for i, h in enumerate(list(history)[:-1]):
            history_lines += (
                f"  [{i+1}] target={h['target']:+.0f} RPM → "
                f"mean={h['velocity_mean']:+.2f}, stdev={h['velocity_stdev']:.2f}, error={h['tracking_error']:.2f} "
                f"| wc={h['wc']:.2f}, b0={h['b0']:.2f}\n"
            )

        prompt = f"""## Current ADRC State
target_velocity: {state['target']:+.0f} RPM | wc: {state['wc']:.2f} | b0: {state['b0']:.2f} | ramp_time: {state['ramp_time']:.2f}

## Current Observation (steady-state, last 5s after 3s settle)
Velocity Mean:    {stats['velocity_mean']:+.2f} RPM
Velocity Stdev:   {stats['velocity_stdev']:.2f} RPM
Tracking Error:   {stats['tracking_error']:.2f} RPM  (|mean − target|)
Current Mean:     {stats['current_mean']:.2f} mA
Current Stdev:    {stats['current_stdev']:.2f} mA
Sample count:     {stats['sample_count']}

## History (oldest → most recent, not including current)
{history_lines.rstrip() if history_lines else "  (no history — first observation)"}

Observe the current performance and tune parameters if needed. Do NOT change the target_velocity unless a USER INSTRUCTION explicitly requests it."""

        user_prompt = ""
        try:
            r = requests.get(f"{BASE_URL}/api/agent_prompt", timeout=1)
            if r.status_code == 200:
                user_prompt = r.json().get("prompt", "")
        except Exception:
            pass

        if user_prompt:
            prompt += f"\n\n*** USER INSTRUCTION (OVERRIDE — honor this exactly) ***\n{user_prompt}\n*****************************************************\n"

        log_to_ui(f"Querying {backend.__class__.__name__}...")
        try:
            result = backend.complete(SYSTEM_PROMPT, prompt)
            log_to_ui(f"Agent [{result.phase}]: {result.reasoning}")

            ai_response = {
                "phase": result.phase,
                "reasoning": result.reasoning,
                "wc": result.wc,
                "b0": result.b0,
                "ramp_time": result.ramp_time,
                "target_velocity": result.target_velocity,
            }
            write_diagnostic(iteration, prompt, ai_response, stats, state, horizon)

            set_adrc(result.wc, result.b0, result.ramp_time, result.target_velocity)

            if user_prompt:
                try:
                    requests.post(f"{BASE_URL}/api/agent_prompt_clear", timeout=1)
                except Exception:
                    pass

        except Exception as e:
            log_to_ui(f"GenAI Error: {e}")
            await asyncio.sleep(2)

if __name__ == "__main__":
    asyncio.run(agent_loop())
