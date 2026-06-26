# -*- coding: utf-8 -*-
import asyncio
import websockets
import json
import statistics
import time
import os
import pathlib
import requests
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
- **wc** (Observer Bandwidth): bounds [1.0, 50.0]. Higher = faster disturbance rejection but amplifies sensor noise and causes oscillation.
- **b0** (System Gain): bounds [1.0, 150.0]. Represents the motor's expected acceleration sensitivity. Too high → controller under-drives (sluggish rise time, large steady-state tracking error, or stall). Too low → controller over-drives (causes high overshoot and oscillation).
- **ramp_time**: bounds [0.0, 5.0]. Use 0.0 for step response testing. Only increase if you want smooth acceleration profiles.

## Tuning Protocol

Observe the current performance (both transient step response metrics and steady-state error/oscillation) and adjust ONE parameter at a time:

| Symptom | Fix |
|---|---|
| High overshoot (peak velocity exceeds target) | Increase b0 (too low b0 causes overdrive/overshoot) or decrease wc slightly |
| Sluggish rise time / settling time is slow | Decrease b0 slightly or increase wc by 20% |
| Tracking error high (steady-state mean far from target) | Decrease b0 |
| Stall / barely moves | Decrease b0 significantly |
| Oscillation high (steady-state stdev > 0.5 RPM) | Decrease wc |
| Error AND oscillation both high | Fix oscillation first (reduce wc), then tracking (reduce b0) |
| Error low but response is sluggish | Increase wc by 20% |

## Analyzing Transient Response Trends
You must track and compare the step response transient metrics across iterations:
- **Rise Time (Tr)**: Time to reach 90% of the target speed. Look at the historical trajectory to verify if Tr is improving or worsening.
- **Settling Time (Ts)**: Time to settle and remain within target tolerance.
- **Overshoot (Os)**: Peak speed exceeding target. If Os is high or increasing, you MUST increase b0 or decrease wc slightly.
Compare these transient metrics to previous iterations to guide your adjustments, rather than focusing only on steady-state mean/error/stdev.

## Step Size - Use Aggressive Steps When the Trend Is Clear

**Do not inch toward the optimum.** If the same symptom has persisted for 2+ iterations, make a larger move:

- First observation of a symptom: adjust 30-40%
- Same symptom persists after adjustment: adjust 50-60%
- Same symptom persists 3+ steps in the same direction: halve (or double) the parameter - move boldly

If you have a **Prior Best** or **Best This Session** on record, use it as your anchor:
- If current performance is worse than the best seen, return directly to the best-seen parameters first, then make small refinements from there.
- Do not wander away from a known-good region - search around it.

When the direction of improvement is clear (e.g., lowering b0 consistently reduces error), continue in that direction with confidence. You do not need to hedge.

## Target Velocity
Do NOT change the target_velocity unless a USER INSTRUCTION explicitly asks you to switch targets or perform a step test.

## Key Insight on b0 at Low RPM
At low RPM (<= 10 RPM), b0 almost always needs to be very low (1-15). If b0 > 20 and oscillation persists, halve it immediately - do not reduce by only 20-30%. b0=50 at 5 RPM will always oscillate.

## User Instructions
If a USER INSTRUCTION is provided, honor it exactly, including any override of target_velocity or parameters. After fulfilling it, resume the standard protocol.
"""


def load_prior_best() -> dict | None:
    """Scan all diagnostic logs and return the best wc/b0 ever found (lowest error+stdev)."""
    best = None
    for log_file in sorted(LOG_DIR.glob("agent_diagnostic_*.jsonl")):
        try:
            with open(log_file) as f:
                for line in f:
                    try:
                        entry = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    s = entry.get("stats", {})
                    st = entry.get("state", {})
                    error = s.get("tracking_error", 999)
                    stdev = s.get("velocity_stdev", 999)
                    score = error + stdev
                    if best is None or score < best["score"]:
                        best = {
                            "score": score,
                            "wc": st.get("wc", 0),
                            "b0": st.get("b0", 0),
                            "tracking_error": error,
                            "velocity_stdev": stdev,
                            "target": st.get("target", 0),
                            "timestamp": entry.get("timestamp", ""),
                            "rise_time": s.get("rise_time"),
                            "settling_time": s.get("settling_time"),
                            "overshoot": s.get("overshoot"),
                            "pct_overshoot": s.get("pct_overshoot"),
                        }
        except Exception:
            continue
    return best


async def measure_telemetry(duration=5.0):
    velocities = []
    currents = []
    horizon = []
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
            current_state["target"] = api_state.get("target_velocity", 0.0)
            current_state["wc"] = api_state.get("adrc_wc", 0.0)
            current_state["b0"] = api_state.get("adrc_b0", 0.0)
            current_state["ramp_time"] = 0.0
        else:
            current_state["target"] = 0.0
            current_state["wc"] = 0.0
            current_state["b0"] = 0.0
            current_state["ramp_time"] = 0.0
    except Exception as e:
        print(f"Error querying motor state: {e}")
        current_state["target"] = 0.0
        current_state["wc"] = 0.0
        current_state["b0"] = 0.0
        current_state["ramp_time"] = 0.0

    if not velocities:
        return None, None, None

    # Focus on the last 200 samples (steady-state, approx last 2.0s) for steady-state stats
    steady_state_vels = velocities[-200:] if len(velocities) >= 200 else velocities
    steady_state_currents = currents[-200:] if len(currents) >= 200 else currents

    target = current_state.get("target", 0.0)
    vel_mean = statistics.mean(steady_state_vels)
    vel_stdev = statistics.stdev(steady_state_vels) if len(steady_state_vels) > 1 else 0.0

    # Calculate initial velocity from the first 10 samples
    v_init = statistics.mean(velocities[:10]) if len(velocities) >= 10 else (velocities[0] if velocities else 0.0)
    step_size = target - v_init

    # Transient analysis
    rise_time = None
    settling_time = None
    overshoot = 0.0
    pct_overshoot = 0.0

    if len(horizon) > 10:
        # 1. Rise Time (10% to 90% or first time reaching 90% of target from v_init)
        if abs(step_size) > 5.0:
            target_90 = v_init + 0.9 * step_size
            for pt in horizon:
                val = pt.get("velocity", 0.0)
                if (step_size > 0 and val >= target_90) or (step_size < 0 and val <= target_90):
                    rise_time = pt["t"]
                    break

        # 2. Overshoot
        if abs(step_size) > 5.0:
            vel_vals = [pt["velocity"] for pt in horizon if "velocity" in pt]
            if step_size > 0:
                peak_val = max(vel_vals)
                overshoot = max(0.0, peak_val - target)
            else:
                peak_val = min(vel_vals)
                overshoot = max(0.0, target - peak_val)
            pct_overshoot = (overshoot / abs(step_size)) * 100.0

        # 3. Settling Time (time after which velocity stays within target ± band)
        # Band size: max(5.0 RPM, 5% of step size)
        band = max(5.0, 0.05 * abs(step_size))
        last_outside_idx = None
        for idx in range(len(horizon) - 1, -1, -1):
            val = horizon[idx].get("velocity", 0.0)
            if abs(val - target) > band:
                last_outside_idx = idx
                break
        
        if last_outside_idx is not None:
            if last_outside_idx == len(horizon) - 1:
                settling_time = duration
            else:
                settling_time = horizon[last_outside_idx + 1]["t"]
        else:
            settling_time = 0.0

    stats = {
        "velocity_mean": vel_mean,
        "velocity_stdev": vel_stdev,
        "tracking_error": abs(vel_mean - target),
        "current_mean": statistics.mean(steady_state_currents) if steady_state_currents else 0.0,
        "current_stdev": statistics.stdev(steady_state_currents) if len(steady_state_currents) > 1 else 0.0,
        "sample_count": len(steady_state_vels),
        "rise_time": rise_time,
        "settling_time": settling_time,
        "overshoot": overshoot,
        "pct_overshoot": pct_overshoot,
        "step_size": step_size,
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
    wc = max(1.0, min(50.0, float(wc)))
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


def _build_trajectory_summary(session_history: list) -> str:
    """Condense full session history into a readable trajectory table."""
    if not session_history:
        return "  (no history — first observation)"
    lines = []
    for h in session_history:
        trend = ""
        if h.get("delta_score") is not None:
            trend = f" {'better' if h['delta_score'] < 0 else 'worse' if h['delta_score'] > 0 else 'same'}"
        
        step_sz = h.get("step_size", 0.0)
        if abs(step_sz) > 5.0:
            tr_val = h.get("rise_time")
            ts_val = h.get("settling_time")
            tr_str = f"{tr_val:.2f}s" if tr_val is not None else "N/A"
            ts_str = f"{ts_val:.2f}s" if ts_val is not None else "N/A"
            transient_str = f"Tr={tr_str}, Ts={ts_str}, Os={h['overshoot']:.1f} RPM ({h['pct_overshoot']:.1f}%)"
        else:
            transient_str = "no step"

        lines.append(
            f"  [{h['iter']:>2}] wc={h['wc']:.2f}, b0={h['b0']:.2f} | "
            f"target={h['target']:+.0f} RPM -> "
            f"mean={h['mean']:+.2f}, stdev={h['stdev']:.2f}, error={h['error']:.2f} | "
            f"{transient_str}{trend}"
        )
    return "\n".join(lines)


async def agent_loop():
    log_to_ui("Starting GenAI Agentic Tuner Loop...")

    # Ensure motor is connected, set to ADRC mode (-2), blended to 100% ADRC, and started
    try:
        log_to_ui("Initializing motor connection and control settings...")
        # Check connection or connect to Virtual Motor as fallback
        try:
            r = requests.post(f"{BASE_URL}/set_op_mode", json={"mode": -2}, timeout=1.0).json()
            if "error" in r and r["error"] == "Not connected":
                requests.post(f"{BASE_URL}/connect", json={"port": "Virtual Motor", "device_id": 48}, timeout=1.0)
                requests.post(f"{BASE_URL}/set_op_mode", json={"mode": -2}, timeout=1.0)
        except Exception:
            requests.post(f"{BASE_URL}/connect", json={"port": "Virtual Motor", "device_id": 48}, timeout=1.0)
            requests.post(f"{BASE_URL}/set_op_mode", json={"mode": -2}, timeout=1.0)

        # Set blend to 100% ADRC
        requests.post(f"{BASE_URL}/set_pid", json={
            "mode": "velocity", "p": 0, "i": 0, "d": 0, "gain_output": 1.0, "limit_i": 30000, "blend": 100
        }, timeout=1.0)
        
        # Reset target velocity to 0 RPM explicitly on startup to prevent sudden motor spin-up on stale speeds
        requests.post(f"{BASE_URL}/set_target", json={
            "mode": "velocity", "value": 0, "min_limit": -4000, "max_limit": 4000
        }, timeout=1.0)
        
        # Start motor
        requests.post(f"{BASE_URL}/start", timeout=1.0)
        log_to_ui("Motor connection, ADRC mode (-2), blend (100%), and drive successfully initialized.")
    except Exception as e:
        log_to_ui(f"Warning: Motor initialization failed: {e}")

    prior_best = load_prior_best()
    if prior_best:
        log_to_ui(
            f"Prior best (from logs): wc={prior_best['wc']:.2f}, b0={prior_best['b0']:.2f} "
            f"→ error={prior_best['tracking_error']:.2f}, stdev={prior_best['velocity_stdev']:.2f} "
            f"@ target={prior_best['target']:.0f} RPM  [{prior_best['timestamp']}]"
        )
    else:
        log_to_ui("No prior diagnostic logs found — starting fresh.")

    session_history: list[dict] = []  # all observations this session
    best_seen: dict | None = None      # best (wc, b0) found this session
    iteration = 0
    prev_score: float | None = None

    await asyncio.sleep(2.0)

    while True:
        iteration += 1
        log_to_ui(f"\n--- Observing 8.0s Step Response (Iteration {iteration}) ---")
        stats, state, horizon = await measure_telemetry(8.0)

        if stats is None:
            log_to_ui("Failed to read telemetry. Retrying...")
            await asyncio.sleep(1)
            continue

        log_to_ui(f"State: target={state['target']} RPM, wc={state['wc']:.2f}, b0={state['b0']:.2f}")
        log_to_ui(f"Stats (Steady-state): mean={stats['velocity_mean']:.2f} RPM, stdev={stats['velocity_stdev']:.2f} RPM, error={stats['tracking_error']:.2f} RPM")
        
        if abs(stats["step_size"]) > 5.0:
            rt_str = f"{stats['rise_time']:.2f}s" if stats['rise_time'] is not None else "N/A (under-drives)"
            st_str = f"{stats['settling_time']:.2f}s" if stats['settling_time'] is not None else "N/A (unsettled)"
            log_to_ui(f"Stats (Transient): rise_time={rt_str}, settling_time={st_str}, overshoot={stats['overshoot']:.2f} RPM ({stats['pct_overshoot']:.1f}%)")
        else:
            log_to_ui(f"Stats (Transient): No speed step change detected (constant target).")

        score = stats["tracking_error"] + stats["velocity_stdev"]
        delta_score = (score - prev_score) if prev_score is not None else None
        prev_score = score

        if best_seen is None or score < best_seen["score"]:
            best_seen = {
                "score": score,
                "wc": state["wc"],
                "b0": state["b0"],
                "tracking_error": stats["tracking_error"],
                "velocity_stdev": stats["velocity_stdev"],
                "iteration": iteration,
                "rise_time": stats["rise_time"],
                "settling_time": stats["settling_time"],
                "overshoot": stats["overshoot"],
                "pct_overshoot": stats["pct_overshoot"],
            }

        session_history.append({
            "iter": iteration,
            "wc": state["wc"],
            "b0": state["b0"],
            "target": state["target"],
            "mean": stats["velocity_mean"],
            "stdev": stats["velocity_stdev"],
            "error": stats["tracking_error"],
            "rise_time": stats["rise_time"],
            "settling_time": stats["settling_time"],
            "overshoot": stats["overshoot"],
            "pct_overshoot": stats["pct_overshoot"],
            "step_size": stats["step_size"],
            "score": score,
            "delta_score": delta_score,
        })

        trajectory = _build_trajectory_summary(session_history[:-1])  # exclude current

        best_session_line = (
            f"wc={best_seen['wc']:.2f}, b0={best_seen['b0']:.2f} -> "
            f"error={best_seen['tracking_error']:.2f}, stdev={best_seen['velocity_stdev']:.2f}"
            + (f", Tr={best_seen['rise_time']:.2f}s, Ts={best_seen['settling_time']:.2f}s, Os={best_seen['overshoot']:.1f} RPM ({best_seen['pct_overshoot']:.1f}%)" if best_seen.get('rise_time') is not None else "")
            + f" (iteration {best_seen['iteration']})"
            if best_seen else "none yet"
        )

        prior_best_line = (
            f"wc={prior_best['wc']:.2f}, b0={prior_best['b0']:.2f} -> "
            f"error={prior_best['tracking_error']:.2f}, stdev={prior_best['velocity_stdev']:.2f}"
            + (f", Tr={prior_best['rise_time']:.2f}s, Ts={prior_best['settling_time']:.2f}s, Os={prior_best['overshoot']:.1f} RPM ({prior_best['pct_overshoot']:.1f}%)" if prior_best and prior_best.get('rise_time') is not None else "")
            + f" @ target={prior_best['target']:.0f} RPM  [{prior_best['timestamp']}]"
            if prior_best else "none"
        )

        consecutive_same_direction = 0
        if len(session_history) >= 3:
            recent = session_history[-3:]
            deltas = [h["delta_score"] for h in recent if h["delta_score"] is not None]
            if all(d > 0 for d in deltas):
                consecutive_same_direction = len(deltas)  # getting worse

        transient_info = ""
        if abs(stats["step_size"]) > 5.0:
            rt_str = f"{stats['rise_time']:.2f} s" if stats['rise_time'] is not None else "Did not reach 90% of target (too sluggish/under-drives)"
            st_str = f"{stats['settling_time']:.2f} s" if stats['settling_time'] is not None else "Did not settle within 5% band (sluggish or oscillating)"
            transient_info = f"""Step Size:        {stats['step_size']:.2f} RPM
Rise Time:        {rt_str}
Settling Time:    {st_str}
Max Overshoot:    {stats['overshoot']:.2f} RPM ({stats['pct_overshoot']:.1f}%)"""
        else:
            transient_info = "Step Response:    No speed step change detected in this iteration."

        prompt = f"""## Prior Best (from previous sessions)
{prior_best_line}

## Best This Session
{best_session_line}

## Full Tuning Trajectory (oldest → most recent, not including current)
{trajectory}

## Current ADRC State
target_velocity: {state['target']:+.0f} RPM | wc: {state['wc']:.2f} | b0: {state['b0']:.2f} | ramp_time: {state['ramp_time']:.2f}

## Current Observation (last 8s trajectory)
{transient_info}

Velocity Mean (steady-state):    {stats['velocity_mean']:+.2f} RPM
Velocity Stdev (steady-state):   {stats['velocity_stdev']:.2f} RPM
Tracking Error (steady-state):   {stats['tracking_error']:.2f} RPM  (|mean − target|)
Current Mean (steady-state):     {stats['current_mean']:.2f} mA
Current Stdev (steady-state):    {stats['current_stdev']:.2f} mA
Steady-state sample count:       {stats['sample_count']}
Performance score (error+stdev): {score:.2f}{f'  [{consecutive_same_direction} consecutive worsening steps — use a larger adjustment]' if consecutive_same_direction >= 2 else ''}

Observe the current performance (both transient rise/settling time and steady-state error/oscillation), consider the full trajectory and prior knowledge above, and tune parameters accordingly. Do NOT change the target_velocity unless a USER INSTRUCTION explicitly requests it."""

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
