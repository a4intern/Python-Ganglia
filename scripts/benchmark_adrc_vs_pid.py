#!/usr/bin/env python3
"""
Benchmark: Best ADRC vs Tuned PID step-response comparison.

Requires the mock server (or real hardware server) to already be running.
Runs step tests at multiple target velocities, captures telemetry, computes
transient / steady-state metrics, and writes a self-contained HTML report
with embedded plots to  sysid_report/benchmark_<timestamp>.html

Usage:
    .venv/bin/python3 scripts/benchmark_adrc_vs_pid.py
    .venv/bin/python3 scripts/benchmark_adrc_vs_pid.py --targets 50 100 200
    .venv/bin/python3 scripts/benchmark_adrc_vs_pid.py --duration 15
"""
import argparse
import asyncio
import base64
import io
import json
import math
import pathlib
import statistics
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import requests
import websockets

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
BASE_URL   = "ws://127.0.0.1:8000"
HTTP_URL   = "http://127.0.0.1:8000"
WS_TEL_URL = f"{BASE_URL}/ws/telemetry"
REPORT_DIR = pathlib.Path(__file__).parent.parent / "sysid_report"

# Best ADRC from empirical analysis and theoretical derivation
# Theoretical b0 = Kt/(J*R) * 60/(2π) ≈ 9.31 RPM/s/V but that saturates
# the 24V rail; empirical sweet spot is 90–130.  Best logged result: wc=3, b0=105.
ADRC_PARAMS = {"wc": 3.0, "b0": 105.0, "ramp_time": 0.0}

# PID computed from motor model:
#   Loop gain K = Kt/(J*R) * (Vmax/PID_COUNTS) * 60/(2π)
#               = 0.8725/(0.18653361*4.8) * (24/4000) * 9.5493
#               ≈ 0.05586  [RPM/s per PID count]
#   Desired closed-loop bandwidth: ωn ≈ 1.5 rad/s (noise-limited)
#   P = ωn / K ≈ 1.5 / 0.05586 ≈ 27   (rounded to 25 to avoid saturation)
#   I = P * ωn / 10 = 25 * 0.15 ≈ 0.3  (slow integral — sigma0=7.87 RPM noise)
#   D = 0  (derivative amplifies sensor noise; omitted)
PID_PARAMS = {"p": 25.0, "i": 0.3, "d": 0.0, "gain_output": 1.0, "limit_i": 5000}

# Dark plot style matching sysid_report
COLORS = {
    "adrc":   "#00f2fe",   # cyan
    "pid":    "#f43f5e",   # rose
    "target": "#ffd32a",   # amber (dashed)
    "bg":     "#080c14",
    "panel":  "#111b2f",
    "grid":   "#1e3050",
    "text":   "#f1f5f9",
    "sub":    "#94a3b8",
}


# ---------------------------------------------------------------------------
# HTTP helpers
# ---------------------------------------------------------------------------

def _post(path, body=None):
    return requests.post(f"{HTTP_URL}{path}", json=body or {}, timeout=5).json()


def _get(path):
    return requests.get(f"{HTTP_URL}{path}", timeout=5).json()


def motor_init(connect_port="Virtual Motor", device_id=48):
    """Connect, set ADRC velocity mode, 100% ADRC blend, start drive."""
    _post("/connect", {"port": connect_port, "device_id": device_id})
    _post("/set_op_mode", {"mode": -2})
    _post("/set_pid", {
        "mode": "velocity", "p": 0, "i": 0, "d": 0,
        "gain_output": 1.0, "limit_i": 30000, "blend": 100,
    })
    _post("/set_adrc", {"mode": "velocity", **ADRC_PARAMS})
    _post("/set_target", {"mode": "velocity", "value": 0,
                         "min_limit": -4000, "max_limit": 4000})
    _post("/start")
    time.sleep(0.5)


def apply_adrc():
    _post("/set_pid", {
        "mode": "velocity", "p": 0, "i": 0, "d": 0,
        "gain_output": 1.0, "limit_i": 30000, "blend": 100,
    })
    payload = {"mode": "velocity", "wc": ADRC_PARAMS["wc"], "b0": ADRC_PARAMS["b0"], "ramp_time": ADRC_PARAMS["ramp_time"]}
    if "wo" in ADRC_PARAMS: payload["wo"] = ADRC_PARAMS["wo"]
    if "filter_alpha" in ADRC_PARAMS: payload["filter_alpha"] = ADRC_PARAMS["filter_alpha"]
    if "dist_alpha" in ADRC_PARAMS: payload["dist_alpha"] = ADRC_PARAMS["dist_alpha"]
    if "eso_alpha" in ADRC_PARAMS: payload["eso_alpha"] = ADRC_PARAMS["eso_alpha"]
    if "eso_delta" in ADRC_PARAMS: payload["eso_delta"] = ADRC_PARAMS["eso_delta"]
    _post("/set_adrc", payload)


def apply_pid():
    _post("/set_pid", {
        "mode": "velocity",
        **PID_PARAMS,
        "blend": 0,
    })


def set_target(rpm: float):
    _post("/set_target", {"mode": "velocity", "value": int(rpm),
                          "min_limit": -4000, "max_limit": 4000})


# ---------------------------------------------------------------------------
# Telemetry capture
# ---------------------------------------------------------------------------

async def _capture(duration: float) -> list[dict]:
    """Stream telemetry for `duration` seconds; return timestamped samples."""
    samples = []
    t0 = time.time()
    try:
        async with websockets.connect(WS_TEL_URL) as ws:
            while time.time() - t0 < duration:
                try:
                    raw = await asyncio.wait_for(ws.recv(), timeout=0.5)
                    pts = json.loads(raw)
                    for pt in pts:
                        if "velocity" in pt:
                            samples.append({
                                "t":        round(time.time() - t0, 4),
                                "velocity": pt["velocity"],
                                "current":  pt.get("current", 0.0),
                            })
                except asyncio.TimeoutError:
                    continue
    except Exception as e:
        print(f"  [WS] {e}", file=sys.stderr)
    return samples


def run_step_test(label: str, target_rpm: float, duration: float,
                  settle_s: float = 3.0) -> dict:
    """
    Reset to 0, wait to settle, apply step, capture telemetry.
    Returns a dict with samples, target, and computed metrics.
    """
    try:
        _post("/api/reset")
    except Exception:
        pass
    print(f"  [{label}] settling at 0 RPM ({settle_s:.0f}s)…")
    set_target(0)
    time.sleep(settle_s)

    print(f"  [{label}] step → {target_rpm:.0f} RPM, capturing {duration:.0f}s…")
    set_target(target_rpm)
    samples = asyncio.run(_capture(duration))

    metrics = _compute_metrics(samples, target_rpm)
    print(f"  [{label}] error={metrics['tracking_error']:.2f} RPM  "
          f"stdev={metrics['velocity_stdev']:.2f}  "
          f"Tr={metrics['rise_time_str']}  "
          f"Os={metrics['pct_overshoot']:.1f}%")
    return {"label": label, "target": target_rpm, "samples": samples, **metrics}


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------

def _compute_metrics(samples: list[dict], target: float) -> dict:
    if not samples:
        return {k: None for k in [
            "tracking_error", "velocity_stdev", "rise_time", "rise_time_str",
            "settling_time", "overshoot", "pct_overshoot", "ITAE",
        ]}

    vels = [s["velocity"] for s in samples]
    t_arr = [s["t"] for s in samples]

    # Steady-state = last 25% of capture window
    ss_start = int(len(vels) * 0.75)
    ss_vels  = vels[ss_start:]
    ss_mean  = statistics.mean(ss_vels) if ss_vels else 0.0
    ss_stdev = statistics.stdev(ss_vels) if len(ss_vels) > 1 else 0.0

    # Steady-state error: distance from the final settled value to the reference.
    # ss_mean IS the center of the settling envelope — it is NOT the target.
    tracking_error = abs(ss_mean - target)

    # Rise time: first time velocity ≥ 90% of target (transient measure).
    t90 = target * 0.9
    rise_time = None
    for s in samples:
        if abs(target) > 1 and s["velocity"] >= t90:
            rise_time = s["t"]
            break
    rise_time_str = f"{rise_time:.2f}s" if rise_time is not None else "N/A"

    # Settling time: last moment the signal is outside the ±band envelope.
    # The envelope is centred on ss_mean (the actual final value), NOT on target.
    # e_ss is then the gap between the envelope centre and the target line.
    # Band half-width = 2% of |target| or 1 RPM minimum.
    band_half = max(1.0, 0.02 * abs(target))
    settling_time = 0.0
    for s in reversed(samples):
        if abs(s["velocity"] - ss_mean) > band_half:
            settling_time = s["t"]
            break

    # Overshoot: peak excursion above (below) the target reference, not ss_mean.
    peak = max(vels) if target > 0 else min(vels)
    overshoot     = max(0.0, peak - target) if target > 0 else max(0.0, target - peak)
    pct_overshoot = (overshoot / abs(target) * 100.0) if abs(target) > 1 else 0.0

    # ITAE from step onset
    ITAE = 0.0
    for i in range(len(samples) - 1):
        tau = max(0.0, t_arr[i])
        dt  = t_arr[i + 1] - t_arr[i]
        e   = target - vels[i]
        ITAE += tau * abs(e) * dt
    ITAE_norm = ITAE / (target ** 2) if abs(target) > 1 else float("inf")

    return {
        "tracking_error":  round(tracking_error, 3),
        "velocity_stdev":  round(ss_stdev, 3),
        "ss_mean":         round(ss_mean, 3),    # final value — centre of settling band
        "band_half":       round(band_half, 3),  # half-width of settling envelope
        "rise_time":       round(rise_time, 3) if rise_time else None,
        "rise_time_str":   rise_time_str,
        "settling_time":   round(settling_time, 3),
        "overshoot":       round(overshoot, 3),
        "pct_overshoot":   round(pct_overshoot, 2),
        "ITAE":            round(ITAE_norm, 5),
    }


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def _style_ax(ax):
    ax.set_facecolor(COLORS["panel"])
    ax.tick_params(colors=COLORS["sub"], labelsize=9)
    ax.spines[:].set_color(COLORS["grid"])
    ax.xaxis.label.set_color(COLORS["sub"])
    ax.yaxis.label.set_color(COLORS["sub"])
    ax.title.set_color(COLORS["text"])
    ax.grid(True, color=COLORS["grid"], linewidth=0.5, linestyle="--", alpha=0.6)


def plot_comparison(adrc_result: dict, pid_result: dict) -> str:
    """Generate a comparison figure; return base64-encoded PNG.

    Settling-band annotation:
    - Dotted horizontal line at each controller's ss_mean (final value).
    - Shaded ±band_half envelope around ss_mean — this is where settling time
      is measured.  The gap between the dotted line and the dashed target line
      is the steady-state error (e_ss).
    - Overshoot is measured from the TARGET line (dashed), not from ss_mean.
    """
    target = adrc_result["target"]

    fig = plt.figure(figsize=(13, 7), facecolor=COLORS["bg"])
    gs  = gridspec.GridSpec(2, 1, height_ratios=[3, 1.2], hspace=0.12)
    ax_vel = fig.add_subplot(gs[0])
    ax_cur = fig.add_subplot(gs[1])

    duration = max(
        (adrc_result["samples"][-1]["t"] if adrc_result["samples"] else 12),
        (pid_result["samples"][-1]["t"]  if pid_result["samples"]  else 12),
    )

    for result, color in ((adrc_result, COLORS["adrc"]), (pid_result, COLORS["pid"])):
        t   = [s["t"]        for s in result["samples"]]
        vel = [s["velocity"] for s in result["samples"]]
        cur = [s["current"]  for s in result["samples"]]

        # Signal trace
        ax_vel.plot(t, vel, color=color, linewidth=1.4, alpha=0.9,
                    label=result["label"])
        ax_cur.plot(t, cur, color=color, linewidth=1.0, alpha=0.75)

        # Final-value (ss_mean) dotted line — the centre of the settling envelope
        ss   = result["ss_mean"]
        half = result["band_half"]
        e_ss = result["tracking_error"]
        ax_vel.axhline(ss, color=color, linewidth=1.0, linestyle=":",
                       alpha=0.7)

        # Settling envelope shaded band around ss_mean (±2% of |target|)
        ax_vel.axhspan(ss - half, ss + half, color=color, alpha=0.07)
        ax_vel.axhline(ss + half, color=color, linewidth=0.6,
                       linestyle="--", alpha=0.35)
        ax_vel.axhline(ss - half, color=color, linewidth=0.6,
                       linestyle="--", alpha=0.35)

        # e_ss annotation: double-headed arrow from ss_mean to target
        if e_ss > 0.5:
            mid_t = duration * 0.88
            ax_vel.annotate(
                "", xy=(mid_t, target), xytext=(mid_t, ss),
                arrowprops=dict(arrowstyle="<->", color=color, lw=1.0),
            )
            ax_vel.text(mid_t + duration * 0.01, (ss + target) / 2,
                        f"$e_{{ss}}$={e_ss:.1f}", color=color,
                        fontsize=7.5, va="center")

    # Target reference line (dashed amber) — NOT where the envelope is centred
    ax_vel.axhline(target, color=COLORS["target"], linewidth=1.3,
                   linestyle="--", alpha=0.9,
                   label=f"Reference {target:.0f} RPM")

    ax_vel.set_ylabel("Velocity (RPM)")
    ax_vel.set_title(f"Step Response — Reference {target:.0f} RPM  "
                     f"(dotted = final value · shaded = ±2% settling band · "
                     f"dashed = ±2% band edge)",
                     fontsize=10, fontweight="bold", pad=10)
    ax_vel.legend(facecolor=COLORS["panel"], edgecolor=COLORS["grid"],
                  labelcolor=COLORS["text"], fontsize=9)
    ax_vel.set_xlim(0, duration)

    ax_cur.set_ylabel("Current (mA)")
    ax_cur.set_xlabel("Time (s)")
    ax_cur.set_xlim(0, duration)
    ax_cur.axhline(0, color=COLORS["grid"], linewidth=0.5)

    for ax in (ax_vel, ax_cur):
        _style_ax(ax)

    ax_vel.tick_params(labelbottom=False)

    buf = io.BytesIO()
    plt.savefig(buf, format="png", dpi=130, bbox_inches="tight",
                facecolor=COLORS["bg"])
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()


def plot_metrics_bar(results_adrc: list, results_pid: list,
                     metric_key: str, ylabel: str, title: str) -> str:
    """Bar chart comparing a metric across targets."""
    targets = [r["target"] for r in results_adrc]
    adrc_vals = [r[metric_key] if r[metric_key] is not None else 0
                 for r in results_adrc]
    pid_vals  = [r[metric_key] if r[metric_key] is not None else 0
                 for r in results_pid]

    x = np.arange(len(targets))
    w = 0.35

    fig, ax = plt.subplots(figsize=(7, 4), facecolor=COLORS["bg"])
    ax.bar(x - w/2, adrc_vals, w, label="ADRC", color=COLORS["adrc"], alpha=0.85)
    ax.bar(x + w/2, pid_vals,  w, label="PID",  color=COLORS["pid"],  alpha=0.85)
    ax.set_xticks(x)
    ax.set_xticklabels([f"{int(t)} RPM" for t in targets])
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=11, fontweight="bold")
    ax.legend(facecolor=COLORS["panel"], edgecolor=COLORS["grid"],
              labelcolor=COLORS["text"], fontsize=9)
    _style_ax(ax)

    buf = io.BytesIO()
    plt.savefig(buf, format="png", dpi=110, bbox_inches="tight",
                facecolor=COLORS["bg"])
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()


# ---------------------------------------------------------------------------
# HTML report
# ---------------------------------------------------------------------------
_HTML_STYLE = """
<style>
  @import url('https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;500;600;700&family=JetBrains+Mono:wght@400;500&display=swap');
  :root {
    --bg:#080c14; --card:rgba(17,27,47,0.7); --border:rgba(43,67,107,0.4);
    --text:#f1f5f9; --sub:#94a3b8; --cyan:#00f2fe; --rose:#f43f5e;
    --amber:#ffd32a; --green:#10b981; --mono:'JetBrains Mono',monospace;
  }
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:var(--bg);color:var(--text);font-family:'Outfit',sans-serif;
       padding:2rem;line-height:1.6}
  h1{font-size:2rem;font-weight:700;color:var(--cyan);margin-bottom:.25rem}
  h2{font-size:1.25rem;font-weight:600;color:var(--text);margin:2rem 0 .75rem}
  h3{font-size:1rem;font-weight:500;color:var(--sub);margin:1rem 0 .5rem}
  .subtitle{color:var(--sub);font-size:.9rem;margin-bottom:2rem}
  .card{background:var(--card);border:1px solid var(--border);border-radius:12px;
        padding:1.5rem;margin-bottom:1.5rem}
  img{width:100%;border-radius:8px;margin-bottom:1rem}
  .metrics-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(220px,1fr));gap:.75rem;margin-top:1rem}
  .metric{background:rgba(0,0,0,0.3);border:1px solid var(--border);border-radius:8px;padding:.75rem 1rem}
  .metric-label{font-size:.75rem;color:var(--sub);text-transform:uppercase;letter-spacing:.05em}
  .metric-row{display:flex;gap:1.5rem;margin-top:.35rem}
  .metric-val{font-family:var(--mono);font-size:1rem;font-weight:500}
  .adrc{color:var(--cyan)}.pid{color:var(--rose)}
  table{width:100%;border-collapse:collapse;font-size:.85rem}
  th{text-align:left;color:var(--sub);font-weight:500;padding:.5rem .75rem;
     border-bottom:1px solid var(--border)}
  td{padding:.5rem .75rem;border-bottom:1px solid rgba(43,67,107,0.2);
     font-family:var(--mono)}
  tr:last-child td{border-bottom:none}
  .better{color:var(--green)}.worse{color:var(--rose)}
  .bar-grid{display:grid;grid-template-columns:1fr 1fr;gap:1rem}
  .tag{display:inline-block;padding:.2rem .6rem;border-radius:4px;
       font-size:.75rem;font-weight:600;letter-spacing:.05em}
  .tag-adrc{background:rgba(0,242,254,.15);color:var(--cyan);border:1px solid rgba(0,242,254,.3)}
  .tag-pid{background:rgba(244,63,94,.15);color:var(--rose);border:1px solid rgba(244,63,94,.3)}
  .params{display:flex;gap:2rem;flex-wrap:wrap;margin:.5rem 0}
  .param{font-family:var(--mono);font-size:.85rem;color:var(--sub)}
  .param span{color:var(--text)}
</style>
"""

def _win(adrc_val, pid_val, lower_is_better=True):
    """Return CSS class for the better value."""
    if adrc_val is None or pid_val is None:
        return "", ""
    if lower_is_better:
        a_cls = "better" if adrc_val <= pid_val else "worse"
        p_cls = "better" if pid_val <= adrc_val else "worse"
    else:
        a_cls = "better" if adrc_val >= pid_val else "worse"
        p_cls = "better" if pid_val >= adrc_val else "worse"
    return a_cls, p_cls


def _metric_block(label: str, adrc_r: dict, pid_r: dict,
                  key: str, fmt: str = ".3f", lower_is_better=True,
                  suffix="") -> str:
    av = adrc_r.get(key)
    pv = pid_r.get(key)
    a_cls, p_cls = _win(av, pv, lower_is_better)
    av_str = f"{av:{fmt}}{suffix}" if av is not None else "N/A"
    pv_str = f"{pv:{fmt}}{suffix}" if pv is not None else "N/A"
    return f"""
    <div class="metric">
      <div class="metric-label">{label}</div>
      <div class="metric-row">
        <span class="metric-val adrc {a_cls}">{av_str}</span>
        <span style="color:var(--border)">vs</span>
        <span class="metric-val pid {p_cls}">{pv_str}</span>
      </div>
    </div>"""


def build_report(adrc_results: list, pid_results: list, timestamp: str, note: str = "") -> str:
    pairs       = list(zip(adrc_results, pid_results))
    step_imgs   = []
    for a, p in pairs:
        step_imgs.append(plot_comparison(a, p))

    bar_tracking = plot_metrics_bar(adrc_results, pid_results,
                                    "tracking_error", "Error (RPM)",
                                    "Steady-State Tracking Error")
    bar_stdev    = plot_metrics_bar(adrc_results, pid_results,
                                    "velocity_stdev",  "Stdev (RPM)",
                                    "Velocity Std Dev (Noise/Oscillation)")
    bar_rise     = plot_metrics_bar(
        [{**r, "rise_time": r["rise_time"] or 99} for r in adrc_results],
        [{**r, "rise_time": r["rise_time"] or 99} for r in pid_results],
        "rise_time", "Rise Time (s)", "Rise Time (10% → 90%)")
    bar_os       = plot_metrics_bar(adrc_results, pid_results,
                                    "pct_overshoot", "Overshoot (%)",
                                    "Percent Overshoot")

    # Build step-response sections
    step_sections = ""
    for (a, p), img in zip(pairs, step_imgs):
        metrics_html = (
            _metric_block("Tracking Error", a, p, "tracking_error",     ".2f", suffix=" RPM") +
            _metric_block("Velocity Stdev", a, p, "velocity_stdev",     ".2f", suffix=" RPM") +
            _metric_block("Rise Time",      a, p, "rise_time",          ".2f", suffix="s") +
            _metric_block("Settling Time",  a, p, "settling_time",      ".2f", suffix="s") +
            _metric_block("Overshoot",      a, p, "pct_overshoot",      ".1f", suffix="%") +
            _metric_block("ITAE (norm.)",   a, p, "ITAE",               ".4f")
        )
        step_sections += f"""
        <div class="card">
          <img src="data:image/png;base64,{img}" alt="Step response {a['target']:.0f} RPM">
          <div class="metrics-grid">{metrics_html}</div>
        </div>"""

    # Summary table
    rows = ""
    for a, p in pairs:
        for result, tag_cls, label in ((a, "tag-adrc", "ADRC"), (p, "tag-pid", "PID")):
            rt = result["rise_time"]
            rows += f"""
            <tr>
              <td><span class="tag {tag_cls}">{label}</span></td>
              <td>{result['target']:.0f}</td>
              <td>{result['tracking_error']:.3f}</td>
              <td>{result['velocity_stdev']:.3f}</td>
              <td>{f"{rt:.2f}" if rt else "N/A"}</td>
              <td>{result['settling_time']:.2f}</td>
              <td>{result['pct_overshoot']:.1f}%</td>
              <td>{result['ITAE']:.5f}</td>
            </tr>"""

    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>ADRC vs PID Benchmark — {timestamp}</title>
  {_HTML_STYLE}
</head>
<body>
  <h1>ADRC vs PID Benchmark</h1>
  <div class="subtitle">Generated {timestamp} &nbsp;|&nbsp;
    Mock motor simulation with fitted model
    (J={0.18653361:.4f} kg·m², Kt={0.8725:.4f} N·m/A, R={4.8:.1f} Ω, σ₀={7.87:.2f} RPM)
    {"&nbsp;|&nbsp; " + note if note else ""}
  </div>

  <div class="card">
    <h2 style="margin-top:0">Controller Parameters</h2>
    <div style="display:grid;grid-template-columns:1fr 1fr;gap:1rem">
      <div>
        <div style="margin-bottom:.5rem"><span class="tag tag-adrc">ADRC</span></div>
        <div class="params">
          <div class="param">wc = <span>{ADRC_PARAMS['wc']}</span></div>
          <div class="param">wo = <span>{ADRC_PARAMS.get('wo', 3.0*ADRC_PARAMS['wc']):.3f}</span></div>
          <div class="param">b0 = <span>{ADRC_PARAMS['b0']}</span></div>
          <div class="param">fa = <span>{ADRC_PARAMS.get('filter_alpha', 0.85):.3f}</span></div>
          <div class="param">da = <span>{ADRC_PARAMS.get('dist_alpha', 0.90):.3f}</span></div>
          <div class="param">ea = <span>{ADRC_PARAMS.get('eso_alpha', 0.75):.3f}</span></div>
          <div class="param">ed = <span>{ADRC_PARAMS.get('eso_delta', 1.0):.3f}</span></div>
          <div class="param">ramp = <span>{ADRC_PARAMS['ramp_time']}</span></div>
          <div class="param">blend = <span>100%</span></div>
        </div>
        <div style="color:var(--sub);font-size:.8rem;margin-top:.5rem">
          Enhanced Nonlinear ADRC (NLESO) with independent observer bandwidth, measurement pre-filtering, and estimated disturbance filtering.
        </div>
      </div>
      <div>
        <div style="margin-bottom:.5rem"><span class="tag tag-pid">PID</span></div>
        <div class="params">
          <div class="param">P = <span>{PID_PARAMS['p']}</span></div>
          <div class="param">I = <span>{PID_PARAMS['i']}</span></div>
          <div class="param">D = <span>{PID_PARAMS['d']}</span></div>
          <div class="param">blend = <span>0%</span></div>
        </div>
        <div style="color:var(--sub);font-size:.8rem;margin-top:.5rem">
          Computed from motor model: K_loop = Kt/(J·R)·(Vmax/4000)·(60/2π) ≈ 0.056.<br>
          P = ωn/K_loop ≈ 25, I = P·ωn/10 ≈ 0.3 (noise-limited, σ₀ = 7.87 RPM).
        </div>
      </div>
    </div>
  </div>

  <h2>Step Response Comparisons</h2>
  <p style="color:var(--sub);font-size:.85rem;margin-bottom:.5rem">
    <span class="adrc">■</span> ADRC &nbsp;
    <span class="pid">■</span> PID &nbsp;
    <span style="color:var(--amber)">- -</span> Reference (target) &nbsp;
    <span style="color:var(--sub)">···</span> Final settled value (ss_mean, per controller)
  </p>
  <p style="color:var(--sub);font-size:.8rem;margin-bottom:1rem;line-height:1.7">
    The <b style="color:var(--text)">shaded band</b> (±2% of reference) is centred on each controller's
    actual final value (<code>ss_mean</code>), not on the reference line.<br>
    <b style="color:var(--text)">Settling time</b> = last moment the signal leaves this band.<br>
    <b style="color:var(--text)">Steady-state error e<sub>ss</sub></b> = gap between the dotted final-value line and
    the dashed reference line — shown with a ↕ annotation on each plot.
  </p>
  {step_sections}

  <h2>Cross-Target Metric Summary</h2>
  <div class="bar-grid card">
    <img src="data:image/png;base64,{bar_tracking}" alt="Tracking Error">
    <img src="data:image/png;base64,{bar_stdev}"    alt="Stdev">
    <img src="data:image/png;base64,{bar_rise}"     alt="Rise Time">
    <img src="data:image/png;base64,{bar_os}"       alt="Overshoot">
  </div>

  <div class="card">
    <h2 style="margin-top:0">Full Results Table</h2>
    <table>
      <thead>
        <tr>
          <th>Controller</th><th>Target (RPM)</th>
          <th>Error (RPM)</th><th>Stdev (RPM)</th>
          <th>Rise Time</th><th>Settle Time</th>
          <th>Overshoot</th><th>ITAE (norm)</th>
        </tr>
      </thead>
      <tbody>{rows}</tbody>
    </table>
    <p style="color:var(--sub);font-size:.75rem;margin-top:.75rem">
      Green = better value for that metric.
      <b>Steady-state error</b> = |ss_mean − reference|, where ss_mean is the mean velocity
      over the last 25% of capture — this is the centre of the settling envelope, not the reference.
      <b>Settling time</b> = last moment |v − ss_mean| &gt; ±2% of reference (band centred on ss_mean).
      <b>Overshoot</b> = (peak − reference) / reference — measured from the reference, not ss_mean.
      ITAE = ∫t·|e(t)|dt / reference² (lower = faster clean response).
    </p>
  </div>
</body>
</html>"""


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--targets", nargs="+", type=float, default=[50.0, 100.0, 200.0],
                        metavar="RPM", help="Target velocities to test (default: 50 100 200)")
    parser.add_argument("--duration", type=float, default=12.0,
                        help="Capture duration per step test in seconds (default: 12)")
    parser.add_argument("--settle", type=float, default=3.0,
                        help="Settle time at 0 RPM between tests (default: 3)")
    parser.add_argument("--port", type=str, default="Virtual Motor",
                        help="Motor port (default: 'Virtual Motor')")
    # Allow optimizer to inject best-found params
    parser.add_argument("--adrc-wc",  type=float, default=None)
    parser.add_argument("--adrc-b0",  type=float, default=None)
    parser.add_argument("--adrc-wo",  type=float, default=None)
    parser.add_argument("--adrc-filter-alpha", type=float, default=None)
    parser.add_argument("--adrc-dist-alpha",   type=float, default=None)
    parser.add_argument("--adrc-eso-alpha",    type=float, default=None)
    parser.add_argument("--adrc-eso-delta",    type=float, default=None)
    parser.add_argument("--pid-p",    type=float, default=None)
    parser.add_argument("--pid-i",    type=float, default=None)
    parser.add_argument("--pid-d",    type=float, default=None)
    parser.add_argument("--note",     type=str,   default="",
                        help="Subtitle note shown in the HTML report")
    args = parser.parse_args()

    # Override module-level defaults if CLI params provided
    if args.adrc_wc is not None: ADRC_PARAMS["wc"]  = args.adrc_wc
    if args.adrc_b0 is not None: ADRC_PARAMS["b0"]  = args.adrc_b0
    if args.adrc_wo is not None: ADRC_PARAMS["wo"]  = args.adrc_wo
    if args.adrc_filter_alpha is not None: ADRC_PARAMS["filter_alpha"] = args.adrc_filter_alpha
    if args.adrc_dist_alpha is not None: ADRC_PARAMS["dist_alpha"] = args.adrc_dist_alpha
    if args.adrc_eso_alpha is not None: ADRC_PARAMS["eso_alpha"] = args.adrc_eso_alpha
    if args.adrc_eso_delta is not None: ADRC_PARAMS["eso_delta"] = args.adrc_eso_delta
    if args.pid_p   is not None: PID_PARAMS["p"]    = args.pid_p
    if args.pid_i   is not None: PID_PARAMS["i"]    = args.pid_i
    if args.pid_d   is not None: PID_PARAMS["d"]    = args.pid_d

    # Verify server is reachable
    try:
        _get("/api/state")
    except Exception:
        print("ERROR: Server not reachable at http://127.0.0.1:8000", file=sys.stderr)
        print("Start it first:  python3 mock_sim.py  or  python3 run_headless.py", file=sys.stderr)
        sys.exit(1)

    print(f"Motor init (port={args.port})…")
    motor_init(connect_port=args.port)

    adrc_results = []
    pid_results  = []

    for target in args.targets:
        print(f"\n{'='*55}")
        print(f" Target: {target:.0f} RPM")
        print(f"{'='*55}")

        # ADRC
        apply_adrc()
        time.sleep(0.3)
        adrc_results.append(
            run_step_test(f"ADRC wc={ADRC_PARAMS['wc']} wo={ADRC_PARAMS.get('wo', 3.0*ADRC_PARAMS['wc']):.2f} b0={ADRC_PARAMS['b0']} fa={ADRC_PARAMS.get('filter_alpha', 0.85):.2f} da={ADRC_PARAMS.get('dist_alpha', 0.90):.2f} ea={ADRC_PARAMS.get('eso_alpha', 0.75):.2f} ed={ADRC_PARAMS.get('eso_delta', 1.0):.2f}",
                          target, args.duration, args.settle)
        )

        # PID
        apply_pid()
        time.sleep(0.3)
        pid_results.append(
            run_step_test(f"PID P={PID_PARAMS['p']} I={PID_PARAMS['i']}",
                          target, args.duration, args.settle)
        )

    # Restore ADRC on exit
    apply_adrc()
    set_target(0)

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    print(f"\nGenerating report…")
    html = build_report(adrc_results, pid_results, timestamp, note=args.note)

    REPORT_DIR.mkdir(exist_ok=True)
    out_path = REPORT_DIR / f"benchmark_{timestamp}.html"
    out_path.write_text(html, encoding="utf-8")
    print(f"Report saved → {out_path}")
    return str(out_path)


if __name__ == "__main__":
    main()
