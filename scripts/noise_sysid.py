"""
noise_sysid.py — Current and Position Sensor Noise Identification
=================================================================
Loads 05_noise_floor telemetry (steady-state at multiple RPM levels) and fits:

  sigma_current(v) = sigma_c0 + sigma_c1 * |v|   (mA)
  sigma_position(v) = sigma_p0 + sigma_p1 * |v|  (rad)

Position noise is estimated by integrating per-window velocity residuals:
  sigma_pos ≈ std(velocity_rpm) * dt_mean * (2π/60)

Updates motor_model.json with sigma_c0, sigma_c1, sigma_p0, sigma_p1.

Usage:
    python scripts/noise_sysid.py [--data-dir sysid_data] [--no-plots] [--dry-run]
"""
import argparse
import json
import pathlib
import sys
import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    print("Note: matplotlib not found — plots will be skipped.")

MODEL_FILE = pathlib.Path("motor_model.json")
REPORT_DIR = pathlib.Path("sysid_report")


# ──────────────────────────────────────────────────────────────────────────────
# Data loading
# ──────────────────────────────────────────────────────────────────────────────
def load_noise_floor(data_dir: pathlib.Path) -> list[dict]:
    files = sorted(data_dir.glob("05_noise_floor*.ndjson"))
    if not files:
        print(f"ERROR: No 05_noise_floor*.ndjson files found in {data_dir}")
        sys.exit(1)

    recs = []
    for f in files:
        with open(f) as fp:
            for line in fp:
                line = line.strip()
                if line:
                    try:
                        recs.append(json.loads(line))
                    except json.JSONDecodeError:
                        pass
        print(f"  Loaded {f.name}  ({len(recs)} total records so far)")
    return recs


def to_arrays(recs: list[dict]):
    t   = np.array([r.get("time", 0.0) for r in recs])
    vel = np.array([r.get("velocity", 0.0) for r in recs])    # RPM
    cur = np.array([r.get("current",  0.0) for r in recs])    # mA
    agent_tgts = np.array([r.get("agent_target", 0.0) for r in recs])
    tgt = agent_tgts if np.std(agent_tgts) > 0.1 else \
          np.array([r.get("target_velocity", 0.0) for r in recs])
    return t, vel, cur, tgt


# ──────────────────────────────────────────────────────────────────────────────
# Group steady-state windows
# ──────────────────────────────────────────────────────────────────────────────
def steady_state_groups(t, vel, cur, tgt, min_pts=30, vel_std_thresh=5.0):
    """
    Group samples by rounded target RPM level, then discard groups where the
    motor is hunting (std(velocity) > vel_std_thresh RPM).  Only genuinely
    settled windows give valid sensor-noise estimates; at low/mid RPM the
    closed-loop controller changes current continuously, inflating both
    std(current) and std(velocity) far above sensor noise.

    Returns list of (target_rpm, vel_window, cur_window, dt_mean).
    """
    unique_tgts = np.unique(np.round(tgt / 10.0) * 10)
    stable, skipped = [], []
    for lvl in unique_tgts:
        mask = np.abs(tgt - lvl) < 20
        if np.sum(mask) < min_pts:
            continue
        v_w = vel[mask]
        c_w = cur[mask]
        idx = np.where(mask)[0]
        dt_mean = float(np.mean(np.diff(t[idx]))) if len(idx) > 1 else 0.02
        v_std = float(np.std(v_w))
        if v_std > vel_std_thresh:
            skipped.append((abs(lvl), v_std))
        else:
            stable.append((float(abs(lvl)), v_w, c_w, dt_mean))

    if skipped:
        print(f"  Skipped {len(skipped)} unstable group(s) "
              f"(σ_v > {vel_std_thresh} RPM): "
              + ", ".join(f"{lvl:.0f} RPM (σ_v={s:.1f})" for lvl, s in skipped[:5])
              + ("…" if len(skipped) > 5 else ""))
    return stable


# ──────────────────────────────────────────────────────────────────────────────
# Current noise fit
# ──────────────────────────────────────────────────────────────────────────────
def fit_current_noise(groups):
    """
    Compute std(current) per steady-state window, then fit:
        sigma_c(v) = sigma_c0 + sigma_c1 * |v|
    """
    print("\n── Current Noise: σ_c(v) = σ_c0 + σ_c1·|v| ──")
    omegas, stdevs = [], []
    for lvl, vel_w, cur_w, _ in groups:
        # Skip windows where current is mostly zero (motor idle/current=0 bug)
        if np.mean(np.abs(cur_w)) < 5.0:
            continue
        s = float(np.std(cur_w))
        print(f"  |v|={lvl:6.0f} RPM  n={len(cur_w):4d}  σ_c={s:.2f} mA")
        omegas.append(lvl)
        stdevs.append(s)

    if len(omegas) < 2:
        mean_std = float(np.mean(stdevs)) if stdevs else 2.0
        print(f"  Too few groups for linear fit — using scalar σ_c={mean_std:.3f} mA")
        return {"sigma_c0": round(mean_std, 4), "sigma_c1": 0.0}

    coeffs = np.polyfit(omegas, stdevs, 1)
    sigma_c1 = float(np.clip(coeffs[0], 0.0, 1.0))
    sigma_c0 = float(np.clip(coeffs[1], 0.01, 500.0))

    # If slope is negligible (< 5% of intercept per 100 RPM), treat as homoscedastic
    if sigma_c1 * 100 < 0.05 * sigma_c0:
        sigma_c0 = float(np.mean(stdevs))
        sigma_c1 = 0.0
        print(f"  Slope negligible → homoscedastic: σ_c0={sigma_c0:.3f} mA")
    else:
        print(f"  σ_c0={sigma_c0:.4f} mA   σ_c1={sigma_c1:.6f} mA/RPM")

    return {"sigma_c0": round(sigma_c0, 6), "sigma_c1": round(sigma_c1, 8),
            "_omegas": omegas, "_stdevs": stdevs}


# ──────────────────────────────────────────────────────────────────────────────
# Position noise fit
# ──────────────────────────────────────────────────────────────────────────────
def fit_position_noise(groups):
    """
    Estimate position noise per sample by integrating velocity noise:
        sigma_pos ≈ std(velocity_rpm) * dt_mean * (2π/60)

    Then fit:
        sigma_p(v) = sigma_p0 + sigma_p1 * |v|

    Note: this captures velocity-noise-driven position uncertainty, not raw
    encoder quantization.  It is an upper bound on true encoder noise.
    """
    print("\n── Position Noise (via velocity integration): σ_p(v) = σ_p0 + σ_p1·|v| ──")
    RPM_TO_RADS = 2.0 * np.pi / 60.0

    omegas, sigma_pos_vals = [], []
    for lvl, vel_w, _, dt_mean in groups:
        sigma_v_rad = float(np.std(vel_w)) * RPM_TO_RADS    # rad/s
        sigma_p = sigma_v_rad * dt_mean                      # rad per sample
        print(f"  |v|={lvl:6.0f} RPM  σ_v={np.std(vel_w):.2f} RPM  "
              f"dt={dt_mean*1000:.1f} ms  σ_p={sigma_p:.5f} rad")
        omegas.append(lvl)
        sigma_pos_vals.append(sigma_p)

    if len(omegas) < 2:
        val = float(np.mean(sigma_pos_vals)) if sigma_pos_vals else 0.002
        print(f"  Too few groups — using scalar σ_p={val:.5f} rad")
        return {"sigma_p0": round(val, 6), "sigma_p1": 0.0}

    coeffs = np.polyfit(omegas, sigma_pos_vals, 1)
    sigma_p1 = float(np.clip(coeffs[0], 0.0, 1.0))
    sigma_p0 = float(np.clip(coeffs[1], 1e-6, 1.0))

    if sigma_p1 * 100 < 0.05 * sigma_p0:
        sigma_p0 = float(np.mean(sigma_pos_vals))
        sigma_p1 = 0.0
        print(f"  Slope negligible → homoscedastic: σ_p0={sigma_p0:.6f} rad")
    else:
        print(f"  σ_p0={sigma_p0:.6f} rad   σ_p1={sigma_p1:.8f} rad/RPM")

    return {"sigma_p0": round(sigma_p0, 8), "sigma_p1": round(sigma_p1, 10),
            "_omegas": omegas, "_stdevs": sigma_pos_vals}


# ──────────────────────────────────────────────────────────────────────────────
# Plots
# ──────────────────────────────────────────────────────────────────────────────
def plot_results(cur_result, pos_result, report_dir: pathlib.Path):
    if not HAS_MPL:
        return
    report_dir.mkdir(exist_ok=True)

    fig, axes = plt.subplots(1, 2, figsize=(11, 4))

    # Current noise
    ax = axes[0]
    if "_omegas" in cur_result:
        xs = np.array(cur_result["_omegas"])
        ys = np.array(cur_result["_stdevs"])
        ax.scatter(xs, ys, color="steelblue", zorder=3, label="data")
        x_fit = np.linspace(0, xs.max() * 1.05, 100)
        y_fit = cur_result["sigma_c0"] + cur_result["sigma_c1"] * x_fit
        ax.plot(x_fit, y_fit, "r--", label=f"fit  σ_c0={cur_result['sigma_c0']:.2f} mA")
    ax.set_xlabel("|velocity| (RPM)")
    ax.set_ylabel("σ (mA)")
    ax.set_title("Current Sensor Noise")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Position noise
    ax = axes[1]
    if "_omegas" in pos_result:
        xs = np.array(pos_result["_omegas"])
        ys = np.array(pos_result["_stdevs"])
        ax.scatter(xs, ys, color="darkorange", zorder=3, label="data")
        x_fit = np.linspace(0, xs.max() * 1.05, 100)
        y_fit = pos_result["sigma_p0"] + pos_result["sigma_p1"] * x_fit
        ax.plot(x_fit, y_fit, "r--", label=f"fit  σ_p0={pos_result['sigma_p0']:.5f} rad")
    ax.set_xlabel("|velocity| (RPM)")
    ax.set_ylabel("σ (rad)")
    ax.set_title("Position Noise Estimate")
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    out = report_dir / "noise_sysid.png"
    fig.savefig(out, dpi=120)
    print(f"\n  Plot saved → {out}")
    plt.close(fig)


# ──────────────────────────────────────────────────────────────────────────────
# Update motor_model.json
# ──────────────────────────────────────────────────────────────────────────────
def update_model(cur_result, pos_result, dry_run: bool):
    keys_to_write = {
        "sigma_c0": cur_result["sigma_c0"],
        "sigma_c1": cur_result["sigma_c1"],
        "sigma_p0": pos_result["sigma_p0"],
        "sigma_p1": pos_result["sigma_p1"],
    }

    print("\n── motor_model.json update ──")
    for k, v in keys_to_write.items():
        print(f"  {k}: {v}")

    if dry_run:
        print("  (dry-run: no file written)")
        return

    if not MODEL_FILE.exists():
        print(f"  WARNING: {MODEL_FILE} not found — skipping update")
        return

    with open(MODEL_FILE) as f:
        model = json.load(f)

    model.update(keys_to_write)

    with open(MODEL_FILE, "w") as f:
        json.dump(model, f, indent=2)
    print(f"  Wrote → {MODEL_FILE}")


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Identify current and position sensor noise")
    parser.add_argument("--data-dir", default="sysid_data", help="Directory with .ndjson files")
    parser.add_argument("--no-plots", action="store_true", help="Skip plot generation")
    parser.add_argument("--dry-run", action="store_true", help="Print results without writing motor_model.json")
    parser.add_argument("--vel-thresh", type=float, default=5.0,
                        help="Max std(velocity) RPM to consider a group settled (default: 5.0)")
    args = parser.parse_args()

    data_dir = pathlib.Path(args.data_dir)
    print(f"Loading noise floor data from {data_dir}/")
    recs = load_noise_floor(data_dir)
    t, vel, cur, tgt = to_arrays(recs)
    print(f"Total records: {len(recs)}")

    groups = steady_state_groups(t, vel, cur, tgt, vel_std_thresh=args.vel_thresh)
    if not groups:
        print("ERROR: No steady-state groups found — check that 05_noise_floor data exists")
        sys.exit(1)
    print(f"Steady-state groups: {len(groups)}")

    cur_result = fit_current_noise(groups)
    pos_result = fit_position_noise(groups)

    if not args.no_plots:
        plot_results(cur_result, pos_result, REPORT_DIR)

    update_model(cur_result, pos_result, args.dry_run)

    print("\nDone.")
    print("  sigma_c0  current noise floor  [mA]")
    print("  sigma_c1  current noise slope   [mA/RPM]")
    print("  sigma_p0  position noise floor  [rad/sample]  ← upper bound from vel noise")
    print("  sigma_p1  position noise slope  [rad/(RPM·sample)]")


if __name__ == "__main__":
    main()
