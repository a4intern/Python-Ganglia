import struct
import time
import threading
import serial.tools.list_ports
import pymodbus.client
import uvicorn
import math
import collections
import pathlib

import json
import random

# ---------------------------------------------------------
# Load fitted motor model (if available)
# ---------------------------------------------------------
_MODEL_PATH = pathlib.Path(__file__).parent / "motor_model.json"

def _load_model():
    defaults = {
        "R": 0.5, "L": 0.002, "Ke": 0.005, "Kt": 0.05,
        "J": 1e-4, "B": 1e-4,
        "Fc": 0.02, "Fs": 0.05, "omega_s_rpm": 5.0,
        "A1": 0.0, "A2": 0.0, "A3": 0.0, "A4": 0.0, "A5": 0.0, "A6": 0.0,
        "N1": 1.0, "N2": 2.0, "N3": 3.0, "N4": 4.0, "N5": 5.0, "N6": 6.0,
        "phi1": 0.0, "phi2": 0.0, "phi3": 0.0, "phi4": 0.0, "phi5": 0.0, "phi6": 0.0,
        "sigma0": 0.5, "sigma1": 0.002,
        "deadzone_pwm_fwd": 200, "deadzone_pwm_rev": 200,
        "max_voltage": 24.0,
        # legacy brake parameters
        "BRAKE_FRICTION": 0.5,
    }
    if _MODEL_PATH.exists():
        try:
            with open(_MODEL_PATH) as f:
                data = json.load(f)
            defaults.update(data)
            print(f"[mock_sim] Loaded motor_model.json — fitted model active.")
        except Exception as e:
            print(f"[mock_sim] Could not load motor_model.json: {e} — using defaults.")
    else:
        print("[mock_sim] motor_model.json not found — using default parameters.")
    return defaults

MOTOR = _load_model()

MAX_VOLTAGE  = MOTOR["max_voltage"]
BRAKE_FRICTION = MOTOR["BRAKE_FRICTION"]  # legacy compat

# Compute physically correct initial b0 from motor model: dω/dt = b0 * V [RPM/s per V]
_b0_initial = MOTOR["Kt"] / (MOTOR["J"] * MOTOR["R"]) * (60.0 / (2 * math.pi))
_b0_initial = max(1.0, min(150.0, _b0_initial))
print(f"[mock_sim] Computed initial b0 = {_b0_initial:.2f} RPM/s/V from motor model")

# ---------------------------------------------------------
# Patching serial.tools.list_ports
# ---------------------------------------------------------
original_comports = serial.tools.list_ports.comports

class MockPortInfo:
    def __init__(self, device):
        self.device = device

def mock_comports():
    ports = original_comports()
    ports.insert(0, MockPortInfo("Virtual Motor"))
    return ports

serial.tools.list_ports.comports = mock_comports

# ---------------------------------------------------------
# Simulation State
# ---------------------------------------------------------
sim_state = {
    "velocity": 0.0,
    "position": 0.0,
    "current": 0.0,
    "target_velocity": 0.0,
    "z1": 0.0,
    "z2": 0.0,
    "z3": 0.0,
    
    "pwm_val": 0,
    "op_mode": 0,
    "brake_active": False,
    "coils": {i: False for i in range(100)},
    
    "pid_p": 0.0,
    "pid_i": 0.0,
    "pid_d": 0.0,
    "pid_integral": 0.0,
    "last_error": 0.0,
    
    "adrc_wc": 3.0,
    "adrc_b0": _b0_initial,
    "adrc_blend": 100,
    "adrc_z1": 0.0,
    "adrc_z2": 0.0,
    "adrc_z3": 0.0,
    "adrc_vel_filtered": 0.0
}

telemetry_history = collections.deque(maxlen=1000)

state_lock = threading.Lock()

def fal(e: float, alpha: float, delta: float) -> float:
    abs_e = abs(e)
    if abs_e > delta:
        return math.copysign(abs_e ** alpha, e)
    return e / max(1e-5, delta ** (1.0 - alpha))

# ---------------------------------------------------------
# Simulation Physics Thread
# ---------------------------------------------------------
def physics_loop():
    dt = 0.01
    obs_velocity = 0.0
    obs_current = 0.0
    obs_position = 0.0
    while True:
        with state_lock:
            mode = sim_state["op_mode"]
            drive_enabled = sim_state.get("drive_enabled", False)
            
            voltage = 0.0
            sim_state["brake_active"] = not drive_enabled
            
            if drive_enabled:
                if mode == 0: # PWM open loop
                    voltage = (sim_state["pwm_val"] / 4000.0) * MAX_VOLTAGE
                elif mode in (1, 2, -2): # PID / ADRC (Velocity/Position/Agent)
                    # Trajectory Generator (Constant Acceleration Ramp)
                    max_accel = 100.0  # RPM/s
                    current_target = sim_state.get("ramped_target", sim_state["velocity"])
                    if current_target < sim_state["target_velocity"]:
                        current_target = min(current_target + max_accel * dt, sim_state["target_velocity"])
                    elif current_target > sim_state["target_velocity"]:
                        current_target = max(current_target - max_accel * dt, sim_state["target_velocity"])
                    sim_state["ramped_target"] = current_target

                    error = current_target - obs_velocity
                    sim_state["pid_integral"] += error * dt
                    derivative = (error - sim_state["last_error"]) / dt
                    sim_state["last_error"] = error
                    
                    pid_out = (sim_state["pid_p"] * error + 
                               sim_state["pid_i"] * sim_state["pid_integral"] + 
                               sim_state["pid_d"] * derivative)
                    
                    pid_voltage = max(min((pid_out / 4000.0) * MAX_VOLTAGE, MAX_VOLTAGE), -MAX_VOLTAGE)
                    
                    # LADRC implementation
                    blend_ratio = sim_state["adrc_blend"] / 100.0
                    wo = sim_state.get("adrc_wo", 3 * sim_state["adrc_wc"])
                    b0 = sim_state["adrc_b0"]
                    applied_u = sim_state.get("last_voltage", 0.0)
                    
                    if mode in (1, -2): # Velocity Mode
                        # 1st-Order System (Velocity Control)
                        beta1 = 2 * wo
                        beta2 = wo**2

                        e = sim_state["adrc_z1"] - sim_state.get("adrc_vel_filtered", obs_velocity)
                        eso_alpha = sim_state.get("adrc_eso_alpha", 0.75)
                        eso_delta = sim_state.get("adrc_eso_delta", 1.0)
                        corr = fal(-e, eso_alpha, eso_delta)

                        sim_state["adrc_z1"] += (sim_state["adrc_z2"] + b0 * applied_u + beta1 * corr) * dt
                        sim_state["adrc_z2"] += (beta2 * corr) * dt
                        sim_state["adrc_z3"] = 0.0  # Not used in 1st order
                        
                        kp = sim_state["adrc_wc"]
                        u0 = kp * (current_target - sim_state["adrc_z1"])
                        
                        z2_val = sim_state["adrc_z2"]
                        z2_filtered = sim_state.get("adrc_z2_filtered", z2_val)
                        dist_alpha = sim_state.get("adrc_dist_filter_alpha", 0.90)
                        z2_filtered = dist_alpha * z2_filtered + (1.0 - dist_alpha) * z2_val
                        sim_state["adrc_z2_filtered"] = z2_filtered
                        
                        if b0 != 0:
                            adrc_voltage = (u0 - z2_filtered) / b0
                        else:
                            adrc_voltage = 0

                        if not math.isfinite(sim_state["adrc_z1"]) or not math.isfinite(sim_state["adrc_z2"]):
                            sim_state["adrc_z1"] = obs_velocity
                            sim_state["adrc_z2"] = 0.0
                            sim_state["adrc_z2_filtered"] = 0.0
                            adrc_voltage = 0.0

                    else:
                        # 2nd-Order System (Position Control)
                        beta1 = 3 * wo
                        beta2 = 3 * (wo**2)
                        beta3 = wo**3
                        
                        e = sim_state["adrc_z1"] - obs_position
                        eso_alpha = sim_state.get("adrc_eso_alpha", 0.75)
                        eso_delta = sim_state.get("adrc_eso_delta", 1.0)
                        corr = fal(-e, eso_alpha, eso_delta)

                        sim_state["adrc_z1"] += (sim_state["adrc_z2"] + beta1 * corr) * dt
                        sim_state["adrc_z2"] += (sim_state["adrc_z3"] + b0 * applied_u + beta2 * corr) * dt
                        sim_state["adrc_z3"] += (beta3 * corr) * dt
                        
                        kp = sim_state["adrc_wc"] ** 2
                        kd = 2 * sim_state["adrc_wc"]
                        u0 = kp * (sim_state["target_position"] - sim_state["adrc_z1"]) - kd * sim_state["adrc_z2"]
                        
                        z3_val = sim_state["adrc_z3"]
                        z3_filtered = sim_state.get("adrc_z3_filtered", z3_val)
                        dist_alpha = sim_state.get("adrc_dist_filter_alpha", 0.90)
                        z3_filtered = dist_alpha * z3_filtered + (1.0 - dist_alpha) * z3_val
                        sim_state["adrc_z3_filtered"] = z3_filtered
                        
                        if b0 != 0:
                            adrc_voltage = (u0 - z3_filtered) / b0
                        else:
                            adrc_voltage = 0

                        if not math.isfinite(sim_state["adrc_z1"]) or not math.isfinite(sim_state["adrc_z2"]) or not math.isfinite(sim_state["adrc_z3"]):
                            sim_state["adrc_z1"] = 0.0
                            sim_state["adrc_z2"] = 0.0
                            sim_state["adrc_z3"] = 0.0
                            sim_state["adrc_z3_filtered"] = 0.0
                            adrc_voltage = 0.0

                    adrc_voltage = max(min(adrc_voltage, MAX_VOLTAGE), -MAX_VOLTAGE)
                    
                    voltage = (1.0 - blend_ratio) * pid_voltage + blend_ratio * adrc_voltage
                    sim_state["last_voltage"] = voltage
            
            # ── Nonlinear Motor Physics ──────────────────────────────────────
            # State: current (A), angular velocity (rad/s), position (rad)
            # All in SI; convert RPM → rad/s at read, rad/s → RPM at write
            M = MOTOR
            R    = M["R"]; L = M["L"]; Ke = M["Ke"]; Kt = M["Kt"]
            J    = M["J"]; B = M["B"]; Fc = M["Fc"]; Fs = M["Fs"]
            ws   = M["omega_s_rpm"] * 2 * math.pi / 60.0  # convert to rad/s

            omega = sim_state["velocity"] * 2 * math.pi / 60.0  # RPM → rad/s
            theta = sim_state["position"] * 2 * math.pi / 60.0  # position in rad
            i_cur = sim_state.get("current_A", 0.0)             # state: A

            # Calculate electromagnetic torque
            Tm = Kt * i_cur

            # Static vs Dynamic friction
            if abs(omega) < 1e-3:
                # Motor is at rest. It only starts moving if EM torque exceeds static friction (stiction).
                # The stiction torque is determined by the dead-zone PWM threshold.
                dz_pwm = M["deadzone_pwm_fwd"] if Tm >= 0 else M["deadzone_pwm_rev"]
                V_dz = (dz_pwm / 4000.0) * MAX_VOLTAGE
                T_stiction = Kt * (V_dz / R)
                
                if abs(Tm) > T_stiction:
                    # Accelerate out of stiction
                    dom_dt = (Tm - math.copysign(T_stiction, Tm)) / max(J, 1e-9)
                else:
                    # Locked by stiction
                    dom_dt = 0.0
                    omega = 0.0
            else:
                # Motor is in motion. Dynamic friction applies (Coulomb + Stribeck + Viscous)
                Tf = (Fc * math.copysign(1, omega) * (1.0 - math.exp(-abs(omega) / (ws + 1e-9)))
                    + Fs * math.exp(-(omega / (ws + 1e-9))**2) * math.copysign(1, omega)
                    + B * omega)
                
                # Cogging harmonics
                for ci in range(1, 7):
                    Tf += M[f"A{ci}"] * math.sin(M[f"N{ci}"] * theta + M[f"phi{ci}"])

                # Brake adds extra friction when drive is disabled
                if sim_state["brake_active"]:
                    Tf += math.copysign(BRAKE_FRICTION, omega)

                dom_dt = (Tm - Tf) / max(J, 1e-9)

            # Electrical ODE: Exact discrete solution to prevent numerical instability
            # since dt (0.01) >> L/R (0.0004)
            if R > 1e-6:
                i_ss = (voltage - Ke * omega) / R
                i_cur = i_ss + (i_cur - i_ss) * math.exp(-(R / max(L, 1e-9)) * dt)
            else:
                di_dt = (voltage - Ke * omega) / max(L, 1e-9)
                i_cur += di_dt * dt
                
            i_cur = max(min(i_cur, 30.0), -30.0)  # hardware current limit (A)
            theta += omega * dt   # use omega BEFORE updating (explicit Euler)
            omega += dom_dt * dt

            # Write back state — reset to zero if physics produced NaN (e.g. extreme wc/b0)
            if not math.isfinite(omega) or not math.isfinite(i_cur):
                omega = 0.0
                i_cur = 0.0
                theta = sim_state["position"] * 2 * math.pi / 60.0
                sim_state["adrc_z1"] = 0.0
                sim_state["adrc_z2"] = 0.0
                sim_state["adrc_z3"] = 0.0
                sim_state["pid_integral"] = 0.0
                sim_state["last_voltage"] = 0.0

            sim_state["current_A"]   = i_cur
            sim_state["current"]     = i_cur * 1000.0           # mA for telemetry
            sim_state["velocity"]    = omega * 60.0 / (2 * math.pi)  # rad/s → RPM
            sim_state["position"]    = theta * 60.0 / (2 * math.pi)

            sim_state["z1"] = sim_state["position"]
            sim_state["z2"] = sim_state["velocity"]
            sim_state["z3"] = dom_dt * 60.0 / (2 * math.pi)  # angular accel in RPM/s

            # Heteroscedastic noise injection
            obs_velocity = sim_state["velocity"] + random.gauss(0, M["sigma0"] + M["sigma1"] * abs(sim_state["velocity"]))
            obs_current  = sim_state["current"]  + random.gauss(0, M["sigma_c0"] + M.get("sigma_c1", 0)*abs(sim_state["velocity"]))
            obs_position = sim_state["position"] + random.gauss(0, M["sigma_p0"] + M.get("sigma_p1", 0)*abs(sim_state["velocity"]))

            # Low-pass filter for ADRC observer input only (telemetry keeps raw noisy signal)
            adrc_fa = sim_state.get("adrc_filter_alpha", 0.85)
            sim_state["adrc_vel_filtered"] = (
                adrc_fa * sim_state["adrc_vel_filtered"]
                + (1.0 - adrc_fa) * obs_velocity
            )


            telemetry_history.append({
                "time": time.time(),
                "position": sim_state["position"] + random.gauss(0, 0.05),
                "velocity": obs_velocity,
                "current":  obs_current,
                "target_velocity": sim_state.get("ramped_target", sim_state["target_velocity"]),
                "z1": sim_state["z1"],
                "z2": sim_state["z2"],
                "z3": sim_state["z3"]
            })
            
        time.sleep(dt)

threading.Thread(target=physics_loop, daemon=True).start()

# ---------------------------------------------------------
# Mock Modbus Client
# ---------------------------------------------------------
class MockResult:
    def __init__(self, registers):
        self.registers = registers
    def isError(self):
        return False

class MockModbusClient:
    def __init__(self, port, framer, baudrate, timeout):
        self.port = port
        self.connected = False

    def connect(self):
        if self.port == "Virtual Motor":
            self.connected = True
            sim_state["op_mode"] = 0   # open-loop, pwm_val=0 → zero voltage on start
            sim_state["pwm_val"] = 0
            return True
        return False

    def close(self):
        self.connected = False

    def read_input_registers(self, address, count, device_id):
        if address == 0 and count == 22:
            with state_lock:
                def _safe_int(val, scale=1.0):
                    v = val * scale
                    return max(-2147483648, min(2147483647, int(v))) if math.isfinite(v) else 0

                vel_raw    = _safe_int(sim_state["velocity"], 10.0)
                cur_raw    = _safe_int(sim_state["current"], 1.0 / 4.698555425)
                target_raw = _safe_int(sim_state["target_velocity"], 10.0)
                
                regs = [0] * 22
                
                def pack_int(val, idx1, idx2):
                    b = struct.pack("<i", val)
                    regs[idx1], regs[idx2] = struct.unpack("<HH", b)
                    
                def pack_float(val, idx1, idx2):
                    b = struct.pack("<f", float(val))
                    regs[idx1], regs[idx2] = struct.unpack("<HH", b)
                
                pack_int(vel_raw, 2, 3)
                pack_int(cur_raw, 4, 5)
                pack_int(target_raw, 18, 19)
                pack_float(sim_state["z1"], 12, 13)
                pack_float(sim_state["z2"], 14, 15)
                pack_float(sim_state["z3"], 16, 17)
                
            return MockResult(regs)
        return MockResult([0]*count)

    def write_coil(self, address, value, device_id):
        with state_lock:
            sim_state["coils"][address] = value
            if address == 11 and value:
                sim_state["position"] = 0.0
            if address == 23 and value:
                sim_state["adrc_z1"] = sim_state["velocity"]
                sim_state["adrc_z2"] = 0.0
                sim_state["adrc_z3"] = 0.0
            if address == 13:
                sim_state["drive_enabled"] = bool(value)
                if bool(value):
                    # Reset controller state on drive enable to prevent spurious motion
                    sim_state["adrc_z1"] = sim_state["velocity"]
                    sim_state["adrc_z2"] = 0.0
                    sim_state["adrc_z3"] = 0.0
                    sim_state["pid_integral"] = 0.0
                    sim_state["last_error"] = 0.0
                    sim_state["last_voltage"] = 0.0
                    sim_state["ramped_target"] = sim_state["velocity"]
                
    def write_register(self, address, value, device_id):
        with state_lock:
            if address == 128:
                sim_state["op_mode"] = struct.unpack("<h", struct.pack("<H", value))[0]
            elif address == 80:
                sim_state["pwm_val"] = struct.unpack("<h", struct.pack("<H", value))[0]

    def write_registers(self, address, values, device_id):
        with state_lock:
            if address in (0, 16, 32):
                if len(values) >= 10:
                    b = struct.pack("<10H", *values[:10])
                    p, i, d, b_gain, limit, fade = struct.unpack("<ffffhH", b)
                    sim_state["pid_p"] = p
                    sim_state["pid_i"] = i
                    sim_state["pid_d"] = d
                
            if address in (10, 26, 42):
                if len(values) >= 6:
                    b = struct.pack("<6H", *values[:6])
                    val, min_l, max_l = struct.unpack("<iii", b)
                    if address == 26:
                        sim_state["target_velocity"] = val / 10.0

            if address in (368, 376, 384):  # ADRC pos/vel/cur
                if len(values) >= 16:
                    b = struct.pack("<16H", *values[:16])
                    wc, b0, ramp_time, wo, filter_alpha, dist_alpha, eso_alpha, eso_delta = struct.unpack("<ffffffff", b)
                    sim_state["adrc_wc"] = max(0.1, min(50.0, wc))
                    sim_state["adrc_b0"] = max(0.1, min(200000.0, b0))
                    sim_state["adrc_wo"] = wo if wo > 0.1 else 3.0 * wc
                    sim_state["adrc_filter_alpha"] = filter_alpha if filter_alpha > 0.1 else 0.85
                    sim_state["adrc_dist_filter_alpha"] = dist_alpha if dist_alpha > 0.1 else 0.90
                    sim_state["adrc_eso_alpha"] = eso_alpha if eso_alpha > 0.0 else 0.75
                    sim_state["adrc_eso_delta"] = eso_delta if eso_delta > 0.0 else 1.0
                elif len(values) >= 12:
                    b = struct.pack("<12H", *values[:12])
                    wc, b0, ramp_time, wo, filter_alpha, dist_alpha = struct.unpack("<ffffff", b)
                    sim_state["adrc_wc"] = max(0.1, min(50.0, wc))
                    sim_state["adrc_b0"] = max(0.1, min(200000.0, b0))
                    sim_state["adrc_wo"] = wo if wo > 0.1 else 3.0 * wc
                    sim_state["adrc_filter_alpha"] = filter_alpha if filter_alpha > 0.1 else 0.85
                    sim_state["adrc_dist_filter_alpha"] = dist_alpha if dist_alpha > 0.1 else 0.90
                elif len(values) >= 8:
                    b = struct.pack("<8H", *values[:8])
                    wc, b0, ramp_time, wo = struct.unpack("<ffff", b)
                    sim_state["adrc_wc"] = max(0.1, min(50.0, wc))
                    sim_state["adrc_b0"] = max(0.1, min(200000.0, b0))
                    sim_state["adrc_wo"] = wo if wo > 0.1 else 3.0 * wc

pymodbus.client.ModbusSerialClient = MockModbusClient

if __name__ == "__main__":
    import main
    import pathlib
    from pydantic import BaseModel
    
    main.UI_PATH = pathlib.Path(__file__).parent / "templates" / "sim_index.html"
    
    class TuneAdrcReq(BaseModel):
        wc: float = None
        b0: float = None
        blend: int = None
        wo: float = None
        filter_alpha: float = None
        dist_alpha: float = None
        eso_alpha: float = None
        eso_delta: float = None

    @main.app.post("/api/tune_adrc")
    async def tune_adrc_endpoint(req: TuneAdrcReq):
        from modbus_handler import agent_state, agent_state_lock, active_ws_queues, active_ws_queues_lock
        with state_lock:
            if req.wc is not None: sim_state["adrc_wc"] = req.wc
            if req.b0 is not None: sim_state["adrc_b0"] = req.b0
            if req.blend is not None: sim_state["adrc_blend"] = req.blend
            if req.wo is not None: sim_state["adrc_wo"] = req.wo
            if req.filter_alpha is not None: sim_state["adrc_filter_alpha"] = req.filter_alpha
            if req.dist_alpha is not None: sim_state["adrc_dist_filter_alpha"] = req.dist_alpha
            if req.eso_alpha is not None: sim_state["adrc_eso_alpha"] = req.eso_alpha
            if req.eso_delta is not None: sim_state["adrc_eso_delta"] = req.eso_delta
            
            update_msg = {
                "type": "tuning_update", 
                "wc": sim_state["adrc_wc"], 
                "b0": sim_state["adrc_b0"], 
                "blend": sim_state["adrc_blend"]
            }
            
        with agent_state_lock:
            if req.wc is not None: agent_state["agent_wc"] = req.wc
            if req.b0 is not None: agent_state["agent_b0"] = req.b0

        with active_ws_queues_lock:
            for q in active_ws_queues:
                try: q.put_nowait(update_msg)
                except: pass
        return {"status": "success", "state": update_msg}
        
    @main.app.get("/api/state")
    async def get_state():
        with state_lock:
            return dict(sim_state)
            
    @main.app.post("/api/reset")
    async def reset_endpoint():
        with state_lock:
            sim_state["velocity"] = 0.0
            sim_state["position"] = 0.0
            sim_state["current"] = 0.0
            sim_state["current_A"] = 0.0
            sim_state["adrc_z1"] = 0.0
            sim_state["adrc_z2"] = 0.0
            sim_state["adrc_z3"] = 0.0
            sim_state["pid_integral"] = 0.0
            sim_state["last_error"] = 0.0
            sim_state["last_voltage"] = 0.0
            sim_state["ramped_target"] = 0.0
            sim_state["adrc_vel_filtered"] = 0.0
            if "adrc_z2_filtered" in sim_state: sim_state["adrc_z2_filtered"] = 0.0
            if "adrc_z3_filtered" in sim_state: sim_state["adrc_z3_filtered"] = 0.0
        return {"status": "success"}
        
    @main.app.get("/api/history")
    async def get_history(count: int = 1000):
        with state_lock:
            hist = list(telemetry_history)[-count:]
        return {"data": hist}
        
    import os
    from fastapi.staticfiles import StaticFiles
    if not os.path.exists(pathlib.Path(__file__).parent / "static"):
        os.makedirs(pathlib.Path(__file__).parent / "static")
    main.app.mount("/static", StaticFiles(directory=str(pathlib.Path(__file__).parent / "static")), name="static")
        
    print("Starting Virtual Motor Simulation (with new Brake Sim UI)...")
    uvicorn.run(main.app, host="127.0.0.1", port=8000)
