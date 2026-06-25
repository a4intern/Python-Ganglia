import requests
import time
import json
import argparse

BASE_URL = "http://127.0.0.1:8000"

def connect_motor():
    print("🔌 Connecting to Virtual Motor...")
    res = requests.post(f"{BASE_URL}/connect", json={"port": "Virtual Motor", "device_id": 48})
    print("Response:", res.json())

def set_op_mode(mode: int):
    print(f"⚙️ Setting Op Mode to {mode}...")
    res = requests.post(f"{BASE_URL}/set_op_mode", json={"mode": mode})
    print("Response:", res.json())

def start_motor():
    print("🚀 Starting motor...")
    res = requests.post(f"{BASE_URL}/start")
    print("Response:", res.json())

def stop_motor():
    print("🛑 Coasting motor to stop...")
    res = requests.post(f"{BASE_URL}/stop")
    print("Response:", res.json())

def set_target(value: float):
    print(f"🎯 Setting Target Velocity to {value} RPM...")
    res = requests.post(f"{BASE_URL}/set_target", json={
        "mode": "velocity", "value": value, "min_limit": -3000, "max_limit": 3000
    })
    print("Response:", res.json())

def tune_adrc(wc: float, b0: float, blend: int = 100):
    print(f"🔧 Tuning ADRC: wc={wc}, b0={b0}, blend={blend}%")
    res = requests.post(f"{BASE_URL}/api/tune_adrc", json={
        "wc": wc, "b0": b0, "blend": blend
    })
    print("Response:", res.json())

def get_history(count: int = 5):
    print(f"📊 Fetching last {count} telemetry ticks...")
    res = requests.get(f"{BASE_URL}/api/history?count={count}")
    data = res.json().get("data", [])
    for d in data:
        print(f"  Time: {d['time']:.2f} | Vel: {d['velocity']:7.2f} | z1: {d['z1']:7.2f} | z2: {d['z2']:7.2f} | z3: {d['z3']:7.2f}")
    print()

def run_sequence():
    connect_motor()
    set_op_mode(-2)
    start_motor()
    
    # Wait for motor to be ready
    time.sleep(0.5)
    
    print("\n--- Phase 1: Baseline Tuning ---")
    tune_adrc(wc=30.0, b0=1.0)
    set_target(1000.0)
    
    # Monitor for 3 seconds
    for _ in range(3):
        time.sleep(1)
        get_history(3)
        
    print("\n--- Phase 2: Aggressive Tuning ---")
    tune_adrc(wc=50.0, b0=2.0)
    
    # Monitor for 3 seconds
    for _ in range(3):
        time.sleep(1)
        get_history(3)
        
    print("\n--- Phase 3: Coasting to Stop ---")
    stop_motor()
    
    # Monitor coast down
    for _ in range(3):
        time.sleep(1)
        get_history(3)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sequence", action="store_true", help="Run the full automated tuning sequence")
    args = parser.parse_args()
    
    if args.sequence:
        run_sequence()
    else:
        print("Run with --sequence to execute the full demonstration.")
