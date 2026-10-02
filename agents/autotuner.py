import requests
import time
import json
import math

BASE_URL = "http://127.0.0.1:8000"

def connect_and_start():
    print("🔌 Connecting and starting motor...")
    requests.post(f"{BASE_URL}/connect", json={"port": "Virtual Motor", "device_id": 48})
    requests.post(f"{BASE_URL}/set_op_mode", json={"mode": -2})
    requests.post(f"{BASE_URL}/start")

def set_target(velocity):
    requests.post(f"{BASE_URL}/set_target", json={
        "mode": "velocity", "value": velocity, "min_limit": -3000, "max_limit": 3000
    })

def tune_adrc(wc, b0):
    requests.post(f"{BASE_URL}/api/tune_adrc", json={"wc": wc, "b0": b0, "blend": 100})

def get_history(count):
    res = requests.get(f"{BASE_URL}/api/history?count={count}")
    return res.json().get("data", [])

def evaluate_step(wc, b0):
    print(f"\n🧪 Testing wc={wc:5.1f}, b0={b0:5.1f} ...")
    
    # 1. Reset to 0 RPM and wait for steady state
    set_target(0)
    time.sleep(1.0)
    
    # 2. Apply new tuning
    tune_adrc(wc, b0)
    
    # 3. Trigger Step to 1000 RPM
    TARGET = 1000.0
    set_target(TARGET)
    
    # Wait for the step to settle
    time.sleep(2.0)
    
    # Read history (approx 200 points for 2 seconds at 10ms dt)
    data = get_history(200)
    if not data:
        return float('inf')
        
    velocities = [d['velocity'] for d in data]
    
    # Metrics
    max_vel = max(velocities)
    overshoot = max(0, max_vel - TARGET)
    overshoot_pct = (overshoot / TARGET) * 100.0
    
    # Calculate Integral Square Error (ISE)
    ise = sum((TARGET - v)**2 for v in velocities) / len(velocities)
    
    # Calculate settling time (time to stay within 2% of target)
    settling_idx = len(velocities)
    for i in range(len(velocities)-1, -1, -1):
        if abs(velocities[i] - TARGET) > 0.02 * TARGET:
            settling_idx = i + 1
            break
            
    settling_time = settling_idx * 0.01 # 10ms per tick
    
    # Cost function: Heavily penalize overshoot > 5%, otherwise prioritize fast settling & low ISE
    cost = ise + (overshoot_pct**2)*100
    if overshoot_pct > 5.0:
        cost += 1000000 # Massive penalty for high overshoot
        
    print(f"   📉 Overshoot: {overshoot_pct:5.2f}% | Settling Time: {settling_time:4.2f}s | Cost: {cost:.0f}")
    return cost

def run_autotuner():
    connect_and_start()
    
    # Parameter Grid
    wc_values = [15.0, 25.0, 35.0]
    b0_values = [1.0, 3.0, 5.0, 8.0]
    
    best_cost = float('inf')
    best_params = (None, None)
    
    print("🤖 Starting AI Autotuning Sequence...")
    for wc in wc_values:
        for b0 in b0_values:
            cost = evaluate_step(wc, b0)
            if cost < best_cost:
                best_cost = cost
                best_params = (wc, b0)
                
    # Phase 2: Fine-Tuning around the best parameters
    print(f"\n🎯 Coarse tune complete. Best so far: wc={best_params[0]}, b0={best_params[1]}. Fine-tuning...")
    best_wc, best_b0 = best_params
    
    fine_wc_values = [best_wc - 5, best_wc, best_wc + 5]
    fine_b0_values = [best_b0 - 1.0, best_b0, best_b0 + 1.0]
    
    for wc in fine_wc_values:
        if wc <= 0: continue
        for b0 in fine_b0_values:
            if b0 <= 0.1: continue
            if (wc, b0) == best_params: continue # Already tested
            cost = evaluate_step(wc, b0)
            if cost < best_cost:
                best_cost = cost
                best_params = (wc, b0)
                
    print(f"\n✨ Optimal Tuning Found!")
    print(f"   👉 wc = {best_params[0]}, b0 = {best_params[1]}")
    
    # Apply optimal tuning and do a final step response
    print("\n🚀 Demonstrating final optimal step response...")
    set_target(0)
    time.sleep(1.0)
    tune_adrc(best_params[0], best_params[1])
    set_target(1000)
    
if __name__ == "__main__":
    run_autotuner()
