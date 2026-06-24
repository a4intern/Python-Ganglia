import asyncio
import websockets
import json
import statistics
import time
import requests

BASE_URL = "http://127.0.0.1:8000"
WS_URL = "ws://127.0.0.1:8000/ws/telemetry"

def set_pid(p, i, d):
    requests.post(f"{BASE_URL}/set_pid", json={
        "mode": "velocity", "p": p, "i": i, "d": d, "gain_output": 1.0, "limit_i": 30000, "blend": 0
    })

def set_adrc(wc, b0):
    requests.post(f"{BASE_URL}/set_adrc", json={
        "mode": "velocity", "wc": wc, "b0": b0, "ramp_time": 0.0
    })

async def measure_performance(duration=2.0):
    velocities = []
    try:
        async with websockets.connect(WS_URL) as ws:
            start_time = time.time()
            while time.time() - start_time < duration:
                try:
                    data = await asyncio.wait_for(ws.recv(), timeout=0.5)
                    pts = json.loads(data)
                    for pt in pts:
                        if "velocity" in pt:
                            velocities.append(pt["velocity"])
                except asyncio.TimeoutError:
                    continue
    except Exception as e:
        print(f"WS Error: {e}")
        return None, None
        
    if not velocities:
        return None, None
    return statistics.mean(velocities), statistics.stdev(velocities) if len(velocities) > 1 else 0

async def online_tune():
    print("Starting Online Tuning (ADRC) for Target = 20 RPM...")
    
    # Enable Velocity mode
    requests.post(f"{BASE_URL}/set_op_mode", json={"mode": -2})
    requests.post(f"{BASE_URL}/start")
    
    # Switch to 100% ADRC mode using the PID endpoint's blend parameter
    requests.post(f"{BASE_URL}/set_pid", json={
        "mode": "velocity", "p": 0, "i": 0, "d": 0, "gain_output": 1.0, "limit_i": 30000, "blend": 100
    })
    
    # We will tune wc and b0
    current_wc = 10.0
    current_b0 = 1.0
    
    best_stdev = float('inf')
    best_params = (current_wc, current_b0)
    
    set_adrc(current_wc, current_b0)
    await asyncio.sleep(1) # wait for stabilize
    
    for iteration in range(5):
        mean, stdev = await measure_performance(1.5)
        if mean is None:
            print("Failed to read telemetry")
            break
            
        print(f"Iter {iteration}: wc={current_wc:.1f}, b0={current_b0:.2f} | Mean: {mean:.2f} RPM, Stdev: {stdev:.3f}")
        
        if stdev < best_stdev:
            best_stdev = stdev
            best_params = (current_wc, current_b0)
            
        error = 20.0 - mean
        
        # Simple ADRC heuristic tuning
        # b0 roughly models system gain (1/inertia). Lower b0 = more aggressive control effort.
        # wc determines the observer/controller bandwidth.
        
        if abs(error) > 1.0:
            current_b0 *= 0.9 # Decrease b0 to force ADRC to push harder
            
        if stdev > 1.0:
            current_wc += 5.0 # Increase bandwidth to react to disturbances faster
        elif stdev < 0.3 and abs(error) <= 1.0:
            pass # Excellent performance
        else:
            current_wc += 2.0
            
        set_adrc(current_wc, current_b0)
        await asyncio.sleep(1.0)
        
    print(f"\nFinished ADRC tuning! Best Stdev: {best_stdev:.3f} with wc={best_params[0]:.1f}, b0={best_params[1]:.2f}")
    set_adrc(best_params[0], best_params[1])

if __name__ == "__main__":
    asyncio.run(online_tune())
