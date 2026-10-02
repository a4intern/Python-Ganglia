import asyncio
import websockets
import json
import statistics
import time
import requests

BASE_URL = "http://127.0.0.1:8000"
WS_URL = "ws://127.0.0.1:8000/ws/telemetry"

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

async def grid_search_smooth():
    print("Starting Deep Search for Smooth ADRC at 20 RPM...")
    
    requests.post(f"{BASE_URL}/set_op_mode", json={"mode": -2})
    requests.post(f"{BASE_URL}/start")
    requests.post(f"{BASE_URL}/set_target", json={
        "mode": "velocity", "value": 50, "min_limit": -4000, "max_limit": 4000
    })
    requests.post(f"{BASE_URL}/set_pid", json={
        "mode": "velocity", "p": 0, "i": 0, "d": 0, "gain_output": 1.0, "limit_i": 30000, "blend": 100
    })
    
    wc_tests = [2.0, 5.0, 10.0]
    b0_tests = [2.0, 5.0, 10.0, 20.0] # Higher b0 -> smaller control effort -> less oscillation
    
    best_stdev = float('inf')
    best_params = (10.0, 1.0)
    
    for wc in wc_tests:
        for b0 in b0_tests:
            set_adrc(wc, b0)
            await asyncio.sleep(1.0) # Let it settle
            
            mean, stdev = await measure_performance(1.5)
            if mean is None:
                continue
                
            print(f"Testing wc={wc:>4.1f}, b0={b0:>4.1f} | Mean: {mean:>5.2f} RPM, Stdev: {stdev:>5.3f}")
            
            # We want minimum oscillation, but mean must be somewhat close to 20
            # If mean drops below 5 RPM, the control effort is too weak (b0 too high)
            if mean > 10.0 and stdev < best_stdev:
                best_stdev = stdev
                best_params = (wc, b0)
                
    print(f"\n✅ Search Complete!")
    print(f"Best Smooth Tuning: wc={best_params[0]:.1f}, b0={best_params[1]:.1f} with Stdev: {best_stdev:.3f}")
    
    # Apply best
    set_adrc(best_params[0], best_params[1])

if __name__ == "__main__":
    asyncio.run(grid_search_smooth())
