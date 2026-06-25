import requests
import json

payload = {
    "mode": "velocity", "value": 50, "min_limit": -4000, "max_limit": 4000
}
try:
    res = requests.post("http://127.0.0.1:8000/set_target", json=payload)
    print("STATUS:", res.status_code)
    print("BODY:", res.text)
except Exception as e:
    print("ERROR:", repr(e))
