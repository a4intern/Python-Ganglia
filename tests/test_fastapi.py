import struct
from pydantic import BaseModel

class TargetRequest(BaseModel):
    mode: str
    value: float
    min_limit: float = -4000.0
    max_limit: float = 4000.0

req = TargetRequest(mode="velocity", value=50, min_limit=-4000, max_limit=4000)

POSITION_TRANSFER_SCALE = 1.0
VELOCITY_TRANSFER_SCALE = 10.0
CURRENT_TRANSFER_SCALE  = 1.0

if req.mode == "position":
    scaled_value = int(req.value * POSITION_TRANSFER_SCALE)
elif req.mode == "velocity":
    scaled_value = int(req.value * VELOCITY_TRANSFER_SCALE)
else:
    scaled_value = int(req.value * CURRENT_TRANSFER_SCALE)

try:
    packed_bytes = struct.pack("<iii", scaled_value, int(req.min_limit), int(req.max_limit))
    print("SUCCESS")
except Exception as e:
    print("ERROR:", repr(e))
