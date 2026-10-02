import struct
scaled_value = int(50.0 * 10.0)
min_limit = int(-4000.0)
max_limit = int(4000.0)
try:
    print(struct.pack("<iii", scaled_value, min_limit, max_limit))
except Exception as e:
    print("ERROR:", repr(e))
