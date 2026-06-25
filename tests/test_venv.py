import struct
try:
    print(struct.pack("<iii", 0, -4000, 4000))
except Exception as e:
    print("ERROR:", repr(e))
