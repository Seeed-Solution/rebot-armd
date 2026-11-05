import socket, struct, time

s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
s.bind(('can0',))
s.settimeout(0.05)

# 1. Clear faults
print("Sending Clear Fault to motors 1..7...")
for mid in range(1, 8):
    s.send(struct.pack('=IB3x8s', mid, 8, bytes([0xFF]*7 + [0xFB])))
time.sleep(0.1)

# Drain
while True:
    try:
        s.recv(16)
    except Exception:
        break

# 2. Ping / Enable
print("Testing each motor 1..7...")
for mid in range(1, 8):
    frame = struct.pack('=IB3x8s', mid, 8, bytes([0xFF]*7 + [0xFD]))
    s.send(frame)
    got = []
    t_end = time.time() + 0.1
    while time.time() < t_end:
        try:
            pkt = s.recv(16)
            cid, dlc, d = struct.unpack('=IB3x8s', pkt)
            got.append(f"0x{cid:02x}: {[hex(b) for b in d]}")
        except Exception:
            pass
    print(f"Motor {mid}: {got if got else 'NO RESPONSE'}")
