import socket, struct, time

s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
s.bind(('can0',))
s.settimeout(0.05)

print("Sending enable to motors 1..7...")
for mid in range(1, 8):
    frame = struct.pack('=IB3x8s', mid, 8, bytes([0xFF]*7 + [0xFC]))
    s.send(frame)
    time.sleep(0.005)

time.sleep(0.05)
replies = {}
t_end = time.time() + 0.3
while time.time() < t_end:
    try:
        pkt = s.recv(16)
        can_id, dlc, data = struct.unpack('=IB3x8s', pkt)
        mid = data[0] & 0x0F
        st = (data[0] >> 4) & 0x0F
        replies[mid] = (hex(can_id), st, data.hex())
    except Exception:
        pass

for m in range(1, 8):
    rep = replies.get(m, "NO REPLY")
    print(f"Motor {m}: {rep}")
