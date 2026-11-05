import socket, struct, time

s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
s.bind(('can0',))
s.settimeout(0.05)

print("Scanning CAN motors 1..10...")
for mid in range(1, 11):
    frame = struct.pack('=IB3x8s', mid, 8, bytes([0xFF]*7 + [0xFC]))
    s.send(frame)
    time.sleep(0.02)
    replies = []
    try:
        while True:
            pkt = s.recv(16)
            can_id, dlc, data = struct.unpack('=IB3x8s', pkt)
            replies.append((hex(can_id), data.hex()))
    except Exception:
        pass
    if replies:
        print(f"Motor ID {mid}: {replies}")
    else:
        print(f"Motor ID {mid}: NO REPLY")
