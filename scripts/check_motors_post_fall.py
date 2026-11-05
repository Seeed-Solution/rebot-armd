import socket, struct, time, os

os.system("ip -details link show can0")

s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
s.bind(('can0',))
s.settimeout(0.05)

print("\n--- Motor Query ---")
for mid in range(1, 8):
    frame = struct.pack('=IB3x8s', mid, 8, bytes([0xFF]*7 + [0xFD]))
    s.send(frame)
    time.sleep(0.01)
    rep = "NO REPLY"
    try:
        pkt = s.recv(16)
        can_id, dlc, data = struct.unpack('=IB3x8s', pkt)
        mid_ret = data[0] & 0x0F
        st = (data[0] >> 4) & 0x0F
        rep = f"CAN=0x{can_id:X}, st={st}, data={data.hex()}"
    except Exception as e:
        pass
    print(f"Motor {mid}: {rep}")
