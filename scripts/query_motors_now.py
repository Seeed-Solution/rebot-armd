import socket, struct, time

def uint_to_float(x_int, x_min, x_max, bits):
    span = (1 << bits) - 1
    offset = float(x_max - x_min)
    return round(float(x_int) * offset / float(span) + x_min, 4)

s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
s.bind(('can0',))
s.settimeout(0.05)

for mid in range(1, 8):
    frame = struct.pack('=IB3x8s', mid, 8, bytes([0xFF]*7 + [0xFD]))
    s.send(frame)
    time.sleep(0.02)
    rep = "NO REPLY"
    try:
        while True:
            pkt = s.recv(16)
            can_id, dlc, data = struct.unpack('=IB3x8s', pkt)
            if can_id in [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]:
                m_ret = data[0] & 0x0F
                st = (data[0] >> 4) & 0x0F
                p_raw = (data[1] << 8) | data[2]
                t_raw = (((data[4] & 0x0F) << 8) | data[5])
                pos = uint_to_float(p_raw, -12.5, 12.5, 16)
                # TMAX read from the B601-DM motors (register 23): 28 on J1-J3, 10 on J4-J7
                tmax = 28.0 if mid <= 3 else 10.0
                tor = uint_to_float(t_raw, -tmax, tmax, 12)
                rep = f"st={st}, pos={pos:.4f} rad ({pos*57.3:.1f} deg), tor={tor:.2f} Nm, temp={data[6]}C"
                break
    except Exception:
        pass
    print(f"Motor {mid}: {rep}")
