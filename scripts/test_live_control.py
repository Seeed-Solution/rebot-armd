import urllib.request, json, time

def call_api(endpoint, data=None):
    url = f"http://127.0.0.1:8765{endpoint}"
    req_data = json.dumps(data).encode() if data is not None else None
    headers = {"Content-Type": "application/json"} if data is not None else {}
    req = urllib.request.Request(url, data=req_data, headers=headers)
    try:
        with urllib.request.urlopen(req) as resp:
            return json.loads(resp.read().decode())
    except urllib.error.HTTPError as e:
        return {"error": e.code, "msg": e.read().decode()}
    except Exception as e:
        return {"error": str(e)}

print("=== 1. Initial Observation ===")
obs = call_api("/api/v1/observation")
print("Obs:", obs)

print("\n=== 2. Arming (POST /api/v1/torque {enable: true}) ===")
arm_res = call_api("/api/v1/torque", {"enable": True})
print("Arm result:", arm_res)

time.sleep(0.5)

print("\n=== 3. Observation after arming ===")
obs_armed = call_api("/api/v1/observation")
print("Obs armed:", obs_armed)

print("\n=== 4. Test Jog J1 (+0.03 rad) ===")
if obs_armed.get("positions"):
    curr_j1 = obs_armed["positions"][0]
    target_pos = list(obs_armed["positions"])
    target_pos[0] = round(curr_j1 + 0.03, 4)
    # /api/v1/motion/jog payload
    jog_res = call_api("/api/v1/motion/jog", {"positions": target_pos})
    print("Jog result:", jog_res)

time.sleep(1.0)

print("\n=== 5. Observation after motion ===")
obs_moved = call_api("/api/v1/observation")
print("Obs moved:", obs_moved)

print("\n=== 6. Disarming (POST /api/v1/torque {enable: false}) ===")
disarm_res = call_api("/api/v1/torque", {"enable": False})
print("Disarm result:", disarm_res)
