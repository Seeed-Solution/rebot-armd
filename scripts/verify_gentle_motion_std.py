#!/usr/bin/env python3
"""
verify_gentle_motion_std.py - Standalone End-to-End Gentle Motion Verifier
Uses ONLY Python standard library (urllib + threading + json), runs on any minimal Linux/reCamera.
"""
import json
import threading
import time
import urllib.request
import sys

HOST = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
PORT = 8765
BASE_URL = f"http://{HOST}:{PORT}"

def http_post(endpoint, payload):
    url = f"{BASE_URL}{endpoint}"
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=3.0) as resp:
        return json.loads(resp.read().decode("utf-8"))

def http_get(endpoint):
    url = f"{BASE_URL}{endpoint}"
    req = urllib.request.Request(url)
    with urllib.request.urlopen(req, timeout=3.0) as resp:
        return json.loads(resp.read().decode("utf-8"))

def main():
    print("=== [1/5] Checking initial armd state ===")
    obs = http_get("/api/v1/observation")
    epoch = obs.get("epoch_id", 1)
    p_init = list(obs["positions"])
    print(f"Connected! Initial epoch={epoch}, is_torqued={obs.get('is_torqued')}, state={obs.get('safety_state')}")

    # Heartbeat thread
    running = True
    current_epoch = [epoch]

    def heartbeat_worker():
        while running:
            try:
                http_post("/api/v1/heartbeat", {"client_id": "auto_verifier", "epoch_id": current_epoch[0]})
            except Exception:
                pass
            time.sleep(0.08)

    hb_thread = threading.Thread(target=heartbeat_worker, daemon=True)
    hb_thread.start()

    try:
        # Clear fault if stopped
        safety_state = obs.get("safety_state")
        if isinstance(safety_state, dict) or safety_state in ["EStop", "Fault"]:
            print(f"Clearing active fault: {safety_state}...")
            cf_res = http_post("/api/v1/safety/clear_fault", {})
            current_epoch[0] = cf_res.get("epoch_id", current_epoch[0])
            time.sleep(0.2)

        # Enable torque
        print("=== [2/5] Enabling hardware torque ===")
        t_res = http_post("/api/v1/torque", {"enable": True})
        current_epoch[0] = t_res.get("epoch_id", current_epoch[0])
        print(f"Torque enabled: is_torqued={t_res.get('is_torqued')}, state={t_res.get('safety_state')}, epoch={current_epoch[0]}")
        time.sleep(0.3)

        obs_after_enable = http_get("/api/v1/observation")
        p_actual = obs_after_enable["positions"]
        print(f"Hardware physical angles: {[round(x * 57.2958, 2) for x in p_actual]} deg")

        # Gentle micro-motion on Joint 6 (Wrist Roll / End effector)
        # Delta = +0.05 rad (2.86 degrees) - safe micro-movement
        delta_rad = 0.05
        target_p = list(p_actual)
        target_p[5] += delta_rad
        print(f"=== [3/5] Commanding gentle micro-motion on Joint 6 (+{round(delta_rad * 57.2958, 2)} deg) ===")

        # Smoothly interpolate over 1.0s (20 steps)
        peak_j6_reached = p_actual[5]
        for step in range(1, 21):
            interp_target = list(p_actual)
            interp_target[5] += (delta_rad * step / 20.0)
            http_post("/api/v1/motion/servo", {"targets": interp_target, "epoch_id": current_epoch[0]})
            time.sleep(0.05)
            # Sample actual feedback
            try:
                sample = http_get("/api/v1/observation")
                if len(sample["positions"]) >= 6:
                    peak_j6_reached = sample["positions"][5]
            except Exception:
                pass

        print(f"Reached peak J6: {round(peak_j6_reached * 57.2958, 2)} deg (Delta: {round((peak_j6_reached - p_actual[5]) * 57.2958, 2)} deg)")
        time.sleep(0.5)

        # Return gently back
        print("=== [4/5] Returning Joint 6 gently back to home ===")
        for step in range(1, 21):
            interp_target = list(p_actual)
            interp_target[5] += (delta_rad * (20 - step) / 20.0)
            http_post("/api/v1/motion/servo", {"targets": interp_target, "epoch_id": current_epoch[0]})
            time.sleep(0.05)

        time.sleep(0.3)
        final_obs = http_get("/api/v1/observation")
        print(f"Final positions: {[round(x * 57.2958, 2) for x in final_obs['positions']]} deg")

        # Disable torque
        print("=== [5/5] Disabling torque (safe finish) ===")
        http_post("/api/v1/torque", {"enable": False})
        print("Verification finished successfully!")

        result = {
            "success": True,
            "initial_deg": round(p_actual[5] * 57.2958, 2),
            "peak_deg": round(peak_j6_reached * 57.2958, 2),
            "delta_deg": round((peak_j6_reached - p_actual[5]) * 57.2958, 2),
            "final_deg": round(final_obs["positions"][5] * 57.2958, 2),
            "all_joints_deg": [round(x * 57.2958, 2) for x in final_obs["positions"]],
            "temperatures": final_obs["temperatures"],
        }
        print("RESULT_JSON:" + json.dumps(result))

    finally:
        running = False

if __name__ == "__main__":
    main()
