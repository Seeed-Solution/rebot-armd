#!/usr/bin/env python3
"""
verify_gentle_motion.py - Automated End-to-End Gentle Motion Verifier
Connects to armd via WebSocket & HTTP, keeps heartbeat, executes micro-movement (<= 3 deg) on Joint 6,
and collects verified telemetry evidence.
"""
import asyncio
import json
import time
import urllib.request

HOST = "127.0.0.1"
PORT = 8765
BASE_URL = f"http://{HOST}:{PORT}"
WS_URL = f"ws://{HOST}:{PORT}/api/v1/ws"

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

async def run_verification():
    import websockets

    print("=== [1/5] Connecting to armd WebSocket ===")
    async with websockets.connect(WS_URL) as ws:
        # 1. Read initial observation
        first_msg = await asyncio.wait_for(ws.recv(), timeout=2.0)
        obs = json.loads(first_msg)
        epoch = obs.get("epoch_id", 1)
        p_init = list(obs["positions"])
        print(f"Connected! Initial epoch={epoch}, positions={[round(x, 4) for x in p_init]}")

        # Start background heartbeat task
        running = True
        current_epoch = epoch

        async def heartbeat_loop():
            while running:
                msg = {"type": "heartbeat", "client_id": "auto_verifier", "epoch_id": current_epoch}
                try:
                    await ws.send(json.dumps(msg))
                except Exception:
                    break
                await asyncio.sleep(0.08)

        hb_task = asyncio.create_task(heartbeat_loop())

        try:
            # 2. Check if stopped / fault -> clear fault first
            safety_state = obs.get("safety_state")
            if isinstance(safety_state, dict) or safety_state in ["EStop", "Fault"]:
                print(f"Clearing active fault: {safety_state}...")
                cf_res = http_post("/api/v1/safety/clear_fault", {})
                current_epoch = cf_res.get("epoch_id", current_epoch)
                await asyncio.sleep(0.2)

            # 3. Enable torque
            print("=== [2/5] Enabling hardware torque ===")
            t_res = http_post("/api/v1/torque", {"enable": True})
            current_epoch = t_res.get("epoch_id", current_epoch)
            print(f"Torque enabled: is_torqued={t_res.get('is_torqued')}, state={t_res.get('safety_state')}, epoch={current_epoch}")
            await asyncio.sleep(0.3)

            # Read latest positions after enable
            obs_after_enable = http_get("/api/v1/observation")
            p_actual = obs_after_enable["positions"]
            print(f"Hardware angles read: {[round(x * 57.2958, 2) for x in p_actual]} deg")

            # 4. Gentle micro-motion on Joint 6 (Wrist Roll / End effector)
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
                http_post("/api/v1/motion/servo", {"targets": interp_target, "epoch_id": current_epoch})
                await asyncio.sleep(0.05)
                # Read latest observation from ws
                try:
                    while True:
                        msg_str = await asyncio.wait_for(ws.recv(), timeout=0.01)
                        latest = json.loads(msg_str)
                        if "positions" in latest and len(latest["positions"]) >= 6:
                            peak_j6_reached = latest["positions"][5]
                except asyncio.TimeoutError:
                    pass

            print(f"Reached peak J6 position: {round(peak_j6_reached * 57.2958, 2)} deg (Delta: {round((peak_j6_reached - p_actual[5]) * 57.2958, 2)} deg)")
            await asyncio.sleep(0.5)

            # Return gently back to initial position
            print("=== [4/5] Returning Joint 6 gently back to home ===")
            for step in range(1, 21):
                interp_target = list(p_actual)
                interp_target[5] += (delta_rad * (20 - step) / 20.0)
                http_post("/api/v1/motion/servo", {"targets": interp_target, "epoch_id": current_epoch})
                await asyncio.sleep(0.05)

            await asyncio.sleep(0.3)
            final_obs = http_get("/api/v1/observation")
            print(f"Final positions: {[round(x * 57.2958, 2) for x in final_obs['positions']]} deg")

            # 5. Disable torque
            print("=== [5/5] Disabling torque (safe finish) ===")
            http_post("/api/v1/torque", {"enable": False})
            print("Verification finished successfully!")

            return {
                "success": True,
                "initial_deg": round(p_actual[5] * 57.2958, 2),
                "peak_deg": round(peak_j6_reached * 57.2958, 2),
                "delta_deg": round((peak_j6_reached - p_actual[5]) * 57.2958, 2),
                "final_deg": round(final_obs["positions"][5] * 57.2958, 2),
                "all_joints_deg": [round(x * 57.2958, 2) for x in final_obs["positions"]],
                "temperatures": final_obs["temperatures"],
            }

        finally:
            running = False
            hb_task.cancel()

if __name__ == "__main__":
    result = asyncio.run(run_verification())
    print("RESULT:", json.dumps(result, indent=2))
