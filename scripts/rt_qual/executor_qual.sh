#!/usr/bin/env bash
# Stage 3 §6 RT executor qualification sequence runner (device or local mock).
#
# Sequence:
#   1. N cold starts: restart armd, verify owner_mode == "executor", loop_active == true,
#      safety_state == "Idle", and record thread scheduling priorities / memory_locked.
#   2. Stationary idle soak (default 60 s) via load.py + analyze.py (+ devstat.sh in device mode).
#   3. J1 motion load at speed_scale 0.3 (smoke 45 s / full 180 s) via load.py + analyze.py
#      (+ devstat.sh in device mode).
#   4. Stop / E-stop checks (at confirmed disarmed Idle state: WS stop + REST emergency_stop
#      latches EStop, advances epoch, and refuses torque enable while E-stopped).
#   5. Collects artifacts under <out-dir>/<run-id>/ and writes summary.md (listing panic/teardown
#      hold-target-changing tests as "covered in sim").

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

MODE="device"          # device | mock
PROFILE="smoke"        # smoke (45 s) | full (180 s)
COLD_STARTS=3
IDLE_S=60
MOTION_S=""
CYCLES=""
SAMPLE_INTERVAL=5
SPEED_SCALE="0.3"
J1_DELTA_RAD="0.15"
HOST="127.0.0.1"
PORT=""
OUT_DIR=""
RUN_ID=""
ARMD_BIN=""
INIT_SCRIPT="/etc/init.d/K92rebot-teleop"
RESTART_CMD=""
CAN_IFACE="can0"
ALLOW_INSECURE_AUTH=0
CLEAR_ESTOP_AFTER_CHECK=""
ENABLE_DEVSTAT=""
ENABLE_MEASURE=""
ENABLE_RECORD=""
ENABLE_BULK_HTTP=""
BULK_PATH=""
ALLOC_TIMING=1

usage() {
    cat <<'EOF'
Usage: executor_qual.sh [OPTIONS]
  --mode device|mock          Target mode (default: device)
  --smoke                     Motion duration 45 s, 2 cycles (default)
  --full                      Motion duration 180 s, 6 cycles
  --cold-starts N             Number of cold starts to verify (default: 3)
  --idle-s S                  Stationary idle soak duration in seconds (default: 60)
  --motion-s S                Override motion duration in seconds
  --cycles N                  Override motion cycle count
  --sample-interval S         Diagnostics sample interval in seconds (default: 5)
  --speed-scale F             Motion speed_scale, <= 0.3 (default: 0.3)
  --j1-delta-rad R            J1 excursion in radians (default: 0.15)
  --host HOST                 armd host (default: 127.0.0.1)
  --port PORT                 armd HTTP/WS port (default: 8765 for device, free port for mock;
                              never 18765/18001/18990)
  --out-dir DIR               Artifact base dir (default: /userdata/rt-qual on device,
                              /tmp/rt-qual-mock in mock mode)
  --run-id ID                 Run identifier (default: exec-qual-YYYYMMDDTHHMMSSZ)
  --armd-bin PATH             Path to armd binary in mock mode (default: target/debug/armd)
  --init-script PATH          Init script on device (default: /etc/init.d/K92rebot-teleop)
  --restart-cmd CMD           Custom restart command for device mode (overrides --init-script)
  --can-iface IFACE           CAN interface for devstat.sh (default: can0)
  --allow-insecure-auth       Pass --allow-insecure-auth to load.py
  --clear-estop-after-check   Clear latched E-stop after E-stop check (default: on in mock, off on device)
  --no-devstat                Disable devstat.sh collection in device mode
  --no-alloc-timing           Do not set ARMD_ALLOC_TIMING=1 in mock mode
  -h, --help                  Show this help
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --mode) MODE="${2:-}"; shift 2 ;;
        --smoke) PROFILE="smoke"; shift ;;
        --full) PROFILE="full"; shift ;;
        --cold-starts) COLD_STARTS="${2:-}"; shift 2 ;;
        --idle-s) IDLE_S="${2:-}"; shift 2 ;;
        --motion-s) MOTION_S="${2:-}"; shift 2 ;;
        --cycles) CYCLES="${2:-}"; shift 2 ;;
        --sample-interval) SAMPLE_INTERVAL="${2:-}"; shift 2 ;;
        --speed-scale) SPEED_SCALE="${2:-}"; shift 2 ;;
        --j1-delta-rad) J1_DELTA_RAD="${2:-}"; shift 2 ;;
        --host) HOST="${2:-}"; shift 2 ;;
        --port) PORT="${2:-}"; shift 2 ;;
        --out-dir) OUT_DIR="${2:-}"; shift 2 ;;
        --run-id) RUN_ID="${2:-}"; shift 2 ;;
        --armd-bin) ARMD_BIN="${2:-}"; shift 2 ;;
        --init-script) INIT_SCRIPT="${2:-}"; shift 2 ;;
        --restart-cmd) RESTART_CMD="${2:-}"; shift 2 ;;
        --can-iface) CAN_IFACE="${2:-}"; shift 2 ;;
        --allow-insecure-auth) ALLOW_INSECURE_AUTH=1; shift ;;
        --clear-estop-after-check) CLEAR_ESTOP_AFTER_CHECK=1; shift ;;
        --no-devstat) ENABLE_DEVSTAT=0; shift ;;
        --no-alloc-timing) ALLOC_TIMING=0; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

case "$MODE" in
    device|mock) ;;
    *) echo "Error: --mode must be 'device' or 'mock' (got '$MODE')" >&2; exit 2 ;;
esac

if [ -z "$MOTION_S" ]; then
    if [ "$PROFILE" = "full" ]; then MOTION_S=180; else MOTION_S=45; fi
fi
if [ -z "$CYCLES" ]; then
    if [ "$PROFILE" = "full" ]; then CYCLES=6; else CYCLES=2; fi
fi
if [ -z "$RUN_ID" ]; then
    RUN_ID="exec-qual-$(date -u +%Y%m%dT%H%M%SZ)"
fi
if [ -z "$OUT_DIR" ]; then
    if [ "$MODE" = "mock" ]; then OUT_DIR="/tmp/rt-qual-mock"; else OUT_DIR="/userdata/rt-qual"; fi
fi
if [ -z "$CLEAR_ESTOP_AFTER_CHECK" ]; then
    if [ "$MODE" = "mock" ]; then CLEAR_ESTOP_AFTER_CHECK=1; else CLEAR_ESTOP_AFTER_CHECK=0; fi
fi
if [ -z "$ENABLE_DEVSTAT" ]; then
    if [ "$MODE" = "device" ]; then ENABLE_DEVSTAT=1; else ENABLE_DEVSTAT=0; fi
fi
if [ -z "$ENABLE_MEASURE" ]; then
    if [ "$MODE" = "device" ]; then ENABLE_MEASURE=1; else ENABLE_MEASURE=0; fi
fi
if [ -z "$ENABLE_RECORD" ]; then
    ENABLE_RECORD=1
fi
if [ -z "$ENABLE_BULK_HTTP" ]; then
    ENABLE_BULK_HTTP=1
fi
if [ -z "$BULK_PATH" ]; then
    if [ "$MODE" = "device" ]; then BULK_PATH="/api/v1/camera/snapshot"; else BULK_PATH="/"; fi
fi

pick_free_port() {
    python3 -c "
import socket
FORBIDDEN = {18765, 18001, 18990}
for _ in range(32):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('127.0.0.1', 0))
        p = s.getsockname()[1]
        if p not in FORBIDDEN:
            print(p)
            raise SystemExit(0)
raise SystemExit(1)
"
}

if [ -z "$PORT" ]; then
    if [ "$MODE" = "mock" ]; then
        PORT="$(pick_free_port)"
    else
        PORT="8765"
    fi
fi

case "$PORT" in
    18765|18001|18990)
        echo "Error: port $PORT is forbidden (never use 18765/18001/18990)" >&2
        exit 2
        ;;
esac

if [ "$MODE" = "mock" ]; then
    if [ -z "$ARMD_BIN" ]; then
        if [ -x "${REPO_ROOT}/target/debug/armd" ]; then
            ARMD_BIN="${REPO_ROOT}/target/debug/armd"
        elif [ -x "${REPO_ROOT}/target/release/armd" ]; then
            ARMD_BIN="${REPO_ROOT}/target/release/armd"
        else
            echo "Error: armd binary not found; build with 'cargo build --bin armd' or pass --armd-bin" >&2
            exit 1
        fi
    fi
    if [ ! -x "$ARMD_BIN" ]; then
        echo "Error: --armd-bin is not executable: $ARMD_BIN" >&2
        exit 1
    fi
fi

# Auto-discover rebot Python SDK from sibling rebot-companion or installed kit path if not already importable.
if ! python3 -c "import rebot" >/dev/null 2>&1; then
    for cand in \
        "/usr/share/rebot-teleop/python" \
        "${REPO_ROOT}/../rebot-companion" \
        "${REPO_ROOT}/../../rebot-companion" \
        "/home/harvest/project/rebot-companion"; do
        if [ -d "${cand}/rebot" ]; then
            export PYTHONPATH="${cand}${PYTHONPATH:+:${PYTHONPATH}}"
            break
        fi
    done
fi

RUN_DIR="${OUT_DIR}/${RUN_ID}"
mkdir -p "${RUN_DIR}/idle" "${RUN_DIR}/motion"
BASE_URL="http://${HOST}:${PORT}"

MOCK_ARMD_PID=""
DEVSTAT_PID=""
MOCK_WORK_DIR=""

stop_pid_cleanly() {
    local pid="$1"
    if [ -z "$pid" ]; then return 0; fi
    if kill -0 "$pid" 2>/dev/null; then
        kill -TERM "$pid" 2>/dev/null || true
        local w=0
        while [ "$w" -lt 40 ] && kill -0 "$pid" 2>/dev/null; do
            sleep 0.1
            w=$((w + 1))
        done
        if kill -0 "$pid" 2>/dev/null; then
            kill -KILL "$pid" 2>/dev/null || true
        fi
        wait "$pid" 2>/dev/null || true
    fi
}

cleanup() {
    stop_pid_cleanly "$DEVSTAT_PID"
    DEVSTAT_PID=""
    stop_pid_cleanly "$MOCK_ARMD_PID"
    MOCK_ARMD_PID=""
}
trap cleanup EXIT INT TERM

wait_armd_ready() {
    local tries=0
    while [ "$tries" -lt 100 ]; do
        if python3 -c "
import json, sys, urllib.request
with urllib.request.urlopen('${BASE_URL}/api/v1/diagnostics/tick', timeout=1.0) as r:
    d = json.loads(r.read().decode())
if d.get('loop_active') and (d.get('releases_total') or 0) >= 3:
    raise SystemExit(0)
raise SystemExit(1)
" 2>/dev/null; then
            return 0
        fi
        sleep 0.1
        tries=$((tries + 1))
    done
    echo "Error: armd at ${BASE_URL} did not become ready with active control loop" >&2
    return 1
}

restart_armd() {
    local idx="$1"
    if [ "$MODE" = "mock" ]; then
        stop_pid_cleanly "$MOCK_ARMD_PID"
        MOCK_ARMD_PID=""
        MOCK_WORK_DIR="${RUN_DIR}/mock-armd-${idx}"
        mkdir -p "${MOCK_WORK_DIR}/logs" "${MOCK_WORK_DIR}/blackbox" "${MOCK_WORK_DIR}/episodes"
        local env_args=("ARMD_GRASP_SETUP=/nonexistent/grasp_setup.json")
        if [ "$ALLOC_TIMING" -eq 1 ]; then
            env_args+=("ARMD_ALLOC_TIMING=1")
        fi
        env "${env_args[@]}" "$ARMD_BIN" \
            --follower mock \
            --owner executor \
            --config "${REPO_ROOT}/kits/recamera-b601dm/profile/rebot-b601dm.json" \
            --port "$PORT" \
            --log-dir "${MOCK_WORK_DIR}/logs" \
            --blackbox-dir "${MOCK_WORK_DIR}/blackbox" \
            --episodes-dir "${MOCK_WORK_DIR}/episodes" \
            --no-operator-auth \
            >"${RUN_DIR}/armd-coldstart-${idx}.log" 2>&1 &
        MOCK_ARMD_PID=$!
    elif [ -n "$RESTART_CMD" ]; then
        sh -c "$RESTART_CMD"
    else
        "$INIT_SCRIPT" restart
    fi
    wait_armd_ready
}

echo "====================================================================="
echo "RT Executor Qualification (${MODE}, profile=${PROFILE}, run_id=${RUN_ID})"
echo "Target: ${BASE_URL} | Output: ${RUN_DIR}"
echo "====================================================================="

# ------------------------------------------------------------------------------
# Step 1: N Cold Starts (restart armd, check owner == executor, Idle, priorities)
# ------------------------------------------------------------------------------
echo "[1/4] Running ${COLD_STARTS} cold start(s)..."
COLD_RECORDS_FILE="${RUN_DIR}/cold_starts.jsonl"
: > "$COLD_RECORDS_FILE"

i=1
while [ "$i" -le "$COLD_STARTS" ]; do
    restart_armd "$i"
    python3 - "$BASE_URL" "$i" "$MODE" >> "$COLD_RECORDS_FILE" <<'PY'
import json, sys, urllib.request
base_url, idx, mode = sys.argv[1], int(sys.argv[2]), sys.argv[3]
with urllib.request.urlopen(f"{base_url}/api/v1/diagnostics/tick", timeout=2.0) as r:
    tick = json.loads(r.read().decode())
with urllib.request.urlopen(f"{base_url}/api/v1/observation", timeout=2.0) as r:
    obs = json.loads(r.read().decode())
owner = tick.get("owner_mode")
loop_active = tick.get("loop_active")
safety = obs.get("safety_state")
if isinstance(safety, dict):
    safety = next(iter(safety.keys()), str(safety))
ctl = tick.get("control_thread") or {}
rec = {
    "cold_start": idx,
    "owner_mode": owner,
    "loop_active": loop_active,
    "safety_state": safety,
    "is_torqued": obs.get("is_torqued"),
    "instance_id": tick.get("instance_id"),
    "boot_id": tick.get("boot_id"),
    "releases_total": tick.get("releases_total"),
    "control_thread": ctl,
    "control_thread_lock_attempts": tick.get("control_thread_lock_attempts"),
    "control_thread_alloc_ops": tick.get("control_thread_alloc_ops"),
    "alloc_timing_enabled": tick.get("alloc_timing_enabled"),
}
print(json.dumps(rec))
errors = []
if owner != "executor":
    errors.append(f"owner_mode={owner!r} != 'executor'")
if loop_active is not True:
    errors.append(f"loop_active={loop_active!r} != True")
if safety != "Idle":
    errors.append(f"safety_state={safety!r} != 'Idle'")
if mode == "device":
    if not ctl.get("realtime"):
        errors.append(f"control_thread.realtime={ctl.get('realtime')!r}")
    if not ctl.get("memory_locked"):
        errors.append(f"control_thread.memory_locked={ctl.get('memory_locked')!r}")
if errors:
    sys.stderr.write(f"Cold start {idx} FAILED: {'; '.join(errors)}\n")
    raise SystemExit(1)
PY
    echo "      cold start ${i}/${COLD_STARTS}: PASS (owner=executor, safety=Idle)"
    i=$((i + 1))
done

python3 - "$COLD_RECORDS_FILE" "${RUN_DIR}/cold_starts.json" <<'PY'
import json, sys
rows = [json.loads(line) for line in open(sys.argv[1], encoding="utf-8") if line.strip()]
distinct_ids = len({r.get("instance_id") for r in rows if r.get("instance_id")})
out = {"cold_starts": rows, "count": len(rows), "distinct_instance_ids": distinct_ids}
with open(sys.argv[2], "w", encoding="utf-8") as f:
    json.dump(out, f, indent=2)
if len(rows) > 1 and distinct_ids != len(rows):
    sys.stderr.write(f"Cold start check FAILED: expected {len(rows)} distinct instance_ids, got {distinct_ids}\n")
    raise SystemExit(1)
PY

# Helper to run devstat.sh in background when enabled.
start_devstat() {
    local sub_id="$1"
    local dur_s="$2"
    if [ "$ENABLE_DEVSTAT" -eq 1 ]; then
        sh "${SCRIPT_DIR}/devstat.sh" \
            --run-id "$sub_id" \
            --out-dir "$RUN_DIR" \
            --interval "$SAMPLE_INTERVAL" \
            --duration "$((dur_s + 20))" \
            --armd-url "$BASE_URL" \
            --can-iface "$CAN_IFACE" \
            >"${RUN_DIR}/${sub_id}/devstat.log" 2>&1 &
        DEVSTAT_PID=$!
    fi
}

stop_devstat() {
    if [ -n "$DEVSTAT_PID" ]; then
        stop_pid_cleanly "$DEVSTAT_PID"
        DEVSTAT_PID=""
    fi
}

AUTH_FLAGS=()
if [ "$ALLOW_INSECURE_AUTH" -eq 1 ]; then
    AUTH_FLAGS+=(--allow-insecure-auth)
fi

# ------------------------------------------------------------------------------
# Step 2: Stationary Idle Soak (default 60 s)
# ------------------------------------------------------------------------------
echo "[2/4] Running stationary idle soak (${IDLE_S}s)..."
start_devstat "idle" "$IDLE_S"
python3 "${SCRIPT_DIR}/load.py" \
    --host "$HOST" --port "$PORT" \
    --run-id "${RUN_ID}-idle" \
    --duration "$IDLE_S" \
    --baseline-s 0 \
    --cooldown-s 0 \
    --sample-interval "$SAMPLE_INTERVAL" \
    --no-motion --no-record --no-measure --no-bulk-http --no-ws-stream \
    --expect-owner executor \
    --mark-baseline \
    "${AUTH_FLAGS[@]}" \
    --output "${RUN_DIR}/idle/load.jsonl"
stop_devstat

ANALYZE_IDLE_ARGS=(--load "${RUN_DIR}/idle/load.jsonl" --output "${RUN_DIR}/idle/report.md")
if [ -f "${RUN_DIR}/idle/devstat.txt" ]; then
    ANALYZE_IDLE_ARGS+=(--devstat "${RUN_DIR}/idle/devstat.txt")
fi
python3 "${SCRIPT_DIR}/analyze.py" "${ANALYZE_IDLE_ARGS[@]}"
IDLE_VERDICT="$(grep -m1 '^\*\*Overall Verdict:' "${RUN_DIR}/idle/report.md" || echo "unknown")"
echo "      idle report: ${IDLE_VERDICT}"

# ------------------------------------------------------------------------------
# Step 3: J1 Motion Qualification Run (speed_scale 0.3, smoke 45 s / full 180 s)
# ------------------------------------------------------------------------------
echo "[3/4] Running J1 motion qualification (${MOTION_S}s, ${CYCLES} cycles, speed_scale=${SPEED_SCALE})..."
MOTION_FLAGS=(
    --host "$HOST" --port "$PORT"
    --run-id "${RUN_ID}-motion"
    --duration "$MOTION_S"
    --cycles "$CYCLES"
    --motion
    --motion-mode joint-j1
    --j1-delta-rad "$J1_DELTA_RAD"
    --speed-scale "$SPEED_SCALE"
    --sample-interval "$SAMPLE_INTERVAL"
    --bulk-path "$BULK_PATH"
    --expect-owner executor
    --mark-baseline
    --output "${RUN_DIR}/motion/load.jsonl"
)
if [ "$ENABLE_MEASURE" -eq 0 ]; then MOTION_FLAGS+=(--no-measure); fi
if [ "$ENABLE_RECORD" -eq 0 ]; then MOTION_FLAGS+=(--no-record); fi
if [ "$ENABLE_BULK_HTTP" -eq 0 ]; then MOTION_FLAGS+=(--no-bulk-http); fi
if [ "$ALLOW_INSECURE_AUTH" -eq 1 ]; then MOTION_FLAGS+=(--allow-insecure-auth); fi

start_devstat "motion" "$MOTION_S"
python3 "${SCRIPT_DIR}/load.py" "${MOTION_FLAGS[@]}"
stop_devstat

ANALYZE_MOTION_ARGS=(--load "${RUN_DIR}/motion/load.jsonl" --output "${RUN_DIR}/motion/report.md")
if [ -f "${RUN_DIR}/motion/devstat.txt" ]; then
    ANALYZE_MOTION_ARGS+=(--devstat "${RUN_DIR}/motion/devstat.txt")
fi
python3 "${SCRIPT_DIR}/analyze.py" "${ANALYZE_MOTION_ARGS[@]}"
MOTION_VERDICT="$(grep -m1 '^\*\*Overall Verdict:' "${RUN_DIR}/motion/report.md" || echo "unknown")"
echo "      motion report: ${MOTION_VERDICT}"

# ------------------------------------------------------------------------------
# Step 4: Stop / E-Stop Checks (confirmed Idle & disarmed before triggering E-stop)
# ------------------------------------------------------------------------------
echo "[4/4] Running stop & E-stop checks..."
python3 - "$BASE_URL" "$CLEAR_ESTOP_AFTER_CHECK" "${RUN_DIR}/estop_check.json" <<'PY'
import json, os, sys, time, urllib.error, urllib.request
from rebot import _ws as rebot_ws

base_url = sys.argv[1]
clear_after = sys.argv[2] == "1"
out_path = sys.argv[3]
ws_url = base_url.replace("http://", "ws://").replace("https://", "wss://") + "/api/v1/ws"

def get_json(path):
    with urllib.request.urlopen(f"{base_url}{path}", timeout=3.0) as r:
        return json.loads(r.read().decode())

def post_json(path, body=None, headers=None):
    data = b"" if body is None else json.dumps(body).encode()
    hdrs = {"Content-Type": "application/json", "Origin": base_url}
    if headers:
        hdrs.update(headers)
    req = urllib.request.Request(f"{base_url}{path}", data=data, headers=hdrs, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=3.0) as r:
            raw = r.read()
            return int(r.status), (json.loads(raw.decode()) if raw else None)
    except urllib.error.HTTPError as e:
        raw = e.read()
        try:
            parsed = json.loads(raw.decode()) if raw else None
        except Exception:
            parsed = raw.decode(errors="replace")
        return int(e.code), parsed

def safety_name(obs):
    s = obs.get("safety_state")
    if isinstance(s, dict):
        return next(iter(s.keys()), str(s))
    return str(s)

# 1. Verify pre-check Idle & disarmed (never trigger E-stop check while energized on hardware).
obs_before = get_json("/api/v1/observation")
s_before = safety_name(obs_before)
if s_before != "Idle" or obs_before.get("is_torqued"):
    raise RuntimeError(f"Pre-check requires Idle & disarmed, got safety={s_before}, is_torqued={obs_before.get('is_torqued')}")
epoch_before = obs_before.get("epoch_id")

# 2. Acquire WS lease and exercise /api/v1/motion/stop on executor owner.
ws = rebot_ws.connect(ws_url, timeout=3.0, origin=True)
req_counter = 0
try:
    def recv_types(wanted, timeout_s=3.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            raw = ws.recv(timeout=max(0.1, deadline - time.monotonic()))
            msg = json.loads(raw)
            if msg.get("type") in wanted:
                return msg
        raise TimeoutError(f"timeout waiting for {wanted}")

    ws.send(json.dumps({"type": "acquire_lease", "seq": 1, "client_id": "executor-qual-estop", "force": True}))
    lease = recv_types({"lease_granted", "lease_rejected"})
    if lease.get("type") != "lease_granted":
        raise RuntimeError(f"lease acquire failed: {lease}")
    gen = lease["generation"]

    def ws_cmd(path, body, epoch_id, operator_token=None):
        global req_counter
        req_counter += 1
        payload = {
            "type": "command",
            "request_id": req_counter,
            "generation": gen,
            "epoch_id": epoch_id,
            "path": path,
            "body": body,
        }
        if operator_token:
            payload["operator_token"] = operator_token
        ws.send(json.dumps(payload))
        return recv_types({"command_ack", "command_reject"})

    ws_stop_reply = ws_cmd("/api/v1/motion/stop", {}, epoch_before)
    if ws_stop_reply.get("type") != "command_ack":
        raise RuntimeError(f"WS /api/v1/motion/stop failed: {ws_stop_reply}")

    # 3. Trigger REST E-stop (POST /api/v1/safety/e_stop) and verify EStop latched + epoch advanced.
    estop_status, estop_body = post_json("/api/v1/safety/e_stop", {})
    if estop_status != 200:
        raise RuntimeError(f"POST /api/v1/safety/e_stop failed: {estop_status} {estop_body}")

    obs_estop = get_json("/api/v1/observation")
    s_estop = safety_name(obs_estop)
    epoch_estop = obs_estop.get("epoch_id")
    if s_estop != "EStop":
        raise RuntimeError(f"expected EStop after safety/e_stop, got {s_estop}")
    if epoch_before is not None and epoch_estop is not None and epoch_estop <= epoch_before:
        raise RuntimeError(f"epoch_id did not advance on E-stop: {epoch_before} -> {epoch_estop}")

    # 4. Verify torque enable is refused while EStop is active.
    en_reply = ws_cmd("/api/v1/torque", {"enable": True}, epoch_estop)
    if en_reply.get("type") != "command_reject":
        raise RuntimeError(f"torque enable must be refused while E-stopped, got {en_reply}")

    cleared = False
    if clear_after:
        pin = os.environ.get("REBOT_OPERATOR_PIN")
        tok = os.environ.get("REBOT_OPERATOR_TOKEN")
        if not tok and pin:
            _, ul = post_json("/api/v1/operator/unlock", {"pin": pin})
            if isinstance(ul, dict):
                tok = ul.get("token")
        clr_reply = ws_cmd("/api/v1/safety/clear_fault", {}, epoch_estop, operator_token=tok)
        if clr_reply.get("type") != "command_ack":
            raise RuntimeError(f"WS clear_fault failed after E-stop check: {clr_reply}")
        obs_after = get_json("/api/v1/observation")
        if safety_name(obs_after) != "Idle":
            raise RuntimeError(f"expected Idle after clear_fault, got {safety_name(obs_after)}")
        cleared = True
finally:
    ws.close()

tick_after = get_json("/api/v1/diagnostics/tick")
result = {
    "ok": True,
    "ws_stop_type": ws_stop_reply.get("type"),
    "estop_http_status": estop_status,
    "epoch_before": epoch_before,
    "epoch_estop": epoch_estop,
    "torque_refused_type": en_reply.get("type"),
    "torque_refused_code": en_reply.get("code"),
    "cleared_after_check": cleared,
    "owner_mode": tick_after.get("owner_mode"),
    "control_thread_lock_attempts": tick_after.get("control_thread_lock_attempts"),
    "control_thread_alloc_ops": tick_after.get("control_thread_alloc_ops"),
}
with open(out_path, "w", encoding="utf-8") as f:
    json.dump(result, f, indent=2)
print(json.dumps(result))
PY
echo "      stop/E-stop check: PASS"

# ------------------------------------------------------------------------------
# Step 5: Write combined summary.md
# ------------------------------------------------------------------------------
python3 - "$RUN_DIR" "$RUN_ID" "$MODE" "$PROFILE" "$IDLE_S" "$MOTION_S" "$SPEED_SCALE" <<'PY'
import json, pathlib, sys
run_dir = pathlib.Path(sys.argv[1])
run_id, mode, profile, idle_s, motion_s, speed_scale = sys.argv[2:]
cold = json.loads((run_dir / "cold_starts.json").read_text(encoding="utf-8"))
estop = json.loads((run_dir / "estop_check.json").read_text(encoding="utf-8"))
idle_rep = (run_dir / "idle/report.md").read_text(encoding="utf-8")
motion_rep = (run_dir / "motion/report.md").read_text(encoding="utf-8")

def first_verdict(md):
    for line in md.splitlines():
        if line.startswith("**Overall Verdict:"):
            return line
    return "**Overall Verdict: unknown**"

idle_summary = json.loads((run_dir / "idle/load.jsonl").read_text(encoding="utf-8").splitlines()[-1])
motion_summary = json.loads((run_dir / "motion/load.jsonl").read_text(encoding="utf-8").splitlines()[-1])

lines = [
    f"# RT Executor Qualification Summary (`{run_id}`)",
    "",
    f"- **Mode**: `{mode}` | **Profile**: `{profile}` (idle `{idle_s}s`, motion `{motion_s}s`, `speed_scale={speed_scale}`)",
    f"- **Cold starts**: `{cold['count']}` passed (`distinct_instance_ids={cold['distinct_instance_ids']}`, all `owner_mode=executor`, `safety_state=Idle`)",
    f"- **Idle soak**: {first_verdict(idle_rep)} | `stop_reason={idle_summary.get('stop_reason')}` | `fault={idle_summary.get('fault')}`",
    f"- **J1 motion load**: {first_verdict(motion_rep)} | `stop_reason={motion_summary.get('stop_reason')}` | `fault={motion_summary.get('fault')}` | `cycles={motion_summary.get('counters', {}).get('motion_cycles_done')}`",
    f"- **Stop / E-stop check**: `ok={estop['ok']}`, WS stop `{estop['ws_stop_type']}`, E-stop HTTP `{estop['estop_http_status']}` (`epoch {estop['epoch_before']} -> {estop['epoch_estop']}`), torque refused `{estop['torque_refused_type']}` (`{estop['torque_refused_code']}`), cleared=`{estop['cleared_after_check']}`",
    "",
    "## Artifacts",
    "",
    "- `cold_starts.json`",
    "- `idle/load.jsonl`, `idle/report.md`",
    "- `motion/load.jsonl`, `motion/report.md`",
    "- `estop_check.json`",
    "",
    "## Panic / Teardown Tests (Covered in Sim)",
    "",
    "Panic and teardown tests that alter hold targets run on mock / Damiao `FakeBus` only and are **covered in sim**:",
    "- Control-thread panic -> SAFE-HOLD (`test_drv_f1_lifecycle`, `test_drv_f1_alloc`, `test_rt_executor_hardening`)",
    "- Non-control panic -> stop, hold, park-to-rest, disable at confirmed rest (`test_drv_f1_lifecycle`)",
    "- Energized handback & startup-failure legacy fallback (`test_drv_f1_lifecycle`, `test_g_real_follower`)",
    "- Thermal disable & deliberate ProcessExit fallback (`test_drv_f1_lifecycle`, `test_rt_executor_hardening`)",
    "- Damiao `FakeBus` executor E2E: enable, J1 motion, rest-pose torque-off (`test_g_real_follower`)",
    "",
]
(run_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")
print("\n".join(lines[:10]))
PY

# Fail non-zero if either report hit FAIL or INVALID.
for rep in "${RUN_DIR}/idle/report.md" "${RUN_DIR}/motion/report.md"; do
    if grep -Eq '^\*\*Overall Verdict: (FAIL|INVALID)' "$rep"; then
        echo "Qualification FAILED: $(grep -m1 '^\*\*Overall Verdict:' "$rep") in $rep" >&2
        exit 1
    fi
done

echo "====================================================================="
echo "RT Executor Qualification complete: ${RUN_DIR}/summary.md"
echo "====================================================================="
