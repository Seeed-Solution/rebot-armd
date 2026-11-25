#!/bin/sh
# Capture Linux ftrace sched_switch + sched_wakeup events for N seconds on device.
# Busybox sh compatible: no bashisms, no arrays, no [[ ]], no process substitution.
# Read-only outside ftrace control files: makes NO sysctl changes.
# Always restores tracing_on=0 (and prior sched event enable states) on exit.
# Exits 2 with a clear message if ftrace is absent or not writable.

set -u

DURATION=10
BUFFER_SIZE_KB=4096
OUT_FILE=""
TRACE_DIR=""

usage() {
    cat <<'EOF'
Usage: sched_trace.sh [OPTIONS] [DURATION_S]
  -d, --duration S         Capture duration in seconds (default: 10)
  -o, --output FILE        Output file path (default: /userdata/rt-qual/sched_trace_<ts>.txt)
  -b, --buffer-size-kb KB  Per-CPU ftrace ring buffer size in KB (default: 4096)
  --trace-dir DIR          Explicit tracefs directory (default: /sys/kernel/tracing or /sys/kernel/debug/tracing)
  -h, --help               Show this help
EOF
}

is_pos_int() {
    case "$1" in
        ''|*[!0-9]*) return 1 ;;
        0) return 1 ;;
        *) return 0 ;;
    esac
}

while [ $# -gt 0 ]; do
    case "$1" in
        -d|--duration)
            [ $# -ge 2 ] || { echo "Missing value for $1" >&2; exit 2; }
            DURATION="$2"; shift 2 ;;
        -o|--output|--out)
            [ $# -ge 2 ] || { echo "Missing value for $1" >&2; exit 2; }
            OUT_FILE="$2"; shift 2 ;;
        -b|--buffer-size-kb|--buffer-kb)
            [ $# -ge 2 ] || { echo "Missing value for $1" >&2; exit 2; }
            BUFFER_SIZE_KB="$2"; shift 2 ;;
        --trace-dir)
            [ $# -ge 2 ] || { echo "Missing value for $1" >&2; exit 2; }
            TRACE_DIR="$2"; shift 2 ;;
        -h|--help)
            usage; exit 0 ;;
        -*)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 2 ;;
        *)
            DURATION="$1"; shift ;;
    esac
done

is_pos_int "$DURATION" || { echo "Invalid duration (expected positive integer seconds): $DURATION" >&2; exit 2; }
is_pos_int "$BUFFER_SIZE_KB" || { echo "Invalid buffer_size_kb (expected positive integer KB): $BUFFER_SIZE_KB" >&2; exit 2; }

if [ -n "$TRACE_DIR" ]; then
    CANDIDATES="$TRACE_DIR"
else
    CANDIDATES="/sys/kernel/tracing /sys/kernel/debug/tracing"
fi

RESOLVED_DIR=""
ABSENT_REASON=""
for d in $CANDIDATES; do
    if [ ! -d "$d" ]; then
        ABSENT_REASON="${ABSENT_REASON} $d(absent)"
        continue
    fi
    if [ ! -e "$d/tracing_on" ] || [ ! -e "$d/trace" ] \
        || [ ! -e "$d/events/sched/sched_switch/enable" ] \
        || [ ! -e "$d/events/sched/sched_wakeup/enable" ]; then
        ABSENT_REASON="${ABSENT_REASON} $d(incomplete_ftrace_files)"
        continue
    fi
    if [ ! -w "$d/tracing_on" ] \
        || [ ! -w "$d/events/sched/sched_switch/enable" ] \
        || [ ! -w "$d/events/sched/sched_wakeup/enable" ] \
        || [ ! -r "$d/trace" ]; then
        ABSENT_REASON="${ABSENT_REASON} $d(not_writable_or_unreadable)"
        continue
    fi
    RESOLVED_DIR="$d"
    break
done

if [ -z "$RESOLVED_DIR" ]; then
    echo "ftrace unavailable: required sched_switch/sched_wakeup tracefs controls not found or not writable (checked:${ABSENT_REASON}). Ensure tracefs/debugfs is mounted and run as root." >&2
    exit 2
fi

if [ -z "$OUT_FILE" ]; then
    TS=$(date -u +%Y%m%dT%H%M%SZ 2>/dev/null || date +%s)
    OUT_FILE="/userdata/rt-qual/sched_trace_${TS}.txt"
fi

OUT_PARENT=$(dirname "$OUT_FILE")
mkdir -p "$OUT_PARENT" || { echo "Failed to create output directory: $OUT_PARENT" >&2; exit 1; }

SW_EN="$RESOLVED_DIR/events/sched/sched_switch/enable"
WU_EN="$RESOLVED_DIR/events/sched/sched_wakeup/enable"
ORIG_SW=$(cat "$SW_EN" 2>/dev/null || echo 0)
ORIG_WU=$(cat "$WU_EN" 2>/dev/null || echo 0)
TRACING_ACTIVE=0

cleanup() {
    # Always ensure tracing_on=0 on exit and restore event enable flags.
    echo 0 > "$RESOLVED_DIR/tracing_on" 2>/dev/null || true
    if [ "$TRACING_ACTIVE" -eq 1 ]; then
        echo "${ORIG_SW:-0}" > "$SW_EN" 2>/dev/null || true
        echo "${ORIG_WU:-0}" > "$WU_EN" 2>/dev/null || true
    fi
}

trap cleanup EXIT INT TERM HUP

TRACING_ACTIVE=1
echo 0 > "$RESOLVED_DIR/tracing_on" || { echo "Failed to write $RESOLVED_DIR/tracing_on" >&2; exit 2; }
if [ -w "$RESOLVED_DIR/current_tracer" ]; then
    echo nop > "$RESOLVED_DIR/current_tracer" 2>/dev/null || true
fi
if [ -w "$RESOLVED_DIR/buffer_size_kb" ]; then
    echo "$BUFFER_SIZE_KB" > "$RESOLVED_DIR/buffer_size_kb" 2>/dev/null || true
fi
if [ -w "$RESOLVED_DIR/trace" ]; then
    printf '' > "$RESOLVED_DIR/trace" 2>/dev/null || true
fi

echo 1 > "$SW_EN" || { echo "Failed to enable sched_switch in $SW_EN" >&2; exit 2; }
echo 1 > "$WU_EN" || { echo "Failed to enable sched_wakeup in $WU_EN" >&2; exit 2; }

echo 1 > "$RESOLVED_DIR/tracing_on" || { echo "Failed to start tracing in $RESOLVED_DIR/tracing_on" >&2; exit 2; }
sleep "$DURATION" || true
echo 0 > "$RESOLVED_DIR/tracing_on" || true

cat "$RESOLVED_DIR/trace" > "$OUT_FILE" || { echo "Failed to read $RESOLVED_DIR/trace into $OUT_FILE" >&2; exit 1; }

BYTES=$(wc -c < "$OUT_FILE" 2>/dev/null | tr -d ' ')
LINES=$(wc -l < "$OUT_FILE" 2>/dev/null | tr -d ' ')
echo "sched_trace.sh finished: trace_dir=$RESOLVED_DIR file=$OUT_FILE duration_s=$DURATION buffer_size_kb=$BUFFER_SIZE_KB bytes=${BYTES:-0} lines=${LINES:-0}"
