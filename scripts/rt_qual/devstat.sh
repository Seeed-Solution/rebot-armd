#!/bin/sh
# SG2002 on-device system & attribution sampler for RT qualification (Stage 3 §6).
# Busybox sh compatible: no bashisms, no arrays, no [[ ]], no process substitution.
#
# Output: /userdata/rt-qual/<run-id>/devstat.txt (override with --out-dir / --run-id).
# Stops cleanly when --duration or --max-bytes is reached or on SIGINT/SIGTERM.

set -u

RUN_ID=""
OUT_DIR="/userdata/rt-qual"
INTERVAL=5
DURATION=300
MAX_BYTES=16777216
ARMD_PID=""
ARMD_URL="http://127.0.0.1:8765"
CAN_IFACE="can0"
STOP=0

usage() {
    cat <<'EOF'
Usage: devstat.sh [OPTIONS]
  --run-id ID        Run identifier (default: UTC timestamp YYYYMMDDTHHMMSSZ)
  --out-dir DIR      Base output directory (default: /userdata/rt-qual)
  --interval N       Sample interval in seconds (default: 5)
  --duration S       Maximum duration in seconds (default: 300)
  --max-bytes B      Maximum output file size in bytes (default: 16777216)
  --armd-pid PID     Explicit armd PID (default: auto-detect)
  --armd-url URL     armd HTTP base URL for version query (default: http://127.0.0.1:8765)
  --can-iface IFACE  CAN interface name (default: can0)
  --help             Show this help
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
        --run-id)
            [ $# -ge 2 ] || { echo "Missing value for $1" >&2; exit 2; }
            RUN_ID="$2"; shift 2 ;;
        --out-dir)
            [ $# -ge 2 ] || { echo "Missing value for $1" >&2; exit 2; }
            OUT_DIR="$2"; shift 2 ;;
        --interval)
            [ $# -ge 2 ] || { echo "Missing value for $1" >&2; exit 2; }
            INTERVAL="$2"; shift 2 ;;
        --duration)
            [ $# -ge 2 ] || { echo "Missing value for $1" >&2; exit 2; }
            DURATION="$2"; shift 2 ;;
        --max-bytes)
            [ $# -ge 2 ] || { echo "Missing value for $1" >&2; exit 2; }
            MAX_BYTES="$2"; shift 2 ;;
        --armd-pid)
            [ $# -ge 2 ] || { echo "Missing value for $1" >&2; exit 2; }
            ARMD_PID="$2"; shift 2 ;;
        --armd-url)
            [ $# -ge 2 ] || { echo "Missing value for $1" >&2; exit 2; }
            ARMD_URL="$2"; shift 2 ;;
        --can-iface)
            [ $# -ge 2 ] || { echo "Missing value for $1" >&2; exit 2; }
            CAN_IFACE="$2"; shift 2 ;;
        --help|-h)
            usage; exit 0 ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 2 ;;
    esac
done

is_pos_int "$INTERVAL" || { echo "Invalid --interval: $INTERVAL" >&2; exit 2; }
is_pos_int "$DURATION" || { echo "Invalid --duration: $DURATION" >&2; exit 2; }
is_pos_int "$MAX_BYTES" || { echo "Invalid --max-bytes: $MAX_BYTES" >&2; exit 2; }

if [ -z "$RUN_ID" ]; then
    RUN_ID=$(date -u +%Y%m%dT%H%M%SZ 2>/dev/null || date +%s)
fi

find_armd_pid() {
    if [ -n "$ARMD_PID" ] && [ -d "/proc/$ARMD_PID" ]; then
        echo "$ARMD_PID"
        return 0
    fi
    if command -v pidof >/dev/null 2>&1; then
        p=$(pidof armd 2>/dev/null | awk '{print $1}')
        if [ -n "$p" ]; then
            echo "$p"
            return 0
        fi
        p=$(pidof rebot-armd 2>/dev/null | awk '{print $1}')
        if [ -n "$p" ]; then
            echo "$p"
            return 0
        fi
    fi
    for c in /proc/[0-9]*/comm; do
        [ -r "$c" ] || continue
        name=$(cat "$c" 2>/dev/null)
        case "$name" in
            armd|rebot-armd)
                p=${c#/proc/}
                p=${p%/comm}
                echo "$p"
                return 0 ;;
        esac
    done
    echo ""
    return 1
}

mono_secs() {
    if [ -r /proc/uptime ]; then
        cut -d. -f1 /proc/uptime
    else
        date +%s
    fi
}

# Print one thread's scheduling info parsed from /proc/.../stat (handles parens/spaces in comm)
# and chrt -p if available.
print_thread_sched() {
    tid="$1"
    stat_path="$2"
    [ -r "$stat_path" ] || return 0
    line=$(cat "$stat_path" 2>/dev/null) || return 0
    # After the last ')', field 1 is state (#3 in proc(5)), field 17 is nice (#19),
    # field 38 is rt_priority (#40), field 39 is policy (#41).
    parsed=$(printf '%s\n' "$line" | awk '{
        i = index($0, "(")
        r = length($0)
        while (r > 0 && substr($0, r, 1) != ")") r--
        if (i == 0 || r <= i) next
        comm = substr($0, i + 1, r - i - 1)
        rest = substr($0, r + 2)
        n = split(rest, f, " ")
        nice = (n >= 17) ? f[17] : "?"
        rtprio = (n >= 38) ? f[38] : "?"
        pol = (n >= 39) ? f[39] : "?"
        polname = (pol == "0") ? "SCHED_OTHER" : (pol == "1") ? "SCHED_FIFO" : (pol == "2") ? "SCHED_RR" : (pol == "3") ? "SCHED_BATCH" : (pol == "5") ? "SCHED_IDLE" : (pol == "6") ? "SCHED_DEADLINE" : pol
        printf "comm=%s policy=%s(%s) rt_prio=%s nice=%s", comm, polname, pol, rtprio, nice
    }')
    chrt_out=""
    if command -v chrt >/dev/null 2>&1; then
        chrt_out=$(chrt -p "$tid" 2>/dev/null | tr '\n' ';' | sed 's/;*$//')
    fi
    if [ -n "$chrt_out" ]; then
        printf 'tid=%s %s chrt="%s"\n' "$tid" "$parsed" "$chrt_out"
    else
        printf 'tid=%s %s\n' "$tid" "$parsed"
    fi
}

dump_once() {
    pid="$1"
    echo "=== ONCE_START ==="
    echo "--- uname ---"
    uname -a 2>&1
    echo "--- armd_version ---"
    if command -v dpkg >/dev/null 2>&1; then
        dpkg -s rebot-armd 2>/dev/null | grep -E '^(Package|Version|Status):' || true
    fi
    if command -v opkg >/dev/null 2>&1; then
        opkg status rebot-armd 2>/dev/null | grep -E '^(Package|Version|Status):' || true
    fi
    if [ -n "$pid" ] && [ -r "/proc/$pid/cmdline" ]; then
        printf 'cmdline='
        tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null
        printf '\n'
    fi
    if [ -n "$pid" ] && [ -L "/proc/$pid/exe" ]; then
        printf 'exe=%s\n' "$(readlink "/proc/$pid/exe" 2>/dev/null || echo unknown)"
    fi
    if command -v curl >/dev/null 2>&1; then
        boots=$(curl -sf --max-time 2 "$ARMD_URL/api/v1/diagnostics/boots" 2>/dev/null || true)
        [ -n "$boots" ] && printf 'boots_json=%s\n' "$boots"
    elif command -v wget >/dev/null 2>&1; then
        boots=$(wget -qO- -T 2 "$ARMD_URL/api/v1/diagnostics/boots" 2>/dev/null || true)
        [ -n "$boots" ] && printf 'boots_json=%s\n' "$boots"
    fi

    echo "--- kernel_config ---"
    if [ -r /proc/config.gz ] && command -v zcat >/dev/null 2>&1; then
        zcat /proc/config.gz 2>/dev/null | grep -E '^CONFIG_(PREEMPT|HZ=|HZ_|NO_HZ|HIGH_RES_TIMERS|IRQ_FORCED_THREADING|SMP=|NR_CPUS=)' || true
    elif [ -r /proc/config.gz ] && command -v gzip >/dev/null 2>&1; then
        gzip -dc /proc/config.gz 2>/dev/null | grep -E '^CONFIG_(PREEMPT|HZ=|HZ_|NO_HZ|HIGH_RES_TIMERS|IRQ_FORCED_THREADING|SMP=|NR_CPUS=)' || true
    elif [ -r "/boot/config-$(uname -r)" ]; then
        grep -E '^CONFIG_(PREEMPT|HZ=|HZ_|NO_HZ|HIGH_RES_TIMERS|IRQ_FORCED_THREADING|SMP=|NR_CPUS=)' "/boot/config-$(uname -r)" || true
    else
        echo "unavailable (/proc/config.gz absent)"
    fi

    echo "--- armd_threads ---"
    if [ -n "$pid" ] && [ -d "/proc/$pid/task" ]; then
        for tdir in /proc/"$pid"/task/*; do
            [ -d "$tdir" ] || continue
            tid=${tdir##*/}
            print_thread_sched "$tid" "$tdir/stat"
        done
    else
        echo "armd_pid_not_found"
    fi

    echo "--- irq_and_rt_threads ---"
    for sfile in /proc/[0-9]*/stat; do
        [ -r "$sfile" ] || continue
        pdir=${sfile%/stat}
        p=${pdir#/proc/}
        comm=""
        [ -r "$pdir/comm" ] && comm=$(cat "$pdir/comm" 2>/dev/null)
        # Include all irq/*, spi*, ksoftirqd*, and any thread running with RT policy (policy != 0).
        want=0
        case "$comm" in
            irq/*|spi*|ksoftirqd*|mcp251*|mmc*|cviusb*) want=1 ;;
        esac
        if [ "$want" -eq 0 ]; then
            pol=$(awk '{
                r = length($0); while (r > 0 && substr($0, r, 1) != ")") r--
                rest = substr($0, r + 2); n = split(rest, f, " ")
                if (n >= 39) print f[39]
            }' "$sfile" 2>/dev/null)
            case "$pol" in
                1|2|6) want=1 ;;
            esac
        fi
        if [ "$want" -eq 1 ]; then
            print_thread_sched "$p" "$sfile"
        fi
    done
    echo "=== ONCE_END ==="
}

dump_sample() {
    idx="$1"
    pid="$2"
    ts_unix=$(date +%s 2>/dev/null || echo 0)
    ts_iso=$(date -u +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo unknown)
    up=$(cat /proc/uptime 2>/dev/null | awk '{print $1}')

    echo "=== SAMPLE $idx ts_unix=$ts_unix ts_iso=$ts_iso uptime=$up ==="
    echo "--- loadavg ---"
    cat /proc/loadavg 2>/dev/null || echo "unavailable"

    echo "--- interrupts ---"
    cat /proc/interrupts 2>/dev/null || echo "unavailable"

    echo "--- softirqs ---"
    cat /proc/softirqs 2>/dev/null || echo "unavailable"

    echo "--- schedstat ---"
    if [ -r /proc/schedstat ]; then
        cat /proc/schedstat 2>/dev/null
    else
        echo "unavailable"
    fi

    echo "--- can_link ---"
    if command -v ip >/dev/null 2>&1; then
        ip -s -d link show "$CAN_IFACE" 2>&1 || true
    else
        echo "ip_command_unavailable"
    fi

    echo "--- armd_tasks ---"
    if [ -n "$pid" ] && [ -d "/proc/$pid/task" ]; then
        for tdir in /proc/"$pid"/task/*; do
            [ -d "$tdir" ] || continue
            tid=${tdir##*/}
            comm=$(cat "$tdir/comm" 2>/dev/null || echo unknown)
            echo "TASK tid=$tid comm=$comm"
            if [ -r "$tdir/stat" ]; then
                printf 'stat: %s\n' "$(cat "$tdir/stat" 2>/dev/null)"
            fi
            if [ -r "$tdir/status" ]; then
                grep -E '^(Name|State|Tgid|Pid|VmSize|VmRSS|VmLck|VmPin|voluntary_ctxt_switches|nonvoluntary_ctxt_switches):' "$tdir/status" 2>/dev/null || true
            fi
            if [ -r "$tdir/sched" ]; then
                grep -E '^(se\.sum_exec_runtime|se\.nr_migrations|se\.statistics\.wait_sum|se\.statistics\.wait_max|se\.statistics\.iowait_sum|nr_switches|nr_voluntary_switches|nr_involuntary_switches|policy|prio)[[:space:]]*:' "$tdir/sched" 2>/dev/null || true
            fi
        done
    else
        echo "armd_pid_not_found"
    fi

    echo "--- irq_tasks ---"
    for cfile in /proc/[0-9]*/comm; do
        [ -r "$cfile" ] || continue
        comm=$(cat "$cfile" 2>/dev/null)
        case "$comm" in
            irq/*|spi*|ksoftirqd*)
                pdir=${cfile%/comm}
                p=${pdir#/proc/}
                ut_st=$(awk '{
                    r = length($0); while (r > 0 && substr($0, r, 1) != ")") r--
                    rest = substr($0, r + 2); n = split(rest, f, " ")
                    if (n >= 13) printf "utime=%s stime=%s", f[12], f[13]
                }' "$pdir/stat" 2>/dev/null)
                exec_ms=""
                if [ -r "$pdir/sched" ]; then
                    exec_ms=$(awk -F: '/^se\.sum_exec_runtime[[:space:]]*:/ {gsub(/[[:space:]]/, "", $2); print $2; exit}' "$pdir/sched" 2>/dev/null)
                fi
                printf 'tid=%s comm=%s %s sum_exec_ms=%s\n' "$p" "$comm" "$ut_st" "${exec_ms:-na}"
                ;;
        esac
    done
    echo "=== SAMPLE_END ==="
}

RUN_DIR="$OUT_DIR/$RUN_ID"
OUT_FILE="$RUN_DIR/devstat.txt"
mkdir -p "$RUN_DIR" || { echo "Failed to create $RUN_DIR" >&2; exit 1; }

trap 'STOP=1' INT TERM HUP

DETECTED_PID=$(find_armd_pid || true)
START_MONO=$(mono_secs)
START_UNIX=$(date +%s 2>/dev/null || echo 0)
START_ISO=$(date -u +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo unknown)

{
    echo "=== META ==="
    echo "run_id=$RUN_ID"
    echo "started_unix=$START_UNIX"
    echo "started_iso=$START_ISO"
    echo "interval_s=$INTERVAL"
    echo "duration_s=$DURATION"
    echo "max_bytes=$MAX_BYTES"
    echo "can_iface=$CAN_IFACE"
    echo "armd_pid=${DETECTED_PID:-none}"
    dump_once "$DETECTED_PID"
} > "$OUT_FILE"

SAMPLE_IDX=0
STOP_REASON="duration_cap"

while [ "$STOP" -eq 0 ]; do
    # Re-detect armd PID if it was missing or restarted.
    if [ -z "$DETECTED_PID" ] || [ ! -d "/proc/$DETECTED_PID" ]; then
        DETECTED_PID=$(find_armd_pid || true)
    fi

    dump_sample "$SAMPLE_IDX" "$DETECTED_PID" >> "$OUT_FILE"
    SAMPLE_IDX=$((SAMPLE_IDX + 1))

    cur_bytes=$(wc -c < "$OUT_FILE" 2>/dev/null || echo 0)
    cur_bytes=$(echo "$cur_bytes" | tr -d ' ')
    if [ "$cur_bytes" -ge "$MAX_BYTES" ]; then
        STOP_REASON="max_bytes"
        break
    fi

    now_mono=$(mono_secs)
    elapsed=$((now_mono - START_MONO))
    if [ "$elapsed" -ge "$DURATION" ]; then
        STOP_REASON="duration_cap"
        break
    fi

    rem=$((DURATION - elapsed))
    step="$INTERVAL"
    if [ "$rem" -lt "$step" ]; then
        step="$rem"
    fi
    sleep "$step" || true
done

if [ "$STOP" -ne 0 ]; then
    STOP_REASON="signal"
fi

END_UNIX=$(date +%s 2>/dev/null || echo 0)
END_ISO=$(date -u +%Y-%m-%dT%H:%M:%SZ 2>/dev/null || echo unknown)
FINAL_BYTES=$(wc -c < "$OUT_FILE" 2>/dev/null | tr -d ' ')

{
    echo "=== END ==="
    echo "ended_unix=$END_UNIX"
    echo "ended_iso=$END_ISO"
    echo "samples=$SAMPLE_IDX"
    echo "bytes=$FINAL_BYTES"
    echo "stop_reason=$STOP_REASON"
} >> "$OUT_FILE"

echo "devstat.sh finished: file=$OUT_FILE samples=$SAMPLE_IDX bytes=$FINAL_BYTES reason=$STOP_REASON"
