#!/bin/sh
# Host-side (and device-side) test of kits/recamera-b601dm/packaging/rootfs/usr/share/rebot-teleop/rebot-vision-run.
# Runs every case under /bin/sh and, when installed, under `busybox ash` (the reCamera shell).
# Timings are shortened with REBOT_VISION_* variables.
# Usage: scripts/test_vision_runner.sh
HERE=$(cd "$(dirname "$0")" && pwd)
KIT=$HERE/../kits/recamera-b601dm/packaging/rootfs
RUNNER=$KIT/usr/share/rebot-teleop/rebot-vision-run
K92=$KIT/etc/init.d/K92rebot-teleop
FAILS=0
PASSES=0

pass() { PASSES=$((PASSES + 1)); echo "  ok   $*"; }
fail() { FAILS=$((FAILS + 1)); echo "  FAIL $*"; }
check() { # check <description> <command...>
    d=$1; shift
    if "$@" >/dev/null 2>&1; then pass "$d"; else fail "$d"; fi
}

wait_for() { # wait_for <seconds> <command...>
    w=$(($1 * 10)); shift
    while [ "$w" -gt 0 ]; do
        "$@" >/dev/null 2>&1 && return 0
        sleep 0.1
        w=$((w - 1))
    done
    return 1
}

line_count_ge() {
    [ "$(wc -l < "$1" 2>/dev/null || echo 0)" -ge "$2" ]
}

run_suite() {
    SH=$1
    echo "== $SH"
    T=$(mktemp -d) || exit 1
    CONF="$T/vision.json"
    : > "$CONF"

    # 1. Self-requested SIGTERM (same path as venc.c:937 kill(getpid(), SIGTERM) -> clean exit 0)
    #    must be restarted by the runner within the initial 1 s backoff.
    BIN="$T/self-term"
    LOG="$T/self-term.log"
    PIDS="$T/self-term.pids"
    cat <<EOF > "$BIN"
#!/bin/sh
echo "\$\$" >> "$PIDS"
trap 'exit 0' TERM
if [ "\$(wc -l < "$PIDS")" -eq 1 ]; then
    sleep 0.2
    kill -TERM "\$\$"
fi
while :; do sleep 0.2; done
EOF
    chmod 0755 "$BIN"
    REBOT_VISION_BIN="$BIN" REBOT_VISION_CONF="$CONF" REBOT_VISION_LOG="$LOG" \
        REBOT_TELEOP_INIT=/nonexistent \
        $SH "$RUNNER" &
    RPID=$!
    check "self-TERM: restarted after clean self-shutdown" wait_for 8 line_count_ge "$PIDS" 2
    check "self-TERM: runner still alive" kill -0 "$RPID"
    check "self-TERM: restart logged with exit 0" grep -q "rebot-vision exited (0); restarting in 1s" "$LOG"
    P1=$(sed -n '1p' "$PIDS" 2>/dev/null)
    P2=$(sed -n '2p' "$PIDS" 2>/dev/null)
    check "self-TERM: new child PID ($P1 -> $P2)" test -n "$P1" -a -n "$P2" -a "$P1" != "$P2"
    kill -TERM "$RPID" 2>/dev/null
    check "self-TERM: explicit SIGTERM stops runner cleanly" wait_for 6 sh -c "! kill -0 $RPID 2>/dev/null"
    wait "$RPID" 2>/dev/null
    RC=$?
    check "self-TERM: runner exited 0 on stop (rc=$RC)" test "$RC" -eq 0
    check "self-TERM: child stopped too" wait_for 4 sh -c "! kill -0 $P2 2>/dev/null"

    # 2. Process-group SIGTERM from the child (kill -TERM 0) must not reach the runner
    #    because setsid isolates the child in its own session/process group.
    if command -v setsid >/dev/null 2>&1; then
        BIN="$T/pg-term"
        LOG="$T/pg-term.log"
        PIDS="$T/pg-term.pids"
        cat <<EOF > "$BIN"
#!/bin/sh
echo "\$\$" >> "$PIDS"
trap 'exit 0' TERM
if [ "\$(wc -l < "$PIDS")" -eq 1 ]; then
    sleep 0.2
    kill -TERM 0
fi
while :; do sleep 0.2; done
EOF
        chmod 0755 "$BIN"
        REBOT_VISION_BIN="$BIN" REBOT_VISION_CONF="$CONF" REBOT_VISION_LOG="$LOG" \
            REBOT_TELEOP_INIT=/nonexistent \
            $SH "$RUNNER" &
        RPID=$!
        check "pgrp-TERM: child isolated by setsid and restarted" wait_for 8 line_count_ge "$PIDS" 2
        check "pgrp-TERM: runner survived child kill -TERM 0" kill -0 "$RPID"
        kill -TERM "$RPID" 2>/dev/null
        wait_for 6 sh -c "! kill -0 $RPID 2>/dev/null"
        wait "$RPID" 2>/dev/null
    fi

    # 3. Repeated crashes trigger exponential backoff and give up after GIVEUP_STARTS in GIVEUP_WINDOW.
    BIN="$T/crash"
    LOG="$T/crash.log"
    PIDS="$T/crash.pids"
    cat <<EOF > "$BIN"
#!/bin/sh
echo "\$\$" >> "$PIDS"
exit 7
EOF
    chmod 0755 "$BIN"
    t0=$(date +%s)
    REBOT_VISION_BIN="$BIN" REBOT_VISION_CONF="$CONF" REBOT_VISION_LOG="$LOG" \
        REBOT_TELEOP_INIT=/nonexistent \
        REBOT_VISION_BACKOFF_MAX=2 REBOT_VISION_GIVEUP_STARTS=3 REBOT_VISION_GIVEUP_WINDOW=60 \
        timeout 20 $SH "$RUNNER"
    RC=$?
    t1=$(date +%s)
    check "give-up: runner exits 1 (rc=$RC)" test "$RC" -eq 1
    check "give-up: started exactly 3 times" test "$(wc -l < "$PIDS")" -eq 3
    check "give-up: backoff 1s then 2s logged" sh -c "grep -q 'exited (7); restarting in 1s' '$LOG' && grep -q 'exited (7); restarting in 2s' '$LOG'"
    check "give-up: waited >= 3s across backoff" test $((t1 - t0)) -ge 3
    check "give-up: give-up message logged" grep -q "3 starts within 60s; giving up" "$LOG"

    # 4. SIGTERM during backoff stops the runner promptly without restarting.
    BIN="$T/backoff-stop"
    LOG="$T/backoff-stop.log"
    PIDS="$T/backoff-stop.pids"
    cat <<EOF > "$BIN"
#!/bin/sh
echo "\$\$" >> "$PIDS"
exit 2
EOF
    chmod 0755 "$BIN"
    REBOT_VISION_BIN="$BIN" REBOT_VISION_CONF="$CONF" REBOT_VISION_LOG="$LOG" \
        REBOT_TELEOP_INIT=/nonexistent \
        REBOT_VISION_BACKOFF_MAX=30 REBOT_VISION_GIVEUP_STARTS=10 \
        $SH "$RUNNER" &
    RPID=$!
    # Wait for first restart (1s) then second exit enters 2s backoff -> let's stop during 2s or 4s backoff
    check "backoff-stop: entered 4s backoff" wait_for 8 grep -q "restarting in 4s" "$LOG"
    N_BEFORE=$(wc -l < "$PIDS")
    t0=$(date +%s)
    kill -TERM "$RPID" 2>/dev/null
    check "backoff-stop: runner exited promptly" wait_for 3 sh -c "! kill -0 $RPID 2>/dev/null"
    t1=$(date +%s)
    wait "$RPID" 2>/dev/null
    RC=$?
    N_AFTER=$(wc -l < "$PIDS")
    check "backoff-stop: exited 0 within 2s (dt=$((t1 - t0))s, rc=$RC)" test "$RC" -eq 0 -a $((t1 - t0)) -le 2
    check "backoff-stop: no extra start ($N_BEFORE == $N_AFTER)" test "$N_BEFORE" -eq "$N_AFTER"

    rm -rf "$T"
}

echo "== syntax"
for f in "$RUNNER" "$K92"; do
    check "sh -n $(basename "$f")" sh -n "$f"
    if command -v busybox >/dev/null 2>&1; then
        check "busybox ash -n $(basename "$f")" busybox ash -n "$f"
    fi
done

run_suite /bin/sh
if command -v busybox >/dev/null 2>&1 && busybox ash -c 'true' >/dev/null 2>&1; then
    run_suite "busybox ash"
fi

echo
echo "$PASSES passed, $FAILS failed"
[ "$FAILS" -eq 0 ]
