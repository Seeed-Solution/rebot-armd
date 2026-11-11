#!/bin/sh
# Host-side test of kits/recamera-b601dm/packaging/rootfs/usr/share/rebot-teleop/rebot-app-run.
# Runs every case under /bin/sh and, when installed, under `busybox sh` (the reCamera shell).
# Needs python3 and timeout. Timings are shortened with the runner's REBOT_APP_* variables.
# Not root: the program runs as the current user (REBOT_APP_USER); as root it runs as nobody.
# Usage: scripts/test_app_runner.sh
HERE=$(cd "$(dirname "$0")" && pwd)
KIT=$HERE/../kits/recamera-b601dm/packaging/rootfs
RUNNER=$KIT/usr/share/rebot-teleop/rebot-app-run
K92=$KIT/etc/init.d/K92rebot-teleop
FAILS=0
PASSES=0
if [ "$(id -u)" -eq 0 ]; then APP_USER=nobody; else APP_USER=$(id -un); fi

pass() { PASSES=$((PASSES + 1)); echo "  ok   $*"; }
fail() { FAILS=$((FAILS + 1)); echo "  FAIL $*"; }
check() { # check <description> <command...>
    d=$1; shift
    if "$@" >/dev/null 2>&1; then pass "$d"; else fail "$d"; fi
}
state_get() { sed -n "s/^$2=//p" "$T/state/rebot-app-$1.state" 2>/dev/null; }
log_has() { grep -q -- "$2" "$T/logs/app-$1.log"; }

# new_app <name> <main.py text>: fresh temp tree
new_app() {
    mkdir -p "$T/apps/$1" "$T/logs" "$T/state"
    printf '%s\n' "$2" > "$T/apps/$1/main.py"
}
# run_fg <name> [env assignments...]: runner in the foreground, at most 30 s; status in RC
run_fg() {
    n=$1; shift
    env REBOT_APPS_DIR="$T/apps" REBOT_APP_LOG_DIR="$T/logs" REBOT_APP_STATE_DIR="$T/state" \
        REBOT_APP_PYTHONPATH=/usr/share/rebot-teleop/python REBOT_APP_USER="$APP_USER" "$@" timeout 30 $SH "$RUNNER" "$n"
    RC=$?
}
# run_bg <name> [env assignments...]: runner in the background, its pid in RPID
run_bg() {
    n=$1; shift
    env REBOT_APPS_DIR="$T/apps" REBOT_APP_LOG_DIR="$T/logs" REBOT_APP_STATE_DIR="$T/state" \
        REBOT_APP_PYTHONPATH=/usr/share/rebot-teleop/python REBOT_APP_USER="$APP_USER" "$@" $SH "$RUNNER" "$n" &
    RPID=$!
}
# wait_for <seconds> <command...>: poll every 0.1 s
wait_for() {
    w=$(($1 * 10)); shift
    while [ "$w" -gt 0 ]; do
        "$@" >/dev/null 2>&1 && return 0
        sleep 0.1
        w=$((w - 1))
    done
    return 1
}
alive() { kill -0 "$1" 2>/dev/null; }
state_is() { [ "$(state_get "$1" state)" = "$2" ]; }
starts_ge() { [ "$(state_get "$1" starts)" -ge "$2" ] 2>/dev/null; }
log_grows() { log_has "$1" "$2"; }

run_suite() {
    SH=$1
    echo "== $SH"
    T=$(mktemp -d) || exit 1

    # 1. exit code 0 with the default restart policy (on-failure): runs once, then stops
    new_app clean 'print("hello from clean")'
    run_fg clean
    check "exit 0: runner exits 0" [ "$RC" -eq 0 ]
    check "exit 0: state stopped" state_is clean stopped
    check "exit 0: one start, last_exit 0" [ "$(state_get clean starts)-$(state_get clean last_exit)" = 1-0 ]
    check "exit 0: output in the log" log_has clean "hello from clean"

    # 2. crash on every start: backoff, then give up after 3 starts
    new_app crash 'import sys; print("boom"); sys.exit(3)'
    t0=$(date +%s)
    run_fg crash REBOT_APP_BACKOFF_MAX=1 REBOT_APP_GIVEUP_STARTS=3
    t1=$(date +%s)
    check "crash: runner exits 1" [ "$RC" -eq 1 ]
    check "crash: state given_up" state_is crash given_up
    check "crash: 3 starts, last_exit 3" [ "$(state_get crash starts)-$(state_get crash last_exit)" = 3-3 ]
    check "crash: backoff waited (>= 2 s for 2 restarts)" [ $((t1 - t0)) -ge 2 ]
    check "crash: give-up line in the log" log_has crash "giving up"

    # 3. restart=never in app.json, entry other than main.py
    new_app never 'print("main.py must not run")'
    printf '%s\n' 'import sys; print("custom entry"); sys.exit(1)' > "$T/apps/never/run.py"
    printf '%s\n' '{"entry": "run.py", "restart": "never"}' > "$T/apps/never/app.json"
    run_fg never REBOT_APP_BACKOFF_MAX=1
    check "never: runs entry from app.json" log_has never "custom entry"
    check "never: main.py not run" sh -c "! grep -q 'main.py must not run' '$T/logs/app-never.log'"
    check "never: one start, state stopped" [ "$(state_get never starts)-$(state_get never state)" = 1-stopped ]

    # 4. restart=always restarts after exit 0; SIGTERM during backoff stops it
    new_app always 'print("again")'
    printf '%s\n' '{"restart": "always"}' > "$T/apps/always/app.json"
    run_bg always REBOT_APP_BACKOFF_MAX=1
    check "always: restarted after exit 0" wait_for 10 starts_ge always 2
    kill -TERM "$RPID"
    check "always: runner exits after SIGTERM" wait_for 5 sh -c "! kill -0 $RPID"
    wait "$RPID" 2>/dev/null
    check "always: state stopped" state_is always stopped

    # 5. environment and working directory of the program
    new_app envapp 'import os, sys; print("HOST=" + os.environ["REBOT_HOST"]); print("PP=" + os.environ["PYTHONPATH"]); print("CWD=" + os.getcwd())'
    run_fg envapp
    check "env: REBOT_HOST=127.0.0.1" log_has envapp "HOST=127.0.0.1$"
    check "env: PYTHONPATH has the SDK dir and the app dir" log_has envapp "PP=/usr/share/rebot-teleop/python:$T/apps/envapp$"
    check "env: cwd is the app directory" log_has envapp "CWD=.*apps/envapp$"

    # 6. SIGTERM reaches the program, which exits on it
    new_app term 'import time, sys; print("up", flush=True); time.sleep(60)'
    run_bg term
    check "term: program running" wait_for 10 state_is term running
    CPID=$(state_get term pid)
    t0=$(date +%s)
    kill -TERM "$RPID"
    check "term: runner exits" wait_for 5 sh -c "! kill -0 $RPID"
    t1=$(date +%s)
    wait "$RPID" 2>/dev/null
    check "term: program gone" sh -c "! kill -0 $CPID"
    check "term: stopped within 3 s" [ $((t1 - t0)) -le 3 ]
    check "term: state stopped" state_is term stopped

    # 7. a program that ignores SIGTERM is sent SIGKILL after the grace time (2 s here)
    new_app stubborn 'import signal, time; signal.signal(signal.SIGTERM, signal.SIG_IGN); print("ignoring", flush=True); time.sleep(60)'
    run_bg stubborn REBOT_APP_KILL_GRACE=2
    check "stubborn: program running" wait_for 10 log_has stubborn ignoring
    CPID=$(state_get stubborn pid)
    t0=$(date +%s)
    kill -TERM "$RPID"
    check "stubborn: still alive 1 s after SIGTERM" sh -c "sleep 1; kill -0 $CPID"
    check "stubborn: runner exits" wait_for 8 sh -c "! kill -0 $RPID"
    t1=$(date +%s)
    wait "$RPID" 2>/dev/null
    check "stubborn: program killed" sh -c "! kill -0 $CPID"
    check "stubborn: stopped after the grace time (2 to 5 s)" sh -c "[ $((t1 - t0)) -ge 2 ] && [ $((t1 - t0)) -le 5 ]"
    check "stubborn: state stopped" state_is stubborn stopped

    # 8. log rotation at start: > LOG_MAX is copied to .1, then .1 to .2; no .3
    new_app rot 'import sys; sys.stdout.write("x" * 2000 + "\n"); sys.exit(1)'
    run_fg rot REBOT_APP_BACKOFF_MAX=1 REBOT_APP_GIVEUP_STARTS=5 REBOT_APP_LOG_MAX=1000
    check "rotate: app-rot.log.1 exists" [ -f "$T/logs/app-rot.log.1" ]
    check "rotate: app-rot.log.2 exists" [ -f "$T/logs/app-rot.log.2" ]
    check "rotate: no .3" [ ! -e "$T/logs/app-rot.log.3" ]

    # 9. log rotation while the program runs (check interval 1 s)
    mkdir -p "$T/apps/rotrun" "$T/logs" "$T/state"
    printf 'import time\nfor i in range(20):\n    print("y" * 300, flush=True)\n    time.sleep(0.2)\ntime.sleep(30)\n' > "$T/apps/rotrun/main.py"
    run_bg rotrun REBOT_APP_LOG_MAX=1000 REBOT_APP_LOG_CHECK=1
    check "rotate while running: .1 appears" wait_for 10 test -f "$T/logs/app-rotrun.log.1"
    kill -TERM "$RPID"; wait "$RPID" 2>/dev/null

    # 10. bad input
    run_fg missing
    check "missing app directory: runner exits 1" [ "$RC" -eq 1 ]
    run_fg '../x'
    check "invalid name: runner exits 2" [ "$RC" -eq 2 ]

    # 10b. app.json values are not glob-expanded (a file named entry=other.py in the cwd)
    new_app glob 'print("must not run")'
    printf '%s\n' 'print("literal entry")' > "$T/apps/glob/o*.py"
    printf '%s\n' 'print("globbed entry")' > "$T/apps/glob/other.py"
    printf '%s\n' '{"entry": "o*.py", "restart": "never"}' > "$T/apps/glob/app.json"
    mkdir -p "$T/cwd" && : > "$T/cwd/entry=other.py"
    (cd "$T/cwd" && run_fg glob)
    check "manifest: entry taken literally" log_has glob "literal entry"
    check "manifest: no glob expansion" sh -c "! grep -q 'globbed entry' '$T/logs/app-glob.log'"

    # 11. program user: REBOT_APP_USER; unknown user or root refuses the start
    new_app who 'import os; print("UID=%d HOME=%s" % (os.getuid(), os.environ["HOME"]))'
    run_fg who
    check "user: runs as REBOT_APP_USER" log_has who "UID=$(id -u "$APP_USER") "
    new_app nouser 'print("must not run")'
    run_fg nouser REBOT_APP_USER=no_such_user_rebot
    check "unknown user: runner exits 1" [ "$RC" -eq 1 ]
    check "unknown user: logged" log_has nouser "user no_such_user_rebot does not exist"
    check "unknown user: program not run" sh -c "! grep -q 'must not run' '$T/logs/app-nouser.log'"
    check "unknown user: state stopped" state_is nouser stopped
    new_app asroot 'print("must not run")'
    run_fg asroot REBOT_APP_USER=root
    check "root user: refused" sh -c "[ $RC -eq 1 ] && grep -q 'is root; not starting' '$T/logs/app-asroot.log' && ! grep -q 'must not run' '$T/logs/app-asroot.log'"

    # 12. process group: stop also ends the processes the program started
    new_app group "import subprocess, time
p = subprocess.Popen(['sleep', '60'])
open('$T/group.child', 'w').write(str(p.pid))
print('spawned', flush=True)
time.sleep(60)"
    run_bg group
    check "group: program and its child running" wait_for 10 test -s "$T/group.child"
    GPID=$(cat "$T/group.child" 2>/dev/null)
    check "group: program is a session leader" sh -c "[ \"\$(ps -o sid= -p $(state_get group pid) | tr -d ' ')\" = $(state_get group pid) ]"
    kill -TERM "$RPID"
    check "group: runner exits" wait_for 5 sh -c "! kill -0 $RPID"
    wait "$RPID" 2>/dev/null
    check "group: the program's child is gone after stop" wait_for 3 sh -c "! kill -0 $GPID"
    # ... and after the program exits on its own
    new_app orphan "import subprocess, sys
p = subprocess.Popen(['sleep', '60'])
open('$T/orphan.child', 'w').write(str(p.pid))
sys.exit(4)"
    printf '%s\n' '{"restart": "never"}' > "$T/apps/orphan/app.json"
    run_fg orphan
    OPID=$(cat "$T/orphan.child" 2>/dev/null)
    check "group: a child left by an exited program is killed" wait_for 3 sh -c "[ -n '$OPID' ] && ! kill -0 '$OPID'"

    rm -rf "$T"
}

echo "== sh -n"
for f in "$RUNNER" "$K92" "$KIT/../debian/postinst" "$KIT/../debian/prerm"; do
    check "sh -n $(basename "$f")" sh -n "$f"
    if command -v busybox >/dev/null 2>&1 && busybox sh -c : 2>/dev/null; then
        check "busybox sh -n $(basename "$f")" busybox sh -n "$f"
    fi
done

run_suite /bin/sh
if command -v busybox >/dev/null 2>&1 && busybox sh -c : 2>/dev/null; then
    run_suite "busybox sh"
else
    echo "== busybox not found; skipped"
fi

echo "passed: $PASSES, failed: $FAILS"
[ "$FAILS" -eq 0 ]
