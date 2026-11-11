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
    # as root the program runs as nobody: it reads the apps and writes its marker files here
    [ "$(id -u)" -eq 0 ] && chmod 1777 "$T"

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

    # 11b. an app's sitecustomize.py runs only in the program (after the privilege drop),
    # never in the root launcher: it records the uid and the process it ran in
    new_app site 'print("main ran", flush=True)'
    printf '%s\n' "import os; open('$T/site.log', 'a').write(str(os.getuid()) + chr(10))" > "$T/apps/site/sitecustomize.py"
    rm -f "$T/site.log"
    (cd "$T/apps/site" && run_fg site PYTHONPATH="$T/apps/site")
    check "sitecustomize: program ran" log_has site "main ran"
    check "sitecustomize: ran exactly once (in the program, not the launcher)" sh -c "[ \$(wc -l < '$T/site.log') -eq 1 ]"
    check "sitecustomize: ran as REBOT_APP_USER" sh -c "[ \"\$(cat '$T/site.log')\" = $(id -u "$APP_USER") ]"
    check "launcher: python3 -I -S and PYTHONPATH not exported" sh -c "grep -q 'python3 -I -S -c \"\$LAUNCH\"' '$RUNNER' && ! grep -q '^ *export PYTHONPATH' '$RUNNER'"

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

    # 12b. root only: a child that calls setsid leaves the group; processes of the app user
    # started after the program are killed on stop and after the program exits, an older
    # process of that user is not
    if [ "$(id -u)" -eq 0 ]; then
        python3 -c 'import os, pwd, sys, time
u = pwd.getpwnam(sys.argv[1]); os.setsid(); os.setgid(u.pw_gid); os.setuid(u.pw_uid)
os.execvp("sleep", ["sleep", "60"])' "$APP_USER" &
        OLDPID=$!
        sleep 0.2
        for run in stop exit stop-fallback exit-fallback; do
            mode=${run%-fallback}
            pidfd=1; [ "$run" = "$mode" ] || pidfd=0
            new_app "escape$mode" "import subprocess, sys, time
p = subprocess.Popen(['sleep', '60'], start_new_session=True)
open('$T/escape.child', 'w').write(str(p.pid))
print('escaped', flush=True)
time.sleep(60 if '$mode' == 'stop' else 0.5)
sys.exit(5)"
            printf '%s\n' '{"restart": "never"}' > "$T/apps/escape$mode/app.json"
            rm -f "$T/escape.child"
            rm -f "$T/logs/app-escape$mode.log"
            run_bg "escape$mode" REBOT_APP_PIDFD=$pidfd
            check "setsid ($run): child started" wait_for 10 test -s "$T/escape.child"
            EPID=$(cat "$T/escape.child" 2>/dev/null)
            check "setsid ($run): child is in another session" sh -c "[ \"\$(ps -o sid= -p $EPID | tr -d ' ')\" = $EPID ]"
            [ "$mode" = stop ] && kill -TERM "$RPID"
            check "setsid ($run): runner exits" wait_for 8 sh -c "! kill -0 $RPID"
            wait "$RPID" 2>/dev/null
            check "setsid ($run): the escaped child is gone" wait_for 3 sh -c "[ -n '$EPID' ] && ! kill -0 '$EPID'"
            if [ "$pidfd" -eq 1 ]; then
                check "setsid ($run): killed through a pidfd, no fallback" sh -c "grep -q 'killed pid $EPID .*; pidfd)' '$T/logs/app-escape$mode.log' && ! grep -q 'falling back' '$T/logs/app-escape$mode.log'"
            else
                check "setsid ($run): fallback logged, killed with kill <pid>" sh -c "grep -q 'pidfd sweep unavailable (exit 3); falling back' '$T/logs/app-escape$mode.log' && grep -q 'killed pid $EPID .*(outside its group)\$' '$T/logs/app-escape$mode.log'"
            fi
        done
        check "setsid: a process of the app user started before the app survives" kill -0 "$OLDPID"
        # 12c. an escaped process that keeps forking: children forked while a sweep runs are
        # caught by the next sweep, none is left
        new_app forker "import os, sys, time
if os.fork() == 0:
    os.setsid()
    open('$T/forker.pid', 'w').write(str(os.getpid()))
    for _ in range(3000):  # bounded, also when the runner leaves it running
        if os.fork() == 0:
            os.execvp('sleep', ['sleep', '6071'])
        time.sleep(0.003)
time.sleep(0.8)
sys.exit(5)"
        printf '%s\n' '{"restart": "never"}' > "$T/apps/forker/app.json"
        run_fg forker
        sleep 0.5
        check "forking escape: no process of it is left" sh -c "! pgrep -u '$APP_USER' -f 'sleep 6071'"
        FPID=$(cat "$T/forker.pid" 2>/dev/null)
        [ -n "$FPID" ] && kill -KILL "$FPID" 2>/dev/null
        pkill -KILL -u "$APP_USER" -f 'sleep 6071' 2>/dev/null
        kill "$OLDPID" 2>/dev/null; wait "$OLDPID" 2>/dev/null
    else
        echo "  skip setsid escape (needs root)"
    fi

    rm -rf "$T"
}

echo "== sh -n"
for f in "$RUNNER" "$K92" "$KIT/../debian/postinst" "$KIT/../debian/prerm"; do
    check "sh -n $(basename "$f")" sh -n "$f"
    if command -v busybox >/dev/null 2>&1 && busybox sh -c : 2>/dev/null; then
        check "busybox sh -n $(basename "$f")" busybox sh -n "$f"
    fi
done

# K92 config parser: values are shell-quoted before eval (a value with ' or $(...) runs nothing)
echo "== K92 config_shell_vars"
CT=$(mktemp -d)
python3 -c '
import json, sys
json.dump({"network": {"sta_ssid": "it'"'"'s $(touch " + sys.argv[1] + "/pwned1)", "sta_password": "`touch " + sys.argv[1] + "/pwned2`; x\"y",
           "ap_ssid": "a b\nc"}, "app": {"enabled": True, "name": "demo"}}, open(sys.argv[1] + "/cfg.json", "w"))' "$CT"
eval "$(sed -n '/^config_shell_vars() {/,/^}/p' "$K92")"
STA_SSID=""; STA_PASS=""; AP_SSID=""; APP_NAME=""
eval "$(config_shell_vars "$CT/cfg.json")"
check "config: value with ' and \$(...) kept literally" [ "$STA_SSID" = "it's \$(touch $CT/pwned1)" ]
check "config: value with backticks kept literally" [ "$STA_PASS" = "\`touch $CT/pwned2\`; x\"y" ]
check "config: value with a space and a newline kept" [ "$AP_SSID" = "a b
c" ]
check "config: app name" [ "$APP_NAME" = demo ]
check "config: nothing was executed" sh -c "[ ! -e '$CT/pwned1' ] && [ ! -e '$CT/pwned2' ]"
rm -rf "$CT"

# leftover sweep helper ($SWEEP, pidfds) on explicit candidate pids of the current user
echo "== rebot-app-run pidfd sweep"
eval "$(sed -n "/^SWEEP='/,/^sys.exit(4 if failed else 0)'\$/p" "$RUNNER")"
check "sweep: helper extracted" [ -n "$SWEEP" ]
starttime_of() { python3 -c 'import sys; print(open("/proc/%s/stat" % sys.argv[1]).read().rsplit(")", 1)[1].split()[19])' "$1"; }
sweep() { python3 -I -S -c "$SWEEP" "$@"; }
if python3 -c 'import os, signal; os.close(os.pidfd_open(os.getpid())); signal.pidfd_send_signal' 2>/dev/null; then
    ME=$(id -u)
    sleep 60 & V1=$!
    sleep 60 & V2=$!
    S1=$(starttime_of "$V1")
    out=$(sweep "$ME" "$S1" SIGKILL 1 "$$" "$V1"); r=$?
    check "sweep: matching pid is killed and printed" sh -c "[ $r -eq 0 ] && [ '$out' = '$V1' ]"
    wait "$V1" 2>/dev/null; rc=$?
    check "sweep: it died of SIGKILL" [ "$rc" -eq 137 ]
    check "sweep: the other candidate (not listed) is alive" alive "$V2"
    S2=$(starttime_of "$V2")
    out=$(sweep "$ME" $((S2 + 1)) SIGKILL 1 "$$" "$V2")
    check "sweep: pid started before <start> (reused-pid stand-in) is skipped" sh -c "[ -z '$out' ] && kill -0 $V2"
    out=$(sweep 4242 "$S2" SIGKILL 1 "$$" "$V2")
    check "sweep: pid of another uid is skipped" sh -c "[ -z '$out' ] && kill -0 $V2"
    out=$(sweep "$ME" "$S2" SIGKILL 1 "$V2" "$V2")
    check "sweep: the excluded pid is skipped" sh -c "[ -z '$out' ] && kill -0 $V2"
    out=$(sweep "$ME" 0 SIGKILL 1 "$$" 999999999 "$V2")
    check "sweep: a pid that does not exist is skipped" sh -c "[ '$out' = '$V2' ]"
    wait "$V2" 2>/dev/null
    # a zombie: a child its parent does not wait for
    ZT=$(mktemp)
    python3 -c 'import os, sys, time
p = os.fork()
if p == 0:
    os._exit(0)
open(sys.argv[1], "w").write(str(p)); time.sleep(3)' "$ZT" & ZH=$!
    wait_for 3 test -s "$ZT"; Z=$(cat "$ZT")
    sleep 0.2
    check "sweep: zombie stand-in is in state Z" sh -c "[ \"\$(cut -d' ' -f3 /proc/$Z/stat)\" = Z ]"
    out=$(sweep "$ME" 0 SIGKILL 1 "$$" "$Z")
    check "sweep: a zombie is skipped" [ -z "$out" ]
    kill "$ZH" 2>/dev/null; wait "$ZH" 2>/dev/null; rm -f "$ZT"
else
    echo "  skip pidfd sweep on candidates (no pidfd on this host)"
fi
sleep 60 & V3=$!
sweep "$(id -u)" 0 SIGKILL 0 "$$" "$V3"; r=$?
check "sweep: forced off (REBOT_APP_PIDFD=0) exits 3, signals nothing" sh -c "[ $r -eq 3 ] && kill -0 $V3"
out=$(python3 -I -S -c 'import os, sys; del os.pidfd_open; sys.argv = sys.argv[1:]; exec(sys.argv[0])' "$SWEEP" "$(id -u)" 0 SIGKILL 1 "$$" "$V3"); r=$?
check "sweep: without os.pidfd_open exits 3, signals nothing" sh -c "[ $r -eq 3 ] && [ -z '$out' ] && kill -0 $V3"
# pidfd_open failing with another errno (ENFILE here; injected): reported, exit 4; ESRCH
# (the pid has exited) is skipped
INJECT='import errno, os, signal, sys
code = getattr(errno, sys.argv[1])
def pidfd_open(pid, flags=0):
    raise OSError(code, os.strerror(code))
os.pidfd_open = pidfd_open
if not hasattr(signal, "pidfd_send_signal"):
    signal.pidfd_send_signal = lambda fd, sig: None
sys.argv = sys.argv[2:]
exec(sys.argv[0])'
out=$(python3 -I -S -c "$INJECT" ENFILE "$SWEEP" "$(id -u)" 0 SIGKILL 1 "$$" "$V3"); r=$?
check "sweep: pidfd_open ENFILE exits 4 and names the pid and errno" sh -c "[ $r -eq 4 ] && [ '$out' = '!$V3:ENFILE' ] && kill -0 $V3"
out=$(python3 -I -S -c "$INJECT" ESRCH "$SWEEP" "$(id -u)" 0 SIGKILL 1 "$$" "$V3"); r=$?
check "sweep: pidfd_open ESRCH is skipped (exit 0, nothing printed)" sh -c "[ $r -eq 0 ] && [ -z '$out' ]"
kill "$V3"; wait "$V3" 2>/dev/null

# build_deb.sh: the recamera flavor without REBOT_SDK_DIR stops before building
echo "== build_deb.sh REBOT_SDK_DIR"
out=$(env -u REBOT_SDK_DIR bash "$HERE/build_deb.sh" 2>&1); rc=$?
check "build_deb: recamera without REBOT_SDK_DIR fails" sh -c "[ $rc -ne 0 ]"
check "build_deb: the error names REBOT_SDK_DIR" sh -c 'printf "%s" "$1" | grep -q "Error: the recamera flavor needs the rebot Python SDK"' _ "$out"
out=$(REBOT_SDK_DIR=/nonexistent bash "$HERE/build_deb.sh" 2>&1)
check "build_deb: REBOT_SDK_DIR without rebot/ fails" sh -c 'printf "%s" "$1" | grep -q "needs the rebot Python SDK"' _ "$out"

# rebot-app-user: creates the program user in temp passwd/group/shadow files, idempotently
echo "== rebot-app-user"
USERTOOL=$KIT/usr/share/rebot-teleop/rebot-app-user
UT=$(mktemp -d)
printf 'root:x:0:0:root:/root:/bin/sh\nrecamera:x:1000:1000::/home/recamera:/bin/sh\n' > "$UT/passwd"
printf 'root:x:0:\nrecamera:x:1000:\n' > "$UT/group"
printf 'root:*:0:0:99999:7:::\n' > "$UT/shadow"
run_tool() { env REBOT_APP_PASSWD="$UT/passwd" REBOT_APP_GROUP="$UT/group" REBOT_APP_SHADOW="$UT/shadow" REBOT_APP_LOCK="$UT/lock" "$@" sh "$USERTOOL"; }
run_tool; r1=$?
cp "$UT/passwd" "$UT/passwd.1"; cp "$UT/group" "$UT/group.1"; cp "$UT/shadow" "$UT/shadow.1"
run_tool; r2=$?
check "app-user: first run succeeds" [ "$r1" -eq 0 ]
check "app-user: passwd entry, no login shell" sh -c "grep -Eq '^rebot-app:x:[0-9]+:[0-9]+:.*:(/bin/false|/sbin/nologin)\$' '$UT/passwd'"
check "app-user: group entry with the same id" sh -c "[ \"\$(sed -n 's/^rebot-app:x:\([0-9]*\):.*/\1/p' '$UT/group')\" = \"\$(sed -n 's/^rebot-app:x:[0-9]*:\([0-9]*\):.*/\1/p' '$UT/passwd')\" ]"
check "app-user: id is a system id not used by recamera" sh -c "! grep -q '^rebot-app:x:1000:' '$UT/passwd'"
check "app-user: locked password in shadow" grep -q '^rebot-app:!:' "$UT/shadow"
check "app-user: second run succeeds" [ "$r2" -eq 0 ]
check "app-user: second run changes nothing" sh -c "cmp -s '$UT/passwd' '$UT/passwd.1' && cmp -s '$UT/group' '$UT/group.1' && cmp -s '$UT/shadow' '$UT/shadow.1'"
check "app-user: exactly one entry" sh -c "[ \$(grep -c '^rebot-app:' '$UT/passwd') -eq 1 ] && [ \$(grep -c '^rebot-app:' '$UT/group') -eq 1 ]"
env REBOT_APP_USER=root REBOT_APP_PASSWD="$UT/passwd" REBOT_APP_GROUP="$UT/group" sh "$USERTOOL" 2>/dev/null
check "app-user: refuses root" [ "$?" -eq 2 ]
# partial accounts from an interrupted run are completed
base_files() {
    printf 'root:x:0:0:root:/root:/bin/sh\nrecamera:x:1000:1000::/home/recamera:/bin/sh\n' > "$UT/passwd"
    printf 'root:x:0:\nwheel:x:10:\nrecamera:x:1000:\n' > "$UT/group"
    printf 'root:*:0:0:99999:7:::\n' > "$UT/shadow"
}
base_files
printf 'rebot-app:x:998:998:rebot user programs:/userdata/rebot-apps:/bin/false\n' >> "$UT/passwd"
printf 'rebot-app:x:998:\n' >> "$UT/group"
run_tool; r=$?
check "app-user: missing shadow entry is added" sh -c "[ $r -eq 0 ] && [ \$(grep -c '^rebot-app:!:' '$UT/shadow') -eq 1 ] && [ \$(grep -c '^rebot-app:' '$UT/passwd') -eq 1 ]"
base_files
printf 'rebot-app:x:997:997:rebot user programs:/userdata/rebot-apps:/bin/false\n' >> "$UT/passwd"
run_tool; r=$?
check "app-user: missing group gets the passwd gid" sh -c "[ $r -eq 0 ] && grep -qx 'rebot-app:x:997:' '$UT/group' && grep -q '^rebot-app:!:' '$UT/shadow'"
base_files
printf 'rebot-app:x:996:\n' >> "$UT/group"
run_tool; r=$?
check "app-user: missing passwd entry gets the group gid" sh -c "[ $r -eq 0 ] && grep -q '^rebot-app:x:[0-9]*:996:' '$UT/passwd' && [ \$(grep -c '^rebot-app:' '$UT/group') -eq 1 ]"
# group gid: a group of the same name with another gid, or a gid owned by another group, is
# refused (exit 4) and nothing is written
for case in groupgid othergroup; do
    base_files
    printf 'rebot-app:x:997:997:rebot user programs:/userdata/rebot-apps:/bin/false\n' >> "$UT/passwd"
    case $case in
        groupgid) printf 'rebot-app:x:995:\n' >> "$UT/group" ;;
        othergroup) printf 'video:x:997:\n' >> "$UT/group" ;;
    esac
    cp "$UT/passwd" "$UT/passwd.1"; cp "$UT/group" "$UT/group.1"; cp "$UT/shadow" "$UT/shadow.1"
    out=$(run_tool 2>&1); r=$?
    check "app-user: gid mismatch ($case) refused (exit 4), nothing written" sh -c "[ $r -eq 4 ] && cmp -s '$UT/passwd' '$UT/passwd.1' && cmp -s '$UT/group' '$UT/group.1' && cmp -s '$UT/shadow' '$UT/shadow.1'"
    check "app-user: gid mismatch ($case) names the gids" sh -c 'printf "%s" "$1" | grep -q "has gid 997"' _ "$out"
done
# native method (stub addgroup/adduser in PATH): a missing group is created with the passwd gid
mkdir -p "$UT/bin"
printf '#!/bin/sh\necho "addgroup $*" >> "%s/calls"\ngid=994\nwhile [ $# -gt 1 ]; do case $1 in -g) gid=$2; shift ;; esac; shift; done\necho "$1:x:$gid:" >> "%s/group"\n' "$UT" "$UT" > "$UT/bin/addgroup"
printf '#!/bin/sh\necho "adduser $*" >> "%s/calls"\nexit 1\n' "$UT" > "$UT/bin/adduser"
chmod +x "$UT/bin/addgroup" "$UT/bin/adduser"
base_files; rm -f "$UT/calls"
printf 'rebot-app:x:997:997:rebot user programs:/userdata/rebot-apps:/bin/false\n' >> "$UT/passwd"
run_tool REBOT_APP_METHOD=native PATH="$UT/bin:$PATH"; r=$?
check "app-user (native): addgroup called with -g <passwd gid> -S" grep -qx 'addgroup -g 997 -S rebot-app' "$UT/calls"
check "app-user (native): group has the passwd gid, account complete" sh -c "[ $r -eq 0 ] && grep -qx 'rebot-app:x:997:' '$UT/group' && [ \$(grep -c '^rebot-app:' '$UT/group') -eq 1 ] && grep -q '^rebot-app:!:' '$UT/shadow'"
# ... and an addgroup that ignores -g (other gid) is caught by the final check
sed -i 's/gid=\$2; shift/shift/' "$UT/bin/addgroup"
base_files; rm -f "$UT/calls"
printf 'rebot-app:x:997:997:rebot user programs:/userdata/rebot-apps:/bin/false\n' >> "$UT/passwd"
run_tool REBOT_APP_METHOD=native PATH="$UT/bin:$PATH" 2>/dev/null; r=$?
check "app-user (native): group created with another gid is refused (exit 4)" [ "$r" -eq 4 ]
# a fresh account via the native method: addgroup without -g, adduser -G
base_files; rm -f "$UT/calls"
run_tool REBOT_APP_METHOD=native PATH="$UT/bin:$PATH"; r=$?
check "app-user (native): fresh account, addgroup -S without -g" sh -c "[ $r -eq 0 ] && grep -qx 'addgroup -S rebot-app' '$UT/calls' && grep -q '^adduser -S -D -H .* -G rebot-app rebot-app' '$UT/calls' && grep -q '^rebot-app:x:[0-9]*:994:' '$UT/passwd'"
# an existing privileged account is refused and left as it is
for case in uid0 gid0 wheelgid wheelmember; do
    base_files
    case $case in
        uid0) printf 'rebot-app:x:0:998::/:/bin/false\n' >> "$UT/passwd"; printf 'rebot-app:x:998:\n' >> "$UT/group" ;;
        gid0) printf 'rebot-app:x:998:0::/:/bin/false\n' >> "$UT/passwd"; printf 'rebot-app:x:998:\n' >> "$UT/group" ;;
        wheelgid) printf 'rebot-app:x:998:10::/:/bin/false\n' >> "$UT/passwd"; printf 'rebot-app:x:998:\n' >> "$UT/group" ;;
        wheelmember) printf 'rebot-app:x:998:998::/:/bin/false\n' >> "$UT/passwd"
            printf 'root:x:0:\nwheel:x:10:recamera,rebot-app\nrecamera:x:1000:\nrebot-app:x:998:\n' > "$UT/group" ;;
    esac
    cp "$UT/passwd" "$UT/passwd.1"; cp "$UT/group" "$UT/group.1"; cp "$UT/shadow" "$UT/shadow.1"
    run_tool 2>/dev/null; r=$?
    check "app-user: privileged account ($case) refused, nothing written" sh -c "[ $r -eq 3 ] && cmp -s '$UT/passwd' '$UT/passwd.1' && cmp -s '$UT/group' '$UT/group.1' && cmp -s '$UT/shadow' '$UT/shadow.1'"
done
# concurrent runs are serialised: one entry each
base_files
pids=""
for i in 1 2 3 4 5 6 7 8; do run_tool & pids="$pids $!"; done
ok=1
for p in $pids; do wait "$p" || ok=0; done
check "app-user: 8 concurrent runs succeed" [ "$ok" -eq 1 ]
check "app-user: concurrent runs leave one entry each" sh -c "[ \$(grep -c '^rebot-app:' '$UT/passwd') -eq 1 ] && [ \$(grep -c '^rebot-app:' '$UT/group') -eq 1 ] && [ \$(grep -c '^rebot-app:' '$UT/shadow') -eq 1 ]"
# a held lock: the run waits, then gives up without writing
base_files
if command -v flock >/dev/null 2>&1; then
    flock "$UT/lock" sleep 3 & holder=$!
    sleep 0.3
else
    mkdir "$UT/lock.d"; holder=""
fi
run_tool REBOT_APP_LOCK_WAIT=1 2>/dev/null; r=$?
check "app-user: held lock gives up (exit 1) and writes nothing" sh -c "[ $r -eq 1 ] && ! grep -q '^rebot-app:' '$UT/passwd'"
[ -n "$holder" ] && { kill "$holder" 2>/dev/null; wait "$holder" 2>/dev/null; }
rmdir "$UT/lock.d" 2>/dev/null
check "runner: default user is rebot-app" grep -q 'RUN_USER=\${REBOT_APP_USER:-rebot-app}' "$RUNNER"
rm -rf "$UT"

run_suite /bin/sh
if command -v busybox >/dev/null 2>&1 && busybox sh -c : 2>/dev/null; then
    run_suite "busybox sh"
else
    echo "== busybox not found; skipped"
fi

echo "passed: $PASSES, failed: $FAILS"
[ "$FAILS" -eq 0 ]
