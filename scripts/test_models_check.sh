#!/bin/sh
# Host-side test of kits/recamera-b601dm/packaging/rootfs/usr/share/rebot-teleop/rebot-models-check
# against a fake rootfs directory. Runs under /bin/sh and (when installed) `busybox ash`.
# Cases: missing, wrong md5, user model present, CDN fallback + CDN unreachable / bad md5.
# Usage: scripts/test_models_check.sh
HERE=$(cd "$(dirname "$0")" && pwd)
KIT=$HERE/../kits/recamera-b601dm/packaging/rootfs
CHECK=$KIT/usr/share/rebot-teleop/rebot-models-check
FETCH=$HERE/fetch_models.sh
POSTINST=$HERE/../kits/recamera-b601dm/packaging/debian/postinst
K92=$KIT/etc/init.d/K92rebot-teleop
FAILS=0
PASSES=0

pass() { PASSES=$((PASSES + 1)); echo "  ok   $*"; }
fail() { FAILS=$((FAILS + 1)); echo "  FAIL $*"; }
check() {
    d=$1; shift
    if "$@" >/dev/null 2>&1; then pass "$d"; else fail "$d"; fi
}

file_md5() { md5sum "$1" | cut -d' ' -f1; }

make_rootfs() {
    root=$1
    mkdir -p "$root/usr/share/rebot-teleop/models" \
             "$root/userdata/local/models" \
             "$root/usr/share/supervisor/models" \
             "$root/cdn"

    printf 'seg384-tuned-payload-v1\n' > "$root/cdn/seg.cvimodel"
    printf 'zipdepth-tabletop-payload-v1\n' > "$root/cdn/depth.cvimodel"
    printf 'os-detect-payload-v1\n' > "$root/usr/share/supervisor/models/det.cvimodel"
    printf 'os-seg640-payload-v1\n' > "$root/usr/share/supervisor/models/seg640.cvimodel"
    printf 'optional-obb-payload-v1\n' > "$root/cdn/obb.cvimodel"

    cp "$root/cdn/seg.cvimodel" "$root/usr/share/rebot-teleop/models/seg.cvimodel"
    cp "$root/cdn/depth.cvimodel" "$root/usr/share/rebot-teleop/models/depth.cvimodel"

    SEG_MD5=$(file_md5 "$root/cdn/seg.cvimodel")
    DEP_MD5=$(file_md5 "$root/cdn/depth.cvimodel")
    DET_MD5=$(file_md5 "$root/usr/share/supervisor/models/det.cvimodel")
    SEG640_MD5=$(file_md5 "$root/usr/share/supervisor/models/seg640.cvimodel")
    OBB_MD5=$(file_md5 "$root/cdn/obb.cvimodel")

    SEG_SZ=$(wc -c < "$root/cdn/seg.cvimodel" | tr -d ' ')
    DEP_SZ=$(wc -c < "$root/cdn/depth.cvimodel" | tr -d ' ')
    DET_SZ=$(wc -c < "$root/usr/share/supervisor/models/det.cvimodel" | tr -d ' ')
    SEG640_SZ=$(wc -c < "$root/usr/share/supervisor/models/seg640.cvimodel" | tr -d ' ')
    OBB_SZ=$(wc -c < "$root/cdn/obb.cvimodel" | tr -d ' ')

    tab=$(printf '\t')
    cat <<EOF > "$root/usr/share/rebot-teleop/models/models.tsv"
# test manifest
detector-segment${tab}seg.cvimodel${tab}${SEG_MD5}${tab}${SEG_SZ}${tab}13.1${tab}http://127.0.0.1:1/seg.cvimodel${tab}bundled${tab}AGPL${tab}seg
detector-segment-fallback${tab}seg640.cvimodel${tab}${SEG640_MD5}${tab}${SEG640_SZ}${tab}19.5${tab}-${tab}os${tab}AGPL${tab}seg640
detector-detect${tab}det.cvimodel${tab}${DET_MD5}${tab}${DET_SZ}${tab}8.5${tab}-${tab}os${tab}AGPL${tab}det
depth${tab}depth.cvimodel${tab}${DEP_MD5}${tab}${DEP_SZ}${tab}20.0${tab}http://127.0.0.1:1/depth.cvimodel${tab}bundled${tab}MIT${tab}depth
detector-obb${tab}obb.cvimodel${tab}${OBB_MD5}${tab}${OBB_SZ}${tab}8.0${tab}http://127.0.0.1:1/obb.cvimodel${tab}optional${tab}AGPL${tab}obb
EOF
}

run_suite() {
    SH=$1
    echo "== $SH"
    T=$(mktemp -d) || exit 1

    # Case 1: missing models -> repaired by linking from /usr/share/rebot-teleop/models
    R1="$T/case1"
    make_rootfs "$R1"
    LOG1="$T/case1.log"
    REBOT_ROOT="$R1" REBOT_NO_SYSLOG=1 $SH "$CHECK" > "$LOG1" 2>&1
    RC=$?
    check "missing: exits 0 (rc=$RC)" test "$RC" -eq 0
    check "missing: seg linked from bundled" test -L "$R1/userdata/local/models/seg.cvimodel"
    check "missing: seg md5 matches" test "$(file_md5 "$R1/userdata/local/models/seg.cvimodel")" = "$SEG_MD5"
    check "missing: depth linked from bundled" test -L "$R1/userdata/local/models/depth.cvimodel"
    check "missing: depth md5 matches" test "$(file_md5 "$R1/userdata/local/models/depth.cvimodel")" = "$DEP_MD5"
    check "missing: optional obb not installed" test ! -e "$R1/userdata/local/models/obb.cvimodel"
    check "missing: repair logged" grep -q "repaired (depth): linked" "$LOG1"
    check "missing: source recorded as bundled" grep -q "^depth.cvimodel	bundled$" "$R1/userdata/local/models/.rebot-model-sources"
    # Second run is idempotent
    REBOT_ROOT="$R1" REBOT_NO_SYSLOG=1 $SH "$CHECK" > "$LOG1.2" 2>&1
    RC2=$?
    check "missing: second run idempotent (rc=$RC2)" test "$RC2" -eq 0
    check "missing: second run logs ok" grep -q "ok (depth):" "$LOG1.2"

    # Case 2: wrong md5 -> original regular file backed up (never deleted), target repaired
    R2="$T/case2"
    make_rootfs "$R2"
    printf 'corrupt-or-old-roomscale-model\n' > "$R2/userdata/local/models/depth.cvimodel"
    OLD_MD5=$(file_md5 "$R2/userdata/local/models/depth.cvimodel")
    LOG2="$T/case2.log"
    REBOT_ROOT="$R2" REBOT_NO_SYSLOG=1 $SH "$CHECK" > "$LOG2" 2>&1
    RC=$?
    check "wrong-md5: exits 0 (rc=$RC)" test "$RC" -eq 0
    check "wrong-md5: active depth md5 repaired" test "$(file_md5 "$R2/userdata/local/models/depth.cvimodel")" = "$DEP_MD5"
    check "wrong-md5: old regular file preserved at .user-bak" test -f "$R2/userdata/local/models/depth.cvimodel.user-bak"
    check "wrong-md5: backup content intact" test "$(file_md5 "$R2/userdata/local/models/depth.cvimodel.user-bak")" = "$OLD_MD5"
    check "wrong-md5: mismatch and backup logged" sh -c "grep -q 'mismatch (depth):' '$LOG2' && grep -q 'backup (depth):' '$LOG2'"

    # Case 3: user models present (custom model untouched, valid regular manifest copy kept)
    R3="$T/case3"
    make_rootfs "$R3"
    printf 'my-private-custom-yolo-model\n' > "$R3/userdata/local/models/user_custom.cvimodel"
    USER_MD5=$(file_md5 "$R3/userdata/local/models/user_custom.cvimodel")
    cp "$R3/cdn/seg.cvimodel" "$R3/userdata/local/models/seg.cvimodel"
    LOG3="$T/case3.log"
    REBOT_ROOT="$R3" REBOT_NO_SYSLOG=1 $SH "$CHECK" > "$LOG3" 2>&1
    RC=$?
    check "user-model: exits 0 (rc=$RC)" test "$RC" -eq 0
    check "user-model: custom model untouched" test -f "$R3/userdata/local/models/user_custom.cvimodel" -a "$(file_md5 "$R3/userdata/local/models/user_custom.cvimodel")" = "$USER_MD5"
    check "user-model: existing valid regular seg kept as regular file" test -f "$R3/userdata/local/models/seg.cvimodel" -a ! -L "$R3/userdata/local/models/seg.cvimodel"
    check "user-model: depth still repaired from bundled" test "$(file_md5 "$R3/userdata/local/models/depth.cvimodel")" = "$DEP_MD5"

    # Case 4a: bundled missing -> CDN fallback succeeds and records source=cdn
    R4="$T/case4a"
    make_rootfs "$R4"
    rm -f "$R4/usr/share/rebot-teleop/models/depth.cvimodel"
    FAKE_FETCH="$T/fake-fetch-ok.sh"
    cat <<EOF > "$FAKE_FETCH"
#!/bin/sh
url=\$1
out=\$2
base=\$(basename "\$url")
cp "$R4/cdn/\$base" "\$out"
EOF
    chmod 0755 "$FAKE_FETCH"
    LOG4A="$T/case4a.log"
    REBOT_ROOT="$R4" REBOT_NO_SYSLOG=1 REBOT_FETCH_CMD="$FAKE_FETCH" $SH "$CHECK" > "$LOG4A" 2>&1
    RC=$?
    check "cdn-fallback: exits 0 (rc=$RC)" test "$RC" -eq 0
    check "cdn-fallback: depth downloaded with right md5" test "$(file_md5 "$R4/userdata/local/models/depth.cvimodel")" = "$DEP_MD5"
    check "cdn-fallback: source recorded as cdn" grep -q "^depth.cvimodel	cdn$" "$R4/userdata/local/models/.rebot-model-sources"
    check "cdn-fallback: download logged" grep -q "repaired (depth): downloaded" "$LOG4A"

    # Case 4b: bundled missing AND CDN unreachable -> exits 1, no partial files, user models untouched
    R4B="$T/case4b"
    make_rootfs "$R4B"
    rm -f "$R4B/usr/share/rebot-teleop/models/depth.cvimodel"
    printf 'user-custom-stays\n' > "$R4B/userdata/local/models/my_user.cvimodel"
    printf 'stale-depth-stays-when-repair-fails\n' > "$R4B/userdata/local/models/depth.cvimodel"
    STALE_MD5=$(file_md5 "$R4B/userdata/local/models/depth.cvimodel")
    LOG4B="$T/case4b.log"
    REBOT_ROOT="$R4B" REBOT_NO_SYSLOG=1 REBOT_CDN_CONNECT_TIMEOUT=1 REBOT_CDN_MAX_TIME=2 \
        $SH "$CHECK" > "$LOG4B" 2>&1
    RC=$?
    check "cdn-unreachable: exits non-zero (rc=$RC)" test "$RC" -ne 0
    check "cdn-unreachable: seg still repaired from bundled" test "$(file_md5 "$R4B/userdata/local/models/seg.cvimodel")" = "$SEG_MD5"
    check "cdn-unreachable: no leftover .part files" sh -c "! ls '$R4B/userdata/local/models/'*.part* >/dev/null 2>&1"
    check "cdn-unreachable: custom user model untouched" test -f "$R4B/userdata/local/models/my_user.cvimodel"
    check "cdn-unreachable: existing mismatched file not deleted on failure" test "$(file_md5 "$R4B/userdata/local/models/depth.cvimodel")" = "$STALE_MD5"
    check "cdn-unreachable: failure logged" grep -q "CDN download failed for depth.cvimodel" "$LOG4B"

    # Case 4c: bundled missing AND CDN returns wrong md5 -> rejected, exits 1
    R4C="$T/case4c"
    make_rootfs "$R4C"
    rm -f "$R4C/usr/share/rebot-teleop/models/depth.cvimodel"
    FAKE_BAD="$T/fake-fetch-bad.sh"
    cat <<'EOF' > "$FAKE_BAD"
#!/bin/sh
printf 'bad-cdn-bytes\n' > "$2"
EOF
    chmod 0755 "$FAKE_BAD"
    LOG4C="$T/case4c.log"
    REBOT_ROOT="$R4C" REBOT_NO_SYSLOG=1 REBOT_FETCH_CMD="$FAKE_BAD" $SH "$CHECK" > "$LOG4C" 2>&1
    RC=$?
    check "cdn-bad-md5: exits non-zero (rc=$RC)" test "$RC" -ne 0
    check "cdn-bad-md5: bad file not installed" test ! -e "$R4C/userdata/local/models/depth.cvimodel"
    check "cdn-bad-md5: md5 mismatch logged" grep -q "CDN download md5 mismatch for depth.cvimodel" "$LOG4C"

    rm -rf "$T"
}

echo "== syntax & ash safety"
for f in "$CHECK" "$FETCH" "$POSTINST" "$K92"; do
    check "sh -n $(basename "$f")" sh -n "$f"
    if command -v busybox >/dev/null 2>&1; then
        check "busybox ash -n $(basename "$f")" busybox ash -n "$f"
    fi
done
check "no blocking wait in rebot-models-check" sh -c "! grep -Eq '^[[:space:]]*wait([[:space:]]|$)' '$CHECK'"

run_suite /bin/sh
if command -v busybox >/dev/null 2>&1 && busybox ash -c 'true' >/dev/null 2>&1; then
    run_suite "busybox ash"
fi

echo
echo "$PASSES passed, $FAILS failed"
[ "$FAILS" -eq 0 ]
