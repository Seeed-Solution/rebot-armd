#!/usr/bin/env sh
# Reproducible generator for tests/fixtures/tracking_off_e4f72b6_trace.json.
# Checks out baseline commit e4f72b6 in a temporary git worktree, runs the
# 7-producer disabled-tracking scenario offline, and records provenance metadata.
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
REPO_ROOT=$(CDPATH= cd -- "$SCRIPT_DIR/../.." && pwd)
BASE_REV="${1:-e4f72b6}"
FULL_COMMIT=$(git -C "$REPO_ROOT" rev-parse "${BASE_REV}^{commit}")
SHORT_COMMIT=$(git -C "$REPO_ROOT" rev-parse --short=7 "$FULL_COMMIT")

FIXTURE_OUT="$SCRIPT_DIR/tracking_off_e4f72b6_trace.json"
META_OUT="$SCRIPT_DIR/tracking_off_e4f72b6_trace.meta.json"
GEN_SRC="$SCRIPT_DIR/gen_tracking_off_e4f72b6.rs"
TARGET_DIR="$REPO_ROOT/target/e4f72b6-fixture-build"

TMP_DIR=$(mktemp -d "${TMPDIR:-/tmp}/armd-tracking-fixture.XXXXXX")
WT_DIR="$TMP_DIR/wt"

cleanup() {
    if [ -d "$WT_DIR" ]; then
        rm -f "$WT_DIR/tests/gen_tracking_off_e4f72b6.rs"
        git -C "$REPO_ROOT" worktree remove --force "$WT_DIR" >/dev/null 2>&1 || true
    fi
    rmdir "$TMP_DIR" >/dev/null 2>&1 || true
    rm -rf "$TARGET_DIR"
}
trap cleanup EXIT INT TERM

git -C "$REPO_ROOT" worktree add --detach "$WT_DIR" "$FULL_COMMIT"
cp "$GEN_SRC" "$WT_DIR/tests/gen_tracking_off_e4f72b6.rs"

CARGO_TARGET_DIR="$TARGET_DIR" \
TRACKING_FIXTURE_OUT="$FIXTURE_OUT" \
TRACKING_FIXTURE_COMMIT="$FULL_COMMIT" \
timeout 900 cargo test --offline --locked \
    --manifest-path "$WT_DIR/Cargo.toml" \
    --test gen_tracking_off_e4f72b6 -- --nocapture

SHA256=$(sha256sum "$FIXTURE_OUT" | awk '{print $1}')
COUNTS=$(python3 -c '
import json, sys
d = json.load(open(sys.argv[1]))
print(len(d["frames"]), len(d["obs"]))
' "$FIXTURE_OUT")
FRAMES=$(echo "$COUNTS" | awk '{print $1}')
OBS=$(echo "$COUNTS" | awk '{print $2}')

cat > "$META_OUT" <<EOF
{
  "source_commit": "$FULL_COMMIT",
  "short_commit": "$SHORT_COMMIT",
  "command": "sh tests/fixtures/generate_tracking_off_e4f72b6.sh $BASE_REV",
  "generator": "tests/fixtures/gen_tracking_off_e4f72b6.rs",
  "fixture": "tests/fixtures/tracking_off_e4f72b6_trace.json",
  "frames": $FRAMES,
  "obs": $OBS,
  "sha256": "$SHA256"
}
EOF

echo "Generated $FIXTURE_OUT (frames=$FRAMES, obs=$OBS, sha256=$SHA256) from $FULL_COMMIT"
