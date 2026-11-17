#!/bin/sh
# Download the rebot-vision models listed in kits/recamera-b601dm/vision/models/models.tsv and check
# their md5. Runs on the reCamera (default target /userdata/local/models) or on a PC (pass a directory).
# Usage: fetch_models.sh [target dir] [kind ...]   e.g. fetch_models.sh /userdata/local/models depth
# The official yolo11n detect / segment models ship with reCamera OS in /usr/share/supervisor/models.
set -eu
here=$(cd "$(dirname "$0")" && pwd)
list=${REBOT_MODELS_TSV:-$here/../kits/recamera-b601dm/vision/models/models.tsv}
dir=${1:-/userdata/local/models}
[ $# -gt 0 ] && shift
mkdir -p "$dir"
fetch() {
    if command -v curl >/dev/null 2>&1; then curl -fL --retry 3 -o "$2" "$1"; else wget -O "$2" "$1"; fi
}
tab=$(printf '\t')
grep -v '^#' "$list" | while IFS="$tab" read -r kind file url md5 license source; do
    [ -n "$kind" ] || continue
    if [ $# -gt 0 ]; then
        want=0
        for k in "$@"; do [ "$k" = "$kind" ] && want=1; done
        [ "$want" = 1 ] || continue
    fi
    out="$dir/$file"
    if [ -f "$out" ] && [ "$(md5sum "$out" | cut -d' ' -f1)" = "$md5" ]; then
        echo "ok      $kind $out"
        continue
    fi
    fetch "$url" "$out.part"
    got=$(md5sum "$out.part" | cut -d' ' -f1)
    if [ "$got" != "$md5" ]; then
        echo "md5 mismatch for $file: $got (expected $md5)" >&2
        rm -f "$out.part"
        exit 1
    fi
    mv "$out.part" "$out"
    echo "fetched $kind $out ($license)"
done
