#!/bin/sh
# Download rebot-vision models listed in kits/recamera-b601dm/vision/models/models.tsv and verify
# their md5 (and size when given). Runs on the reCamera (default target /userdata/local/models)
# or on a build host (pass a cache directory).
# Usage: fetch_models.sh [target dir] [role|ship|kind ...]
#   e.g. fetch_models.sh target/models-cache bundled
#        fetch_models.sh /userdata/local/models depth
# Stock detect / 640x640 segment models ship with reCamera OS in /usr/share/supervisor/models.
set -eu
here=$(cd "$(dirname "$0")" && pwd)
list=${REBOT_MODELS_TSV:-$here/../kits/recamera-b601dm/vision/models/models.tsv}
dir=${1:-/userdata/local/models}
[ $# -gt 0 ] && shift
timeout_s=${REBOT_FETCH_TIMEOUT:-120}
connect_s=${REBOT_FETCH_CONNECT_TIMEOUT:-15}
mkdir -p "$dir"

fetch() {
    if command -v curl >/dev/null 2>&1; then
        curl -fL --retry 3 --connect-timeout "$connect_s" --max-time "$timeout_s" -o "$2" "$1"
    else
        wget -T "$connect_s" -O "$2" "$1"
    fi
}

matches_filter() {
    f_role=$1
    f_ship=$2
    shift 2
    [ $# -eq 0 ] && return 0
    for k in "$@"; do
        [ "$k" = "$f_role" ] && return 0
        [ "$k" = "$f_ship" ] && return 0
        # Legacy kind shorthands (segment, obb, depth)
        case "$k:$f_role" in
            segment:detector-segment|obb:detector-obb) return 0 ;;
        esac
    done
    return 1
}

tab=$(printf '\t')
grep -v '^[[:space:]]*#' "$list" | while IFS="$tab" read -r role file md5 size ion_mb url ship license notes; do
    [ -n "$role" ] || continue
    case "$url" in
        ''|-|os) continue ;;
    esac
    matches_filter "$role" "$ship" "$@" || continue
    out="$dir/$file"
    if [ -f "$out" ] && [ "$(md5sum "$out" | cut -d' ' -f1)" = "$md5" ]; then
        echo "ok      $role $out"
        continue
    fi
    fetch "$url" "$out.part"
    got=$(md5sum "$out.part" | cut -d' ' -f1)
    if [ "$got" != "$md5" ]; then
        echo "md5 mismatch for $file: $got (expected $md5)" >&2
        rm -f "$out.part"
        exit 1
    fi
    if [ -n "$size" ] && [ "$size" != "-" ]; then
        got_size=$(wc -c < "$out.part" | tr -d ' ')
        if [ "$got_size" != "$size" ]; then
            echo "size mismatch for $file: $got_size (expected $size)" >&2
            rm -f "$out.part"
            exit 1
        fi
    fi
    mv "$out.part" "$out"
    echo "fetched $role $out ($license)"
done
