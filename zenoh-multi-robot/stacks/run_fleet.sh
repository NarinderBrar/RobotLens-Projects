#!/usr/bin/env bash
set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
count=${1:-5}

find_stack() {
    local name=$1
    if [[ -n "${ZMS_BIN:-}" && -x "$ZMS_BIN/$name" ]]; then
        printf '%s' "$ZMS_BIN/$name"
        return
    fi
    if command -v "$name" >/dev/null 2>&1; then
        command -v "$name"
        return
    fi
    for folder in "../../../Zenoh-motion-stacks/build/system" "../../../motion-stacks/build/system"; do
        if [[ -x "$here/$folder/$name" ]]; then
            printf '%s' "$here/$folder/$name"
            return
        fi
    done
    printf 'cannot find %s; set ZMS_BIN or install Zenoh-motion-stacks\n' "$name" >&2
    exit 1
}

navigation=$(find_stack zms_navigation)
pids=()
stop_fleet() {
    kill -TERM "${pids[@]}" 2>/dev/null || true
    wait "${pids[@]}" 2>/dev/null || true
}
trap stop_fleet EXIT INT TERM

for index in $(seq 1 "$count"); do
    config="$here/navigation-robot$index.json"
    "$navigation" --check-config "$config"
    "$navigation" --config "$config" &
    pids+=($!)
done
printf 'started %s navigation stacks; press Ctrl+C to stop\n' "$count"
wait
