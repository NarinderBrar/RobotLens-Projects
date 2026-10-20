#!/usr/bin/env bash
set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

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
    printf 'cannot find %s; set ZMS_BIN or install motion-stacks\n' "$name" >&2
    exit 1
}

pids=()
stop_stacks() {
    kill -TERM "${pids[@]}" 2>/dev/null || true
    wait "${pids[@]}" 2>/dev/null || true
}
trap stop_stacks EXIT INT TERM

navigation=$(find_stack zms_navigation)
"$navigation" --check-config "$here/navigation.json"
"$navigation" --config "$here/navigation.json" &
pids+=($!)
manipulation=$(find_stack zms_manipulation)
"$manipulation" --check-config "$here/manipulation.json"
"$manipulation" --config "$here/manipulation.json" &
pids+=($!)
wait
