#!/usr/bin/env bash
set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
count=${1:-3}

find_navigation() {
    if [[ -n "${ZMS_BIN:-}" && -x "$ZMS_BIN/zms_navigation" ]]; then
        printf '%s' "$ZMS_BIN/zms_navigation"
        return
    fi
    if command -v zms_navigation >/dev/null 2>&1; then
        command -v zms_navigation
        return
    fi
    for folder in ../../../Zenoh-motion-stacks/build/system ../../../motion-stacks/build/system; do
        if [[ -x "$here/$folder/zms_navigation" ]]; then
            printf '%s' "$here/$folder/zms_navigation"
            return
        fi
    done
    printf 'Cannot find zms_navigation; set ZMS_BIN to its directory.\n' >&2
    return 1
}

navigation=$(find_navigation)
for ((n = 1; n <= count; n++)); do
    "$navigation" --check-config "$here/navigation-robot$n.json"
done

python=${ZMS_PYTHON:-python3}
if ! "$python" -c 'import zenoh' 2>/dev/null; then
    if [[ -z "${ZMS_PYTHON:-}" && -x "$here/../../../RobotLens-Automation/.venv/bin/python" ]]; then
        python="$here/../../../RobotLens-Automation/.venv/bin/python"
    fi
    "$python" -c 'import zenoh' || {
        printf 'Set ZMS_PYTHON to a Python interpreter with eclipse-zenoh installed.\n' >&2
        exit 1
    }
fi
"$python" "$here/../scripts/generate_map.py"

pids=()
stop_components() {
    if ((${#pids[@]})); then
        kill -TERM "${pids[@]}" 2>/dev/null || true
        wait "${pids[@]}" 2>/dev/null || true
    fi
}
trap stop_components EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
for ((n = 1; n <= count; n++)); do
    "$navigation" --config "$here/navigation-robot$n.json" &
    pids+=($!)
    "$python" "$here/../scripts/publish_map.py" --config "$here/navigation-robot$n.json" &
    pids+=($!)
done
printf 'Started %d navigation stacks and map publishers; press Ctrl+C to stop.\n' "$count"
wait -n "${pids[@]}"
printf 'A navigation component exited; stopping the remaining components.\n' >&2
exit 1
