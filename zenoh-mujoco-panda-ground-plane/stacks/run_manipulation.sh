#!/usr/bin/env bash
set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

find_manipulation() {
    if [[ -n "${ZMS_BIN:-}" && -x "$ZMS_BIN/zms_manipulation" ]]; then
        printf '%s' "$ZMS_BIN/zms_manipulation"
        return
    fi
    if command -v zms_manipulation >/dev/null 2>&1; then
        command -v zms_manipulation
        return
    fi
    for folder in ../../../Zenoh-motion-stacks/build/system ../../../motion-stacks/build/system; do
        if [[ -x "$here/$folder/zms_manipulation" ]]; then
            printf '%s' "$here/$folder/zms_manipulation"
            return
        fi
    done
    printf 'Cannot find zms_manipulation; set ZMS_BIN to its directory.\n' >&2
    return 1
}

manipulation=$(find_manipulation)
"$manipulation" --check-config "$here/manipulation.json"
printf 'Starting manipulation; press Ctrl+C to stop.\n'
exec "$manipulation" --config "$here/manipulation.json"
