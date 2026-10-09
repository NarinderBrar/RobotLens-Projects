#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PYTHON="${PROJECT_ROOT}/.venv/bin/python"

if [[ ! -x "$PYTHON" ]]; then
    printf 'Project environment is missing. Create it and install training/requirements.txt first.\n' >&2
    exit 2
fi

exec "$PYTHON" "$PROJECT_ROOT/training/microduck/ppo_external_trainer.py" "$@"
