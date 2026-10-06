#!/usr/bin/env bash
#
# pr2's companion Gazebo fixture runner: starts (or stops) the
# headless, robot-less pr2_world.sdf plus the ros_gz_bridge, purely so
# RobotLens's Simulation source has a /clock heartbeat to Attach to while
# pr2.launch.py drives PR2 kinematically (see ../README.md's
# "Option A" note and pr2_world.sdf's header comment -- PR2 itself is never
# spawned into this world).
#
# Mirrors ../../gazebo/run_gazebo_example.sh's env handling verbatim: see
# that script's header for why `gz` needs a minimal whitelisted environment
# instead of whatever shell this script was run from (sourcing ROS 2 Jazzy
# can intermittently break the `gz` CLI's subcommand dispatch).
#
# Usage:
#   robots_worlds/pr2/gazebo/run_pr2_gazebo.sh up
#   robots_worlds/pr2/gazebo/run_pr2_gazebo.sh down
#   robots_worlds/pr2/gazebo/run_pr2_gazebo.sh status

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_DIR="${TMPDIR:-/tmp}/robotlens_pr2_gazebo"
WORLD_FILE="$SCRIPT_DIR/pr2_world.sdf"
BRIDGE_YAML="$SCRIPT_DIR/clock_tf.yaml"

usage() {
    cat >&2 <<EOF
Usage: $(basename "$0") up
       $(basename "$0") down
       $(basename "$0") status
EOF
}

require_ros() {
    if ! command -v ros2 >/dev/null 2>&1; then
        echo "error: 'ros2' not on PATH. Source the ROS 2 Jazzy setup first:" >&2
        echo "  source /opt/ros/jazzy/setup.bash" >&2
        exit 1
    fi
    if ! command -v gz >/dev/null 2>&1; then
        echo "error: 'gz' (Gazebo Harmonic) not on PATH." >&2
        exit 1
    fi
}

# A pid file is only "alive" if its pid still exists AND its cmdline still
# matches the expected role -- see ../../gazebo/run_gazebo_example.sh for why.
pid_alive() {
    local role="$1"
    local pidfile="$STATE_DIR/$role.pid"
    [ -f "$pidfile" ] || return 1
    local pid
    pid="$(cat "$pidfile")"
    if kill -0 "$pid" 2>/dev/null &&
        tr '\0' ' ' </proc/"$pid"/cmdline 2>/dev/null | grep -q -- "$2"; then
        echo "$pid"
        return 0
    fi
    rm -f "$pidfile"
    return 1
}

teardown() {
    for role in gz_sim bridge; do
        local pidfile="$STATE_DIR/$role.pid"
        if [ -f "$pidfile" ]; then
            kill "$(cat "$pidfile")" 2>/dev/null || true
            rm -f "$pidfile"
        fi
    done
    pkill -f "gz s[i]m" 2>/dev/null || true
    pkill -f "parameter_bridg[e]" 2>/dev/null || true
    sleep 1
}

GZ_ENV=(env -i HOME="$HOME" PATH="/usr/bin:/bin")

cmd_up() {
    require_ros
    mkdir -p "$STATE_DIR"
    if pid_alive gz_sim "gz sim" || pid_alive bridge "parameter_bridge"; then
        echo "a fixture is already running; run '$(basename "$0")' down first" >&2
        exit 1
    fi

    if ! "${GZ_ENV[@]}" gz sim --help 2>&1 | grep -q "Run and manage Gazebo"; then
        echo "error: 'gz sim' is not available even in a clean environment." >&2
        echo "Check your Gazebo install: ${GZ_ENV[*]} gz sim --help" >&2
        exit 1
    fi

    echo "starting gz sim (world: pr2_ground)..."
    setsid nohup "${GZ_ENV[@]}" gz sim -s --headless-rendering -r \
        "$WORLD_FILE" </dev/null >"$STATE_DIR/gz_sim.log" 2>&1 &
    echo $! >"$STATE_DIR/gz_sim.pid"

    echo "starting ros_gz_bridge (config: $(basename "$BRIDGE_YAML"))..."
    setsid nohup ros2 run ros_gz_bridge parameter_bridge --ros-args \
        -p "config_file:=$BRIDGE_YAML" </dev/null >"$STATE_DIR/bridge.log" 2>&1 &
    echo $! >"$STATE_DIR/bridge.pid"

    echo "waiting for /clock over the bridge..."
    local deadline=$((SECONDS + 20))
    while [ "$SECONDS" -lt "$deadline" ]; do
        if timeout 2 ros2 topic list 2>/dev/null | grep -qx '/clock'; then
            echo "OK: /clock present. Use 'status' to inspect; connect RobotLens via Attach."
            return 0
        fi
        if ! pid_alive gz_sim "gz sim" >/dev/null; then
            echo "error: gz sim exited early; see $STATE_DIR/gz_sim.log" >&2
            teardown
            return 1
        fi
        if ! pid_alive bridge "parameter_bridge" >/dev/null; then
            echo "error: ros_gz_bridge exited early; see $STATE_DIR/bridge.log" >&2
            teardown
            return 1
        fi
        sleep 1
    done
    echo "warning: no /clock within 20s; see $STATE_DIR/gz_sim.log and $STATE_DIR/bridge.log" >&2
    teardown
    return 1
}

cmd_down() {
    teardown
    echo "stopped (any stragglers signalled)."
}

cmd_status() {
    local gz_pid bridge_pid
    if gz_pid=$(pid_alive gz_sim "gz sim"); then
        echo "gz sim:      running (pid $gz_pid)"
    else
        echo "gz sim:      not running"
    fi
    if bridge_pid=$(pid_alive bridge "parameter_bridge"); then
        echo "bridge:      running (pid $bridge_pid)"
    else
        echo "bridge:      not running"
    fi
    if [ -f "$STATE_DIR/gz_sim.log" ]; then
        echo "gz_sim.log:  $STATE_DIR/gz_sim.log"
    fi
    if [ -f "$STATE_DIR/bridge.log" ]; then
        echo "bridge.log:  $STATE_DIR/bridge.log"
    fi
    echo "ros topics:  $(timeout 3 ros2 topic list 2>/dev/null | sort | tr '\n' ' ' || true)"
}

ACTION="${1:-}"
case "$ACTION" in
up)
    cmd_up
    ;;
down)
    cmd_down
    ;;
status)
    cmd_status
    ;;
*)
    usage
    exit 2
    ;;
esac
