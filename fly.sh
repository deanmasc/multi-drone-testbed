#!/usr/bin/env bash
# fly.sh -- the three-terminal hardware ritual in one command.
#
#   ./fly.sh testbed_hexagon_hybrid --real drone1,drone4 --duration 95
#   ./fly.sh testbed_fig4 --real drone1,drone4 --no-gui
#   ./fly.sh testbed_hexagon --real none          # everything simulated, via ROS
#   ./fly.sh testbed_fig4 --real drone1 --check   # validate + show plan, launch nothing
#
# Runs, in order, what used to be terminals 1-3:
#   1. ros2 launch crazyflie launch.py backend:=cflib      (radio + VICON bridge)
#      ...and waits for /poses to appear before going on
#   2. ros2 launch drone_testbed hardware_hybrid.launch.py (the flight)
#   3. python3 tools/metrics_recorder.py                   (--no-metrics to skip)
#
# One Ctrl-C shuts them down in the REVERSE order: the flight launch is
# interrupted first so every crazyflie_node lands (its main() lands in a
# finally block), then the recorder gets its Ctrl-C so it writes the analysis,
# then the radio server goes away. Killing them all at once is what strands a
# drone in the air with no server to land it through.
#
# Options:
#   --real LIST      comma-separated algorithm ids to fly for real (drone1,drone4).
#                    "none" simulates every drone. Default: drone1.
#                    The Crazyswarm / VICON name is derived as droneN -> drone_N,
#                    which is the convention config/crazyflies.yaml already uses,
#                    so cf_name and mocap_name are no longer typed by hand. To
#                    fly an algorithm slot on a different airframe, say so
#                    explicitly: --real drone1,drone4=drone_2
#   --duration SEC   flight_duration for the launch file (0 = until Ctrl-C)
#   --no-gui         live_visualizer off (headless / no display forwarding)
#   --no-metrics     do not start metrics_recorder
#   --record         also start tools/flight_recorder.py for each real drone
#   --check | -n     validate configs and print the plan, then exit
#   --yes  | -y      skip the "press enter to fly" confirmation
#   -- ARGS          anything after -- is passed through to the hybrid launch,
#                    e.g. -- takeoff_height:=0.8,1.2 geofence:=1.2
#
# Every process's console output is tee'd into logs/<config>_<stamp>/, next to
# the metrics file and a copy of the config as flown.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="$REPO/ros2_ws/src/drone_testbed/config"

CONFIG=""; REAL="drone1"; DURATION=""; GUI=true; METRICS=true; RECORD=false
CHECK_ONLY=false; ASSUME_YES=false; EXTRA=()

die() { echo "fly: $*" >&2; exit 1; }
log() { echo "[fly] $*"; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --real)       REAL="$2"; shift 2 ;;
    --duration)   DURATION="$2"; shift 2 ;;
    --no-gui)     GUI=false; shift ;;
    --no-metrics) METRICS=false; shift ;;
    --record)     RECORD=true; shift ;;
    --check|-n)   CHECK_ONLY=true; shift ;;
    --yes|-y)     ASSUME_YES=true; shift ;;
    --)           shift; EXTRA=("$@"); break ;;
    -h|--help)    sed -n '2,40p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    -*)           die "unknown option $1 (use -- before raw launch args)" ;;
    *)            [[ -z "$CONFIG" ]] || die "two configs given: $CONFIG and $1"
                  CONFIG="$1"; shift ;;
  esac
done
[[ -n "$CONFIG" ]] || die "no config given. Try: ./fly.sh testbed_fig4 --real drone1,drone4"

# ---- resolve the config -----------------------------------------------------
# Bare name, name.yaml, or a path all resolve to the SOURCE tree, not the colcon
# install dir. load_config() tries an absolute path first, so editing the yaml
# no longer needs a rebuild to take effect.
CONFIG_PATH=""
for c in "$CONFIG" "$CONFIG.yaml" "$CONFIG_DIR/$CONFIG" "$CONFIG_DIR/$CONFIG.yaml"; do
  if [[ -f "$c" ]]; then
    CONFIG_PATH="$(cd "$(dirname "$c")" && pwd)/$(basename "$c")"; break
  fi
done
[[ -n "$CONFIG_PATH" ]] || die "no such config: $CONFIG (looked in $CONFIG_DIR)"
CONFIG_NAME="$(basename "$CONFIG_PATH" .yaml)"

# ---- ROS environment --------------------------------------------------------
# ROS 2's setup.bash reads unset variables, so -u has to come off around it.
if [[ -z "${ROS_DISTRO:-}" ]]; then
  set +u
  for d in jazzy humble; do
    [[ -d /opt/ros/$d ]] && { source "/opt/ros/$d/setup.bash"; break; }
  done
  set -u
fi
[[ -n "${ROS_DISTRO:-}" ]] || die "no ROS 2 found in /opt/ros -- run setup.sh first"
if [[ -f "$REPO/ros2_ws/install/setup.bash" ]]; then
  set +u; source "$REPO/ros2_ws/install/setup.bash"; set -u
else
  die "ros2_ws/install/setup.bash missing -- build first: cd ros2_ws && colcon build --symlink-install"
fi

# ---- the fleet ----------------------------------------------------------------
# droneN (algorithm id, testbed yaml) <-> drone_N (Crazyswarm robot key AND the
# VICON rigid body, crazyflies.yaml). Same string for cf_name and mocap_name.
REAL_IDS=()
if [[ "$REAL" != "none" ]]; then
  IFS=',' read -r -a REAL_IDS <<< "$REAL"
fi
CF_NAMES=()
for k in "${!REAL_IDS[@]}"; do
  entry="${REAL_IDS[$k]}"
  if [[ "$entry" == *=* ]]; then          # explicit droneN=drone_M
    REAL_IDS[$k]="${entry%%=*}"; CF_NAMES+=("${entry#*=}")
  elif [[ "$entry" =~ ^drone([0-9]+)$ ]]; then
    CF_NAMES+=("drone_${BASH_REMATCH[1]}")
  else
    die "--real entries must be droneN or droneN=drone_M, got '$entry'"
  fi
done
join() { local IFS=','; echo "$*"; }

# Validate before anything is armed: every real id must be in the testbed
# config, and its drone_N must exist AND be enabled in crazyflies.yaml -- an
# enabled-but-unpowered airframe stalls the server, and a disabled one is
# silently never connected while the launch waits on it.
PAIRS=()
for k in "${!REAL_IDS[@]}"; do PAIRS+=("${REAL_IDS[$k]}=${CF_NAMES[$k]}"); done
python3 - "$CONFIG_PATH" "$REPO/config/crazyflies.yaml" "${PAIRS[@]+"${PAIRS[@]}"}" <<'PY'
import sys, yaml
cfg_path, cf_path, *pairs = sys.argv[1:]
real = dict(p.split('=', 1) for p in pairs)      # algorithm id -> crazyswarm/VICON name
cfg = yaml.safe_load(open(cfg_path))
cfs = yaml.safe_load(open(cf_path))
ids = [d['id'] for d in cfg['drones']]
robots = cfs.get('robots', {})
errs = []
for rid, key in real.items():
    if rid not in ids:
        errs.append(f"{rid} is not in {cfg_path} (has {ids})")
    r = robots.get(key)
    if r is None:
        errs.append(f"{key} is not in config/crazyflies.yaml")
    elif not r.get('enabled', False):
        errs.append(f"{key} is 'enabled: false' in config/crazyflies.yaml")
for key, r in (robots.items() if real else []):   # no server without real drones
    if r.get('enabled', False) and key not in real.values():
        errs.append(f"{key} is enabled in crazyflies.yaml but not in --real: "
                    "the server will wait for it. Disable it or add it.")
uris = [r['uri'] for r in robots.values() if r.get('enabled', False)]
if len(uris) != len(set(uris)):
    errs.append(f"duplicate radio URIs among enabled robots: {uris}")
algo = cfg['algorithm']['name']
print(f"[fly] {cfg_path.split('/')[-1]}: {algo}, {len(ids)} agents")
for d in cfg['drones']:
    rid = d['id']; tag = 'REAL' if rid in real else 'sim '
    extra = ''
    if rid in real:
        key = real[rid]
        extra = f"  {key:8s} {robots.get(key, {}).get('uri', '?')}"
    x, y = d.get('initial_position', [0, 0])
    print(f"[fly]   {rid:8s} {tag}{extra}   start ({x:+.3f}, {y:+.3f})")
if errs:
    sys.stdout.flush()
    print("[fly] REFUSING TO LAUNCH:", file=sys.stderr)
    for e in errs: print("[fly]   -", e, file=sys.stderr)
    sys.exit(1)
PY

# ---- run directory ------------------------------------------------------------
RUN="$REPO/logs/${CONFIG_NAME}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN"
cp "$CONFIG_PATH" "$RUN/"
log "logs -> ${RUN#$REPO/}"

LAUNCH_ARGS=(
  "config:=$CONFIG_PATH"
  "hw_drone:=$(join "${REAL_IDS[@]+"${REAL_IDS[@]}"}")"
  "cf_name:=$(join "${CF_NAMES[@]+"${CF_NAMES[@]}"}")"
  "mocap_name:=$(join "${CF_NAMES[@]+"${CF_NAMES[@]}"}")"
  "gui:=$GUI"
)
[[ -n "$DURATION" ]] && LAUNCH_ARGS+=("flight_duration:=$DURATION")
LAUNCH_ARGS+=("${EXTRA[@]+"${EXTRA[@]}"}")
log "launch args: ${LAUNCH_ARGS[*]}"

if $CHECK_ONLY; then
  log "--check: not launching"; rmdir "$RUN" 2>/dev/null || true; exit 0
fi

# ---- crazyflies.yaml into the Crazyswarm2 share dir ---------------------------
# Only when it differs, so a repeat flight does not prompt for sudo.
if [[ ${#REAL_IDS[@]} -gt 0 ]]; then
  SHARE="/opt/ros/$ROS_DISTRO/share/crazyflie/config"
  for f in crazyflies.yaml motion_capture.yaml; do
    if [[ -d "$SHARE" ]] && ! cmp -s "$REPO/config/$f" "$SHARE/$f"; then
      log "installing config/$f -> $SHARE (sudo)"
      sudo cp "$REPO/config/$f" "$SHARE/$f"
    fi
  done
fi

if [[ ${#REAL_IDS[@]} -gt 0 ]] && ! $ASSUME_YES; then
  echo
  echo "  emergency stop from another terminal:"
  echo "    ros2 topic pub --once /sim/abort std_msgs/String '{data: manual}'"
  echo
  read -r -p "  press enter to fly, ctrl-c to abort... "
fi

# ---- process management -------------------------------------------------------
PIDS=(); NAMES=()
start() {   # start NAME CMD... -- background, output to console and to a log
  local name="$1"; shift
  log "starting $name"
  ( "$@" 2>&1 | tee "$RUN/$name.log" ) &
  PIDS+=($!); NAMES+=("$name")
}
# The subshell owns the pipeline, so signal the whole process group of it.
stop() {   # stop INDEX TIMEOUT
  local pid="${PIDS[$1]}" name="${NAMES[$1]}" t="$2"
  kill -0 "$pid" 2>/dev/null || return 0
  log "stopping $name"
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  for ((i = 0; i < t * 10; i++)); do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.1
  done
  log "$name did not exit in ${t}s, killing"
  kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
}
set -m   # job control on, so each background pipeline gets its own process group

shutting_down=false
shutdown() {
  $shutting_down && return; shutting_down=true
  trap - INT TERM
  echo
  # Reverse order of start(), so the flight lands before the server goes.
  for ((k = ${#PIDS[@]} - 1; k >= 0; k--)); do
    case "${NAMES[$k]}" in
      flight)  stop "$k" 25 ;;   # takeoff hand-back + LAND_DURATION + margin
      *)       stop "$k" 10 ;;
    esac
  done
  log "done. logs in ${RUN#$REPO/}"
}
trap shutdown INT TERM

# 1. Crazyswarm2, then wait for the VICON bridge to actually publish.
if [[ ${#REAL_IDS[@]} -gt 0 ]]; then
  start crazyswarm ros2 launch crazyflie launch.py backend:=cflib
  log "waiting for /poses"
  for ((i = 0; i < 600; i++)); do
    if ros2 topic list 2>/dev/null | grep -qx '/poses'; then break; fi
    kill -0 "${PIDS[0]}" 2>/dev/null || { log "crazyswarm exited"; shutdown; exit 1; }
    sleep 0.1
  done
  ros2 topic list 2>/dev/null | grep -qx '/poses' || { log "/poses never appeared (60s)"; shutdown; exit 1; }
  # Give the server a moment to finish parameter setup on every airframe;
  # crazyflie_node's Crazyswarm() client blocks on its services anyway.
  sleep 3
fi

# 2. The flight.
start flight ros2 launch drone_testbed hardware_hybrid.launch.py "${LAUNCH_ARGS[@]}"
FLIGHT_IDX=$(( ${#PIDS[@]} - 1 ))

# 3. Recorders. metrics_recorder analyses on Ctrl-C, so it must be stopped
#    gracefully, which stop() does.
if $METRICS; then
  sleep 2
  start metrics python3 "$REPO/tools/metrics_recorder.py" --config "$CONFIG_PATH" --out-dir "$RUN"
fi
if $RECORD; then
  for i in "${!REAL_IDS[@]}"; do
    start "record_${REAL_IDS[$i]}" python3 "$REPO/tools/flight_recorder.py" \
      --drone-id "${REAL_IDS[$i]}" --cf-name "${CF_NAMES[$i]}" \
      --out "$RUN/flight_${REAL_IDS[$i]}.txt"
  done
fi

# Block until the flight launch ends on its own (flight_duration elapsed, or
# an abort landed everything) or until Ctrl-C arrives.
while kill -0 "${PIDS[$FLIGHT_IDX]}" 2>/dev/null; do sleep 0.5; done
log "flight launch exited"
shutdown
