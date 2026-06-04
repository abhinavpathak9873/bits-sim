#!/usr/bin/env bash
set -eo pipefail

WORLD="${WORLD:-mall}"
ROBOT="${ROBOT:-tb3_burger}"
SEED="${SEED:-1}"
GUI="${GUI:-true}"
DYNAMIC_OBSTACLES="${DYNAMIC_OBSTACLES:-false}"
CROWD_DENSITY="${CROWD_DENSITY:-low}"
SPAWN_METHOD="${SPAWN_METHOD:-include}"
if [[ "${GUI}" == "true" && -z "${DISPLAY:-}" && -S /tmp/.X11-unix/X0 ]]; then
  export DISPLAY=:0
fi

source /usr/share/gazebo/setup.bash
source /opt/ros/humble/setup.bash
if [[ -f /sim/install/setup.bash ]]; then
  source /sim/install/setup.bash
fi
set -u

world_file="/sim/install/dynamic_nav_worlds/share/dynamic_nav_worlds/worlds/classic/${WORLD}.world"
tb3_share="$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo"

case "${ROBOT}" in
  tb3_burger) model_name="turtlebot3_burger" ;;
  tb3_waffle) model_name="turtlebot3_waffle" ;;
  tb3_waffle_pi) model_name="turtlebot3_waffle_pi" ;;
  *) echo "Unsupported ROBOT=${ROBOT}. Use tb3_burger, tb3_waffle, or tb3_waffle_pi." >&2; exit 2 ;;
esac

model_file="${tb3_share}/models/${model_name}/model.sdf"
export GAZEBO_MODEL_PATH="${tb3_share}/models:${GAZEBO_MODEL_PATH:-}"
if [[ ! -f "${world_file}" ]]; then
  echo "World file not found: ${world_file}" >&2
  exit 2
fi
if [[ ! -f "${model_file}" ]]; then
  echo "TurtleBot3 model file not found: ${model_file}" >&2
  exit 2
fi

cleanup() {
  jobs -pr | xargs -r kill 2>/dev/null || true
  [[ -n "${temp_world:-}" && -f "${temp_world}" ]] && rm -f "${temp_world}"
}
trap cleanup EXIT INT TERM

server_world="${world_file}"
if [[ "${SPAWN_METHOD}" == "include" ]]; then
  temp_world="$(mktemp /tmp/dynamic_nav_${WORLD}_${ROBOT}_XXXX.world)"
  python3 - "${world_file}" "${temp_world}" "${model_name}" <<'PY'
from pathlib import Path
import sys

src = Path(sys.argv[1])
dst = Path(sys.argv[2])
model = sys.argv[3]
text = src.read_text(encoding='utf-8')
include = f"""
    <include>
      <uri>model://{model}</uri>
      <name>robot</name>
      <pose>-11.5 -7.0 0.01 0 0 0</pose>
    </include>
"""
if '</world>' not in text:
    raise SystemExit(f'No </world> tag found in {src}')
dst.write_text(text.replace('</world>', include + '\n  </world>', 1), encoding='utf-8')
PY
  server_world="${temp_world}"
fi

gzserver "${server_world}" \
  --seed="${SEED}" \
  -s /opt/ros/humble/lib/libgazebo_ros_init.so \
  -s /opt/ros/humble/lib/libgazebo_ros_factory.so \
  -s /opt/ros/humble/lib/libgazebo_ros_state.so \
  -s /opt/ros/humble/lib/libgazebo_ros_force_system.so \
  > /tmp/dynamic_nav_gzserver.log 2>&1 &
server_pid=$!

if [[ "${GUI}" == "true" ]]; then
  sleep 2
  gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so \
    > /tmp/dynamic_nav_gzclient.log 2>&1 &
fi

if [[ "${SPAWN_METHOD}" == "factory" ]]; then
  echo "Starting Gazebo and waiting briefly before spawning the robot..."
  sleep 8
  ros2 run gazebo_ros spawn_entity.py \
    -entity robot \
    -file "${model_file}" \
    -x -11.5 -y -7.0 -z 0.01 -Y 0.0 \
    -robot_namespace robot \
    -timeout 90
else
  echo "Started Gazebo with ${ROBOT} included directly in the world."
fi

if [[ "${DYNAMIC_OBSTACLES}" == "true" ]]; then
  ros2 run dynamic_nav_actors actor_manager --ros-args \
    -p world:="${WORLD}" \
    -p backend:=classic \
    -p crowd_density:="${CROWD_DENSITY}" \
    -p dynamic_obstacles:=true \
    -p seed:="${SEED}" \
    -p max_actors:=4 \
    -p update_rate_hz:=2.0 &
fi

echo "Simulation running. DISPLAY=${DISPLAY:-unset}, WORLD=${WORLD}, ROBOT=${ROBOT}, GUI=${GUI}, SPAWN_METHOD=${SPAWN_METHOD}"
echo "Use Ctrl-C in this terminal to stop it."
wait "${server_pid}"
