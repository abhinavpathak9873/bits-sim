#!/usr/bin/env bash
set -eo pipefail

WORLD="${WORLD:-mall}"
ROBOT="${ROBOT:-tb3_burger}"
SEED="${SEED:-1}"
GUI="${GUI:-true}"
DYNAMIC_OBSTACLES="${DYNAMIC_OBSTACLES:-false}"
CROWD_DENSITY="${CROWD_DENSITY:-low}"
SPEED_PROFILE="${SPEED_PROFILE:-low}"
MAX_ACTORS="${MAX_ACTORS:-8}"
ACTOR_UPDATE_RATE="${ACTOR_UPDATE_RATE:-10.0}"
SPAWN_METHOD="${SPAWN_METHOD:-include}"
ACTOR_CONTROL="${ACTOR_CONTROL:-plugin}"
HUMAN_SCALE="${HUMAN_SCALE:-1.0}"
GOAL_MARKER="${GOAL_MARKER:-true}"
if [[ "${GUI}" == "true" && -z "${DISPLAY:-}" && -S /tmp/.X11-unix/X0 ]]; then
  export DISPLAY=:0
fi

source /usr/share/gazebo/setup.bash
source /opt/ros/humble/setup.bash
if [[ -f /sim/install/setup.bash ]]; then
  source /sim/install/setup.bash
fi
set -u
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

world_file="/sim/install/dynamic_nav_worlds/share/dynamic_nav_worlds/worlds/classic/${WORLD}.world"
scenario_file="/sim/install/dynamic_nav_worlds/share/dynamic_nav_worlds/config/scenarios/${WORLD}.yaml"
tb3_share="$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo"

case "${ROBOT}" in
  tb3_burger) model_name="turtlebot3_burger" ;;
  tb3_waffle) model_name="turtlebot3_waffle" ;;
  tb3_waffle_pi) model_name="turtlebot3_waffle_pi" ;;
  *) echo "Unsupported ROBOT=${ROBOT}. Use tb3_burger, tb3_waffle, or tb3_waffle_pi." >&2; exit 2 ;;
esac

model_file="${tb3_share}/models/${model_name}/model.sdf"
export GAZEBO_MODEL_PATH="${tb3_share}/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="/sim/install/dynamic_nav_gazebo_plugins/lib:${GAZEBO_PLUGIN_PATH:-}"
if [[ ! -f "${world_file}" ]]; then
  echo "World file not found: ${world_file}" >&2
  exit 2
fi
if [[ ! -f "${model_file}" ]]; then
  echo "TurtleBot3 model file not found: ${model_file}" >&2
  exit 2
fi
if [[ ! -f "${scenario_file}" ]]; then
  echo "Scenario config not found: ${scenario_file}" >&2
  exit 2
fi

read -r robot_x robot_y robot_yaw < <(python3 - "${scenario_file}" <<'PY'
import os
import sys
import yaml

with open(sys.argv[1], 'r', encoding='utf-8') as stream:
    scenario = yaml.safe_load(stream) or {}
start = scenario.get('robot_start', {})

def read_pose(env_name, key, fallback):
    return float(os.environ.get(env_name, start.get(key, fallback)))

print(
    f"{read_pose('ROBOT_X', 'x', 0.0):.4f} "
    f"{read_pose('ROBOT_Y', 'y', 0.0):.4f} "
    f"{read_pose('ROBOT_YAW', 'yaw', 0.0):.4f}"
)
PY
)

cleanup() {
  jobs -pr | xargs -r kill 2>/dev/null || true
  [[ -n "${temp_world:-}" && -f "${temp_world}" ]] && rm -f "${temp_world}"
}
trap cleanup EXIT INT TERM

server_world="${world_file}"
if [[ "${SPAWN_METHOD}" == "include" ]]; then
  temp_world="$(mktemp /tmp/dynamic_nav_${WORLD}_${ROBOT}_XXXX.world)"
  build_args=(
    "${world_file}" "${temp_world}" "${model_name}" "${scenario_file}"
    --seed "${SEED}"
    --crowd-density "${CROWD_DENSITY}"
    --max-actors "${MAX_ACTORS}"
    --speed-profile "${SPEED_PROFILE}"
    --actor-control "${ACTOR_CONTROL}"
    --robot-x "${robot_x}"
    --robot-y "${robot_y}"
    --robot-yaw "${robot_yaw}"
    --human-scale "${HUMAN_SCALE}"
  )
  if [[ "${DYNAMIC_OBSTACLES}" == "true" ]]; then
    build_args+=(--dynamic-obstacles)
  fi
  python3 "${script_dir}/build_classic_world.py" "${build_args[@]}"
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
    -x "${robot_x}" -y "${robot_y}" -z 0.01 -Y "${robot_yaw}" \
    -robot_namespace robot \
    -timeout 90
else
  echo "Started Gazebo with ${ROBOT} included directly in the world."
fi

if [[ "${DYNAMIC_OBSTACLES}" == "true" && "${ACTOR_CONTROL}" != "plugin" ]]; then
  ros2 run dynamic_nav_actors actor_manager --ros-args \
    -p world:="${WORLD}" \
    -p backend:=classic \
    -p crowd_density:="${CROWD_DENSITY}" \
    -p dynamic_obstacles:=true \
    -p seed:="${SEED}" \
    -p max_actors:="${MAX_ACTORS}" \
    -p update_rate_hz:="${ACTOR_UPDATE_RATE}" \
    -p speed_profile:="${SPEED_PROFILE}" \
    -p control_mode:=existing_models &
fi

if [[ "${GOAL_MARKER}" == "true" ]]; then
  ros2 run dynamic_nav_benchmark goal_marker --ros-args \
    -p goal_topic:=/goal_pose \
    -p marker_name:=dynamic_nav_goal_marker \
    > /tmp/dynamic_nav_goal_marker.log 2>&1 &
fi

echo "Simulation running. DISPLAY=${DISPLAY:-unset}, WORLD=${WORLD}, ROBOT=${ROBOT}, ROBOT_POSE=${robot_x},${robot_y},${robot_yaw}, GUI=${GUI}, SPAWN_METHOD=${SPAWN_METHOD}, DYNAMIC_OBSTACLES=${DYNAMIC_OBSTACLES}, SPEED_PROFILE=${SPEED_PROFILE}, ACTOR_CONTROL=${ACTOR_CONTROL}, HUMAN_SCALE=${HUMAN_SCALE}, GOAL_MARKER=${GOAL_MARKER}"
echo "Use Ctrl-C in this terminal to stop it."
wait "${server_pid}"
