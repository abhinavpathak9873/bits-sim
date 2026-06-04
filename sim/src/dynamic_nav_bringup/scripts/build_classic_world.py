#!/usr/bin/env python3
import argparse
import math
import random
from pathlib import Path

import yaml


def pose_on_route(route, segment, progress):
    start = route[segment]
    end = route[(segment + 1) % len(route)]
    x = start[0] + (end[0] - start[0]) * progress
    y = start[1] + (end[1] - start[1]) * progress
    yaw = math.atan2(end[1] - start[1], end[0] - start[0])
    return x, y, yaw


def plugin_block(
    route, speed, z, segment, progress, sway_phase, sway_amplitude, sway_rate,
    bob_amplitude, speed_phase, avoid_radius, lane_offset, direction, static_obstacles
):
    waypoints = '\n'.join(
        f'        <waypoint><x>{point[0]:.3f}</x><y>{point[1]:.3f}</y></waypoint>' for point in route
    )
    obstacles = '\n'.join(
        '        <static_obstacle>'
        f'<x>{obstacle["x"]:.3f}</x><y>{obstacle["y"]:.3f}</y>'
        f'<sx>{obstacle["sx"]:.3f}</sx><sy>{obstacle["sy"]:.3f}</sy>'
        '</static_obstacle>'
        for obstacle in static_obstacles
    )
    return f"""
      <plugin name="scripted_motion" filename="libdynamic_nav_scripted_actor.so">
        <speed>{speed:.4f}</speed>
        <z>{z:.4f}</z>
        <segment>{segment}</segment>
        <progress>{progress:.4f}</progress>
        <direction>{direction:.1f}</direction>
        <sway_phase>{sway_phase:.4f}</sway_phase>
        <sway_amplitude>{sway_amplitude:.4f}</sway_amplitude>
        <sway_rate>{sway_rate:.4f}</sway_rate>
        <bob_amplitude>{bob_amplitude:.4f}</bob_amplitude>
        <speed_phase>{speed_phase:.4f}</speed_phase>
        <avoid_radius>{avoid_radius:.4f}</avoid_radius>
        <lane_offset>{lane_offset:.4f}</lane_offset>
        <robot_name>robot</robot_name>
        <robot_avoid_radius>{max(avoid_radius + 0.25, 1.00):.4f}</robot_avoid_radius>
        <robot_hard_radius>0.42</robot_hard_radius>
{waypoints}
{obstacles}
      </plugin>
"""


def apply_lane_offset(x, y, yaw, lane_offset):
    return x - math.sin(yaw) * lane_offset, y + math.cos(yaw) * lane_offset


def far_enough(x, y, radius, placed_actors):
    for other_x, other_y, other_radius in placed_actors:
        if math.hypot(x - other_x, y - other_y) < radius + other_radius + 0.85:
            return False
    return True


def clear_static_obstacles(x, y, radius, static_obstacles):
    padding = radius + 0.35
    for obstacle in static_obstacles:
        blocked_x = abs(x - obstacle['x']) <= obstacle['sx'] * 0.5 + padding
        blocked_y = abs(y - obstacle['y']) <= obstacle['sy'] * 0.5 + padding
        if blocked_x and blocked_y:
            return False
    return True


def pedestrian_model(name, x, y, z, yaw, radius, height, shirt_color, head_color, head_radius_cap, motion):
    torso_height = height * 0.72
    torso_radius = radius * 0.62
    head_radius = min(radius * 0.58, head_radius_cap)
    head_z = torso_height * 0.52
    return f"""
    <model name="{name}">
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose>
      <static>false</static>
      <link name="body">
        <kinematic>true</kinematic>
        <inertial>
          <mass>70.0</mass>
          <inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="collision">
          <geometry><cylinder><radius>{radius:.3f}</radius><length>{height:.3f}</length></cylinder></geometry>
        </collision>
        <visual name="torso">
          <pose>0 0 {-height * 0.08:.3f} 0 0 0</pose>
          <geometry><cylinder><radius>{torso_radius:.3f}</radius><length>{torso_height:.3f}</length></cylinder></geometry>
          <material><ambient>{shirt_color}</ambient><diffuse>{shirt_color}</diffuse></material>
        </visual>
        <visual name="head">
          <pose>0 0 {head_z:.3f} 0 0 0</pose>
          <geometry><sphere><radius>{head_radius:.3f}</radius></sphere></geometry>
          <material><ambient>{head_color}</ambient><diffuse>{head_color}</diffuse></material>
        </visual>
      </link>
{motion}
    </model>
"""


def cart_model(name, x, y, z, yaw, color, motion):
    return f"""
    <model name="{name}">
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose>
      <static>false</static>
      <link name="body">
        <kinematic>true</kinematic>
        <inertial>
          <mass>25.0</mass>
          <inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>0.75 0.45 0.55</size></box></geometry>
        </collision>
        <visual name="cart_body">
          <geometry><box><size>0.75 0.45 0.55</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
        <visual name="handle">
          <pose>-0.45 0 0.25 0 0 0</pose>
          <geometry><box><size>0.08 0.55 0.08</size></box></geometry>
          <material><ambient>0.12 0.12 0.12 1</ambient><diffuse>0.12 0.12 0.12 1</diffuse></material>
        </visual>
      </link>
{motion}
    </model>
"""


def add_pedestrians(args, scenario, static_obstacles, rng, profile, speed_scale, insertions):
    shirt_colors = [
        '0.20 0.32 0.75 1',
        '0.20 0.55 0.42 1',
        '0.64 0.27 0.30 1',
        '0.48 0.40 0.74 1',
        '0.74 0.50 0.22 1',
        '0.25 0.25 0.28 1',
    ]
    head_colors = [
        '0.74 0.55 0.38 1',
        '0.90 0.72 0.55 1',
        '0.48 0.31 0.20 1',
        '0.64 0.43 0.29 1',
    ]
    cart_limit = min(int(profile['carts']), max(0, args.max_actors // 4))
    if int(profile['carts']) > 0 and args.max_actors >= 4:
        cart_limit = max(1, cart_limit)
    pedestrian_limit = max(0, args.max_actors - cart_limit)
    placed_actors = [(args.robot_x, args.robot_y, 0.65)]
    for index in range(min(int(profile['pedestrians']), pedestrian_limit)):
        speed = rng.uniform(0.55, 1.25) * speed_scale
        radius = rng.uniform(0.16, 0.22) * args.human_scale
        height = rng.uniform(1.55, 1.85) * args.human_scale
        route = None
        segment = 0
        progress = 0.0
        lane_offset = 0.0
        x = y = yaw = 0.0
        for _ in range(80):
            route = [tuple(point) for point in rng.choice(scenario['pedestrian_routes'])]
            if rng.random() < 0.5:
                route = list(reversed(route))
            segment = rng.randrange(max(1, len(route) - 1))
            progress = rng.random()
            lane_options = [-1.05, -0.64, 0.64, 1.05]
            lane_offset = lane_options[(index + rng.randrange(len(lane_options))) % len(lane_options)]
            lane_offset += rng.uniform(-0.04, 0.04)
            route_x, route_y, yaw = pose_on_route(route, segment, progress)
            x, y = apply_lane_offset(route_x, route_y, yaw, lane_offset)
            if far_enough(x, y, radius, placed_actors) and clear_static_obstacles(x, y, radius, static_obstacles):
                break
        sway_phase = rng.uniform(0.0, math.tau)
        sway_amplitude = rng.uniform(0.004, 0.018)
        sway_rate = rng.uniform(0.8, 1.6)
        bob_amplitude = rng.uniform(0.006, 0.018)
        speed_phase = rng.uniform(0.0, math.tau)
        motion = ''
        if args.actor_control == 'plugin':
            motion = plugin_block(
                route, speed, height / 2.0, segment, progress,
                sway_phase, sway_amplitude, sway_rate, bob_amplitude, speed_phase,
                radius + 0.70 * args.human_scale, lane_offset, 1.0, static_obstacles
            )
        insertions.append(pedestrian_model(
            f'pedestrian_{index:03d}',
            x,
            y,
            height / 2.0,
            yaw,
            radius,
            height,
            shirt_colors[index % len(shirt_colors)],
            head_colors[(index + args.seed) % len(head_colors)],
            0.115 * args.human_scale,
            motion,
        ))
        placed_actors.append((x, y, radius))
    return cart_limit


def add_carts(args, scenario, static_obstacles, rng, cart_limit, speed_scale, insertions):
    for index in range(cart_limit):
        route = [tuple(point) for point in scenario['cart_routes'][index % len(scenario['cart_routes'])]]
        progress = ((index % 4) / 4.0 + 0.12) % 1.0
        speed = rng.uniform(0.35, 0.75) * speed_scale
        segment = rng.randrange(max(1, len(route) - 1))
        sway_phase = rng.uniform(0.0, math.tau)
        sway_amplitude = rng.uniform(0.0, 0.035)
        sway_rate = rng.uniform(0.25, 0.55)
        speed_phase = rng.uniform(0.0, math.tau)
        x, y, yaw = pose_on_route(route, segment, progress)
        if not clear_static_obstacles(x, y, 0.45, static_obstacles):
            progress = (progress + 0.35) % 1.0
            x, y, yaw = pose_on_route(route, segment, progress)
        motion = ''
        if args.actor_control == 'plugin':
            motion = plugin_block(
                route, speed, 0.75 / 2.0, segment, progress,
                sway_phase, sway_amplitude, sway_rate, 0.0, speed_phase, 0.80, 0.0, 1.0, static_obstacles
            )
        insertions.append(cart_model(
            f'service_cart_{index:03d}', x, y, 0.75 / 2.0, yaw, '0.85 0.58 0.16 1', motion
        ))


def build_world(args):
    world_text = args.source_world.read_text(encoding='utf-8')
    insertions = [f"""
    <include>
      <uri>model://{args.model_name}</uri>
      <name>robot</name>
      <pose>{args.robot_x:.3f} {args.robot_y:.3f} 0.01 0 0 {args.robot_yaw:.3f}</pose>
    </include>
"""]
    if args.dynamic_obstacles:
        scenario = yaml.safe_load(args.scenario.read_text(encoding='utf-8'))
        if not isinstance(scenario, dict):
            raise SystemExit(f'Scenario config must contain a YAML mapping: {args.scenario}')
        profile = scenario['density_profiles'][args.crowd_density]
        static_obstacles = [
            {
                'x': float(obstacle['x']),
                'y': float(obstacle['y']),
                'sx': float(obstacle['sx']),
                'sy': float(obstacle['sy']),
            }
            for obstacle in scenario.get('static_obstacles', [])
        ]
        speed_scale = {
            'static': 0.0,
            'low': 0.55,
            'medium': 1.0,
            'mid': 1.0,
            'high': 1.55,
        }[args.speed_profile]
        rng = random.Random(args.seed)
        cart_limit = add_pedestrians(args, scenario, static_obstacles, rng, profile, speed_scale, insertions)
        add_carts(args, scenario, static_obstacles, rng, cart_limit, speed_scale, insertions)
    if '</world>' not in world_text:
        raise SystemExit(f'No </world> tag found in {args.source_world}')
    args.destination_world.write_text(
        world_text.replace('</world>', ''.join(insertions) + '\n  </world>', 1),
        encoding='utf-8',
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source_world', type=Path)
    parser.add_argument('destination_world', type=Path)
    parser.add_argument('model_name')
    parser.add_argument('scenario', type=Path)
    parser.add_argument('--dynamic-obstacles', action='store_true')
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--crowd-density', required=True)
    parser.add_argument('--max-actors', type=int, required=True)
    parser.add_argument('--speed-profile', required=True)
    parser.add_argument('--actor-control', required=True)
    parser.add_argument('--robot-x', type=float, required=True)
    parser.add_argument('--robot-y', type=float, required=True)
    parser.add_argument('--robot-yaw', type=float, required=True)
    parser.add_argument('--human-scale', type=float, required=True)
    build_world(parser.parse_args())


if __name__ == '__main__':
    main()
