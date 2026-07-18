#!/usr/bin/env python3

import argparse
import math
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description='Move a traffic cone in a running Gazebo Fortress world.')
    parser.add_argument('--world', default='origincar_5m_map_world')
    parser.add_argument('--name', default='traffic_cone')
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--z', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--timeout', type=int, default=3000)
    args = parser.parse_args()

    half_yaw = args.yaw * 0.5
    req = (
        f'name: "{args.name}" '
        f'position {{x: {args.x} y: {args.y} z: {args.z}}} '
        f'orientation {{z: {math.sin(half_yaw)} w: {math.cos(half_yaw)}}}'
    )
    service = f'/world/{args.world}/set_pose'
    cmd = [
        'ign', 'service',
        '-s', service,
        '--reqtype', 'ignition.msgs.Pose',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', str(args.timeout),
        '--req', req,
    ]
    result = subprocess.run(cmd, check=False)
    return result.returncode


if __name__ == '__main__':
    sys.exit(main())
