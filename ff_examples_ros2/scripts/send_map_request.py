#!/usr/bin/env python3
 
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import json
import argparse

import rclpy
from rclpy.node import Node

from rmf_fleet_msgs.msg import MapRequest



def main(argv = sys.argv):
    '''
    Example map request:
    - fleet_name: magni
    - robot_name: magni123
    - map_number: 2
    '''

    default_fleet_name = 'fleet_name'
    default_robot_name = 'robot_name'
    default_map_number = 1
    default_topic_name = 'map_request'
    default_level_name = 'level1'

    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--fleet-name', default=default_fleet_name)
    parser.add_argument('-r', '--robot-name', default=default_robot_name)
    parser.add_argument('-l', '--level-name', default=default_level_name)
    parser.add_argument('-i', '--map-number', default=default_map_number)
    parser.add_argument('-t', '--topic-name', default=default_topic_name)
    args = parser.parse_args(argv[1:])

    print('fleet_name: {}'.format(args.fleet_name))
    print('robot_name: {}'.format(args.robot_name))
    print('level_name: {}'.format(args.level_name))
    print('map_number: {}'.format(args.map_number))
    print('topic_name: {}'.format(args.topic_name))

    rclpy.init()
    node = rclpy.create_node('send_map_request_node')
    pub = node.create_publisher(MapRequest, args.topic_name, 10)

    msg = MapRequest()
    msg.fleet_name = args.fleet_name
    msg.robot_name = args.robot_name
    msg.level_name = args.level_name
    msg.map_number = int(args.map_number)

    rclpy.spin_once(node, timeout_sec=2.0)
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.5)
    print('all done!')


if __name__ == '__main__':
    main(sys.argv)
