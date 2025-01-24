#!/usr/bin/env python3

# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import math
import time
import psutil
import numpy as np

import rclpy
from simple_node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros.ros_logs import set_ros_loggers
from yasmin_viewer import YasminViewerPub
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL
from moon_exploration.states import ExplorationFSM
from moon_exploration.states import GeneratePointsState

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from gazebo_msgs.srv import GetEntityState
from moon_exploration_msgs.action import RunMoonExploration


class MoonExplorationNode(Node):

    def __init__(self) -> None:
        super().__init__("moon_exploration_node")

        self.init_pose = None
        self.gt_init_pose = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.gz_client = self.create_client(GetEntityState, "/get_entity_state")

        self.declare_parameter("radius", 10)
        self.radius = self.get_parameter("radius").get_parameter_value().integer_value

        self.declare_parameter("waypoints_nums", 30)
        self.waypoints_nums = (
            self.get_parameter("waypoints_nums").get_parameter_value().integer_value
        )

        self.declare_parameter("exploration_mode", "spiral")
        self.exploration_mode = (
            self.get_parameter("exploration_mode").get_parameter_value().string_value
        )

        self.declare_parameter("nav2_planner", "SmacHybrid")
        self.nav2_planner = (
            self.get_parameter("nav2_planner").get_parameter_value().string_value
        )

        self.declare_parameter("nav2_controller", "RPP")
        self.nav2_controller = (
            self.get_parameter("nav2_controller").get_parameter_value().string_value
        )

        self.declare_parameter("use_low_moon", False)
        self.use_low_moon = (
            self.get_parameter("use_low_moon").get_parameter_value().bool_value
        )

        # create a state machine
        set_ros_loggers()

        self.sm = StateMachine(outcomes=[SUCCEED, CANCEL])

        # add states
        self.sm.add_state(
            "GENERATING_POINTS",
            GeneratePointsState(),
            transitions={
                SUCCEED: "EXPLORING",
            },
        )

        self.sm.add_state(
            "EXPLORING",
            ExplorationFSM(self),
            transitions={SUCCEED: SUCCEED, CANCEL: CANCEL},
        )

        YasminViewerPub("MOON_EXPLORATION", self.sm, node=self)

        pcl_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.pcl_size = -1
        self.create_subscription(
            PointCloud2, "/cloud_map", self.cloud_map_cb, pcl_qos_profile
        )

        map_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.map_size = -1
        self.create_subscription(OccupancyGrid, "/map", self.map_cb, map_qos_profile)

        self.exploration_server = self.create_action_server(
            RunMoonExploration,
            "run_moon_exploration",
            self.run_moon_exploration,
            self.cancel_moon_exploration,
        )

    def cloud_map_cb(self, msg: PointCloud2) -> None:
        data = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)
        xyz = data[:, :3]
        self.pcl_size = xyz.shape[0]

    def map_cb(self, msg: OccupancyGrid) -> None:
        data = [cell for cell in msg.data if cell != -1]
        self.map_size = len(data) * msg.info.resolution * msg.info.resolution
        self.map_cells = len(data)

    def get_data(self) -> None:

        # get data
        if self.pcl_size <= 0 or self.map_size <= 0:
            return

        try:
            tr = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except TransformException as ex:
            return

        req = GetEntityState.Request()
        req.name = "rover"
        res: GetEntityState.Response = self.gz_client.call(req)

        # init poses
        if self.gt_init_pose is None:
            self.gt_init_pose = res.state.pose

        if self.init_pose is None:
            self.init_pose = tr.transform

        # data
        t = time.time()
        cpu = psutil.cpu_percent()
        ram = psutil.virtual_memory()[2]

        x = tr.transform.translation.x - self.init_pose.translation.x
        y = tr.transform.translation.y - self.init_pose.translation.y
        z = tr.transform.translation.z - self.init_pose.translation.z

        angle = (
            euler_from_quaternion(
                [
                    tr.transform.rotation.x,
                    tr.transform.rotation.y,
                    tr.transform.rotation.z,
                    tr.transform.rotation.w,
                ]
            )[2]
            - euler_from_quaternion(
                [
                    self.init_pose.rotation.x,
                    self.init_pose.rotation.y,
                    self.init_pose.rotation.z,
                    self.init_pose.rotation.w,
                ]
            )[2]
        )

        gt_x = res.state.pose.position.x - self.gt_init_pose.position.x
        gt_y = res.state.pose.position.y - self.gt_init_pose.position.y
        gt_z = res.state.pose.position.z - self.gt_init_pose.position.z

        gt_angle = (
            euler_from_quaternion(
                [
                    res.state.pose.orientation.x,
                    res.state.pose.orientation.y,
                    res.state.pose.orientation.z,
                    res.state.pose.orientation.w,
                ]
            )[2]
            - euler_from_quaternion(
                [
                    self.gt_init_pose.orientation.x,
                    self.gt_init_pose.orientation.y,
                    self.gt_init_pose.orientation.z,
                    self.gt_init_pose.orientation.w,
                ]
            )[2]
        )

        p_error = math.sqrt(
            math.pow(x - gt_x, 2) + math.pow(y - gt_y, 2) + math.pow(z - gt_z, 2)
        )

        diff = abs(angle - gt_angle) % (2 * math.pi)
        o_error = min(diff, 2 * math.pi - diff)

        distance = 0
        gt_distance = 0
        elapsed_time = 0

        if self.data:
            distance = math.sqrt(
                math.pow(x - self.data[-1]["x"], 2)
                + math.pow(y - self.data[-1]["y"], 2)
                + math.pow(z - self.data[-1]["z"], 2)
            )

            gt_distance = math.sqrt(
                math.pow(gt_x - self.data[-1]["gt_x"], 2)
                + math.pow(gt_y - self.data[-1]["gt_y"], 2)
                + math.pow(gt_z - self.data[-1]["gt_z"], 2)
            )

            elapsed_time = t - self.data[-1]["timestamp"]

        self.data.append(
            {
                "timestamp": t,
                "elapsed_time": elapsed_time,
                "cpu": cpu,
                "ram": ram,
                "x": x,
                "y": y,
                "z": z,
                "angle": angle,
                "gt_x": gt_x,
                "gt_y": gt_y,
                "gt_z": gt_z,
                "gt_angle": gt_angle,
                "position_error": p_error,
                "orientation_error": o_error,
                "distance": distance,
                "gt_distance": gt_distance,
                "pcl_size": self.pcl_size,
                "map_cells": self.map_cells,
                "map_size": self.map_size,
            }
        )

    def run_moon_exploration(self, goal_handle) -> None:

        self.get_logger().info("Starting Exploration Experiment")

        blackboard = Blackboard()
        result = RunMoonExploration.Result()

        blackboard.exploration_method = self.exploration_mode  # spiral, radial
        blackboard.max_radius = self.radius
        blackboard.max_waypoints = self.waypoints_nums
        blackboard.sample_dir = "~/exploration_samples"

        self.data = []
        timer = self.create_timer(0.5, self.get_data)

        result.succeed = self.sm(blackboard) == SUCCEED
        timer.destroy()

        # save results
        moon_type = "moon"

        if self.use_low_moon:
            moon_type = "low_moon"

        csv_path = (
            f"exploration_{self.radius}_{self.waypoints_nums}_{self.exploration_mode}"
        )
        csv_path += f"_{self.nav2_planner}_{self.nav2_controller}_{moon_type}.csv"

        parsed_data = "Timestamp,Time,CPU,RAM,x,y,z,Angle,GT x,GT y,GT z,GT Angle,"
        parsed_data += "Position Error,Orientation Error,Distance,GT Distance,Cloud Map Size,Map Cells,Map Size\n"
        for ele in self.data:
            parsed_data += f"{ele['timestamp']},{ele['elapsed_time']},"
            parsed_data += f"{ele['cpu']},{ele['ram']},"
            parsed_data += f"{ele['x']},{ele['y']},{ele['z']},{ele['angle']},"
            parsed_data += f"{ele['gt_x']},{ele['gt_y']},{ele['gt_z']},{ele['gt_angle']},"
            parsed_data += f"{ele['position_error']},{ele['orientation_error']},"
            parsed_data += f"{ele['distance']},{ele['gt_distance']},{ele['pcl_size']},"
            parsed_data += f"{ele['map_cells']},{ele['map_size']}\n"

        f = open(csv_path, "a")
        f.write(parsed_data)
        f.close()

        goal_handle.succeed()
        return result

    def cancel_moon_exploration(self) -> None:
        self.sm.cancel_state()


def main():
    rclpy.init()
    node = MoonExplorationNode()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
