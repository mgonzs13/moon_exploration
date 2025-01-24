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


import os
import cv2
from cv_bridge import CvBridge
from simple_node import Node
from rclpy.qos import qos_profile_sensor_data
from yasmin_ros import MonitorState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin.blackboard import Blackboard
from sensor_msgs.msg import Image


class TakeSampleState(MonitorState):

    def __init__(self, node: Node) -> None:

        super().__init__(
            Image,
            "/camera/image_raw",
            [SUCCEED, ABORT],
            self.take_sample,
            qos_profile_sensor_data,
            node=node,
        )
        self.cv2_bridge = CvBridge()
        self.samples_counter = 0
        self.node = node

    def take_sample(self, blackboard: Blackboard, msg: Image) -> str:

        image = self.cv2_bridge.imgmsg_to_cv2(msg)
        abs_path = os.path.expanduser(blackboard.sample_dir)

        if not os.path.exists(abs_path):
            os.makedirs(abs_path)

        succ = cv2.imwrite(f"{abs_path}/img_{self.samples_counter}.png", image)

        if succ:
            # blackboard.progress_bar.update(1)
            # self.node.get_logger().info(str(blackboard.progress_bar))

            self.samples_counter += 1
            return SUCCEED

        return ABORT
