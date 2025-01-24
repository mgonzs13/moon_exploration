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


from yasmin import State
from yasmin import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped


class GetNextPointState(State):

    def __init__(self) -> None:

        super().__init__([SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard) -> str:

        if blackboard.waypoints:
            wp = blackboard.waypoints.pop(0)
            orientation = quaternion_from_euler(0, 0, wp[2])

            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.pose.position.x = wp[0]
            msg.pose.position.y = wp[1]
            msg.pose.orientation.x = orientation[0]
            msg.pose.orientation.y = orientation[1]
            msg.pose.orientation.z = orientation[2]
            msg.pose.orientation.w = orientation[3]

            blackboard.waypoint = msg
            return SUCCEED

        return ABORT
