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
import numpy as np
from typing import List
from yasmin import State
from yasmin import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED


class GeneratePointsState(State):

    def __init__(self) -> None:
        super().__init__([SUCCEED])

    def generate_spiral_points(
        self, size: int = 20, num_points: int = 20, start_angle: int = 90
    ) -> List[List[float]]:

        x, y = 0, 0
        angle = start_angle
        points = [(x, y, angle)]

        for i in range(1, num_points + 1):
            # Calculate the distance for each step in the spiral
            step = size * math.sqrt(i / num_points)

            # Calculate the new coordinates based on the current angle
            x += step * math.cos(math.radians(angle))
            y += step * math.sin(math.radians(angle))

            # Increment the angle (90 degrees for a perfect circular spiral)
            angle += 90
            # Append the new point to the list of points
            points.append((x, y, np.radians(angle - 90)))

        return points[1:]

    def generate_radial_points(
        self, radius: int = 10, num_points: int = 20, start_angle: int = 0
    ) -> List[List[float]]:

        points = []
        angle_increment = 2 * math.pi / num_points

        for i in range(num_points):
            angle = np.radians(start_angle) + i * angle_increment
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            points.append((x, y, angle))

            points.append((0.0, 0.0, angle - np.pi))

        return points

    def execute(self, blackboard: Blackboard) -> str:
        blackboard.waypoints = []

        if blackboard.exploration_method == "spiral":
            blackboard.waypoints.extend(
                self.generate_spiral_points(
                    blackboard.max_radius * 2, blackboard.max_waypoints
                )
            )

        elif blackboard.exploration_method == "radial":
            blackboard.waypoints.extend(
                self.generate_radial_points(
                    blackboard.max_radius, int(blackboard.max_waypoints / 2)
                )
            )

        return SUCCEED
