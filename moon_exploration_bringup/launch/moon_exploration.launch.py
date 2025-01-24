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
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1"
    )

    radius = LaunchConfiguration("radius")
    radius_cmd = DeclareLaunchArgument(
        "radius",
        default_value="10",
        description="Integer value of the radius to explore",
    )

    waypoints_nums = LaunchConfiguration("waypoints_nums")
    waypoints_nums_cmd = DeclareLaunchArgument(
        "waypoints_nums",
        default_value="30",
        description="Number of waypoints to explore",
    )

    exploration_mode = LaunchConfiguration("exploration_mode")
    exploration_mode_cmd = DeclareLaunchArgument(
        "exploration_mode",
        default_value="spiral",
        choices=["spiral", "radial"],
        description="Mode to explore the area (spiral, radial)",
    )

    nav2_planner = LaunchConfiguration("nav2_planner")
    nav2_planner_cmd = DeclareLaunchArgument(
        "nav2_planner",
        default_value="SmacHybrid",
        choices=["SmacHybrid", "SmacLattice"],
        description="Nav2 planner (SmacHybrid or SmacLattice)",
    )

    nav2_controller = LaunchConfiguration("nav2_controller")
    nav2_controller_cmd = DeclareLaunchArgument(
        "nav2_controller",
        default_value="RPP",
        choices=["RPP", "TEB"],
        description="Nav2 controller (RPP or TEB)",
    )

    use_low_moon = LaunchConfiguration("use_low_moon")
    use_low_moon_cmd = DeclareLaunchArgument(
        "use_low_moon",
        default_value="False",
        description="Whether launch the low_moon instead of the normal moon world",
    )

    launch_yasmin_viewer = LaunchConfiguration("launch_yasmin_viewer")
    launch_yasmin_viewer_cmd = DeclareLaunchArgument(
        "launch_yasmin_viewer",
        default_value="True",
        description="Whether launch YASMIN Viewer",
    )

    rover_gazebo_moon_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rover_gazebo"), "launch", "moon.launch.py"
            )
        ),
        launch_arguments={
            "nav2_planner": nav2_planner,
            "nav2_controller": nav2_controller,
        }.items(),
        condition=UnlessCondition(use_low_moon),
    )

    rover_gazebo_low_moon_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rover_gazebo"),
                "launch",
                "low_moon.launch.py",
            )
        ),
        launch_arguments={
            "nav2_planner": nav2_planner,
            "nav2_controller": nav2_controller,
        }.items(),
        condition=IfCondition(use_low_moon),
    )

    moon_exploration_node_cmd = Node(
        package="moon_exploration",
        executable="moon_exploration_node",
        output="both",
        parameters=[
            {
                "radius": radius,
                "waypoints_nums": waypoints_nums,
                "exploration_mode": exploration_mode,
                "nav2_planner": nav2_planner,
                "nav2_controller": nav2_controller,
                "use_low_moon": use_low_moon,
            }
        ],
    )

    yasmin_viewer_cmd = Node(
        package="yasmin_viewer",
        executable="yasmin_viewer_node",
        output="both",
        condition=IfCondition(launch_yasmin_viewer),
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(radius_cmd)
    ld.add_action(waypoints_nums_cmd)
    ld.add_action(exploration_mode_cmd)
    ld.add_action(nav2_planner_cmd)
    ld.add_action(nav2_controller_cmd)
    ld.add_action(use_low_moon_cmd)
    ld.add_action(launch_yasmin_viewer_cmd)

    ld.add_action(rover_gazebo_moon_cmd)
    ld.add_action(rover_gazebo_low_moon_cmd)
    ld.add_action(moon_exploration_node_cmd)
    ld.add_action(yasmin_viewer_cmd)

    return ld
