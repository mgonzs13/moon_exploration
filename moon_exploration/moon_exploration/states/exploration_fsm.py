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


from simple_node import Node

from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT

from moon_exploration.states import DriveState
from moon_exploration.states import GetNextPointState


class ExplorationFSM(StateMachine):

    def __init__(self, node: Node) -> None:
        super().__init__(outcomes=[SUCCEED, CANCEL])

        self.add_state(
            "GETTING_NEXT_POINT",
            GetNextPointState(),
            transitions={
                SUCCEED: "DRIVING",
                ABORT: SUCCEED,
            },
        )

        self.add_state(
            "DRIVING",
            DriveState(node),
            transitions={
                SUCCEED: "GETTING_NEXT_POINT",
                ABORT: "DRIVING",
                CANCEL: CANCEL,
            },
        )
