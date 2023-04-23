import logging
import math
from collections import namedtuple
from pprint import pformat
from typing import Iterator, List, Optional, Tuple

import numpy as np
from ai2thor.server import Event

orientation_bindings = {0: (0, 1), 90: (1, 0), 180: (0, -1), 270: (-1, 0)}

Pos2D = namedtuple("Pos2D", ["x", "z"])


def get_rotation(degrees: float) -> dict:

    if degrees > 0:
        if degrees < 180:
            return dict(action="RotateRight", degrees=degrees)
        else:
            return dict(action="RotateLeft", degrees=360 - degrees)
    else:
        if degrees > -180:
            return dict(action="RotateLeft", degrees=-degrees)
        else:
            return dict(action="RotateRight", degrees=360 + degrees)


class Action:
    rotate_angle: int = 15

    def __init__(self, actions: List[dict]):

        self.actions = actions
        self.api_actions = []
        for api_action in actions:
            if api_action["action"].startswith("Rotate") or api_action[
                "action"
            ].startswith("Look"):
                degrees = api_action.get("degrees", 90)
                while degrees > self.rotate_angle:
                    self.api_actions.append(
                        dict(action=api_action["action"],
                             degrees=self.rotate_angle)
                    )
                    degrees -= self.rotate_angle
                self.api_actions.append(
                    dict(action=api_action["action"], degrees=degrees)
                )
            else:
                self.api_actions.append(api_action)

    def __str__(self) -> str:
        return "Action:\n" + pformat(self.api_actions, indent=2)

    @property
    def cost(self) -> float:
        """Returns the cost of the action, TO BE IMPLEMENTED"""
        return 1


class NavigationState:
    step_size = 0.05
    invalid_positions = set()

    def __init__(self, x: float, z: float, theta: int = 0):
        self.x = x
        self.z = z
        self.theta = theta

    def __eq__(self, other) -> bool:
        """Only considers the position, no rotation"""
        return (
            isinstance(other, NavigationState)
            and other.x == self.x
            and other.z == self.z
        )

    def __hash__(self) -> int:
        return hash(self.x + self.z)

    def __str__(self) -> str:
        return "<NavigationState ({}, {})@{}>".format(self.x, self.z, self.theta)

    def __sub__(self, other: "NavigationState") -> float:
        """manhattan distance"""

        if isinstance(other, NavigationState):
            return abs(other.x - self.x) + abs(other.z - self.z)
        else:
            raise ValueError(
                "can't subtract {} from NavigationState".format(type(other))
            )

    @staticmethod
    def from_event(event: Event) -> "NavigationState":
        """Initializes a navigation state directly from the simulator output"""
        return NavigationState(
            event.metadata["agent"]["position"]["x"],
            event.metadata["agent"]["position"]["z"],
            int(event.metadata["agent"]["rotation"]["y"]),
        )

    @staticmethod
    def snap_action(event: Event) -> Action:
        """
        Return the action needed for snapping to the grid
        """
        new_state = NavigationState.from_event(event)
        actions = []
        if new_state.theta not in orientation_bindings.keys():
            diff = min(
                (new_state.theta - key for key in orientation_bindings.keys()), key=abs
            )
            actions.append(get_rotation(diff))
        if not math.isclose(event.metadata["agent"]["cameraHorizon"], 0):
            actions.append(
                dict(action="LookUp",
                     degrees=event.metadata["agent"]["cameraHorizon"])
            )
        return Action(actions)

    @staticmethod
    def add_invalid(state: "NavigationState"):

        NavigationState.invalid_positions.add((state.x, state.z))

    def get_successors(self) -> Iterator[Tuple["NavigationState", Action]]:
        """
        Gets the successors of current state and their corresponding action sequences
        """

        for target_theta, (dx, dz) in orientation_bindings.items():
            new_x = self.x + dx * self.step_size
            new_z = self.z + dz * self.step_size
            if (new_x, new_z) not in NavigationState.invalid_positions:
                actions = []
                if target_theta != self.theta:
                    actions.append(get_rotation(target_theta - self.theta))
                actions.append(
                    dict(action="MoveAhead", moveMagnitude=self.step_size))
                yield (
                    NavigationState(new_x, new_z, target_theta),
                    Action(actions),
                )


# class VisualSearchState:
#     def __init__(self, theta: float, horizon: float):
#         self.theta = theta
#         self.horizon = horizon
#
#     def __hash__(self) -> int:
#         return hash(self.theta + self.horizon)
#
#     def __eq__(self, other) -> bool:
#         return (
#             isinstance(other, VisualSearchState)
#             and other.theta == self.theta
#             and other.horizon == self.horizon
#         )
#
#     @staticmethod
#     def from_event(self, event: Event) -> "VisualSearchState":
#         return VisualSearchState(
#             event.metadata["agent"]["rotation"]["y"],
#             event.metadata["agent"]["cameraHorizon"],
#         )
#


def get_obj_loc(event: Event, object_id: str) -> Optional[Pos2D]:

    pos = None
    for object_info in event.metadata["objects"]:
        if object_info["objectId"] == object_id:
            center = object_info["axisAlignedBoundingBox"]["center"]
            pos = Pos2D(center["x"], center["z"])
    if pos is None:
        logging.warning("{} not found in scene".format(object_id))
    return pos


def get_obj_in_frame(event: Event, object_id: str) -> Optional[Pos2D]:

    try:
        mask = event.instance_masks[object_id]
    except KeyError:
        if object_id.count("|") > 3:
            trimed_id = "|".join(object_id.split("|")[:-1])
            res = get_obj_in_frame(event, trimed_id)
            if res is not None:
                return res
        logging.warning("{} not found in frame".format(object_id))
        return None
    else:
        size = mask.shape[0]
        xs, zs = mask.nonzero()
        return Pos2D(xs.mean() / size, zs.mean() / size)
