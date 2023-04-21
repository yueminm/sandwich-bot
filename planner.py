import logging
from typing import Optional

import numpy as np
from ai2thor.server import Event

import utils
from interface import Env
from utils import NavigationState, Pos2D


class NavigationPlanner:
    def __init__(self, env: Env, goal: Pos2D):
        self.env = env
        self.goal = NavigationState(*goal)
        self.heuristics = {}

        self.plan(env.event)

    @staticmethod
    def go_to_obj(env: Env, object_id: str):

        goal = utils.get_obj_loc(env.event, object_id)
        NavigationPlanner(env, goal)

    def get_heuristics(self, state: Optional[NavigationState]) -> float:

        try:
            return self.heuristics[state]
        except KeyError:
            if state:
                self.heuristics[state] = state - self.goal
                return self.heuristics[state]
            else:
                return None

    def plan2(self, event: Event, k: int):
        """LRTA* with K=k"""
        raise NotImplementedError

    def plan(self, event: Event):
        """LRTA* with K=1"""

        snap_action = NavigationState.snap_action(event)
        self.env.step(snap_action)
        current = NavigationState.from_event(self.env.event)

        while True:

            successor = None
            successor_action = None
            successor_f = None
            for succ, action in current.get_successors():
                heuristic = self.get_heuristics(succ)
                f_value = heuristic + action.cost
                if successor_f is None or f_value < successor_f:
                    successor = succ
                    successor_action = action
                    successor_f = f_value

            if successor is None or current - self.goal < 1.0:
                # goal check
                return

            if self.env.step(successor_action):
                self.heuristics[current] = successor_f
                current = successor
            else:
                NavigationState.add_invalid(successor)
                current = NavigationState.from_event(self.env.event)


# class VisualSearchPlanner:
#     def __init__(self, env: Env, object_id: str):
#         self.env = env
#         self.object_id = object_id


def handle_look_at(env: Env, object_id: str):

    # rotate
    print("something")
    current = NavigationState.from_event(env.event)
    object_pos = utils.get_obj_loc(env.event, object_id)
    dx = object_pos.x - current.x
    dz = object_pos.z - current.z
    target_theta = np.arctan2(dx, dz) / np.pi * 180
    action_dict = utils.get_rotation(target_theta - current.theta)
    logging.debug("generated action dict: {}".format(action_dict))
    env.step(utils.Action([action_dict]))

    # update horizon
    while env.event.metadata["agent"]["cameraHorizon"] < 60 and (
        object_id not in env.objects_visible
        or utils.get_obj_in_frame(env.event, object_id)[0] > 0.7
    ):
        env.api_step(action="LookDown", degrees=10)
    if object_id not in env.objects_visible:
        while env.event.metadata["agent"]["cameraHorizon"] > -30 and (
            object_id not in env.objects_visible
            or utils.get_obj_in_frame(env.event, object_id)[0] > 0.3
        ):
            env.api_step(action="LookUp", degrees=10)
    if object_id not in env.objects_visible:
        logging.warning("object {} not found".format(object_id))


def set_object_pose(env: Env, positions: dict, rotations: dict):

    objects = [
        dict(
            objectName=x["name"],
            position=positions.get(x["objectId"], x["position"]),
            rotation=rotations.get(x["objectId"], x["rotation"]),
        )
        for x in env.event.metadata["objects"]
        if x["moveable"] or x["pickupable"]
    ]
    env.api_step(action="SetObjectPoses", objectPoses=objects)
    env.api_step(action="Done")


def handle_put_obj(env: Env, recep_id: str):
    """Manually handle putting down sliced objects"""

    try:
        object_in_hand = env.event.metadata["inventoryObjects"][0]["objectId"]
    except IndexError:
        logging.warning("agent has no object in hand to be put down")
    else:
        event = env.api_step(action="PutObject", objectId=recep_id)
        if event.metadata["lastActionSuccess"]:
            if "Slice" in object_in_hand:
                current_object = event.get_object(object_in_hand)
                target_object = event.get_object(recep_id)
                target_bbox = target_object["axisAlignedBoundingBox"]
                rotation = current_object["rotation"]
                position = current_object["position"]
                rotation["x"] = 90
                position["y"] = (
                    max(pt[1] for pt in target_bbox["cornerPoints"])
                    + target_bbox["size"]["y"] / 2
                )
                set_object_pose(
                    env,
                    {object_in_hand: position},
                    {object_in_hand: rotation},
                )
        elif "Slice" in object_in_hand:
            target_object = event.get_object(recep_id)
            target_bbox = target_object["axisAlignedBoundingBox"]
            rotation = current_object["rotation"]
            position = target_bbox["center"]
            rotation["x"] = 90
            position["y"] = (
                max(pt[1] for pt in target_bbox["cornerPoints"])
                + target_bbox["size"]["y"]
            )
            set_object_pose(
                env,
                {object_in_hand: position},
                {object_in_hand: dict(x=90, y=0, z=0)},
            )
            env.api_step(action="DropHandObject", forceAction=True)
        else:
            logging.warning(
                "putting {} onto {} failed".format(object_in_hand, recep_id)
            )
