import logging
from typing import Optional

import numpy as np
from ai2thor.server import Event

# import utils
# from interface import Env
# from utils import NavigationState, Pos2D

import utils_initial as utils
from interface import Env
from utils_initial import NavigationState, Pos2D


class NavigationPlanner:
    def __init__(self, env: Env, goal: Pos2D):
        self.env = env
        self.goal = NavigationState(*goal)
        self.heuristics = {}

        self.plan2(env.event, 1)
        # self.plan(env.event)

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
        snap_action = NavigationState.snap_action(event)
        self.env.step(snap_action)
        this_state = NavigationState.from_event(self.env.event)
        
        while True:
            current = this_state
            """Dictionary to store information of states"""
            expanded_state_dict = {current : dict(parent_state = None, parent_action = None, all_expanded = False, goal_reached = False)}
            """List to track the order of states being expanded"""
            expanded_state_order = []
            expanded_state_order.append(current)
            expanding_id = 0
            count = 0
            
            if this_state - self.goal <= 1.0:
                return

            """Expand k states and store corresponding information into dictionary"""
            while count < k:
                current = expanded_state_order[expanding_id]
                print("in loop")
                print(expanded_state_dict[current])
                if expanded_state_dict[current]["goal_reached"]:
                    break
                print("here")
                for succ, action in current.get_successors():
                    if count >= k:
                        break
                    if succ is None:
                        break
                    if succ in expanded_state_dict:
                        continue
                    if not self.env.step(action): 
                        NavigationState.add_invalid(succ)
                        continue
                    
                    expanded_state_info = dict(parent_state = current, parent_action = action, all_expanded = False, goal_reached = False)
                    expanded_state_dict[succ] = expanded_state_info
                    expanded_state_order.append(succ)
                    count += 1
                    if succ - self.goal <= 1.0:
                        expanded_state_dict[succ]["goal_reached"] = True
                        
                expanding_id += 1
                expanded_state_dict[current]["all_expanded"] = True
            
            min_f = np.inf
            min_f_state = None
            for state in expanded_state_order[::-1]:
                if expanded_state_dict[state]["all_expanded"]:
                    break
                if expanded_state_dict[state]["goal_reached"]:
                    min_f_state = state
                    self.heuristics[state] = 0.0
                    break
                
                for succ, action in state.get_successors():
                    if succ in expanded_state_dict:
                        continue
                    heuristic = self.get_heuristics(succ)
                    f_value = heuristic + action.cost
                    if f_value <= min_f:
                        min_f = f_value
                        min_f_state = state
            
            self.heuristics[min_f_state] = min_f
            next_state = min_f_state
            while (not expanded_state_dict[next_state]["parent_state"] is None):
                backtrack_state = expanded_state_dict[next_state]["parent_state"]
                self.heuristics[backtrack_state] =  self.heuristics[next_state] \
                    + expanded_state_dict[next_state]["parent_action"].cost
                next_state = backtrack_state
            
            this_state = next_state
                    
        
                
            
            

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
