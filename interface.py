import logging
from pprint import pformat
from time import sleep
from typing import List

import numpy as np
from ai2thor import controller
from ai2thor.server import Event

import utils
from utils import Action, NavigationState


class Env:
    controller: controller.Controller
    event: Event
    interval: float = 0.15

    def __init__(
        self,
        width: int = 600,
        height: int = 600,
        log_file: str = "log",
        floorplan: str = "FloorPlan3",
    ):

        self.controller = controller.Controller(
            scene=floorplan,
            width=width,
            height=height,
            gridSize=0.05,
            snapToGrid=False,
            fieldOfView=60,
            renderInstanceSegmentation=True,
        )
        self.event = self.controller.step(action="Done")
        logging.info("environment started")
        logging.debug("all objects: \n {}".format(pformat(self.objects, indent=2)))

    @property
    def objects(self) -> List[str]:
        """Return all object ids in the environment"""
        return [obj_info["objectId"] for obj_info in self.event.metadata["objects"]]

    @property
    def objects_visible(self) -> List[str]:
        """Return all visible object ids in the environment"""
        return [
            obj_info["objectId"]
            for obj_info in self.event.metadata["objects"]
            if obj_info["visible"]
        ]

    def api_step(self, *args, **kwargs) -> Event:
        self.event = self.controller.step(*args, **kwargs)
        sleep(self.interval)
        return self.event

    def step(self, action: Action) -> bool:
        """Attempts to perform action, return if the action is successful"""

        for api_action in action.api_actions:
            sleep(self.interval)
            logging.debug("executing {}".format(api_action))
            self.event = self.controller.step(api_action)
            if not self.event.metadata["lastActionSuccess"]:
                logging.info(
                    "last action unsuccessful: \n {}".format(
                        pformat(api_action, indent=2)
                    )
                )
                return False
        return True

    def look_at(self, object_id: str):

        # rotate
        current = NavigationState.from_event(self.event)
        object_pos = utils.get_obj_loc(self.event, object_id)
        dx = object_pos.x - current.x
        dz = object_pos.z - current.z
        target_theta = np.arctan2(dx, dz) / np.pi * 180
        action_dict = utils.get_rotation(target_theta - current.theta)
        logging.debug("generated action dict: {}".format(action_dict))
        self.step(utils.Action([action_dict]))

        # update horizon
        while self.event.metadata["agent"]["cameraHorizon"] < 60 and (
            object_id not in self.objects_visible
            or utils.get_obj_in_frame(self.event, object_id)[0] > 0.7
        ):
            self.api_step(action="LookDown", degrees=10)
        if object_id not in self.objects_visible:
            while self.event.metadata["agent"]["cameraHorizon"] > -30 and (
                object_id not in self.objects_visible
                or utils.get_obj_in_frame(self.event, object_id)[0] > 0.3
            ):
                self.api_step(action="LookUp", degrees=10)
        if object_id not in self.objects_visible:
            logging.warning("object {} not found".format(object_id))

    def _set_object_pose(self, positions: dict, rotations: dict):

        objects = [
            dict(
                objectName=x["name"],
                position=positions.get(x["objectId"], x["position"]),
                rotation=rotations.get(x["objectId"], x["rotation"]),
            )
            for x in self.event.metadata["objects"]
            if x["moveable"] or x["pickupable"]
        ]
        self.api_step(action="SetObjectPoses", objectPoses=objects)
        self.api_step(action="Done")

    def put_obj(self, recep_id: str):
        """Manually handle putting down sliced objects"""

        try:
            object_in_hand = self.event.metadata["inventoryObjects"][0]["objectId"]
        except IndexError:
            logging.warning("agent has no object in hand to be put down")
        else:
            event = self.api_step(action="PutObject", objectId=recep_id)
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
                    self._set_object_pose(
                        {object_in_hand: position},
                        {object_in_hand: rotation},
                    )
            elif "Slice" in object_in_hand:
                current_object = event.get_object(object_in_hand)
                target_object = event.get_object(recep_id)
                target_bbox = target_object["axisAlignedBoundingBox"]
                rotation = current_object["rotation"]
                position = target_bbox["center"]
                rotation["x"] = 90
                position["y"] = (
                    max(pt[1] for pt in target_bbox["cornerPoints"])
                    + target_bbox["size"]["y"]
                )
                self._set_object_pose(
                    {object_in_hand: position},
                    {object_in_hand: dict(x=90, y=0, z=0)},
                )
                self.api_step(action="DropHandObject", forceAction=True)
            else:
                logging.warning(
                    "putting {} onto {} failed".format(object_in_hand, recep_id)
                )
