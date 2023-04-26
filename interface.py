import json
import logging
import os
from pprint import pformat
from time import sleep
from typing import List

from ai2thor import controller
from ai2thor.server import Event

from utils import Action

# from utils_initial import Action


class Env:
    controller: controller.Controller
    event: Event
    interval: float = 0.15
    reachables: list

    def __init__(
        self,
        width: int = 600,
        height: int = 600,
        log_file: str = "log",
        floorplan: str = "FloorPlan3",
    ):

        self.floorplan = floorplan
        self.width = width
        self.height = height

        self.controller = controller.Controller(
            scene=floorplan,
            width=width,
            height=height,
            gridSize=0.05,
            snapToGrid=False,
            fieldOfView=60,
            renderInstanceSegmentation=True,
        )

        if os.path.isfile("poses/{}.json".format(floorplan)):
            self.event = self.controller.step(
                action="SetObjectPoses",
                objectPoses=json.load(open("poses/{}.json".format(floorplan))),
            )

        self.reachables = self.controller.step(action="GetReachablePositions").metadata[
            "actionReturn"
        ]

        self.event = self.controller.step(action="Done")
        self.path_length = 0
        logging.info("environment started")
        logging.debug("all objects: \n {}".format(pformat(self.objects, indent=2)))

    def reset(self):
        self.controller.reset(
            scene=self.floorplan,
            width=self.width,
            height=self.height,
            gridSize=0.05,
            snapToGrid=False,
            fieldOfView=60,
            renderInstanceSegmentation=True,
        )
        if os.path.isfile("poses/{}.json".format(self.floorplan)):
            self.event = self.controller.step(
                action="SetObjectPoses",
                objectPoses=json.load(open("poses/{}.json".format(self.floorplan))),
            )
        self.reachables = self.controller.step(action="GetReachablePositions").metadata[
            "actionReturn"
        ]

        self.event = self.controller.step(action="Done")
        logging.info("environment reset")
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
        sleep(self.interval)
        self.event = self.controller.step(*args, **kwargs)
        if (
            len(args) > 0
            and isinstance(args[0], dict)
            and args[0].get("action", "").startswith("Move")
        ):
            self.path_length += args[0]["moveMagnitude"]
        if kwargs.get("action", "").startswith("Move"):
            self.path_length += kwargs["moveMagnitude"]
        return self.event

    def step(self, action: Action) -> bool:
        """Attempts to perform action, return if the action is successful"""
        for api_action in action.api_actions:
            logging.debug("executing {}".format(api_action))
            self.event = self.api_step(api_action)
            if not self.event.metadata["lastActionSuccess"]:
                logging.info(
                    "last action unsuccessful: \n {}".format(
                        pformat(api_action, indent=2)
                    )
                )
                logging.warning(
                    "{}".format(pformat(self.event.metadata["errorMessage"], indent=2))
                )
                return False
        return True
