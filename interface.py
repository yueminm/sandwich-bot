import logging
from pprint import pformat
from time import sleep
from typing import List

from ai2thor import controller
from ai2thor.server import Event

from utils import Action


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
        return [obj_info["objectId"] for obj_info in self.event.metadata["objects"]]

    @property
    def objects_visible(self) -> List[str]:
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
