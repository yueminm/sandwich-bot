import logging
from typing import List, Tuple

from fire import Fire

from interface import Env
from planner import NavigationPlanner, handle_look_at, handle_put_obj


def set_logging(level: str = "INFO"):
    logging.basicConfig(
        force=True,
        level=level,
        format="[%(levelname)s] (%(module)s@%(process)d) %(message)s",
    )


set_logging("INFO")


class Agent:
    def __init__(self, env: Env):
        self.env = env

    def execute(self, tasks: List[Tuple[str, str]]):
        for func, arg in tasks:
            getattr(self, func)(arg)

    def go_to_obj(self, object_id: str):
        """Assumes the object is reachable"""
        NavigationPlanner.go_to_obj(self.env, object_id)

    def look_at_obj(self, object_id: str):
        """Assumes the agent is reasonable close to object"""
        handle_look_at(self.env, object_id)

    def pick_obj(self, object_id: str):
        """Assumes the agent has the object in the frame"""

        if object_id in self.env.objects_visible:
            self.env.api_step(action="PickupObject", objectId=object_id)
        else:
            logging.warning("{} not found in scene".format(object_id))

    def put_obj(self, recep_id: str):
        """
        Assumes the agent is holding the object to be put down and has the recepticle in
        the frame
        Manually handle sliced object
        """
        handle_put_obj(self.env, recep_id)

    def cut_obj(self, object_id: str):
        """
        Assumes the agent is holding a knife and has the object to be cut in the frame
        """

        if object_id in self.env.objects_visible:
            self.env.api_step(action="SliceObject", objectId=object_id)
        else:
            logging.warning("{} not found in scene".format(object_id))


def test():
    set_logging("DEBUG")
    env = Env(floorplan="FloorPlan3")
    A = Agent(env)
    tasks = [
        ("go_to_obj", "Knife|+00.72|+01.32|-02.20"),
        ("look_at_obj", "Knife|+00.72|+01.32|-02.20"),
        ("pick_obj", "Knife|+00.72|+01.32|-02.20"),
        ("go_to_obj", "Bread|-01.51|+01.38|+00.66"),
        ("look_at_obj", "Bread|-01.51|+01.38|+00.66"),
        ("cut_obj", "Bread|-01.51|+01.38|+00.66"),
        ("go_to_obj", "Sink|-01.99|+01.14|-00.98|SinkBasin"),
        ("look_at_obj", "Sink|-01.99|+01.14|-00.98|SinkBasin"),
        ("put_obj", "Sink|-01.99|+01.14|-00.98|SinkBasin"),
        ("go_to_obj", "Bread|-01.51|+01.38|+00.66|BreadSliced_2"),
        ("look_at_obj", "Bread|-01.51|+01.38|+00.66|BreadSliced_2"),
        ("pick_obj", "Bread|-01.51|+01.38|+00.66|BreadSliced_2"),
        ("go_to_obj", "Plate|-01.47|+01.31|+00.24"),
        ("look_at_obj", "Plate|-01.47|+01.31|+00.24"),
        ("put_obj", "Plate|-01.47|+01.31|+00.24"),
    ]

    A.execute(tasks)
    event = env.api_step(action="Done")
    import pdb

    # pdb.set_trace()


if __name__ == "__main__":
    Fire()
