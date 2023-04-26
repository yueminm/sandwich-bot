import logging
from pprint import pformat
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

        self.bindings = {
            "Bread": None,
            "Lettuce": None,
            "Tomato": None,
            "Knife": None,
            "Plate": None,
            "Sink": None,
            "Pot": None,
            "SinkBasin": None,
        }
        for obj_info in self.env.event.metadata["objects"]:
            if obj_info["objectType"] in self.bindings.keys():
                self.bindings[obj_info["objectType"]] = obj_info["name"]
                if obj_info["objectType"] in {"Bread", "Lettuce", "Tomato"}:
                    for i in range(1, 4):
                        self.bindings[
                            obj_info["objectType"][0] + "S" + str(i)
                        ] = obj_info["assetId"] + "_Slice_{}".format(i + 1)
        self.bindings["plate"] = self.bindings["Plate"]
        logging.debug("got bindings: \n{}".format(pformat(self.bindings, indent=2)))

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

    def name_to_id(self, name: str) -> str:

        for obj_info in self.env.event.metadata["objects"]:
            if obj_info["name"] == name:
                return obj_info["objectId"]

    def parse(self, string: str) -> Tuple[str]:

        string = string[string.index("(") + 1 : string.index(")")]
        return tuple(string.split(","))

    def run_plan(self, plan_file: str):

        plan = open(plan_file).readlines()
        plan = plan[plan.index("Plan:\n") + 1 :]
        logging.info("got plan: \n{}".format(pformat(plan, indent=2)))

        for step in plan:
            logging.debug("executing {}".format(step))
            if step.startswith("PickKnife"):
                self.go_to_obj(self.name_to_id(self.bindings["Knife"]))
                self.look_at_obj(self.name_to_id(self.bindings["Knife"]))
                self.pick_obj(self.name_to_id(self.bindings["Knife"]))
            elif step.startswith("CutBread"):
                self.go_to_obj(self.name_to_id(self.bindings["Bread"]))
                self.look_at_obj(self.name_to_id(self.bindings["Bread"]))
                self.cut_obj(self.name_to_id(self.bindings["Bread"]))
            elif step.startswith("CutTomato"):
                self.go_to_obj(self.name_to_id(self.bindings["Tomato"]))
                self.look_at_obj(self.name_to_id(self.bindings["Tomato"]))
                self.cut_obj(self.name_to_id(self.bindings["Tomato"]))
            elif step.startswith("CutLettuce"):
                self.go_to_obj(self.name_to_id(self.bindings["Lettuce"]))
                self.look_at_obj(self.name_to_id(self.bindings["Lettuce"]))
                self.cut_obj(self.name_to_id(self.bindings["Lettuce"]))
            elif step.startswith("PutKnife"):
                self.go_to_obj(self.name_to_id(self.bindings["SinkBasin"]))
                self.look_at_obj(self.name_to_id(self.bindings["SinkBasin"]))
                self.put_obj(self.name_to_id(self.bindings["SinkBasin"]))
            elif step.startswith("PickSlice"):
                symbol = self.parse(step)[0]
                # if symbol not in self.bindings.keys():
                #     object_type = {
                #         "BS": "BreadSliced",
                #         "LS": "LettuceSliced",
                #         "TS": "TomatoSliced",
                #     }[symbol[:2]]
                #     for obj_info in self.env.event.metadata["objects"]:
                #         if (
                #             obj_info["objectType"] == object_type
                #             and obj_info["name"] not in self.bindings.values()
                #         ):
                #             self.bindings[symbol] = obj_info["name"]
                #             break
                self.go_to_obj(self.name_to_id(self.bindings[symbol]))
                self.look_at_obj(self.name_to_id(self.bindings[symbol]))
                self.pick_obj(self.name_to_id(self.bindings[symbol]))
            elif step.startswith("PutSlice"):
                symbol = self.parse(step)[-1]
                self.go_to_obj(self.name_to_id(self.bindings[symbol]))
                self.look_at_obj(self.name_to_id(self.bindings[symbol]))
                self.put_obj(self.name_to_id(self.bindings[symbol]))

        logging.info("total path length: {}".format(self.env.path_length))


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
    event = env.api_step(action="Done")  # noqa


def run_plan(plan_file: str, floorplan: str = "FloorPlan3"):

    set_logging("DEBUG")
    env = Env(floorplan=floorplan)
    Agent(env).run_plan(plan_file)


if __name__ == "__main__":
    Fire()
