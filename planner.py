import logging
from typing import Optional

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
                logging.info("goal reached")
                return

            if self.env.step(successor_action):
                self.heuristics[current] = successor_f
                current = successor
            else:
                NavigationState.add_invalid(successor)
                current = NavigationState.from_event(self.env.event)
