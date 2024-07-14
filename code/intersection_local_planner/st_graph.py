from ctypes import Union
from typing import Tuple
from common.scenario.frenet import FrenetState


from configuration_parameters import parameters, paras_control, paras_planner


class StGraph:
    """Build ST Graph, search paths, and other actions related to path planning."""

    def __init__(
        self,
        t: float = 0.0,
        s: float = 0.0,
        s_d: float = 0.0,
        s_dd: float = 0.0,
        s_ddd: float = 0.0,
        d: float = 0.0,
        d_d: float = 0.0,
        d_dd: float = 0.0,
        d_ddd: float = 0.0,
        flag_zero_spd: bool = False,
        flag_max_spd: bool = False,
        flag_min_acc: bool = False,
        flag_max_acc: bool = False,
    ):
        self.frenet_state = FrenetState(t=t, s=s, s_d=s_d, s_dd=s_dd, s_ddd=s_ddd, d=d, d_d=d_d, d_dd=d_dd, d_ddd=d_ddd)
        # self.ft = ft
        self.x = None
        self.y = None

        self.occupancy_map = None

    def build_st_graph():
        pass

    def Get_Obstacle_Block_Segment():
        pass

    def generateInitialSpeedProfile():
        pass
