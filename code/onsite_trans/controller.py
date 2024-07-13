import math
import sys

from unicodedata import category

from onsite_trans.observation import Observation
from onsite_trans.opendrive2discretenet.opendriveparser.parser import parse_opendrive
from onsite_trans.opendrive2discretenet.network import Network

import os
import xml.dom.minidom
from lxml import etree
import json
import re
import numpy as np
import copy
from shapely.geometry import Polygon
from itertools import combinations

sys.path.append("..")


class ReplayInfo:
    """Store all data used to control the background vehicle during playback tests
    obstacle  vehicle_traj
    vehicle_traj = {
        "vehicle_id_0":{
            "shape":{
                "wid":4,
                "len":5
            },
            "t_0":{
                "x":10,
                "y":10,
                "v":10,
                "a":0,
                "yaw":0
            },
            "t_1":{...},
            ...
        },
        "vehicle_id_1":{...},
        ...
    }

    The ego vehicle track information contains only the current frame information
    ego_info = {
        "x":,
        "y":,
        "v":,
        "a":,
        "yaw":,
        "length":,
        "width":,
    }

    light_info = {}

    road_info = {}

    test_setting = {
        "t":,
        "dt":,
        "max_t",
        "goal":{
                "x":[],
                "y":[]
            }
        "end":,
        "scenario_type":,
        "scenario_name":,
        "map_type":
    }
    """

    def __init__(self):
        self.vehicle_traj = {}
        self.ego_info = {"length": 4.924, "width": 1.872, "x": 0, "y": 0, "v": 0, "a": 0, "yaw": 0}
        self.light_info = {}
        self.road_info = {}
        self.test_setting = {
            "t": 0,
            "dt": 0.01,
            "max_t": 10,
            "goal": {"x": [-10, 10], "y": [-10, 10]},
            "end": -1,
            "scenario_name": None,
            "scenario_type": None,
            "map_type": None,
        }

    def add_vehicle(self, id, t, x=None, y=None, v=None, a=None, yaw=None, length=None, width=None):
        """This function fulfills the functionality of adding background vehicle track information to vehicle_trajectiry."""
        if id == "ego":
            self._add_vehicle_ego(x, y, v, a, yaw, length, width)
        else:
            if id not in self.vehicle_traj.keys():
                self.vehicle_traj[id] = {}
                self.vehicle_traj[id]["shape"] = {}
            if t not in self.vehicle_traj[id].keys():
                self.vehicle_traj[id][t] = {}
            for key, value in zip(["x", "y", "v", "a", "yaw"], [x, y, v, a, yaw]):
                if value is not None:
                    self.vehicle_traj[id][t][key] = value
            for key, value in zip(["length", "width"], [length, width]):
                if value is not None:
                    self.vehicle_traj[id]["shape"][key] = value

    def add_settings(self, scenario_name=None, scenario_type=None, dt=None, max_t=None, goal_x=None, goal_y=None):

        for key, value in zip(["scenario_name", "scenario_type", "dt", "max_t"], [scenario_name, scenario_type, dt, max_t]):
            if value is not None:
                self.test_setting[key] = value
        for key, value in zip(["x", "y"], [goal_x, goal_y]):
            if value is not None:
                self.test_setting["goal"][key] = value

    def _add_vehicle_ego(self, x=None, y=None, v=None, a=None, yaw=None, length=None, width=None):

        for key, value in zip(["x", "y", "v", "a", "yaw", "length", "width"], [x, y, v, a, yaw, length, width]):
            if value is not None:
                self.ego_info[key] = value

    def _get_dt_maxt(self):

        max_t = 0
        for i in self.vehicle_traj.keys():
            t_i = list(self.vehicle_traj[i].keys())
            max_t_i = float(t_i[-1])
            if max_t_i > max_t:
                max_t = max_t_i
        dt = np.around(float(t_i[-1]) - float(t_i[-2]), 3)
        self.add_settings(dt=dt, max_t=max_t)


class ReplayParser:
    """Parse scene file"""

    def __init__(self):
        self.replay_info = ReplayInfo()

    def parse(self, senario_data: str) -> ReplayInfo:
        # .xosc .xodr .json
        path_json = ""
        input_dir = senario_data["params"]
        for item in os.listdir(input_dir):
            if item.split(".")[-1] == "xosc":
                path_openscenario = input_dir + "/" + item
            if item.split(".")[-1] == "xodr":
                path_opendrive = input_dir + "/" + item
            if item.split(".")[-1] == "json":
                path_json = input_dir + "/" + item

        self.replay_info.add_settings(scenario_name=senario_data["scene_name"], scenario_type="replay")

        self._parse_openscenario(path_openscenario)
        self._parse_opendrive(path_opendrive)
        if path_json:
            self._parse_light_json(path_json)
        return self.replay_info

    def _parse_light_json(self, file_dir: str) -> None:
        with open(file_dir, "r") as read_f:
            self.replay_info.light_info = json.load(read_f)
        return

    def _parse_openscenario(self, file_dir: str):
        opens = xml.dom.minidom.parse(file_dir).documentElement

        wl_node = opens.getElementsByTagName("Dimensions")
        for num, wl in zip(range(len(wl_node)), wl_node):
            if num == 0:
                self.replay_info.add_vehicle(id="ego", t=-1, width=float(wl.getAttribute("width")), length=float(wl.getAttribute("length")))
            else:
                self.replay_info.add_vehicle(id=num, t=-1, width=float(wl.getAttribute("width")), length=float(wl.getAttribute("length")))

        ego_node = opens.getElementsByTagName("Private")[0]
        ego_init = ego_node.childNodes[3].data
        ego_v, ego_x, ego_y, ego_head = [float(i.split("=")[1]) for i in ego_init.split(",")]
        ego_v = abs(ego_v)
        # ego_head = (ego_head + 2 * math.pi) if -math.pi <= ego_head < 0 else ego_head
        # ego_head = math_utils.unify_angle_range(ego_head)
        ego_head = (ego_head + np.pi) % (2 * np.pi) - np.pi
        self.replay_info.add_vehicle(id="ego", t=-1, x=ego_x, y=ego_y, v=ego_v, a=0, yaw=ego_head)

        act_list = opens.getElementsByTagName("Act")
        for id, act in zip(np.arange(1, len(act_list) + 1), act_list):
            t_list, x_list, y_list, yaw_list = [[] for i in range(4)]
            for point in act.getElementsByTagName("Vertex"):
                t_list.append(round(float(point.getAttribute("time")), 3))
                loc = point.getElementsByTagName("WorldPosition")[0]
                x_list.append(float(loc.getAttribute("x")))
                y_list.append(float(loc.getAttribute("y")))
                yaw = float(loc.getAttribute("h"))
                # yaw = (yaw + 2 * math.pi) if -math.pi <= yaw < 0 else yaw  #  (0, 2pi)
                yaw = (yaw + np.pi) % (2 * np.pi) - np.pi  #  [-PI, pi]
                yaw_list.append(yaw)

            x_diff = np.diff(x_list)
            y_diff = np.diff(y_list)
            t_diff = np.diff(t_list)
            v_list = np.sqrt(x_diff**2 + y_diff**2) / t_diff
            v_list = list(np.around(v_list, 2))
            v_list.append(v_list[-1])
            # acc
            v_diff = np.diff(v_list)
            a_list = v_diff / t_diff
            a_list = list(np.around(a_list, 2))
            a_list.append(0.00)

            t_list = [str(t) for t in t_list]
            for t, x, y, v, a, yaw in zip(t_list, x_list, y_list, v_list, a_list, yaw_list):
                self.replay_info.add_vehicle(id=id, t=t, x=round(x, 2), y=round(y, 2), v=round(v, 2), a=round(a, 2), yaw=round(yaw, 3))

        goal_init = ego_node.childNodes[5].data
        goal = [float(i) for i in re.findall("-*\d+\.\d+", goal_init)]
        self.replay_info.add_settings(goal_x=goal[:2], goal_y=goal[2:])

        self.replay_info._get_dt_maxt()

        return self.replay_info

    def _parse_opendrive(self, path_opendrive: str) -> None:
        """Parse the opendrive network information and store it in self.replay_info.road_info."""
        fh = open(path_opendrive, "r")

        root = etree.parse(fh).getroot()
        openDriveXml = parse_opendrive(root)
        fh.close()

        self.loadedRoadNetwork = Network()
        self.loadedRoadNetwork.load_opendrive(openDriveXml)

        open_drive_info = self.loadedRoadNetwork.export_discrete_network(
            filter_types=["driving", "onRamp", "offRamp", "exit", "entry"]
        )  # -> <class> DiscreteNetwork
        self.replay_info.road_info = open_drive_info

        name = root.find("header").attrib["name"]
        if name in ["highway", "highD"]:
            self.replay_info.test_setting["map_type"] = "highway"
        elif name in ["", "SinD"]:
            self.replay_info.test_setting["map_type"] = "intersection"
        elif name in ["NDS_ramp"]:
            self.replay_info.test_setting["map_type"] = "ramp"


class ReplayController:
    def __init__(self):
        self.control_info = ReplayInfo()

    def init(self, control_info: ReplayInfo) -> Observation:
        self.control_info = control_info
        return self._get_initial_observation()

    def step(self, action, old_observation: Observation, goal_pose) -> Observation:
        action = self._action_cheaker(action)
        new_observation = self._update_ego_and_t(action, old_observation)
        new_observation = self._update_other_vehicles_to_t(new_observation)
        new_observation = self._update_end_status(new_observation, goal_pose)
        if self.control_info.light_info:
            new_observation = self._update_light_info_to_t(new_observation)
        return new_observation

    def _action_cheaker(self, action):
        a = np.clip(action[0], -15, 15)
        rad = np.clip(action[1], -1, 1)
        return (a, rad)

    def _get_initial_observation(self) -> Observation:
        observation = Observation()
        # vehicle_info
        observation.vehicle_info["ego"] = self.control_info.ego_info
        observation = self._update_other_vehicles_to_t(observation)
        # road_info
        observation.road_info = self.control_info.road_info
        # test_setting
        observation.test_setting = self.control_info.test_setting

        points = np.empty(shape=[0, 2])
        for discrete_lane in observation.road_info.discretelanes:
            points = np.concatenate([discrete_lane.left_vertices, discrete_lane.right_vertices, discrete_lane.center_vertices, points], axis=0)
        points_x = points[:, 0]
        points_y = points[:, 1]
        observation.test_setting["x_max"] = np.max(points_x)
        observation.test_setting["x_min"] = np.min(points_x)
        observation.test_setting["y_max"] = np.max(points_y)
        observation.test_setting["y_min"] = np.min(points_y)
        observation = self._update_end_status(observation)
        # light_info
        if self.control_info.light_info:
            observation.test_setting["t"] = float(("%.2f" % observation.test_setting["t"]))
            observation.light_info = self.control_info.light_info[str(np.around(observation.test_setting["t"], 3))]
        return observation

    def _update_light_info_to_t(self, old_observation: Observation) -> Observation:
        new_observation = copy.deepcopy(old_observation)
        new_observation.light_info = self.control_info.light_info[str(np.around(old_observation.test_setting["t"], 3))]
        return new_observation

    def _update_ego_and_t(self, action: tuple, old_observation: Observation) -> Observation:

        new_observation = copy.deepcopy(old_observation)
        new_observation.test_setting["t"] = float(old_observation.test_setting["t"] + old_observation.test_setting["dt"])

        a, rot = action
        dt = old_observation.test_setting["dt"]
        x, y, v, yaw, width, length = [float(old_observation.vehicle_info["ego"][key]) for key in ["x", "y", "v", "yaw", "width", "length"]]

        new_observation.vehicle_info["ego"]["x"] = x + v * np.cos(yaw) * dt
        new_observation.vehicle_info["ego"]["y"] = y + v * np.sin(yaw) * dt
        yaw_new = yaw + v / length * 1.7 * np.tan(rot) * dt
        new_observation.vehicle_info["ego"]["yaw"] = (yaw_new + np.pi) % (2 * np.pi) - np.pi
        new_observation.vehicle_info["ego"]["v"] = v + a * dt
        if new_observation.vehicle_info["ego"]["v"] < 0:
            new_observation.vehicle_info["ego"]["v"] = 0

        new_observation.vehicle_info["ego"]["a"] = a
        return new_observation

    def _update_other_vehicles_to_t(self, old_observation: Observation) -> Observation:
        new_observation = copy.deepcopy(old_observation)
        new_observation.vehicle_info = {}
        new_observation.vehicle_info["ego"] = old_observation.vehicle_info["ego"]
        t = old_observation.test_setting["t"]
        t = str(np.around(t, 3))
        for vehi in self.control_info.vehicle_traj.items():
            id = vehi[0]
            info = vehi[1]
            if t in info.keys():
                new_observation.vehicle_info[id] = {}
                for key in ["x", "y", "v", "a", "yaw"]:
                    new_observation.vehicle_info[id][key] = info[t][key]
                for key in ["width", "length"]:
                    new_observation.vehicle_info[id][key] = info["shape"][key]
        return new_observation

    def _update_end_status(self, observation: Observation, goal_pose: tuple[float, float, float] = None) -> Observation:
        """calculates time T, tests whether it terminates, and updates the end value in observation.test_setting.

        end=
        1: Playback test run is complete;
        2: Collision occurs;
        """
        status_list = [-1]

        #   Check whether there is a collision between the main car and the background car,status=2
        if self._collision_detect(observation):
            status_list += [2]

        # Check whether the scenario end time has been reached. max_t,status=1
        if observation.test_setting["t"] >= self.control_info.test_setting["max_t"]:
            status_list += [1]

        # Check if you are out of map range with status=1.5
        if self.control_info.test_setting["map_type"] in ["highway", "ramp"]:
            if (
                observation.vehicle_info["ego"]["x"] > observation.test_setting["x_max"]
                or observation.vehicle_info["ego"]["x"] < observation.test_setting["x_min"]
            ):
                status_list += [1.5]
        elif self.control_info.test_setting["map_type"] in ["intersection"]:
            if (
                observation.vehicle_info["ego"]["x"] > observation.test_setting["x_max"]
                or observation.vehicle_info["ego"]["x"] < observation.test_setting["x_min"]
                or observation.vehicle_info["ego"]["y"] > observation.test_setting["y_max"]
                or observation.vehicle_info["ego"]["y"] < observation.test_setting["y_min"]
            ):
                status_list += [1.5]

        # Check whether the target area has been reached with status=1.5, exceeding goal_pose in the longitudinal direction
        if goal_pose is not None:
            Yaw2 = goal_pose[2]
            dx = observation.vehicle_info["ego"]["x"] - goal_pose[0]
            dy = observation.vehicle_info["ego"]["y"] - goal_pose[1]
            longitudinal_error = math.sin(Yaw2) * dy + math.cos(Yaw2) * dx
            if longitudinal_error >= 0.0:
                status_list += [1.5]

        observation.test_setting["end"] = max(status_list)
        return observation

    def _collision_detect(self, observation: Observation) -> bool:
        poly_zip = []
        self.vehicle_index = []

        if observation.test_setting["t"] > 0.5:
            for index, vehi in observation.vehicle_info.items():
                self.vehicle_index += [index]
                poly_zip += [self._get_poly(vehi)]

        for a, b in combinations(poly_zip, 2):
            if self.vehicle_index[poly_zip.index(a)] == "ego" or self.vehicle_index[poly_zip.index(b)] == "ego":
                if a.intersects(b):
                    return True
                else:
                    continue
        return False

    def _get_poly(self, vehicle: dict) -> Polygon:
        x, y, yaw, width, length = [float(vehicle[i]) for i in ["x", "y", "yaw", "width", "length"]]

        alpha = np.arctan(width / length)
        diagonal = np.sqrt(width**2 + length**2)
        x0 = x + diagonal / 2 * np.cos(yaw + alpha)
        y0 = y + diagonal / 2 * np.sin(yaw + alpha)
        x2 = x - diagonal / 2 * np.cos(yaw + alpha)
        y2 = y - diagonal / 2 * np.sin(yaw + alpha)
        x1 = x + diagonal / 2 * np.cos(yaw - alpha)
        y1 = y + diagonal / 2 * np.sin(yaw - alpha)
        x3 = x - diagonal / 2 * np.cos(yaw - alpha)
        y3 = y - diagonal / 2 * np.sin(yaw - alpha)

        poly = Polygon(((x0, y0), (x1, y1), (x2, y2), (x3, y3), (x0, y0))).convex_hull
        return poly


def _add_vehicle_to_observation(env, old_observation: Observation) -> Observation:
    new_observation = old_observation
    for i in range(len(env.vehicle_list)):
        name = env.vehicle_list[i]
        data = env.vehicle_array[0, i, :]
        for key, value in zip(["x", "y", "v", "yaw", "length", "width"], data):
            if name not in new_observation.vehicle_info.keys():
                new_observation.vehicle_info[name] = {"x": -1, "y": -1, "v": -1, "a": 0, "yaw": -1, "length": -1, "width": -1}
            new_observation.vehicle_info[name][key] = value
    return new_observation


class Controller:
    def __init__(self) -> None:
        self.observation = Observation()
        self.parser = None
        self.control_info = None
        self.controller = None
        self.mode = "replay"

    def init(self, scenario: dict) -> Observation:
        """Initializes the running scenario, given the observations at the initial time

        Parameters
            ----------
            input_dir : str
            Test the location of the input file
            Playback test: Contains.xodr,.xosc files
            Interactive testing:
            mode : str
            Specify test mode
            replay the test :replay
            Interaction test :interact
        Returns
            -------
            observation : Observation
            Observation information at the initial time, returned as an object of the Observation class.
        """
        self.mode = scenario["test_settings"]["mode"]
        if self.mode == "replay":
            self.parser = ReplayParser()
            self.controller = ReplayController()
            self.control_info = self.parser.parse(scenario["data"])
            self.observation = self.controller.init(self.control_info)
            self.traj = self.control_info.vehicle_traj
        return self.observation, self.traj

    def step(self, action, goal_pose):
        self.observation = self.controller.step(action, self.observation, goal_pose)
        return self.observation


if __name__ == "__main__":
    demo_input_dir = r"demo/demo_inputs"
    demo_output_dir = r"demo/demo_outputs"
    from onsite_trans.scenarioOrganizer import ScenarioOrganizer
