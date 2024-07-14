# import lib
import sys
import time
import math
import os
import copy
from typing import Dict, List, Union, Tuple


import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def add_path_to_sys(target: str):
    abs_path = os.path.abspath(target)
    if abs_path not in sys.path:
        sys.path.append(abs_path)


dir_current_file = os.path.dirname(__file__)
dir_parent_1 = os.path.dirname(dir_current_file)
add_path_to_sys(dir_current_file)
add_path_to_sys(dir_parent_1)

import common.tool as tool
from common.scenario.frenet import FrenetState, GoalSamplingFrenetTrajectory, State
from common.geometry.cubic_spline import CubicSpline2D
from common.vehicle.vehicle import Vehicle
from onsite_trans.observation import Observation
from intersection_local_planner.frenet_optimal_planner import (
    FrenetOptimalPlanner,
    FrenetOptimalPlannerSettings,
    Stats,
)

from intersection_local_planner.jerk_space_sampling_planner import (
    JerkSpaceSamplingPlanner,
    JerkSpaceSamplingPlannerSettings,
)
from configuration_parameters import parameters, paras_control, paras_planner


class PlannerSettings(object):
    def __init__(self):
        """Initializes the planner with some parameters."""
        self.dt = 0.1  # time tick [s]   time resolution between two planned waypoints

        #! Some constraint parameters of ego vehicle
        self.speed_max = parameters["vehicle_para"]["speed_max"]
        self.accel_max = parameters["vehicle_para"]["lon_accel_max"]
        # self.decel_max = -3
        self.jerk_max = parameters["vehicle_para"]["lon_jerk_max"]
        self.lateral_accel_max = parameters["vehicle_para"]["lateral_accel_max"]
        # self.lateral_deccel_max = -0.5
        self.lateral_jerk_max = parameters["vehicle_para"]["lateral_jerk_max"]

        self.centripetal_accel_max = parameters["vehicle_para"]["centripetal_accel_max"]

        # ! road parameters
        self.max_road_width = 3.5  # maximum road width [m]
        self.speeds_ref = None


class IntersectionPlanner:
    def __init__(
        self,
        planner_settings: PlannerSettings,
        observation: Observation,
        method: str = "frenet",
    ):
        self.fplanner_settings = planner_settings
        self.frenet_traj = dict()
        self.time = 0
        self.dt = observation["test_setting"]["dt"]

        self.ego_state = State()
        self.ego_states = list()
        self.ego_frenet_state = FrenetState()
        self.ego_frenet_states = list()
        self.ego_vehicle = None  # 自车参数
        self.time_list = []

        # global planning
        self.route_candidates = list()
        self.refline = None
        self.frenet_traj = dict()
        self.cubic_spline = None  # Cubic spline combination of reference path multi-segment lines.

        self.method = method
        self.init_local_planner(method, observation)

    def init_local_planner(self, method, observation):
        """Select which planning method to use for local trajectory planning."""
        self.ego_vehicle = self._get_ego_vehicle_parameters(observation)
        self.ego_state = self._get_ego_states(observation)

        self.stats = Stats()
        if method == "frenet":
            # baseline
            planner_settings = FrenetOptimalPlannerSettings(
                num_width=paras_planner["planner_frenet"]["num_width"],
                num_speed=paras_planner["planner_frenet"]["num_speed"],
                num_t=paras_planner["planner_frenet"]["num_t"],
                highest_speed=self.fplanner_settings.speed_max,
                lowest_speed=0.0,
                min_planning_t=paras_planner["planner_frenet"]["min_t"],
                max_planning_t=paras_planner["planner_frenet"]["max_t"],
            )
            self.fplanner = FrenetOptimalPlanner(planner_settings, self.ego_vehicle)

        else:
            print("##log## ERROR: Planning method entered is not recognized!")
            raise ValueError

    def _get_ego_vehicle_parameters(self, observation: Observation) -> Vehicle:
        self.ego_vehicle = Vehicle(
            observation["vehicle_info"]["ego"]["length"],
            observation["vehicle_info"]["ego"]["width"],
            longitudinal_v_max=self.fplanner_settings.speed_max,
            longitudinal_a_max=self.fplanner_settings.accel_max,
            longitudinal_jerk_max=self.fplanner_settings.jerk_max,
            lateral_accel_max=self.fplanner_settings.lateral_accel_max,
            lateral_jerk_max=self.fplanner_settings.lateral_jerk_max,
            centripetal_accel_max=self.fplanner_settings.centripetal_accel_max,
        )
        return self.ego_vehicle

    def _get_ego_states(self, observation: Observation) -> State:
        self.ego_state = State(
            t=observation["test_setting"]["t"],
            x=observation["vehicle_info"]["ego"]["x"],
            y=observation["vehicle_info"]["ego"]["y"],
            yaw=observation["vehicle_info"]["ego"]["yaw"],
            v=observation["vehicle_info"]["ego"]["v"],
            a=observation["vehicle_info"]["ego"]["a"],
        )
        return self.ego_state

    def _get_ego_frenet_state(self, ego_states: State = None) -> FrenetState:
        self.ego_frenet_state.from_state(ego_states, self.ref_ego_lane_pts)
        return self.ego_frenet_state

    def generate_frenet_frame(self, centerline_pts: np.ndarray):
        """The parametric curve is modeled (cubic spline curve) based on the sparse point sequence of the reference path center line,
            and then the values of the reference path center line are re-discretized.
        Args:
            centerline_pts (np.ndarray): sparse point sequence of the reference path center line [x, y, yaw , width]*i.

        Returns:
            cubic_spline (CubicSpline2D): cubic spline  .
            (ref_xy, ref_yaw, ref_rk(np.ndarray) : reference path center line are re-discretized,0.1m [x, y, yaw , curve]*N.
        """
        self.cubic_spline = CubicSpline2D(centerline_pts[:, 0], centerline_pts[:, 1])

        s = np.arange(0, self.cubic_spline.s_list[-1], 0.1)
        ref_xy = [self.cubic_spline.calc_position(i_s) for i_s in s]
        # the range of angles [-PI,PI)
        ref_yaw = [self.cubic_spline.calc_yaw(i_s) for i_s in s]
        ref_rk = [self.cubic_spline.calc_curvature(i_s) for i_s in s]
        self.ref_ego_lane_pts = np.column_stack((ref_xy, ref_yaw, ref_rk))

        return self.cubic_spline, self.ref_ego_lane_pts

    def plan_global_route(self, observation, road_info, discretelanes):
        """global path of ego vehicle within this context..

        return global path center line, [x, y, yaw , width].
        1:self.refline ,Read the reference path and concatenate in the offline map file,[x, y, yaw , width];
        2:self.ref_ego_lane_pts, The parametric curve is modeled (cubic spline curve) based on
            the sparse point sequence of the reference path center line,
            and then the values of the reference path center line are re-discretized;[x, y, yaw , curve]*N;
        """
        road_id_list = road_info["road_id_list"]
        lane_id_list = road_info["lane_id_list"]
        road_list = road_info["road_list"]

        start_lane_id = tool.location([self.ego_state.x, self.ego_state.y], discretelanes)
        x_list, y_list = (
            observation["test_setting"]["goal"]["x"],
            observation["test_setting"]["goal"]["y"],
        )

        x_avg = (x_list[0] + x_list[1]) / 2
        y_avg = (y_list[0] + y_list[1]) / 2
        goal_lane_id = tool.location([x_avg, y_avg], discretelanes)  # lane id:"<road.id>.<lane_section.id>.<lane.id>.<width.id>"
        lane_id_parts = goal_lane_id.split(".")
        goal_lane_id_list = []
        goal_lane_id_list.append(goal_lane_id)

        # Search all lanes of the entire cross-section to find the lane that connects to the starting point
        # right 3 lanes
        for lane_number in range(3):
            temp_lane_id = lane_id_parts[0] + "." + lane_id_parts[1] + "." + str(int(lane_id_parts[-2]) - lane_number - 1) + "." + lane_id_parts[-1]
            if temp_lane_id in lane_id_list:
                goal_lane_id_list.append(temp_lane_id)
            else:
                break
        # left 3 lanes
        for lane_number in range(3):
            temp_lane_id = lane_id_parts[0] + "." + lane_id_parts[1] + "." + str(int(lane_id_parts[-2]) + lane_number + 1) + "." + lane_id_parts[-1]
            if temp_lane_id in lane_id_list:
                goal_lane_id_list.append(temp_lane_id)
            else:
                break

        # Search all key area lanes and find those that connect with the starting point;
        # todo  modified to evaluate the route according to cost func and select the optimal route;
        road_found = False
        for road_index, road in enumerate(road_id_list):
            if start_lane_id in road:
                for goal_lane_id in goal_lane_id_list:
                    if goal_lane_id in road:
                        target_road_id = road_index
                        candidate_path_dict = {
                            "target_road_id": target_road_id,
                            "lane_ids": road_info["road_id_list"][target_road_id],
                        }
                        self.route_candidates.append(candidate_path_dict)
                        road_found = True
                        break  # get target lane ID
            if road_found:
                break  #  get road (target lane ID)

        # If no direct connection is found, find an alternative path
        if not road_found:
            possible_paths = tool.getPossiblePath([self.ego_state.x, self.ego_state.y], discretelanes, road_id_list)
            min_dist = np.inf
            target_road_id = -1

            for path_id in possible_paths:
                if goal_lane_id in road_id_list[path_id]:
                    target_road_id = path_id
                    break
            if target_road_id == -1:
                # If you can't find it, go through all possible path points and find the path closest to the target point.
                start_r_id = tool.getRoadID(start_lane_id, road_id_list)
                for r_id in start_r_id:
                    for lane_id in road_id_list[r_id]:
                        discretelane = tool.getDiscreteLane(lane_id, discretelanes)
                        for pt in discretelane.center_vertices:
                            temp_dist = math.hypot(pt[0] - x_avg, pt[1] - y_avg)
                            if temp_dist < min_dist:
                                min_dist = temp_dist
                                target_road_id = r_id
            candidate_path_dict = {
                "target_road_id": road_index,
                "lane_ids": road_info["road_id_list"][target_road_id],
            }
            self.route_candidates.append(candidate_path_dict)

        refline_temp = road_list[target_road_id]
        refline_temp = np.array(refline_temp)
        self.route_path_best = {
            "method_find_path": "nomal",
            "target_road_id": target_road_id,
            "lane_ids": road_info["road_id_list"][target_road_id],
        }

        # Reorder centerline points: Use the unique index found (sorted) to select a unique point in the centerline array,
        refline_temp = np.array(refline_temp, dtype=float)
        _, unqiue_indices = np.unique(refline_temp, return_index=True, axis=0)
        concat_centerline = refline_temp[np.sort(unqiue_indices)]

        # Calculate the orientation of each centerline point
        diff_y = np.diff(concat_centerline[:, 1].flatten())
        diff_x = np.diff(concat_centerline[:, 0].flatten())
        yaws = np.arctan2(diff_y, diff_x)  #!range [-PI,PI)
        yaws = np.append(yaws, [yaws[-1]], axis=0)

        # Calcuate the width of each centerline point, and remove duplicates
        widths = np.concatenate(
            [
                np.array(
                    [np.linalg.norm(discretelane.left_vertices[i] - discretelane.right_vertices[i]) for i in range(len(discretelane.left_vertices))]
                )
                for discretelane in road_info["discretelane_lets_list"][target_road_id]
            ]
        )[np.sort(unqiue_indices)]

        concat_centerline = np.hstack((concat_centerline, yaws[:, np.newaxis], widths[:, np.newaxis]))

        self.refline = concat_centerline

        goal_pose = (x_avg, y_avg, concat_centerline[-1, 2])

        return concat_centerline, goal_pose

    def _plot_global_path(self, concat_centerline):
        plt.figure()
        plt.plot(concat_centerline[:, 0], concat_centerline[:, 1], ".", label="Original points")

        plt.legend()
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.axis("equal")
        plt.title("concat centerline")
        plt.show()
        plt.savefig("concat centerline.png")

    def get_ego_ref_speed(self):
        self.fplanner_settings.speeds_ref = None

    def plan(self, observation: Observation, traj_future: Dict = None, ego_update_mode="kinematics"):
        """Core function of local trajectory planner."""
        self.ego_state = self._get_ego_states(observation)
        self.ego_states.append(self.ego_state)
        if ego_update_mode == "planner_expected_value":
            if observation["test_setting"]["t"] <= 0.05:
                self._get_ego_frenet_state(ego_states=self.ego_state)
            else:
                self.ego_frenet_state = self.fplanner.best_traj.frenet_state_at_time_step(t=1)
        else:
            self._get_ego_frenet_state(ego_states=self.ego_state)
        self.ego_frenet_states.append(self.ego_frenet_state)

        if self.method == "frenet":
            # ! one of baseline  : Frenet Optimal Planner
            # [1] Werling M, Ziegler J, Kammel S, et al. Optimal trajectory generation for dynamic street scenarios
            # in a frenet frame[C/OL]. IEEE, 2010: 987-993.
            # code: https://github.com/SS47816/fiss_plus_planner
            best_traj_ego = self.fplanner.plan_traj(self.ego_frenet_state, self.ego_state, obstacles=traj_future, observation=observation)
            self.stats += self.fplanner.stats

            if best_traj_ego is None:
                print("###log### No solution available for problem:")
                raise ValueError

        elif self.method == "JSSP":
            best_traj_ego = self.fplanner.plan_traj(self.ego_frenet_state, self.ego_state, obstacles=traj_future, observation=observation)

            if best_traj_ego is None:
                print("###log### No solution available for problem:")
                # raise ValueError
                best_traj_ego = self.fplanner.all_trajs_temp[0]
                best_traj_ego.cost_total = 10000.0
                self.fplanner.best_traj = best_traj_ego

        else:
            pass

        plan_acc_result = best_traj_ego.s_dd[1]
        return plan_acc_result

    def motion_control(self, best_traj_ego: GoalSamplingFrenetTrajectory, observation: Observation, length: float, delta_t: float = 0.1):
        x0, y0, yaw0, v0 = (
            observation["vehicle_info"]["ego"]["x"],
            observation["vehicle_info"]["ego"]["y"],
            observation["vehicle_info"]["ego"]["yaw"],
            observation["vehicle_info"]["ego"]["v"],
        )
        x1, y1, yaw1, v1 = best_traj_ego.x[1], best_traj_ego.y[1], best_traj_ego.yaw[1], best_traj_ego.s_d[1]
        a = (v1 - v0) / delta_t

        delta_yaw = yaw1 - yaw0
        delta_yaw = (delta_yaw + np.pi) % (2 * np.pi) - np.pi

        if v0 != 0:
            delta = np.arctan((delta_yaw * length) / (1.7 * v0 * delta_t))
        else:
            delta = 0

        return a, delta

    @staticmethod
    def find_lookahead_point_by_curve_distance(pos: np.array, ref_pos: np.array, ld: float = 5.0):
        """Find the preview point according to the 'curve' preview distance.curve distance

        Args:
            pos (np.array): indicates the current position of the vehicle. The format is [x, y].
            v (float): speed, expressed in meters per second.
            ref_pos (np.array): sequence of waypoints to be tracked [x, y]*N;
            ld (float): curve preview distance, expressed in meters.

        Returns:
            tuple: Contains the coordinates of the preview point and its index in the reference path array.
        """
        dist = np.linalg.norm(ref_pos - pos, axis=1)
        idx = np.argmin(dist)

        l_steps = 0
        size_of_ref_pos = len(ref_pos)

        while l_steps < ld and idx + 1 < size_of_ref_pos:
            l_steps += np.linalg.norm(ref_pos[idx + 1] - ref_pos[idx])
            idx += 1

        lookahead_point = ref_pos[idx]

        return lookahead_point, idx

    @staticmethod
    def find_lookahead_point_by_linear_distance(pos: np.array, ref_pos: np.array, ld: float = 5.0):
        """Find the preview point according to the 'linear' preview distance. linear distance

        Args:
            pos (np.array): indicates the current position of the vehicle. The format is [x, y].
            v (float): speed, expressed in meters per second.
            ref_pos (np.array): sequence of waypoints to be tracked [x, y]*N;
            ld (float): linear preview distance, expressed in meters.

        Returns:
            tuple: Contains the coordinates of the preview point and its index in the reference path array.
        """
        # Compute the straight-line distances from the current position to each point on the reference path
        dist = np.linalg.norm(ref_pos - pos, axis=1)

        # Find the index of the first point on the path that is at least ld meters away
        idx = np.where(dist >= ld)[0]

        # If such a point exists, use the first one. Otherwise, use the last point on the path.
        if idx.size > 0:
            idx = idx[0]
            lookahead_point = ref_pos[idx]
        else:
            idx = len(ref_pos) - 1
            lookahead_point = ref_pos[-1]

        return lookahead_point, idx

    def pure_pursuit_control(self, kv: float = 0.25, ld0: float = 2.8):
        """Path tracking control, pure tracking controller. Gain control: Front wheel steering Angle

        Args:
            kv (float, optional): Pre-viewing distance factor, used to adjust the pre-viewing distance according to the vehicle speed.
            ld0 (float, optional): Lower limit of preview distance to ensure a minimum preview distance at low speeds. Defaults to 5.0.

        Returns:
            float: steer_angle
        """
        kv = paras_control["pure_pursuit"]["kv_ld0"][0]
        ld0 = paras_control["pure_pursuit"]["kv_ld0"][1]
        coff_wheel_base = paras_control["pure_pursuit"]["coff_wheel_base"]

        if parameters["sim_config"]["kinematics_path_tracing_type"] == "ref_path":
            ref_pos = self.ref_ego_lane_pts[:, 0:2]
        elif parameters["sim_config"]["kinematics_path_tracing_type"] == "plan_path":
            ref_pos = np.column_stack((self.fplanner.best_traj.x, self.fplanner.best_traj.y))
        else:
            raise Exception("#log#kinematics_path_tracing_type error")

        ld = kv * self.ego_state.v + ld0
        pos = np.array([self.ego_state.x, self.ego_state.y])
        # l = self.ego_vehicle.l / 1.7

        # lookahead_point, idx = self.find_lookahead_point_by_linear_distance(pos, ref_pos, ld)
        lookahead_point, idx = self.find_lookahead_point_by_curve_distance(pos, ref_pos, ld)
        if idx < len(ref_pos):
            point_temp = lookahead_point
        else:
            point_temp = ref_pos[-1]

        alpha = np.arctan2(point_temp[1] - pos[1], point_temp[0] - pos[0]) - self.ego_state.yaw
        wheel_base = self.ego_vehicle.l / coff_wheel_base
        steer_angle = np.arctan2(2 * wheel_base * np.sin(alpha), ld)

        return steer_angle

    @staticmethod
    def update_ego_states_from_planner(best_traj_ego: GoalSamplingFrenetTrajectory, observation: Observation) -> Observation:
        observation["vehicle_info"]["ego"]["x"] = copy.deepcopy(best_traj_ego.x[1])
        observation["vehicle_info"]["ego"]["y"] = copy.deepcopy(best_traj_ego.y[1])
        observation["vehicle_info"]["ego"]["yaw"] = copy.deepcopy(best_traj_ego.yaw[1])
        observation["vehicle_info"]["ego"]["v"] = copy.deepcopy(best_traj_ego.s_d[1])
        observation["vehicle_info"]["ego"]["a"] = copy.deepcopy(best_traj_ego.s_dd[1])
        return observation


if __name__ == "__main__":
    # test for spd planner
    pass
