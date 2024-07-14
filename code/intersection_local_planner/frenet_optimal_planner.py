import copy
import math
import os, sys
import time

from shapely.geometry import Polygon
from shapely import affinity
import numpy as np


def add_path_to_sys(target: str):
    abs_path = os.path.abspath(target)
    if abs_path not in sys.path:
        sys.path.append(abs_path)


dir_current_file = os.path.dirname(__file__)
dir_parent_1 = os.path.dirname(dir_current_file)
add_path_to_sys(dir_current_file)
add_path_to_sys(dir_parent_1)

from common.cost.cost_function import GoalSamplingCostFunction
from common.geometry.cubic_spline import CubicSpline2D
from common.geometry.polynomial import QuarticPolynomial, QuinticPolynomial
from common.scenario.frenet import State, FrenetState, GoalSamplingFrenetTrajectory
from common.vehicle.vehicle import Vehicle
from common.plot_scripts.plot_polyvt_sampling import plot_jerk_sampling_of_best_traj
from configuration_parameters import parameters, paras_control, paras_planner


class Stats(object):
    def __init__(self):
        self.num_iter = 0
        self.num_trajs_generated = 0
        self.num_trajs_validated = 0
        self.num_collison_checks = 0
        # self.best_traj_costs = [] # float("inf")

    def __add__(self, other):
        self.num_iter += other.num_iter
        self.num_trajs_generated += other.num_trajs_generated
        self.num_trajs_validated += other.num_trajs_validated
        self.num_collison_checks += other.num_collison_checks
        # self.best_traj_costs.extend(other.best_traj_costs)
        return self

    def average(self, value: int):
        self.num_iter /= value
        self.num_trajs_generated /= value
        self.num_trajs_validated /= value
        self.num_collison_checks /= value
        return self


class FrenetOptimalPlannerSettings(object):
    def __init__(
        self,
        num_width: int = 5,
        num_speed: int = 5,
        num_t: int = 5,
        delta_t: float = 0.1,
        max_road_width: float = 3.5,
        highest_speed: float = 13.4112,
        lowest_speed: float = 0.0,
        min_planning_t: float = 5.0,
        max_planning_t: float = 8.0,
    ):
        # time resolution between two planned waypoints
        self.delta_t = delta_t  # time tick [s]

        # sampling parameters
        self.num_width = num_width  # road width  sampling number
        self.num_t = num_t  # time sampling number
        self.num_speed = num_speed  # speed sampling number

        self.max_road_width = max_road_width  # maximum road width [m]
        self.highest_speed = highest_speed  # highest sampling speed [m/s]
        self.lowest_speed = lowest_speed  # lowest sampling speed [m/s]
        self.min_t = min_planning_t
        self.max_t = max_planning_t

        # self.check_obstacle = True  # True if check collison with obstacles
        # self.check_boundary = True  # True if check collison with road boundaries


class FrenetOptimalPlanner(object):
    def __init__(self, planner_settings: FrenetOptimalPlannerSettings, ego_vehicle: Vehicle):
        self.settings = planner_settings
        self.ego_vehicle = ego_vehicle
        self.cost_function = GoalSamplingCostFunction("WX1", vehicle_info=ego_vehicle, max_road_width=self.settings.max_road_width)
        self.cubic_spline = None
        self.best_traj = None
        self.all_trajs = []

        self.stats = Stats()

    def sampling_frenet_trajectories(self, frenet_state: FrenetState) -> list:
        frenet_trajectories = []

        sampling_width = self.settings.max_road_width - self.ego_vehicle.w
        #! lateral sampling
        traj_per_timestep = []
        for path_id, di in enumerate(np.linspace(-sampling_width / 2, sampling_width / 2, self.settings.num_width)):
            # for di in np.linspace(-sampling_width / 2, sampling_width / 2, self.settings.num_width):

            #! time sampling
            # xx=np.linspace(self.settings.min_t, self.settings.max_t, self.settings.num_t)
            for Ti in np.linspace(self.settings.min_t, self.settings.max_t, self.settings.num_t):
                # aa = abs(Ti % 0.1)
                # if aa > 1e-3 and aa < 0.09:  # epsilon = 1e-10
                #     raise Exception("##log##:error.")
                ft = GoalSamplingFrenetTrajectory()
                ft.path_id = path_id

                lat_qp = QuinticPolynomial(frenet_state.d, frenet_state.d_d, frenet_state.d_dd, di, 0.0, 0.0, Ti)
                ft.t = [t for t in np.arange(0.0, Ti, self.settings.delta_t)]
                ft.d = [lat_qp.calc_point(t) for t in ft.t]
                ft.d_d = [lat_qp.calc_first_derivative(t) for t in ft.t]
                ft.d_dd = [lat_qp.calc_second_derivative(t) for t in ft.t]
                ft.d_ddd = [lat_qp.calc_third_derivative(t) for t in ft.t]

                #! longitudinal sampling
                for tv in np.linspace(self.settings.lowest_speed, self.settings.highest_speed, self.settings.num_speed):
                    tft = copy.deepcopy(ft)

                    lon_qp = QuarticPolynomial(frenet_state.s, frenet_state.s_d, frenet_state.s_dd, tv, 0.0, Ti)
                    tft.s = [lon_qp.calc_point(t) for t in ft.t]
                    tft.s_d = [lon_qp.calc_first_derivative(t) for t in ft.t]
                    tft.s_dd = [lon_qp.calc_second_derivative(t) for t in ft.t]
                    tft.s_ddd = [lon_qp.calc_third_derivative(t) for t in ft.t]

                    # Compute the final cost
                    tft.cost_total = self.cost_function.cost_total(
                        tft,
                        self.settings.highest_speed,
                        t_max=self.settings.max_t,
                        t_min=self.settings.min_t,
                    )
                    frenet_trajectories.append(tft)
                    traj_per_timestep.append(tft)

        self.all_trajs.append(traj_per_timestep)

        return frenet_trajectories

    def calc_global_paths(self, ftlist: list[GoalSamplingFrenetTrajectory]) -> list:
        passed_ftlist = []
        for ft in ftlist:
            # Calculate global positions for each point in the  Frenet trajectory ft
            for i in range(len(ft.s)):
                ix, iy = self.cubic_spline.calc_position(ft.s[i])
                if ix is None:  #! The distance s from the end of the reference path is exceeded
                    break
                # Calculate global yaw (orientation) from Frenet s coordinate
                i_yaw = self.cubic_spline.calc_yaw(ft.s[i])
                di = ft.d[i]
                # Calculate the final x, y coordinates considering lateral offset (d)
                fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
                fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
                ft.x.append(fx)
                ft.y.append(fy)

            if len(ft.x) >= 2:
                #  Convert lists to numpy arrays for easier mathematical operations
                # calc yaw and ds
                ft.x = np.array(ft.x)
                ft.y = np.array(ft.y)
                # # Calculate differences between consecutive x and y coordinates
                x_d = np.diff(ft.x)
                y_d = np.diff(ft.y)
                # Calculate yaw angles and their differences
                ft.yaw = np.arctan2(y_d, x_d)
                ft.ds = np.hypot(x_d, y_d)
                ft.yaw = np.append(ft.yaw, ft.yaw[-1])
                # Calculate curvature (c), its first derivative (c_d), and second derivative (c_dd)
                dt = self.settings.delta_t
                ft.c = np.divide(np.diff(ft.yaw), ft.ds)  #! Make sure yaw is -pi to pi.
                ft.c_d = np.divide(np.diff(ft.c), dt)
                ft.c_dd = np.divide(np.diff(ft.c_d), dt)

                # Append the trajectory with calculated global parameters to the list
                passed_ftlist.append(ft)
        return ftlist

    def check_constraints(self, trajs: list[GoalSamplingFrenetTrajectory]) -> list:
        """Performs constraint checks on the trajectory.

            1) Max curvature check.
            2) Max Speed Check: Ensure that the speed on the track does not exceed the maximum speed limit of the vehicle at any point.
            This is done by checking whether the longitudinal velocity s_d on each point in the trajectory exceeds self.ego_vehicle.max_speed.
            3) Max Acceleration Check: Ensure that the acceleration on the track does not exceed the maximum acceleration limit of the vehicle.
            This is done by checking whether the longitudinal acceleration s_dd at each point in the trajectory exceeds self.ego_vehicle.max_accel.

        Args:
            Trajs [GoalSamplingFrenetTrajectory] (list) : GoalSamplingFrenetTrajectory list. In fact, only one value is passed in at a time

        Returns:
            The function returns a list of tracks that have passed all checks.
        """
        passed = []
        for i, traj in enumerate(trajs):
            # 1）Max curvature check
            # if any([abs(c) > self.ego_vehicle.max_curvature for c in traj.c]):
            #     continue
            # if any([abs(c_d) > self.ego_vehicle.max_kappa_d for c_d in traj.c_d]):
            #     continue
            # if any([abs(c_dd) > self.ego_vehicle.max_kappa_dd for c_dd in traj.c_dd]):
            #     continue
            # 2）Max speed check
            # if any([v > self.ego_vehicle.max_speed for v in traj.s_d]):
            #     continue
            # 3）Max accel check
            if any([abs(a) > self.ego_vehicle.max_accel for a in traj.s_dd]):
                continue
            # 4）max jerk check;
            if any([abs(jerk) > self.ego_vehicle.max_jerk for jerk in traj.s_ddd]):
                continue
            # 5) max_centripetal_accel check #!bug,Cut out all the fast ones in the corners
            # if any([abs(curvature * v**2) > self.ego_vehicle.max_centripetal_accel for curvature, v in zip(traj.c, traj.s_d)]):
            #     continue

            # todo
            traj.constraint_passed = True
            passed.append(i)
        return [trajs[i] for i in passed]

    def construct_polygon(self, polygon: Polygon, x: float, y: float, yaw: float) -> Polygon:
        polygon_translated = affinity.translate(polygon, xoff=x, yoff=y)
        polygon_rotated = affinity.rotate(polygon_translated, yaw, use_radians=True)

        return polygon_rotated

    def get_vehicle_polygon(self, l, w):
        # footprint coordinates (in clockwise direction)
        corners: list[tuple[float, float]] = [
            (l / 2, w / 2),  # front left corner's coordinates in box_center frame [m]
            (l / 2, -w / 2),  # front right corner's coordinates in box_center frame [m]
            (-l / 2, -w / 2),  # rear right corner's coordinates in box_center frame [m]
            (-l / 2, w / 2),  # rear left corner's coordinates in box_center frame [m]
            (l / 2, w / 2),  # front left corner's coordinates in box_center frame [m] (to enclose the polygon)
        ]
        vehicle_polygon: Polygon = Polygon(corners)

        return vehicle_polygon

    def has_collision(
        self, traj: GoalSamplingFrenetTrajectory, obstacles: list, time_step_now: int = 0, check_res: int = 1, observation: dict = None
    ) -> tuple:

        num_polys = 0
        if len(obstacles) <= 0:
            return False, 0

        final_time_step = int(observation["test_setting"]["max_t"] / observation["test_setting"]["dt"])  # 100 step = 10.0s
        t_step_max = min(len(traj.x), final_time_step - time_step_now)
        for i in range(t_step_max):
            if i % check_res == 0:
                # construct a polygon for the ego ego_vehicle at time step i
                try:
                    ego_polygon = self.construct_polygon(self.ego_vehicle.polygon, traj.x[i], traj.y[i], traj.yaw[i])
                except:
                    print(f"Failed to create Polygon for t={i} x={traj.x[i]}, y={traj.y[i]}, yaw={traj.y[i]}")
                    return True, num_polys
                else:
                    # construct a polygon for the obstacle at time step i
                    t_step = i + time_step_now
                    str_time = str(round(t_step * observation["test_setting"]["dt"], 2))
                    for key, obstacle in obstacles.items():
                        if str_time in obstacle.keys():
                            state = obstacle[str_time]  # state['y']
                            box_polygon = self.get_vehicle_polygon(l=obstacle["shape"]["length"], w=obstacle["shape"]["width"])
                            obstacle_polygon = self.construct_polygon(box_polygon, x=state["x"], y=state["y"], yaw=state["yaw"])
                            num_polys += 1
                            if ego_polygon.intersects(obstacle_polygon):  # Returns True if geometries intersect, else False
                                # plot_collision(ego_polygon, obstacle_polygon, t_step)
                                return True, num_polys

        return False, num_polys

    def check_collisions(self, trajs: list, obstacles: list, time_step_now: int = 0, observation: dict = None) -> list:
        passed = []

        for i, traj in enumerate(trajs):
            # Collision check
            collision, num_polys = self.has_collision(traj, obstacles, time_step_now, 2, observation)
            if collision:
                continue
            traj.collision_passed = True
            passed.append(i)

        return [trajs[i] for i in passed]

    def plan_traj(self, frenet_state: FrenetState, ego_state: State, obstacles: list, observation) -> GoalSamplingFrenetTrajectory:
        """The core function of the frenet planning method.

        Args:
            frenet_state (FrenetState): indicates the frene status of the car
            obstacles (list): obstacles.
            observation (_type_): Some state data of the scene.

        Returns:
            GoalSamplingFrenetTrajectory: planning results.
        """

        time_step_now = int(observation["test_setting"]["t"] / observation["test_setting"]["dt"])
        self.ego_state = ego_state

        start_time = time.time()
        ftlist = self.sampling_frenet_trajectories(frenet_state)
        print("##log##", len(ftlist), "trajectories are sampled.")
        end_time_1 = time.time()
        print(f"###log### sampling time:{end_time_1 - start_time}\n")

        ftlist = self.calc_global_paths(ftlist)
        end_time_2 = time.time()
        print(f"###log### calc_global_paths time:{end_time_2 - end_time_1}\n")

        self.stats.num_trajs_generated = len(ftlist)
        self.stats.num_trajs_validated = len(ftlist)
        self.stats.num_collison_checks = len(ftlist)
        ftlist = self.check_constraints(ftlist)
        print(f"##log## {len(ftlist)} trajectories passed constraint check")
        ftlist = self.check_collisions(ftlist, obstacles, time_step_now, observation)
        print(f"##log## {len(ftlist)} trajectories passed collision check")
        end_time_3 = time.time()
        print(f"###log### check_collisions time:{end_time_3 - end_time_2}\n")

        # find minimum cost path
        min_cost = float("inf")
        for ft in ftlist:
            if min_cost >= ft.cost_total:
                min_cost = ft.cost_total
                self.best_traj = ft

        if True:  # ---------- paper plot figure: spd_planning_cache ----------
            # 1) Take out all the tracks corresponding to path_id for the best track, and use black marks for the best track
            trajs_one_path = []
            id_path = self.best_traj.path_id
            trajs_one_path.append(self.best_traj)
            best_traj_id = 0
            for traj in self.all_trajs[time_step_now]:
                if traj.path_id == id_path:
                    if traj is not self.best_traj:
                        trajs_one_path.append(traj)

            def check_dir(target_dir):
                if not os.path.exists(target_dir):
                    os.makedirs(target_dir)

            dir_parent_2 = os.path.dirname(dir_parent_1)
            OUTPUT_DIR = os.path.join(dir_parent_2, "paper_data", "figure_output")
            method_local_planner = paras_planner["planner_type"]
            scenario_name = observation["test_setting"]["scenario_name"]
            result_path = os.path.join(OUTPUT_DIR, "spd_planning_cache", method_local_planner, scenario_name)
            check_dir(result_path)
            step = int(observation["test_setting"]["t"] / observation["test_setting"]["dt"])
            fig_path = os.path.join(result_path, "{time_step}.png".format(time_step=step))
            plot_jerk_sampling_of_best_traj(trajectories=trajs_one_path, best_traj_id=best_traj_id, dir_png=fig_path)

        return self.best_traj
