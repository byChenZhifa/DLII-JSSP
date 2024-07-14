import copy
import math
from queue import PriorityQueue
import os, sys
import time

from shapely.geometry import Polygon
from shapely import affinity
import numpy as np
from scipy.integrate import cumtrapz as cumtrapz


def add_path_to_sys(target: str):
    abs_path = os.path.abspath(target)
    if abs_path not in sys.path:
        sys.path.append(abs_path)


dir_current_file = os.path.dirname(__file__)
dir_parent_1 = os.path.dirname(dir_current_file)
add_path_to_sys(dir_current_file)
add_path_to_sys(dir_parent_1)

from common.cost.cost_function import JerkSpaceSamplingCostFunction
from common.geometry.cubic_spline import CubicSpline2D
from common.geometry.polynomial import QuarticPolynomial, QuinticPolynomial
from common.scenario.frenet import State, FrenetState, JerkSpaceSamplingFrenetTrajectory
from common.vehicle.vehicle import Vehicle
from spatio_temporal_graph_node import StNode
from configuration_parameters import parameters, paras_control, paras_planner
from common.plot_scripts.plot_polyvt_sampling import plot_jerk_sampling_of_best_traj
from intersection_local_planner.polyvt_sampling import PolyVTSampling


class JerkSpaceSamplingPlannerSettings(object):
    def __init__(
        self,
        num_width: int = 5,
        num_jerk: int = 5,
        delta_t: float = 0.1,
        max_road_width: float = 3.5,
        highest_jerk: float = 13.4112,
        lowest_jerk: float = 0.0,
        plan_horizon: float = 5.0,
        traj_num_max: int = 5,
    ):
        self.delta_t = delta_t  # time tick [s] , time resolution between two planned waypoints
        self.plan_horizon = plan_horizon

        # sampling parameters
        self.max_road_width = max_road_width  # maximum road width [m]
        self.num_width = num_width  # road width sampling number

        self.highest_jerk = highest_jerk  # highest sampling speed [m/s]
        self.lowest_jerk = lowest_jerk  # lowest sampling speed [m/s]
        self.num_jerk = num_jerk  # speed sampling number

        self.jerk_values_sampling = np.linspace(
            -self.highest_jerk,
            self.highest_jerk,
            self.num_jerk,
        )

        self.traj_num_max = traj_num_max


class JerkSpaceSamplingPlanner(object):
    def __init__(self, planner_settings: JerkSpaceSamplingPlannerSettings, ego_vehicle: Vehicle):
        self.settings = planner_settings
        self.ego_vehicle = ego_vehicle
        self.ego_frenet_state = None
        self.cost_function = JerkSpaceSamplingCostFunction("WX1", vehicle_info=ego_vehicle, max_road_width=self.settings.max_road_width)
        self.cubic_spline = None
        self.best_traj = None
        self.all_trajs = []
        self.all_path_sampling_trajs = []

        self.time_step_max = int(round(self.settings.plan_horizon / self.settings.delta_t))

    def sampling_frenet_trajectories(self, frenet_state: FrenetState, obstacles, time_step_now, observation) -> list:

        trajs_per_timestep = []

        #! longitudinal sampling
        pvt_sampler = PolyVTSampling(
            total_time=self.settings.plan_horizon,
            delta_t=self.settings.delta_t,
            jerk_min=-self.settings.highest_jerk,
            jerk_max=self.settings.highest_jerk,
            a_min=-parameters["vehicle_para"]["lon_accel_max"],
            a_max=parameters["vehicle_para"]["lon_accel_max"],
            v_max=parameters["vehicle_para"]["speed_max"],
            num_samples=self.settings.num_jerk,
        )
        seeds_temp = pvt_sampler.sampling(s0=frenet_state.s, v0=frenet_state.s_d, a0=frenet_state.s_dd)  # 不同at线形直线拼接采样种子.
        seeds_stop = pvt_sampler.sampling_stop_seeds_supplement(s0=frenet_state.s, v0=frenet_state.s_d, a0=frenet_state.s_dd)
        seeds = seeds_temp + seeds_stop

        #! lateral sampling
        if self.settings.num_width <= 1:
            di_list = [0.0]
        else:
            sampling_width = self.settings.max_road_width - self.ego_vehicle.w
            di_list = np.linspace(-sampling_width / 2, sampling_width / 2, self.settings.num_width)

        for path_id, di in enumerate(di_list):
            ft = JerkSpaceSamplingFrenetTrajectory()
            ft.path_id = path_id
            lat_qp = QuinticPolynomial(
                xs=frenet_state.d,
                vxs=frenet_state.d_d,
                axs=frenet_state.d_dd,
                xe=di,
                vxe=0.0,
                axe=0.0,
                time=self.settings.plan_horizon,
            )
            ft.t = np.linspace(0.0, self.settings.plan_horizon, self.time_step_max + 1).tolist()
            ft.d = [lat_qp.calc_point(t) for t in ft.t]
            ft.d_d = [lat_qp.calc_first_derivative(t) for t in ft.t]
            ft.d_dd = [lat_qp.calc_second_derivative(t) for t in ft.t]
            ft.d_ddd = [lat_qp.calc_third_derivative(t) for t in ft.t]

            all_traj = pvt_sampler.get_all_frenet_trajectory(s0=frenet_state.s, v0=frenet_state.s_d, a0=frenet_state.s_dd, seeds=seeds, ft=ft)
            trajs_per_timestep.extend(all_traj)

        self.all_path_sampling_trajs.append(trajs_per_timestep)

        return trajs_per_timestep

    def calc_global_paths(self, ftlist: list[JerkSpaceSamplingFrenetTrajectory]) -> list:

        passed_ftlist = []
        for ft in ftlist:
            # Calculate global positions for each point in the  Frenet trajectory ft
            for i in range(len(ft.s)):
                ix, iy = self.cubic_spline.calc_position(ft.s[i])
                if ix is None:
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
                ft.c = np.divide(np.diff(ft.yaw), ft.ds)  # !  yaw  [-pi  ,pi].
                ft.c_d = np.divide(np.diff(ft.c), dt)
                ft.c_dd = np.divide(np.diff(ft.c_d), dt)

                # Append the trajectory with calculated global parameters to the list
                passed_ftlist.append(ft)
        return ftlist

    def check_constraints_curvature(self, trajs: list[JerkSpaceSamplingFrenetTrajectory]) -> list:
        passed = []
        for i, traj in enumerate(trajs):
            # 1）Max curvature check
            if any([abs(c) > self.ego_vehicle.max_curvature for c in traj.c]):
                continue

            traj.constraint_passed = True
            passed.append(i)
        return [trajs[i] for i in passed]

    def check_constraints(self, trajs: list[JerkSpaceSamplingFrenetTrajectory]) -> list:

        passed = []
        for i, traj in enumerate(trajs):
            # 1）Max curvature check
            # if any([abs(c) > self.ego_vehicle.max_curvature for c in traj.c]):
            #     continue
            # if any([abs(c_d) > self.ego_vehicle.max_kappa_d for c_d in traj.c_d]):
            #     continue
            # if any([abs(c_dd) > self.ego_vehicle.max_kappa_dd for c_dd in traj.c_dd]):
            #     continue
            # 2）Max speed check,
            # if any([v > self.ego_vehicle.max_speed for v in traj.s_d]):
            #     continue
            # 3）Max accel check
            if any([abs(a) > self.ego_vehicle.max_accel for a in traj.s_dd]):
                continue
            # 4）max jerk check;
            if any([abs(jerk) > self.ego_vehicle.max_jerk for jerk in traj.s_ddd]):
                continue
            # 5) max_centripetal_accel check
            # if any([abs(curvature * v**2) > self.ego_vehicle.max_centripetal_accel for curvature, v in zip(traj.c, traj.s_d)]):
            #     continue

            # todo
            traj.constraint_passed = True
            passed.append(i)
        return [trajs[i] for i in passed]

    @staticmethod
    def construct_polygon(polygon: Polygon, x: float, y: float, yaw: float) -> Polygon:
        polygon_translated = affinity.translate(polygon, xoff=x, yoff=y)
        polygon_rotated = affinity.rotate(polygon_translated, yaw, use_radians=True)

        return polygon_rotated

    @staticmethod
    def get_vehicle_polygon(l, w):
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

    @staticmethod
    def has_collision(
        ego_vehicle,
        traj: JerkSpaceSamplingFrenetTrajectory,
        obstacles: list,
        time_step_now: int = 0,
        check_res: int = 1,
        observation: dict = None,
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
                    ego_polygon = JerkSpaceSamplingPlanner.construct_polygon(ego_vehicle.polygon, traj.x[i], traj.y[i], traj.yaw[i])
                except:
                    print(f"Failed to create Polygon for t={i} x={traj.x[i]}, y={traj.y[i]}, yaw={traj.y[i]}")
                    return True, num_polys
                else:
                    # construct a polygon for the obstacle at time step i
                    t_step = i + time_step_now
                    str_time = str(round(t_step * observation["test_setting"]["dt"], 2))
                    for key, obstacle in obstacles.items():
                        if str_time in obstacle.keys():
                            state = obstacle[str_time]
                            # !Obstacle increases expansion value
                            box_polygon = JerkSpaceSamplingPlanner.get_vehicle_polygon(
                                l=obstacle["shape"]["length"] + 0.1, w=obstacle["shape"]["width"] + 0.1
                            )
                            obstacle_polygon = JerkSpaceSamplingPlanner.construct_polygon(box_polygon, x=state["x"], y=state["y"], yaw=state["yaw"])
                            num_polys += 1
                            if ego_polygon.intersects(obstacle_polygon):  # Returns True if geometries intersect, else False
                                # plot_collision(ego_polygon, obstacle_polygon, t_step)
                                return True, num_polys

        return False, num_polys

    def check_collisions(self, trajs: list, obstacles: list, time_step_now: int = 0, observation: dict = None) -> list:

        passed = []

        for i, traj in enumerate(trajs):
            # Collision check
            collision, num_polys = JerkSpaceSamplingPlanner.has_collision(self.ego_vehicle, traj, obstacles, time_step_now, 2, observation)
            if collision:
                continue
            traj.collision_passed = True
            passed.append(i)

        return [trajs[i] for i in passed]

    def calc_all_trajs_cost(self, trajs: list) -> tuple[PriorityQueue, list]:
        candidate_trajs = PriorityQueue()
        for id_trajs, traj in enumerate(trajs):
            traj = self.cost_function.cost_total(traj=traj, target_speed=parameters["vehicle_para"]["speed_max"])
            candidate_trajs.put((traj.cost_total, traj, id_trajs))

        return candidate_trajs, trajs

    def plan_traj(self, frenet_state: FrenetState, ego_state: State, obstacles: list, observation) -> JerkSpaceSamplingFrenetTrajectory:

        self.ego_frenet_state = frenet_state
        self.ego_state = ego_state
        time_step_now = int(observation["test_setting"]["t"] / observation["test_setting"]["dt"])

        start_time = time.time()
        ftlist = self.sampling_frenet_trajectories(frenet_state, obstacles, time_step_now, observation)
        print("##log##", len(ftlist), "trajectories are sampled.")
        end_time_1 = time.time()
        print(f"###log### sampling time:{end_time_1 - start_time}\n")

        ftlist = self.calc_global_paths(ftlist)
        end_time_2 = time.time()
        print(f"###log### calc_global_paths time:{end_time_2 - end_time_1}\n")

        ftlist = self.check_constraints_curvature(ftlist)
        ftlist = self.check_collisions(ftlist, obstacles, time_step_now, observation)
        end_time_3 = time.time()
        print(f"###log### check_collisions time:{end_time_3 - end_time_2}\n")
        # ftlist = self.check_collision_using_multi_process(ftlist, obstacles, time_step_now, observation)
        # end_time_3 = time.time()
        # print(f"###log### check_collision_using_multi_process time:{end_time_3 - end_time_2}\n")

        ftqueue, ftlist = self.calc_all_trajs_cost(ftlist)
        self.all_trajs.append(ftlist)

        print("##log##", len(ftlist), "trajectories passed collision check.")

        # find minimum cost path
        if not ftqueue.empty():
            _, self.best_traj, best_traj_id_ftlist = ftqueue.get()

            # ---------- paper plot: spd_planning_cache ----------
            #! note: Turn off when drawing trajectories;
            if False:
                max_cost_traj = max(ftlist, key=lambda x: x.cost_total)

                trajs_one_path = []
                best_traj_ftlist = ftlist[best_traj_id_ftlist]
                id_path = best_traj_ftlist.path_id
                trajs_one_path.append(best_traj_ftlist)
                best_traj_id = 0
                for traj in self.all_path_sampling_trajs[time_step_now]:
                    if traj.path_id == id_path:
                        if traj.cost_total == float("inf"):
                            traj.cost_total = max_cost_traj.cost_total * 1.5
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

        else:
            print("##log## No solution.")

        return self.best_traj

    def generate_frenet_frame(self, centerline_pts: np.ndarray):
        self.cubic_spline = CubicSpline2D(centerline_pts[:, 0], centerline_pts[:, 1])
        s = np.arange(0, self.cubic_spline.s_list[-1], 0.1)
        ref_xy = [self.cubic_spline.calc_position(i_s) for i_s in s]
        ref_yaw = [self.cubic_spline.calc_yaw(i_s) for i_s in s]
        ref_rk = [self.cubic_spline.calc_curvature(i_s) for i_s in s]

        return self.cubic_spline, np.column_stack((ref_xy, ref_yaw, ref_rk))

    def init_speed_limit_lookup_for_frenet_path(self):
        """# todo ,constant 20.0 mps"""
        pass
