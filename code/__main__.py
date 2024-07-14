# Python library
import time
import sys
import os
from typing import Dict, List, Union, Tuple

# Third-party library
# import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


def add_path_to_sys(target: str):
    abs_path = os.path.abspath(target)
    if abs_path not in sys.path:
        sys.path.append(abs_path)


dir_current_file = os.path.dirname(__file__)
dir_parent_1 = os.path.dirname(dir_current_file)
add_path_to_sys(dir_current_file)
add_path_to_sys(os.path.join(dir_current_file, "common"))
add_path_to_sys(os.path.join(dir_current_file, "onsite_trans"))
add_path_to_sys(os.path.join(dir_current_file, "intersection_local_planner"))

# Local library
from onsite_trans import scenarioOrganizer, env
from onsite_trans.controller import ReplayParser
import common.tool as tool
import common.utils.math_utils as math_utils
from intersection_local_planner.intersection_planner import IntersectionPlanner, PlannerSettings
from intersection_local_planner.visualizer_for_paper_plan import PlanVisualizer
from configuration_parameters import parameters, paras_planner


def check_dir(target_dir):
    if not os.path.exists(target_dir):
        os.makedirs(target_dir)


if __name__ == "__main__":
    # 指定输入输出文件夹位置
    input_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../inputs"))
    output_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../outputs"))
    check_dir(output_dir)
    tic = time.time()
    # Instantiate the Scene Management module (ScenairoOrganizer) and Scene Test Module (Env)
    so = scenarioOrganizer.ScenarioOrganizer()
    envi = env.Env()
    so.load(input_dir, output_dir)
    print(f"###test paras###\n{so.config}")

    while True:
        # Use the scenario management module to give the next scenario to be tested
        scenario_to_test = so.next()
        if scenario_to_test is None:
            break  # all scenarios have been tested.
        print(f"<scene-{scenario_to_test['data']['scene_name']}>")
        try:
            observation, traj = envi.make(scenario=scenario_to_test)
            parser = ReplayParser()
            replay_info = parser.parse(scenario_to_test["data"])
            observation_paper = dict()
            observation_paper["vehicle_info"] = observation["vehicle_info"]
            observation_paper["test_setting"] = observation["test_setting"]
            observation_paper["replay_info"] = replay_info

            # ========== Code development ==========
            # Road map information preprocessing part: path splicing
            lane_id_list = list()
            for discrete_lane in replay_info.road_info.discretelanes:
                lane_id_list.append(discrete_lane.lane_id)
            road_id_list, road_list, discretelane_lets_list, x_min, x_max, y_min, y_max = tool.modifyLane(replay_info.road_info.discretelanes)
            road_info = dict()
            road_info["lane_id_list"] = lane_id_list
            road_info["road_id_list"] = road_id_list
            road_info["road_list"] = road_list
            road_info["discretelane_lets_list"] = discretelane_lets_list
            road_info["road_info_from_parser"] = replay_info.road_info
            observation_paper["roads_limit"] = {"x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max}

            # Partial data repair :1) Calculate yaw rate; 2) Smooth the yaw of the vehicle, and there is a jump
            traj_add_yawRate = dict()
            traj_add_yawRate = tool.calc_yaw_rate_other_vehicle(traj, observation["test_setting"]["dt"])
            traj = traj_add_yawRate

            tool.initDiscreteLaneBox(replay_info.road_info.discretelanes)
            tool.initDiscreteLaneBoxForPredict(replay_info.road_info.discretelanes)
            tool.envErrorDetect(replay_info.road_info.discretelanes, replay_info.vehicle_traj)
            SCENE = tool.scene_recognition(observation)

            if SCENE == tool.SceneType.INTERSECTION:

                planner_settings = PlannerSettings()
                method_local_planner = paras_planner["planner_type"]
                planner = IntersectionPlanner(planner_settings, observation, method_local_planner)
                ego_route_pts, goal_pose = planner.plan_global_route(observation, road_info, replay_info.road_info.discretelanes)  # 全局规划器

                planner.fplanner.cubic_spline, ref_ego_lane_pts = planner.generate_frenet_frame(ego_route_pts)

                # paper plot figure
                if paras_planner["is_visualize_plan"]:
                    visualizer_plan = PlanVisualizer()
                    plot_figure_for_method = paras_planner["plot_figure_for_method"]
                    visualizer_plan.init(
                        observation=observation_paper, goal_pose=goal_pose, visilize=True, method_local_planner=plot_figure_for_method
                    )
                    visualizer_plan.plot_scenario(
                        observation=observation_paper, planner=planner, draw_route_reference_path=False, draw_route_lanelets=False
                    )

            elif SCENE == tool.SceneType.HIGHWAY_RAMP:
                pass  # This scenario is outside the scope of this paper
            elif SCENE == tool.SceneType.HIGHWAY_BASE:
                pass  # This scenario is outside the scope of this paper
            else:
                print("###log### scene recognition error!")

            scence_dt = observation["test_setting"]["dt"]
            time_list = []
            final_time_step = int(observation["test_setting"]["max_t"] / observation["test_setting"]["dt"])

            while observation["test_setting"]["end"] == -1:  # ========== single scene time-step replay test ==========
                if SCENE == tool.SceneType.INTERSECTION:
                    ################ planner start #############
                    start_time = time.time()
                    # Test Planner. Future trajectories of vehicle obstacles use truth values
                    plan_acc = planner.plan(observation, traj, ego_update_mode=parameters["sim_config"]["ego_update_mode"])
                    end_time = time.time()

                    time_list.append(end_time - start_time)
                    time_step = len(time_list)
                    print(f"###log### time step:{time_step}/{final_time_step},{method_local_planner} planning time:{end_time - start_time}\n")

                    if parameters["sim_config"]["ego_update_mode"] == "kinematics":
                        # Motion control part, path tracker, calculating front wheel Angle
                        plan_omege = planner.pure_pursuit_control()
                        action = (plan_acc, plan_omege)
                    elif parameters["sim_config"]["ego_update_mode"] == "planner_expected_value":
                        action = (0.0, 0.0)
                    else:
                        raise Exception("##log## ego_update_mode error,please check!")
                    ################ planner end #############

                    # paper plot figure
                    if paras_planner["is_visualize_plan"]:
                        # --------- Scene basic elements updated draw ---------
                        visualizer_plan.update_base_info(observation, planner)
                        if paras_planner["plot_figure_for_method"] == "JSSP":
                            # ---------  Render local trajectory planning results  ---------
                            visualizer_plan.update_local_planning_JSSP(observation, planner)
                        elif paras_planner["plot_figure_for_method"] == "frenet":
                            visualizer_plan.update_local_planning_frenet(observation, planner)
                        # visualizer_predict.update_unimodal_prediction()
                        visualizer_plan.save_figure_as_png()
                        visualizer_plan.save_figure_as_svg()

                    # Update the scene based on the vehicle's action and return new observations.
                    observation = envi.step(
                        action=action,
                        observation_last=100,
                        traj=traj,
                        best_traj_ego=planner.fplanner.best_traj,
                        ego_update_mode=parameters["sim_config"]["ego_update_mode"],
                        goal_pose=goal_pose,
                    )
                elif SCENE == tool.SceneType.HIGHWAY_RAMP:
                    pass  # This scenario is outside the scope of this paper
                else:
                    pass  # This scenario is outside the scope of this paper

        except Exception as e:
            print(repr(e))
        finally:
            # If the test is completed, pass the test results back to the ScenarioOrganizer
            so.add_result(scenario_to_test, observation["test_setting"]["end"])
            # Close the visual interface at the end of each test to avoid multiple visualizations at the same time
            plt.close()
