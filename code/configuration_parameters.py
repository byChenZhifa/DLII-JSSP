# simulation environment setting
parameters = {
    "sim_config": {
        "ego_update_mode": "planner_expected_value",  #!NOTE: kinematics , planner_expected_value
        "kinematics_path_tracing_type": "plan_path",  #!NOTE: ref_path , plan_path
    },
    "vehicle_para": {
        "speed_max": 18.0,
        "lon_accel_max": 8.0,  # longitudinal acceleration
        "lon_jerk_max": 10.0,
        "lateral_accel_max": 1.2,  # lateral acceleration
        "lateral_jerk_max": 2.0,
        "centripetal_accel_max": 2.0,  # turn centripetal acceleration threshold
    },
    "para_threshold": {  # Planning algorithm penalty threshold
        "speed_max": 18.0,
        "lon_accel_max": 3.0,
        "lon_jerk_max": 6.0,
        "lateral_accel_max": 0.5,
        "lateral_jerk_max": 1.0,
        "centripetal_accel_max": 1.0,
    },
}


# Parameter configuration: Path tracking controller
paras_control = {
    "controller_type": "pure_pursuit",  # only one
    "pure_pursuit": {
        "kv_ld0": [0.35, 4.8],
        # Wheelbase calculation factor # l = self.ego_vehicle.l / 1.7
        "coff_wheel_base": 1.7,
    },
}


# Parameter configuration: local trajectory planner
paras_planner = {
    "is_visualize_plan": "True",
    "planner_type": "JSSP",  #!NOTE: "frenet";"JSSP"
    "plot_figure_for_method": "JSSP",
    "planner_frenet": {
        "num_width": 3,
        "num_speed": 250,
        "num_t": 10,
        "min_t": 5.0,
        "max_t": 5.5,
        "cost_wights": [1.0, 0.0, 99.4, 0.2, 0.1, 0.8, 0.4, 10.0],
        # w_time ;w_distance;w_laneoffset;w_lateral_accel;w_lateral_jerk;w_longitudinal_a;w_longitudinal_jerk;w_longitudinal_v
        "max_distance": 140.0,
    },
    "planner_JSSP": {
        "num_width": 3,
        "num_jerk": 25,
        "plan_horizon": 5.0,
        "cost_wights": [0.0, 3.0, 5.4, 0.2, 0.1, 0.8, 0.4, 4.0],
        # w_time ;w_distance;w_laneoffset;w_lateral_accel;w_lateral_jerk;w_longitudinal_a;w_longitudinal_jerk;w_longitudinal_v
        # ! w_time is None
        "max_distance": 140.0,
        "traj_num_max": 50,
    },
}


# Parameter configuration: Predictor
paras_prediction = {
    "controller_type": "NONE",
    "is_visualize_predict": "True",
    "plot_figure_for_method": "LANE_INTENTION_INFERENCE",  # CVCYR_PREDICTOR;"LANE_INTENTION_INFERENCE"
    "plot_figure_for_veh_ids": [8],  # Turn right
    # "plot_figure_for_veh_ids":[3], # Change lanes
    # "plot_figure_for_veh_ids":[4], # Turn left
    "predictor_papra": {},
}
