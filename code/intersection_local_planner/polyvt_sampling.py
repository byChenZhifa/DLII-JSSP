import copy
import math
import os, sys

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.collections import LineCollection
from tqdm import tqdm


def add_path_to_sys(target: str):
    abs_path = os.path.abspath(target)
    if abs_path not in sys.path:
        sys.path.append(abs_path)


dir_current_file = os.path.dirname(__file__)
dir_parent_1 = os.path.dirname(dir_current_file)
dir_parent_2 = os.path.dirname(dir_parent_1)
add_path_to_sys(dir_current_file)
add_path_to_sys(dir_parent_1)
add_path_to_sys(dir_parent_2)

from common.scenario.frenet import JerkSpaceSamplingFrenetTrajectory


class PolyVTSampling(object):
    """Sampling Algorithm based on Linear design.
    Longitudinal Sampling Utilizing Predefined Maneuver Modes
    """

    def __init__(self, total_time, delta_t, jerk_min, jerk_max, a_min, a_max, v_max, num_samples) -> None:

        self.total_time = total_time
        self.delta_t = delta_t
        self.jerk_min = jerk_min
        self.jerk_max = jerk_max
        self.a_min = a_min
        self.a_max = a_max
        self.v_max = v_max
        self.num_samples = num_samples
        self.time_step_max = int(round(self.total_time / self.delta_t))
        self.time_array = np.linspace(0.0, total_time, int(total_time / delta_t) + 1).tolist()
        self.jerk_values = np.linspace(jerk_min, jerk_max, num_samples)
        self.t1_array_delta = 0.5000
        self.t2_array_delta = 0.8000
        self.t3_array_delta = 1.0000
        self.t4_array_delta = 1.5000
        self.t1_array = np.arange(0.0, self.total_time + 1e-9, self.t1_array_delta)
        self.t2_array = np.concatenate((np.array([5.0]), np.arange(0.0, self.total_time + 1e-9, self.t2_array_delta)))
        self.t3_array = np.arange(0.0, self.total_time + 1e-9, self.t3_array_delta)
        self.t4_array = np.arange(0.0, self.total_time + 1e-9, self.t4_array_delta)

        self.jerk_min_3 = jerk_min * 0.6
        self.jerk_max_3 = jerk_max * 0.6
        self.jerk_min_5 = jerk_min * 0.3
        self.jerk_max_5 = jerk_max * 0.3

    @staticmethod
    def calculate_st_by_jt(s0, v0, a0, j, t):
        return s0 + v0 * t + 0.5 * a0 * t * t + (1 / 6.0) * j * t * t * t

    @staticmethod
    def calculate_st_by_jt_clamp(s0, v0, a0, j, t):
        s = s0 + v0 * t + 0.5 * a0 * t * t + (1 / 6.0) * j * t * t * t
        if s < s0:
            return s0
        else:
            return s

    @staticmethod
    def calculate_vt_by_jt(v0, a0, j, t):
        return v0 + a0 * t + 0.5 * j * t * t

    @staticmethod
    def calculate_vt_by_jt_clamp(v0, a0, j, t):
        v = v0 + a0 * t + 0.5 * j * t * t
        if v < 0.0:
            return 0.0
        else:
            return v

    @staticmethod
    def calculate_at_by_jt(a0, j, t):
        return a0 + j * t

    def check_state(self, st, vt, at, s0, v0, a0):
        """Check whether the terminal state exceeds dynamic constraints.

        Args:
            s0, v0, a0: indicates the initial status value.
            st, vt, at: state value after t time;

        Returns:
            bool: _description_
        """
        # check acceleration
        ACC_MAX = self.a_max - 1e-9
        if at > ACC_MAX or at < -ACC_MAX:
            return False

        # check velocity
        VEL_MAX = self.v_max - 1e-9
        # if vt > VEL_MAX or vt < 0.0 + 1e-9:
        if vt > VEL_MAX or vt < 0.0 - 1e-9:
            return False

        if st < s0 - 1e-9:
            # if st < s0 + 1e-9:
            return False

        return True

    def check_state_stop(self, s2, v2, a2, s1):
        if a2 > 0.0 or a2 < -1.0:
            return False

        # clamp range[-0.5,0.0]
        if v2 > 0.0 or v2 < -0.5:
            return False

        if s2 < s1 - 0.1:
            return False

        return True

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
                ft.c = np.divide(np.diff(ft.yaw), ft.ds)
                ft.c_d = np.divide(np.diff(ft.c), dt)
                ft.c_dd = np.divide(np.diff(ft.c_d), dt)

                # Append the trajectory with calculated global parameters to the list
                passed_ftlist.append(ft)
        return ftlist

    def sampling(self, s0, v0, a0) -> list[tuple]:
        """j-t space sampling strategy based on a-t space linear design.

        Short planning time domain (such as 5 seconds): In short planning time domain, good vertical planning does not carry out
        frequent acceleration and deceleration mode switching;

        Design 5 AT-line splicing, corresponding to different driving modes:
        1) Curve a rises; 2)a curve is constant;
        3) Curve a declines \ rise in the opposite direction; 4) Curve a is constant again;
        5) Curve a declines in the opposite direction;
        Corresponding jerk value and maintenance time of each a-t line :j1, t1, j2, t2, j3, t3, j4, t4, j5, t5

        Args:
            s0, v0, a0: Initial value.

        Returns:
            list[tuple]: The sampling  seeds of  different lines -- (j1, t1, j2, t2, j3, t3, j4, t4, j5, t5).
        """
        seeds = []
        TINTER_MIN = 0.55
        for j1 in tqdm(np.linspace(self.jerk_min, self.jerk_max, self.num_samples)):
            # for t1 in np.linspace(0, self.total_time, self.time_step_max + 1):
            for t1 in self.t1_array:
                if t1 < TINTER_MIN:
                    continue

                s1 = self.calculate_st_by_jt(s0, v0, a0, j1, t1)
                v1 = self.calculate_vt_by_jt(v0, a0, j1, t1)
                a1 = self.calculate_at_by_jt(a0, j1, t1)

                if not self.check_state(s1, v1, a1, s0, v0, a0):
                    continue

                j2 = 0.0
                for t2 in self.t2_array:
                    if t1 + t2 >= self.total_time - self.t2_array_delta * 3:
                        continue

                    s2 = self.calculate_st_by_jt(s1, v1, a1, j2, t2)
                    v2 = self.calculate_vt_by_jt(v1, a1, j2, t2)
                    a2 = self.calculate_at_by_jt(a1, j2, t2)

                    if not self.check_state(s2, v2, a2, s1, v1, a1):
                        continue

                    if t1 + t2 > self.total_time + 1e-9:
                        continue

                    for j3 in np.linspace(self.jerk_min_3, self.jerk_max_3, int(self.num_samples / 4)):
                        if j3 * j1 > 0:
                            continue  # Linear design, does not allow only the same direction of acceleration change

                        for t3 in self.t3_array:
                            if t1 + t2 + t3 > self.total_time - self.t3_array_delta * 2:
                                continue

                            s3 = self.calculate_st_by_jt(s2, v2, a2, j3, t3)
                            v3 = self.calculate_vt_by_jt(v2, a2, j3, t3)
                            a3 = self.calculate_at_by_jt(a2, j3, t3)

                            if not self.check_state(s3, v3, a3, s2, v2, a2):
                                continue

                            if a3 * a1 > 0:
                                continue  # Make sure the third paragraph pattern has changed to the opposite vertical motion model

                            if a3 * a2 < 0:  #! Check the poles of [a3,0.0,a2]
                                t2_5 = (0.0 - a2) / j3
                                v2_5 = self.calculate_vt_by_jt(v2, a2, j3, t2_5)
                                if v2_5 > self.v_max or v2_5 < 0.0 + 1e-9:
                                    continue

                            j4 = 0.0
                            for t4 in self.t4_array:
                                if t1 + t2 + t3 + t4 > self.total_time - self.t4_array_delta:
                                    continue

                                s4 = self.calculate_st_by_jt(s3, v3, a3, j4, t4)
                                v4 = self.calculate_vt_by_jt(v3, a3, j4, t4)
                                a4 = self.calculate_at_by_jt(a3, j4, t4)

                                if not self.check_state(s4, v4, a4, s3, v3, a3):
                                    continue

                                t5 = self.total_time - t1 - t2 - t3 - t4
                                j5 = (0.0 - a4) / t5
                                if j5 > self.jerk_max_5 or j5 < self.jerk_min_5:
                                    continue

                                s5 = self.calculate_st_by_jt(s4, v4, a4, j5, t5)
                                v5 = self.calculate_vt_by_jt(v4, a4, j5, t5)
                                a5 = self.calculate_at_by_jt(a4, j5, t5)
                                if not self.check_state(s5, v5, a5, s4, v4, a4):
                                    continue

                                seeds.append((j1, t1, j2, t2, j3, t3, j4, t4, j5, t5))
        return seeds

    def sampling_stop_seeds_supplement(self, s0, v0, a0):
        """Supplements the existing seeds with emergency stop trajectories.
        Add the scram sampling track and slow down until it stops.
        Maneuver design :a decreases, then increases to 0; speed is small to 0
        """
        seeds_emer_stop = []
        j3, j4, j5 = 0.0, 0.0, 0.0
        t4, t5 = 0.0, 0.0
        for j1 in tqdm(np.linspace(self.jerk_min, self.jerk_max, self.num_samples * 2)):
            if j1 > 0:
                continue

            for t1 in np.arange(0.0, self.total_time + 1e-9, 0.1):
                s1 = self.calculate_st_by_jt(s0, v0, a0, j1, t1)
                v1 = self.calculate_vt_by_jt(v0, a0, j1, t1)
                a1 = self.calculate_at_by_jt(a0, j1, t1)

                if not self.check_state(s1, v1, a1, s0, v0, a0):
                    continue

                for j2 in np.linspace(self.jerk_min, self.jerk_max, self.num_samples * 2):
                    if j2 < 0:
                        continue

                    for t2 in np.arange(0.0, self.total_time + 1e-9, 0.1):
                        if t1 + t2 >= self.total_time:
                            continue
                        a2 = self.calculate_at_by_jt(a1, j2, t2)
                        if a2 > 0.0 or a2 < -0.3:  # clamp  [-0.5,0.0]
                            continue
                        v2 = self.calculate_vt_by_jt(v1, a1, j2, t2)
                        if v2 > 0.0 or v2 < -0.3:  # clamp  [-0.5,0.0]
                            continue
                        s2 = self.calculate_st_by_jt(s1, v1, a1, j2, t2)
                        if s2 < s1 - 0.1:
                            continue

                        t3 = self.total_time - t1 - t2

                        seeds_emer_stop.append((j1, t1, j2, t2, j3, t3, j4, t4, j5, t5))
        return seeds_emer_stop

    def get_all_frenet_trajectory(self, s0, a0, v0, seeds, ft: JerkSpaceSamplingFrenetTrajectory):
        """Generate velocity trajectories in batches"""
        return [self.get_one_frenet_trajectory(s0=s0, v0=v0, a0=a0, seed=seed, jssft=ft) for seed in seeds]

    def get_one_frenet_trajectory(self, s0, v0, a0, seed, jssft: JerkSpaceSamplingFrenetTrajectory) -> JerkSpaceSamplingFrenetTrajectory:
        """A velocity trajectory is generated based on the jerk-t space sampling results"""
        ft = copy.deepcopy(jssft)
        j1, t1, j2, t2, j3, t3, j4, t4, j5, t5 = seed

        s1 = self.calculate_st_by_jt(s0, v0, a0, j1, t1)
        v1 = self.calculate_vt_by_jt(v0, a0, j1, t1)
        a1 = self.calculate_at_by_jt(a0, j1, t1)

        s2 = self.calculate_st_by_jt(s1, v1, a1, j2, t2)
        v2 = self.calculate_vt_by_jt(v1, a1, j2, t2)
        a2 = self.calculate_at_by_jt(a1, j2, t2)

        s3 = self.calculate_st_by_jt(s2, v2, a2, j3, t3)
        v3 = self.calculate_vt_by_jt(v2, a2, j3, t3)
        a3 = self.calculate_at_by_jt(a2, j3, t3)

        s4 = self.calculate_st_by_jt(s3, v3, a3, j4, t4)
        v4 = self.calculate_vt_by_jt(v3, a3, j4, t4)
        a4 = self.calculate_at_by_jt(a3, j4, t4)

        for t in self.time_array:
            dt = 0
            if t < t1 - 1e-9:
                ft.t.append(t)

                dt = t
                ft.s_ddd.append(j1)
                ft.s_dd.append(self.calculate_at_by_jt(a0, j1, dt))
                ft.s_d.append(self.calculate_vt_by_jt(v0, a0, j1, dt))
                ft.s.append(self.calculate_st_by_jt(s0, v0, a0, j1, dt))

            elif t < t1 + t2 + 1e-9:
                ft.t.append(t)

                dt = t - t1
                ft.s_ddd.append(j2)
                ft.s_dd.append(self.calculate_at_by_jt(a1, j2, dt))
                ft.s_d.append(self.calculate_vt_by_jt(v1, a1, j2, dt))
                ft.s.append(self.calculate_st_by_jt(s1, v1, a1, j2, dt))

            elif t < t1 + t2 + t3 + 1e-9:
                ft.t.append(t)

                dt = t - t1 - t2
                ft.s_ddd.append(j3)
                ft.s_dd.append(self.calculate_at_by_jt(a2, j3, dt))
                ft.s_d.append(self.calculate_vt_by_jt(v2, a2, j3, dt))
                # if self.calculate_vt_by_jt(v2, a2, j3, dt) < 0.0:
                #     aaa = 1
                ft.s.append(self.calculate_st_by_jt(s2, v2, a2, j3, dt))

            elif t < t1 + t2 + t3 + t4 + 1e-9:
                ft.t.append(t)

                dt = t - t1 - t2 - t3
                ft.s_ddd.append(j4)
                ft.s_dd.append(self.calculate_at_by_jt(a3, j4, dt))
                ft.s_d.append(self.calculate_vt_by_jt(v3, a3, j4, dt))
                # if self.calculate_vt_by_jt(v3, a3, j4, dt) < 0.0:
                #     aa = 1
                ft.s.append(self.calculate_st_by_jt(s3, v3, a3, j4, dt))

            else:
                ft.t.append(t)

                dt = t - t1 - t2 - t3 - t4
                ft.s_ddd.append(j5)
                ft.s_dd.append(self.calculate_at_by_jt(a4, j5, dt))
                ft.s_d.append(self.calculate_vt_by_jt(v4, a4, j5, dt))
                # if self.calculate_vt_by_jt(v4, a4, j5, dt) < 0.0:
                #     aa = 1
                ft.s.append(self.calculate_st_by_jt(s4, v4, a4, j5, dt))

        return ft


if __name__ == "__main__":
    pass
