import copy
import sys
from onsite_trans.observation import Observation
from onsite_trans.controller import Controller
from onsite_trans.recorder import Recorder

sys.path.append("..")


class Env:
    def __init__(self):
        self.controller = Controller()
        self.recorder = Recorder()

    def make(self, scenario: dict, read_only=False) -> Observation:
        observation, traj = self.controller.init(scenario)
        self.recorder.init(observation, scenario["file_info"]["output"], read_only)

        return observation.format(), traj

    def step(self, action, observation_last, traj, best_traj_ego, ego_update_mode, goal_pose) -> Observation:

        observation = self.controller.step(action, goal_pose)  # Use vehicle kinematics model to update the scene in a single step;
        if ego_update_mode == "planner_expected_value":
            #! Update self vehicle status according to ego planning expected value.
            observation.vehicle_info["ego"]["x"] = copy.deepcopy(best_traj_ego.x[1])
            observation.vehicle_info["ego"]["y"] = copy.deepcopy(best_traj_ego.y[1])
            observation.vehicle_info["ego"]["yaw"] = copy.deepcopy(best_traj_ego.yaw[1])
            observation.vehicle_info["ego"]["v"] = copy.deepcopy(best_traj_ego.s_d[1])
            observation.vehicle_info["ego"]["a"] = copy.deepcopy(best_traj_ego.s_dd[1])
        self.recorder.record(observation)

        return observation.format()


if __name__ == "__main__":
    import time

    demo_input_dir = r"demo/demo_inputs"
    demo_ouput_dir = r"demo/demo_outputs"
    tic = time.time()
    env = Env()

    from onsite_trans.scenarioOrganizer import ScenarioOrganizer

    so = ScenarioOrganizer()
    so.load(demo_input_dir, demo_ouput_dir)
    num_scenario = len(so.scenario_list)
    for i in range(num_scenario):
        scenario_to_test = so.next()
        print(scenario_to_test)
        observation = env.make(scenario_to_test, demo_ouput_dir, visilize=True)
        while observation.test_setting["end"] == -1:
            observation = env.step([-1, 0])
    toc = time.time()
    print(toc - tic)
