import os
import time
import shutil
import sys
import numpy as np

sys.path.append("..")


class ScenarioOrganizer:
    def __init__(self, use_socket=False):
        self.socket_mode = use_socket
        self.test_mode = ""
        self.scenario_list = []
        self.test_matrix = np.array([])
        self.test_num = 0
        self.scaler = None
        self.adaptive_method = None

    def load(self, input_dir: str, output_dir: str) -> None:
        """Read the configuration file and prepare the scene contents according to the configuration file (py format)"""
        self._check_output_dir(output_dir)
        self.scenario_list = []

        sys.path.append(input_dir)
        from test_conf import config

        self.config = config
        self.test_mode = config["test_settings"]["mode"]

        self.config["file_info"] = {"input": input_dir, "output": output_dir}

        self.config["test_settings"].setdefault("visualize", False)

        if self.test_mode == "replay":
            self.config["test_settings"].setdefault("skip_exist_scene", False)
            for item in os.listdir(input_dir):
                if item.split(".")[-1] != "py":
                    if item != "__pycache__" and item[0] != ".":
                        if self.config["test_settings"]["skip_exist_scene"] and os.path.exists(os.path.join(output_dir, item + "_result.csv")):
                            continue
                        sce_path = input_dir + "/" + item
                        sce = self.config.copy()
                        sce["data"] = {"scene_name": item, "params": sce_path}
                        self.scenario_list += [sce]
            self.test_num = len(self.scenario_list)

    def next(self):
        """Give the next scenario and test mode. If there is no scenario, the name of the scenario to be tested is None"""
        if self.test_mode == "replay":
            if self.scenario_list:
                scenario_to_test = self.scenario_list.pop(0)
            else:
                scenario_to_test = None
        return scenario_to_test

    def add_result(self, concrete_scenario: dict, res: float) -> None:
        if self.test_mode == "replay":
            return

    def _check_output_dir(self, output_dir: str) -> None:
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)


if __name__ == "__main__":
    demo_replay = r"demo/demo_inputs_adaptive"
    demo_ouput_dir = r"demo/demo_outputs"
    so = ScenarioOrganizer()
    so.load(demo_replay, demo_ouput_dir)
    while True:
        scenario_to_test = so.next()
        if scenario_to_test is None:
            break
        if scenario_to_test["test_settings"]["mode"] == "adaptive":
            res = scenario_to_test["data"]["params"][0] ** 2 - scenario_to_test["data"]["params"][1] ** 2
        else:
            res = 1
        print(scenario_to_test, res)
        so.add_result(scenario_to_test, res)
