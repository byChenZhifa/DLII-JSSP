from onsite_trans.observation import Observation
import pandas as pd
from functools import reduce
import copy
import sys

sys.path.append("..")
import warnings


class DataRecord:
    def __init__(self):
        self.data = {}
        self.vehicle_column = ["x", "y", "v", "a", "yaw", "width", "length"]

    def add_data(self, observation: Observation):
        stored_vehicles = self.data.keys()

        t = observation.test_setting["t"]

        for vehicle_name, values in observation.vehicle_info.items():
            if vehicle_name not in stored_vehicles:
                self._add_vehicle_frame(vehicle_name)

            sub_frame = pd.DataFrame(values, columns=self.vehicle_column, index=[t])
            sub_frame.columns = list(self.data[vehicle_name].columns)

            with warnings.catch_warnings():
                warnings.simplefilter("ignore", category=FutureWarning)
                self.data[vehicle_name] = pd.concat([self.data[vehicle_name], sub_frame])

    def merge_frame(self) -> pd.DataFrame:
        vehicle_dataframe_group = [self.data[vehi_name] for vehi_name in list(self.data.keys())]
        return reduce(lambda x, y: pd.merge(x, y, how="outer", left_index=True, right_index=True), vehicle_dataframe_group)

    def _add_vehicle_frame(self, vehicle_name: str):
        self.data[vehicle_name] = pd.DataFrame(None, columns=[i + "_" + str(vehicle_name) for i in self.vehicle_column])


class Recorder:
    def __init__(self):
        self.output_dir = ""
        self.file_name = ""
        self.data_record = DataRecord()

    def init(self, observation: Observation, output_dir: str, read_only: bool = False) -> None:
        if output_dir[-1] != "/":
            output_dir += "/"
        self.data_record = DataRecord()
        self.output_dir = output_dir
        self.read_only = read_only
        self.record(observation)

    def record(self, observation: Observation) -> None:
        self.data_record.add_data(observation)
        data_output = copy.deepcopy(self.data_record.merge_frame())
        data_output.loc[:, "end"] = -1
        t = observation.test_setting["t"]
        if observation.test_setting["end"] == 1.5:
            data_output.drop(t, axis=0, inplace=True)
            data_output.iloc[-1, -1] = 1
        else:
            data_output.iloc[-1, -1] = observation.test_setting["end"]
        if not self.read_only:
            data_output.to_csv(self.output_dir + observation.test_setting["scenario_name"] + "_result.csv")
        if observation.test_setting["end"] != -1:
            print(observation.test_setting)


if __name__ == "__main__":
    data = DataRecord()
    observe = Observation()
    data.add_data(observe)
    print(data.data)
