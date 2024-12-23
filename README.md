---

# DLII-JSSP

> This repository serves as the implementation for the paper titled:
>
> **"Efficient Sampling-based Trajectory Planning with Dual-Layer Probabilistic Intention Prediction for Autonomous Driving in Complex Intersections"**
>
> DLII-JSSP stands for Dual-Layer Intention Inference (DLII) and Jerk Space Sampling Planner (JSSP). However, in the future, we plan to revise it to the Sampling Planner based on Predefined Maneuver Modes (SPPMM).
>
> #### To-Do List
>
> - [x] Results [Completed]
> - [x] Datasets [Completed]
> - [x] Code [ Completed]

## 0. Introduction

Navigating complex urban intersections remains a significant challenge for autonomous vehicles due to highly dynamic and dense traffic environments. This paper presents a novel trajectory planning framework designed to address these challenges by integrating a dual-layer probabilistic rad-lane intention prediction model with an efficient sampling-based trajectory planner. The proposed dual-layer model comprises three components: short-term vehicle kinematic prediction-based road-level intention inference, Interactive Multiple Model (IMM)-based lane-level intention inference, and target-lane-based trajectory generation. This architecture broadens the application scope and validates the effectiveness of the IMM-based multi-model probabilistic fusion framework in urban intersection scenarios. By incorporating both road-level and lane-level contextual information, the proposed trajectory prediction method significantly enhances prediction accuracy. Additionally, a longitudinal sampling strategy based on predefined maneuver modes is employed to improve the probabilistic completeness of existing parametric curve-based sampling techniques, facilitating rapid and effective obstacle avoidance. A notable advantage of the proposed sampling strategy is its ability to efficiently and probabilistically generate safe and feasible trajectories in challenging intersection scenarios. Extensive simulation results demonstrate that the proposed framework outperforms existing methods in terms of both efficiency and safety in urban intersection vehicle conflict scenarios.

The contributions of this study are summarized as follows:

(1) **IMM-based Dual-Layer Intention Inference:** A novel approach is proposed for hierarchical, fine-grained prediction of obstacle vehicles' road-lane selection intentions, generating multimodal prediction trajectories. This approach broadens the application scope and validates the effectiveness of the IMM-based multi-model probabilistic fusion framework in urban intersection scenarios.

(2) **Efficient, Probabilistically Complete Longitudinal Sampling based on Predefined Maneuver Modes in Acceleration-Time Space**: A new method is introduced that ensures probabilistic completeness of the solution space, in contrast to parametric curve-based sampling methods. This approach improves the performance of sampling-based longitudinal obstacle avoidance and enables the efficient generation of effective avoidance trajectories for autonomous vehicles in intersection vehicle conflict scenarios.

## 1. Preparation

### 1.1 Download the project code

- Clone this repo from the GitHub.

```bash
 git clone git@github.com:byChenZhifa/DLII-JSSP.git
```

- Download `the 8 scenarios` or `all scenarios data` .

  - 8 scenes demo: `scenes_demo(8 for planning ablation study).zip` ,[download here](https://github.com/byChenZhifa/DLII-JSSP/tree/main/data_scenarios)] and save it to project directory `./data_scenarios` .
  - all the intersection scenes: `scenes_all(800).tar.xz` , [download website](https://github.com/byChenZhifa/archive/tree/main/scenarios_onsite_v1_intersection).

### 1.2 Set up your virtual environment

- Create a virtual environment using conda (python3.9)

  ```bash
  conda create -n dlii python=3.9
  ```

- Activate the virtual environment

  ```bash
  conda activate dlii
  ```

- Install python dependency packages via pip. Open the project directory as follows `../setup/requirements.txt`, install dependency package.

  ```bash
  pip install -r setup/requirements.txt
  # Speed up (use pip source in China)
  pip install -r setup/requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple

  ```

- 1.3. run.

## 2. simulation tool introduction: OnSite

### 2.1 OnSite introduction

The simulation testing tool provided by OnSite's first edition is a scenario-based closed-loop simulation environment. This simulation environment primarily includes the following features:

- **Replay Testing Mode**: The simulation environment utilizes a replay testing mode with closed-loop simulation and non-reactive agents.

- **Automated Scenario Testing**: It automatically and cyclically reads and tests all scenarios in the library.

- **Standardized Data Formats**: The scenarios are formatted in the OpenSCENARIO (e.g., 8_3_4_565_exam.xosc) data exchange format, and the maps are formatted in the OpenDRIVE data exchange format (e.g., 8_3_4_565.xodr) [46].

- **Integrated Module Testing**: This simulation environment supports the integrated testing of multiple autonomous driving modules, including prediction, planning, and motion control.

- **Vehicle Model**: The ego vehicle’s model in the closed-loop simulation uses the kinematic bicycle model. The state-space function of the kinematic bicycle model is as follows:
  ${x_{k + 1}} = {v_{lon,k}} \cdot \cos ({\theta _k}) \cdot T$ (1a)

  ${y_{k + 1}} = {v_{lon,k}} \cdot \sin ({\theta _k}) \cdot T$ (1b)

  $ {\theta _{k + 1}} = {\theta \_k} + \frac{{{v_{lon,k}} \cdot \tan ({\delta _k})}}{{{L_{wb}}}} \cdot T$ (1c)

  $ {v*{lon,k + 1}} = {v*{lon,k}} + {a\_{lon,k}} \cdot T $ (1d)

  where T represents the discrete time step set in the simulation. The ego vehicle's states $ {{\bf{x}}_k} = [{x_k},{y_k},{\theta \_k},{v_{lon,k}}]$ represent the x position, y position, yaw angle, and longitudinal velocity respectively. The control inputs ${{\bf{u}}_k} = [{a_{lon,k}},{\delta _k}]$ denote the longitudinal acceleration and front wheel steering angle as output by the planning or control algorithm. $L_{wb}$ represents the vehicle's wheelbase.

### 2.2 Suggested structure for the project code

> TODO

## 3. Our test results

Note: This framework initiates with a **high-layer inference for the target road** and subsequently **refines this prediction for the target lane**.

<img src="./test_results/Fig.%202.%20Schematic%20diagram%20of%20dual-layer%20intention%20inference%20for%20road%20layer%20and%20lane%20layer.png" style="zoom:17%;" />

Fig. 2. Schematic diagram of dual-layer intention inference for road layer and lane layer

<img src="./test_results/fig4-Cartesian frame to Frenet frame.png" style="zoom:23%;" />

**Fig. 4.** Schematics on the conversion from Cartesian frame to Frenet frame.

<img src="./test_results/fig-6.png" style="zoom:30%;" />

**Fig. 6.** Multiple Lane Centerline Models based on Lane Frenet States inside One Road

### 3.1 More results for trajectory predictor

- Method 1: A predictor based on Constant Velocity and Constant Yaw Rate (CVCYR).

- Method 2: Our predictor utilizing Interactive Multiple Model (IMM) Kalman filters.

- Method 3: A predictor called MTP with angle, as described in [13]. It is a deep convolutional network-based Multimodal Trajectory Prediction (MTP) method.
  [13] H. Cui et al., “Multimodal Trajectory Predictions for Autonomous Driving Using Deep Convolutional Networks,” in Proceedings of the 2019 International Conference on Robotics and Automation (ICRA), IEEE, 2019, pp. 2090–2096.

#### Test scenario :7_28_1_89, vehicle #4 turn left test

- method 1 : CVCYR

![image-](./test_results/CVCYR_PREDICTOR-7_28_1_89.gif)

- method 2 : Our predictor with IMM

![image-](./test_results/LANE_INTENTION_INFERENCE-7_28_1_89.gif)

- method 3 : MTP with angle

![image-](./test_results/all-3-gifs/CNN_PREDICTOR-7_28_1_89.gif)

#### Test scenario :8_2_1_563, vehicle #8 going straight and changing lanes.

- method 1 : CVCYR

![image-](./test_results/CVCYR_PREDICTOR-8_2_1_563.gif)

-Our predictor with IMM

![image-](./test_results/LANE_INTENTION_INFERENCE-8_2_1_563.gif)

- method 3 : MTP with angle

![image-](./test_results/all-3-gifs/CNN_PREDICTOR-8_2_1_563.gif)

#### Test scenario :8_3_1_54, vehicle # 3 turn right.

- method 1 : CVCYR

![image-](./test_results/CVCYR_PREDICTOR-8_3_1_54.gif)

- method 2 : Our predictor with IMM

![image-](./test_results/LANE_INTENTION_INFERENCE-8_3_1_54.gif)

- method 3 : MTP with angle

![image-](./test_results/all-3-gifs/CNN_PREDICTOR-8_3_1_54.gif)

### 3.2 More results for trajectory planner

Note: Some results of trajectory planning using predefined linear strategies .

- Test scenario : JSSP-8_3_4_565

![image-20240426202932398](./test_results/JSSP-8_3_4_565.gif)

- Test scenario : JSSP-8_8_1_135

![image-20240426202932398](./test_results/JSSP-8_8_1_135.gif)

- Test scenario : JSSP-8_32_left_straight_in_opposite_36

![image-20240426202932398](./test_results/JSSP-8_32_left_straight_in_opposite_36.gif)

- Test scenario : JSSP-8_9_2_332

![image-20240426202932398](./test_results/JSSP-8_9_2_332.gif)

- Test scenario : JSSP-10_60_straight_in_adjacent_left_61

![image-20240426202932398](./test_results/JSSP-10_60_straight_in_adjacent_left_61.gif)

## Contact us

If you have any issues with the code, please contact to this email: [chenzhifa@buaa.edu.cn](chenzhifa@buaa.edu.cn)

## Citation

If you find our work useful for your research, please consider citing the paper.

> TODO

## reference

- https://github.com/SS47816/fiss_plus_planner
- https://github.com/daeheepark/PathPredictNusc
- https://github.com/nutonomy/nuscenes-devkit
- [13] H. Cui et al., “Multimodal trajectory predictions for autonomous driving using deep convolutional networks,” in 2019 International Conference on Robotics and Automation (ICRA), IEEE, 2019, pp. 2090–2096.
- [36] S. Sun, Z. Liu, H. Yin, and M. H. Ang, “FISS: a trajectory planning framework using fast iterative search and sampling strategy for autonomous driving,” IEEE Robot. Autom. Lett., vol. 7, no. 4, pp. 9985–9992, Oct. 2022, doi: 10.1109/LRA.2022.3191940.
- [37] S. Sun et al., “FISS+: efficient and focused trajectory generation and refinement using fast iterative search and sampling strategy,” in 2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Oct. 2023, pp. 10527–10534. doi: 10.1109/IROS55552.2023.10341498.

```

```
