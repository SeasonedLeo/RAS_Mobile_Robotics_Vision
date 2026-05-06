---
layout: default
math: mathjax
title: "Report 3"
parent: Project
nav_order: 3
---

# Report 3 — Final Documentation & Analysis

{: .no_toc }

This report is structured to match the Milestone 3 requirements for the final scientific dossier. It focuses on validation, benchmarking, ethical analysis, and auditable attribution of each custom technical contribution.

---

## Table of Contents

{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 1. Graphical Abstract

<!-- Required by Milestone 3: include a single visual summary for the course gallery. -->
<!-- TODO: Replace this placeholder with the final graphical abstract image and caption. -->

<div align="center">
  <img src="{{ '/assets/images/report3_graphical_abstract_placeholder.png' | relative_url }}"
       alt="Graphical abstract placeholder for the final project"
       width="90%">
</div>

**Figure 1.1.** Graphical abstract summarizing the project mission, core perception-and-planning pipeline, and final mission outcome.

### 1.1 Project Mission

This project addresses the problem of localizing a target object using a mobile robot in an indoor environment without relying on a pre-built map. The robot observes the scene with onboard vision sensors, estimates the object pose in its local odometry frame, and evaluates whether that estimate is reliable enough for use. When confidence is low, the robot autonomously moves to a better nearby viewpoint and repeats the estimation process. The mission objective is to achieve more reliable object localization through closed-loop active perception rather than a single static observation


---

## 2. Algorithm

This section presents the algorithmic design rationale and execution logic of the custom modules and ROS 2 nodes used in the system. Although Report 2 outlined the main components and their core functions, this section examines them in greater detail, with emphasis on internal data flow, decision-making logic, and the way the modules interact to support active perception.



### 2.1 System Components and Their Interactions

### 2.1.1 Perception Module 

The perception module structure and node-level connectivity were introduced in Report 2. In this report, the emphasis is on the final implementation logic and on the role of perception within the active perception loop. The module consists of the object-specific detection nodes (`cylinder_finder.py` and `box_finder.py`) together with the downstream `pose_estimator.py` node, which converts segmented target observations into pose estimates in the planning frame (`odom`).

Both detection nodes follow a common preprocessing pipeline based on spatial filtering, voxel downsampling, and floor removal. After that, each node applies object-specific reasoning. The final box finder differs from the earlier design outlined in Report 2. Rather than relying on a PCA-only interpretation of the segmented object, the implemented node first filters candidate points using a brown color model appropriate for cardboard-like box surfaces, then forms Euclidean clusters, and finally fits an upright oriented box by analyzing the horizontal footprint and vertical extent of each candidate cluster. Geometric constraints on dimensions and support size are used to reject implausible hypotheses, and the best valid cluster is selected as the box target. This change was introduced to make the final box detection stage more object-specific, since the earlier PCA-only approach did not provide sufficiently reliable detection performance

The segmented target cloud is then passed to the pose estimator, which computes the target centroid and dominant horizontal orientation, transforms the estimate into the `odom` frame, and publishes both the final pose and a pose-estimate sample for downstream confidence evaluation. In this way, the perception module provides the geometric target information required by the rest of the active perception system.


### 2.1.2 Planning Module

The planning module structure and its node-level role within the active perception loop were already described in Report 2. In the final system, the implementation logic of this module remained largely unchanged, with the confidence evaluator using a simplified four-term scoring scheme. The module consists of `confidence_evaluator.py` and `nbv_planner.py`, which together determine whether the current target estimate is sufficiently reliable or whether an additional viewpoint should be planned.

The `confidence_evaluator.py` node receives a recent history of pose-estimate samples and computes a confidence score from four quantities: positional variance, yaw variance, mean point count, and mean anisotropy ratio. These terms are combined into a single score that determines whether the robot should stop or continue the active perception process.

If the confidence threshold is not reached, the `nbv_planner.py` node generates candidate viewpoints around the current target estimate in the planning frame and scores them according to radius error, travel distance, and heading change. The best candidate is returned as the next-best-view goal, while marker outputs are published for visualization in RViz.

Together, these two nodes provide the decision-making layer that closes the active perception loop by determining whether the current estimate is sufficient or whether a new observation pose is required.




### 2.1.2 (Visual Odometry or Estimation & Localization Module) Individual Localization Section (Vikas Narang): ORB-SLAM3 + EKF

This section documents the localization module contribution based on ORB-SLAM3 stereo visual odometry integrated with EKF fusion.

#### Camera Used in ORB-SLAM Pipeline

The ORB-SLAM3 node uses the TurtleBot4 OAK-D stereo camera pair as its primary localization sensor:

- Left camera image topic: `/oakd/left/image_raw`
- Right camera image topic: `/oakd/right/image_raw`
- Left camera calibration topic: `/oakd/left/camera_info`
- Right camera calibration topic: `/oakd/right/camera_info`

Stereo intrinsics and baseline are consumed through calibration/configuration parameters (`fx`, `fy`, `cx`, `cy`, image size, and baseline term such as `Camera.bf`) so that feature correspondences can be triangulated at metric scale.

#### How ORB-SLAM3 Works in This Project

At each synchronized stereo frame pair, ORB-SLAM3 performs the following processing stages:

1. Extract ORB keypoints and descriptors from left/right images.
2. Match descriptors to build stereo correspondences and temporal feature tracks.
3. Estimate camera motion from geometric constraints and tracked features.
4. Triangulate sparse 3D map points from stereo disparity.
5. Refine pose/map consistency through internal local optimization.
6. Publish local visual odometry as ROS 2 pose/odometry outputs for downstream fusion.

The localization output is relative to startup and can drift over long operation. To stabilize deployment for navigation, the VO estimate is fused with wheel odometry in EKF (`robot_localization`), providing a smoother and more robust `odom` estimate for planning/control.

#### ORB-SLAM3 Features and Parameters Used

The ORB-SLAM3 configuration relies on the standard ORB feature extractor and stereo parameters:

- `ORBextractor.nFeatures`: target number of features extracted per frame.
- `ORBextractor.scaleFactor`: image pyramid scaling between levels.
- `ORBextractor.nLevels`: number of pyramid levels for multi-scale matching.
- `ORBextractor.iniThFAST`: initial FAST threshold for keypoint detection.
- `ORBextractor.minThFAST`: fallback FAST threshold in low-texture regions.
- `Camera.fps`: expected image frame rate for timing consistency.
- `ThDepth`/stereo depth threshold terms: constrain valid stereo depth range.

Practically, these parameters control the accuracy/robustness trade-off: higher feature counts and tuned FAST thresholds improve tracking in texture-limited indoor scenes but increase compute load.




### 2.1.3 Navigation, Control, Actuation Module

The navigation and actuation role of the system was initially intended to be handled through Nav2, as discussed in Report 2. In the final implementation, however, Nav2 was replaced by a lightweight local controller implemented in `odom_controller.py`. This change was made because the final system operates without a global map and expresses all perception and planning outputs directly in the `odom` frame. Since Nav2 typically assumes a map-based navigation setup, it was not well aligned with the final local active perception pipeline.

An alternative attempt was made to use SLAM Toolbox in order to provide the mapping layer required for Nav2, but this integration was not brought to a stable operational state during the project. For this reason, the final system adopted a simpler local-control formulation that directly drives the TurtleBot4 to a goal pose in the `odom` frame.

The `odom_controller.py` node subscribes to an `odom`-frame navigation goal and the robot odometry feedback, then publishes velocity commands to drive the robot toward the requested viewpoint. The controller uses proportional control for linear motion and PD control for heading correction, with bounded linear and angular velocities, a rotate-in-place mode for large heading errors, and separate tolerances for final position and yaw convergence. In this way, the module provides the final execution link between the viewpoint selected by the planner and the physical robot motion required to acquire the next observation.



### 2.1.4 Orchestrator

The orchestrator structure was already introduced in Report 2, and its core role remained unchanged in the final system. Its main responsibility is to connect the perception, planning, and motion-execution modules into a closed-loop active perception process. In the final implementation, this role was extended to include the local navigation executor: once a next-best-view pose is selected, the orchestrator publishes the corresponding `odom`-frame goal and waits for the navigation status before resuming perception.

Operationally, the orchestrator stores a bounded history of pose-estimate samples, waits until a minimum history length is available, calls the confidence-evaluation service, and either terminates the active perception loop or requests a next-best-view from the planner. If a new viewpoint is required, the selected goal is sent to the local controller, and the orchestrator returns to the observation state after navigation completes successfully. This makes the orchestrator the supervisory logic that manages state transitions between observation, evaluation, replanning, and motion execution.

Let the robot pose in the planning frame be

$$
\mathbf{x}_k =
\begin{bmatrix}
x_k \\
y_k \\
\theta_k
\end{bmatrix},
$$

and let the target pose estimate at step \(k\) be

$$
\hat{\mathbf{o}}_k =
\begin{bmatrix}
\hat{x}_k \\
\hat{y}_k \\
\hat{\psi}_k
\end{bmatrix}.
$$

Let the recent history of pose-estimate samples be

$$
\mathcal{H}_k = \{s_1, \dots, s_N\},
$$

where each sample contains pose, point count, and anisotropy ratio. From this history, the confidence evaluator computes

$$
\sigma_p^2 = \frac{1}{N}\sum_{i=1}^{N}
\left\|
\begin{bmatrix}
x_i \\
y_i
\end{bmatrix}
-
\begin{bmatrix}
\bar{x} \\
\bar{y}
\end{bmatrix}
\right\|^2,
$$

$$
\sigma_\psi^2 = \frac{1}{N}\sum_{i=1}^{N}
\operatorname{wrap}(\psi_i-\bar{\psi})^2,
$$

$$
\bar{n} = \frac{1}{N}\sum_{i=1}^{N} n_i,
\qquad
\bar{a} = \frac{1}{N}\sum_{i=1}^{N} a_i.
$$

Here, \(n_i\) denotes the point count of the \(i\)-th pose-estimate sample, and \(a_i\) denotes its anisotropy ratio. Therefore, \(\bar{n}\) is the mean point count over the history window, and \(\bar{a}\) is the mean anisotropy ratio over the same window.

These quantities are converted into bounded component scores:

$$
S_p = \frac{1}{1+\sigma_p^2/\eta_p}, \qquad \eta_p = 0.005,
$$

$$
S_\psi = \frac{1}{1+\sigma_\psi^2/\eta_\psi}, \qquad \eta_\psi = 0.08,
$$

$$
S_n = \operatorname{clip}\left(\frac{\bar{n}}{\eta_n}, 0, 1\right),
\qquad \eta_n = 200,
$$

$$
S_a = \operatorname{clip}\left(\frac{\bar{a}}{\eta_a}, 0, 1\right),
\qquad \eta_a = 0.5.
$$

The final confidence score is

$$
C_k = w_p S_p + w_\psi S_\psi + w_n S_n + w_a S_a,
$$

where the current implementation uses

$$
w_p = 0.3,\qquad
w_\psi = 0.2,\qquad
w_n = 0.1,\qquad
w_a = 0.15.
$$

The orchestrator stops active perception if

$$
N \geq N_{\min}
\quad \text{and} \quad
C_k \geq \tau,
$$

otherwise it requests a next-best-view.

$$
N_{\min} = 10
$$

in the current implementation.

For next-best-view planning, let the candidate viewpoint set be

$$
\mathcal{V} = \{v_i\}_{i=1}^{M},
$$

where each \(v_i\) is a candidate robot observation pose generated around the current target estimate. Each candidate is then scored by

$$
J(v_i)=
\alpha \frac{|r_i-r_d|}{r_d}
+
\beta \frac{d_i}{r_{\max}}
+
\gamma \frac{|\operatorname{wrap}(\theta_i-\theta_k)|}{\pi},
$$

In this cost function, \(r_i\) denotes the candidate radius from the target, \(r_d\) denotes the desired observation radius, \(d_i\) denotes the robot travel distance from its current pose to the candidate, and \(\theta_i\) denotes the candidate viewing yaw. The term \(\theta_k\) denotes the robot’s current yaw in the planning frame. The planner then selects

$$
v_k^\star = \arg\min_{v_i \in \mathcal{V}_{\text{valid}}} J(v_i),
$$

after rejecting candidates whose travel distance is below a minimum threshold. The current implementation uses

$$
\alpha = 0.2,\qquad \beta = 0.1,\qquad \gamma = 0.2.
$$

When radius randomization is enabled, each candidate radius is sampled in the interval \([r_{\min}, r_{\max}]\); otherwise a fixed or adaptive base radius is used.

### 2.3 System Logic Suedocode

<!-- TODO: Replace this outline with the exact logic of the custom algorithm. -->

```text
Input: recent pose estimates, robot pose, planner parameters
Output: stop decision or next-best-view goal

1. Collect a fixed window of recent target pose estimates
2. Compute confidence metrics from spatial and angular consistency
3. If confidence exceeds threshold, stop and report final estimate
4. Otherwise generate candidate viewpoints around the target
5. Score each candidate using travel cost, visibility, and safety terms
6. Select the best candidate and send it to navigation
```



## 3. Benchmarking & Results

This section must present empirical evidence from the final mission and evaluate how well the system performed, even if the final demonstration was only partially successful.

### 3.1 Experimental Setup

<!-- TODO: Describe hardware, software version, environment, trial conditions, and what counts as success. -->

Document the robot configuration, sensing stack, environment conditions, object type(s), and any reset procedure used between trials.


<!-- TODO: Milestone 3 expects reliability data across at least 10 trials. -->

| Trial | Mission Outcome | Detection Success | Pose Quality | NBV / Navigation Success | Notes |
| :--- | :--- | :--- | :--- | :--- | :--- |
| 1 | `TODO` | `TODO` | `TODO` | `TODO` | `TODO` |
| 2 | `TODO` | `TODO` | `TODO` | `TODO` | `TODO` |
| 3 | `TODO` | `TODO` | `TODO` | `TODO` | `TODO` |
| 4 | `TODO` | `TODO` | `TODO` | `TODO` | `TODO` |
| 5 | `TODO` | `TODO` | `TODO` | `TODO` | `TODO` |
| 6 | `TODO` | `TODO` | `TODO` | `TODO` | `TODO` |
| 7 | `TODO` | `TODO` | `TODO` | `TODO` | `TODO` |
| 8 | `TODO` | `TODO` | `TODO` | `TODO` | `TODO` |
| 9 | `TODO` | `TODO` | `TODO` | `TODO` | `TODO` |
| 10 | `TODO` | `TODO` | `TODO` | `TODO` | `TODO` |

### 3.5 Discussion of Results

<!-- TODO: Interpret the evidence rather than only listing numbers. -->

Summarize the main quantitative findings, identify bottlenecks, and explain whether the system met the project objective under the tested conditions.

---

## 4. Ethical Impact Statement

<!-- Milestone 3 requests roughly 300 words covering privacy, safety, and bias. -->

This section should critically evaluate how the system affects people, environments, and downstream decision-making.



---

## 5. Custom Module Code Links

Every custom module discussed in this report must include a direct repository hyperlink to the relevant source file and, where possible, the exact line range implementing the logic.

| Module | Role in System | Direct Link |
| :--- | :--- | :--- |
| `cylinder_finder.py` | Cylinder detection from point clouds | `TODO` |
| `box_finder.py` | Box detection from point clouds | `TODO` |
| `pose_estimator.py` | Pose estimation in the planning frame | `TODO` |
| `confidence_evaluator.py` | Confidence scoring service | `TODO` |
| `nbv_planner.py` | Next-best-view planning | `TODO` |
| `orchestrator.py` | Active perception coordination | `TODO` |
| `orb_vo_node.py` | Visual odometry integration | `TODO` |
| `ekf.yaml` / localization config | Sensor fusion configuration | `TODO` |

> Note: Replace each `TODO` entry with a clickable GitHub URL to the exact file and line number used in your implementation.
{: .note }

### 5.1 Localization Module Links (Vikas Narang)

- `orb_vo_node.py`: `https://github.com/mohammadnsr1/MobileRobots_Active_Perception/tree/main/src/ORB_EKF/orb_ekf/orb_vo_node.py`
- `orb_vo.yaml`: `https://github.com/mohammadnsr1/MobileRobots_Active_Perception/tree/main/src/ORB_EKF/config/orb_vo.yaml`
- `ekf.yaml`: `https://github.com/mohammadnsr1/MobileRobots_Active_Perception/tree/main/src/ORB_EKF/config/ekf.yaml`

---

## 6. Individual Contribution & Audit Appendix

This appendix should make authorship auditable and match the Milestone 3 requirement for individual technical accountability.

| Team Member | Primary Technical Role | Key Git Commits / PRs | Specific File(s) Authorship |
| :--- | :--- | :--- | :--- |
| Student Name 1 | `TODO` | `TODO` | `TODO` |
| Student Name 2 | `TODO` | `TODO` | `TODO` |
| Student Name 3 | `TODO` | `TODO` | `TODO` |
| Vikas Narang | `Localization_Module (VO + EKF)` | `1 parent 3b519bc, 61fc196 (report contribution), ORB_EKF commits` | `src/ORB_EKF/orb_ekf/orb_vo_node.py, src/ORB_EKF/config/orb_vo.yaml, src/ORB_EKF/config/ekf.yaml` |
| Mohammad Nasr | `Perception_Module` | `970c915, b180cfd, 4a9a162, 03b2bf6` | `cylinder_finder.py, box_finder.py, pose_estimator.py, confidence_evaluator.py, nbv_planner.py, orchestrator.py` |
| Khaled | `Navigation_Module` | `e5e9469` | `navigation subsystem design and Nav2 integration artifacts` |
