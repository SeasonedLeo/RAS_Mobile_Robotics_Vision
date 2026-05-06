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

<!-- TODO: Summarize the project objective in 2-4 sentences for an interdisciplinary audience. -->

State the problem the robot solves, the operating setting, and the mission objective in concise non-specialist language.

### 1.2 Key Technical Idea

<!-- TODO: Briefly explain the main algorithmic idea shown in the graphic. -->

Summarize the custom perception, estimation, and decision logic highlighted in the graphical abstract.

### 1.3 Final Outcome Snapshot

<!-- TODO: Add 1 short paragraph describing the final mission result shown in the abstract. -->

Provide a concise outcome statement supported by the main result visual or benchmark.

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

<!-- TODO: Define the robot state, object state, observations, and control/action variables. -->



### 2.1.4 Orchestrator

<!-- TODO: Replace the example equations below with the project-specific algorithm used in Milestone 3. -->

Let the robot pose in the planning frame be

$$
\mathbf{x}_k =
\begin{bmatrix}
x_k \\
y_k \\
\theta_k
\end{bmatrix},
$$

and let the object estimate at step \(k\) be

$$
\hat{\mathbf{o}}_k =
\begin{bmatrix}
\hat{x}_k \\
\hat{y}_k \\
\hat{\psi}_k
\end{bmatrix}.
$$

If the system maintains a confidence score over a window of \(N\) estimates, one generic formulation is

$$
C_k = w_p S_p + w_\psi S_\psi + w_n S_n + w_a S_a,
$$

where \(S_p\) is positional stability, \(S_\psi\) is yaw stability, \(S_n\) is observation support, and \(S_a\) is a shape-quality or anisotropy term.

If a next-best-view candidate set \(\mathcal{V} = \{v_i\}_{i=1}^{M}\) is evaluated, the planner may select

$$
v^\star = \arg\min_{v_i \in \mathcal{V}} J(v_i),
$$

with

$$
J(v_i) = \alpha d(v_i, \mathbf{x}_k) + \beta h(v_i, \hat{\mathbf{o}}_k) + \gamma r(v_i),
$$

where \(d\) penalizes travel cost, \(h\) rewards view geometry, and \(r\) captures risk or constraint violations.

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


