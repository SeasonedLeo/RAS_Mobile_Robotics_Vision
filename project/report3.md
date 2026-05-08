---
layout: default
math: mathjax
title: "Report 3"
parent: Project
nav_order: 3
---

# Report 3 — Final Documentation & Analysis

{: .no_toc }

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
  <img src="{{ '/assets/images/Graphical_Abstract_v2.png' | relative_url }}"
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




### 2.1.3 (Visual Odometry or Estimation & Localization Module) Individual Localization Section: ORB-SLAM3 + EKF

This section documents the localization module contribution based on ORB-SLAM3 stereo visual odometry integrated with EKF fusion.

#### Camera Used in ORB-SLAM Pipeline

The ORB-SLAM3 node uses the TurtleBot4 OAK-D stereo camera pair as its primary localization sensor:

- Left camera image topic: `/oakd/left/image_raw`
- Right camera image topic: `/oakd/right/image_raw`
- Left camera calibration topic: `/oakd/left/camera_info`
- Right camera calibration topic: `/oakd/right/camera_info`

Stereo intrinsics and baseline are consumed through calibration/configuration parameters (`fx`, `fy`, `cx`, `cy`, image size, and baseline term such as `Camera.bf`) so that feature correspondences can be triangulated at metric scale.

#### How ORB-SLAM3 Works in This Project

ORB-SLAM3 estimates the camera pose by repeatedly solving a geometric consistency problem between image observations and a 3D map.

```mermaid
%%{init: {"themeVariables": {"fontSize": "20px"}, "flowchart": {"nodeSpacing": 45, "rankSpacing": 65}}}%%
flowchart TB
  A["Stereo Images"] --> B["ORB Features"]
  B --> C["Stereo + Temporal Matching"]
  C --> D["Depth from Disparity"]
  D --> E["Pose Optimization"]
  E --> F["Local Bundle Adjustment"]
  F --> G["Publish VO Pose/Odom"]
  G --> H["EKF Fusion"]
```

The Mermaid pipeline above can be read as eight mathematical blocks:

##### Block A: Stereo Images

At time \(k\), the input is a synchronized rectified stereo pair \((I_L^k, I_R^k)\). Rectification enforces horizontal epipolar geometry:

$$
v_L \approx v_R,
$$

which simplifies correspondence search to mostly the \(u\)-direction.

##### Block B: ORB Features

ORB combines FAST keypoint detection with oriented BRIEF binary descriptors. A BRIEF bit is generated by intensity comparison:

$$
b_m =
\begin{cases}
1, & I(\mathbf{p}+\mathbf{a}_m) < I(\mathbf{p}+\mathbf{b}_m),\\
0, & \text{otherwise},
\end{cases}
$$

and the descriptor is a binary vector \(\mathbf{q}=[b_1,\dots,b_M]\). In words: each keypoint is encoded as a compact binary signature for fast matching.

##### Block C: Stereo + Temporal Matching

Features are matched (1) left-right at the same time and (2) across time \(k-1 \to k\). Similarity uses Hamming distance:

$$
d_H(\mathbf{q}_i,\mathbf{q}_j)=\sum_{m=1}^{M}\mathbf{1}[q_{i,m}\oplus q_{j,m}],
$$

then geometric checks reject outliers. In words: keep only correspondences that are both visually similar and geometrically plausible.

##### Block D: Depth from Disparity

From stereo correspondence:

$$
d = u_L-u_R,\qquad Z=\frac{f_x b}{d},
$$

and back-projection gives 3D coordinates:

$$
X=\frac{(u-c_x)Z}{f_x},\qquad Y=\frac{(v-c_y)Z}{f_y}.
$$

In words: stereo disparity converts 2D features into metric-scale 3D map points.

##### Block E: Pose Optimization

Camera pose \(\mathbf{T}_k\in SE(3)\) is estimated by minimizing reprojection error:

$$
\mathbf{T}_k^*=
\arg\min_{\mathbf{T}_k}
\sum_i
\rho\!\left(
\left\|
\mathbf{u}_{i,k}-\pi\!\left(\mathbf{K}\mathbf{T}_k\mathbf{X}_i\right)
\right\|^2
\right).
$$

In words: find the rigid transform that best aligns projected 3D landmarks with observed pixels.

##### Block F: Local Bundle Adjustment

A local window jointly refines poses and landmarks:

$$
\min_{\{\mathbf{T}_j\},\{\mathbf{X}_i\}}
\sum_{(i,j)\in\mathcal{O}}
\rho\!\left(
\left\|
\mathbf{u}_{i,j}-\pi\!\left(\mathbf{K}\mathbf{T}_j\mathbf{X}_i\right)
\right\|^2
\right).
$$

In words: optimize a small local graph to reduce drift and improve consistency.

##### Block G: Publish VO Pose/Odom

The backend returns a camera/world transform; for ROS publishing, the pose is expressed in the VO/world frame (e.g., \(\mathbf{T}_{wc}=\mathbf{T}_{cw}^{-1}\)) and emitted as `/vo/pose` and `/vo/odometry`.

In words: this is the direct visual-odometry estimate before sensor fusion.

##### Block H: EKF Fusion

EKF fuses VO with wheel odometry. With state \(\mathbf{x}_k\), process model \(f\), and measurement model \(h\):

$$
\mathbf{x}_{k|k-1}=f(\mathbf{x}_{k-1|k-1},\mathbf{u}_k),\qquad
\mathbf{P}_{k|k-1}=\mathbf{F}_k\mathbf{P}_{k-1|k-1}\mathbf{F}_k^\top+\mathbf{Q}_k,
$$

$$
\mathbf{K}_k=\mathbf{P}_{k|k-1}\mathbf{H}_k^\top(\mathbf{H}_k\mathbf{P}_{k|k-1}\mathbf{H}_k^\top+\mathbf{R}_k)^{-1},
$$

$$
\mathbf{x}_{k|k}=\mathbf{x}_{k|k-1}+\mathbf{K}_k\!\left(\mathbf{z}_k-h(\mathbf{x}_{k|k-1})\right).
$$

In words: prediction comes from motion model; correction comes from VO/odometry measurements, yielding a smoother and more stable navigation pose.

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




### 2.1.4 Navigation, Control, Actuation Module

The navigation and actuation role of the system was initially intended to be handled through Nav2, as discussed in Report 2. In the final implementation, however, Nav2 was replaced by a lightweight local controller implemented in `odom_controller.py`. This change was made because the final system operates without a global map and expresses all perception and planning outputs directly in the `odom` frame. Since Nav2 typically assumes a map-based navigation setup, it was not well aligned with the final local active perception pipeline.

An alternative attempt was made to use SLAM Toolbox in order to provide the mapping layer required for Nav2, but this integration was not brought to a stable operational state during the project. For this reason, the final system adopted a simpler local-control formulation that directly drives the TurtleBot4 to a goal pose in the `odom` frame.

The `odom_controller.py` node subscribes to an `odom`-frame navigation goal and the robot odometry feedback, then publishes velocity commands to drive the robot toward the requested viewpoint. The controller uses proportional control for linear motion and PD control for heading correction, with bounded linear and angular velocities, a rotate-in-place mode for large heading errors, and separate tolerances for final position and yaw convergence. In this way, the module provides the final execution link between the viewpoint selected by the planner and the physical robot motion required to acquire the next observation.



### 2.1.5 Orchestrator

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

where each sample contains a pose estimate, the number of points in the segmented target cloud, and the anisotropy ratio returned by the pose-estimation stage. Over this history window, the confidence evaluator computes the position variance, the yaw variance, the mean point count, and the mean anisotropy ratio.

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
| `orb_vo.yaml` | localization config | `https://github.com/mohammadnsr1/MobileRobots_Active_Perception/tree/main/src/ORB_EKF/config/orb_vo.yaml` |
| `orb_vo_node.py` | Visual odometry integration | `https://github.com/mohammadnsr1/MobileRobots_Active_Perception/tree/main/src/ORB_EKF/orb_ekf/orb_vo_node.py` |
| `ekf.yaml` / localization config | Sensor fusion configuration | `https://github.com/mohammadnsr1/MobileRobots_Active_Perception/tree/main/src/ORB_EKF/config/ekf.yaml` |

> Note: Replace each `TODO` entry with a clickable GitHub URL to the exact file and line number used in your implementation.
{: .note }


---

## 6. Individual Contribution & Audit Appendix

This appendix should make authorship auditable and match the Milestone 3 requirement for individual technical accountability.

| Team Member | Primary Technical Role | Key Git Commits / PRs | Specific File(s) Authorship |
| :--- | :--- | :--- | :--- |
| Mohammad Nasr | `Perception & Planning Module` | `970c915, b180cfd, 4a9a162, 03b2bf6` | `cylinder_finder.py, box_finder.py, pose_estimator.py, confidence_evaluator.py, nbv_planner.py, orchestrator.py, odom_controller.py` |
| Vikas Narang | `Localization_Module (VO + EKF)` | `1 parent 3b519bc, 61fc196 (report contribution), ORB_EKF commits` | `src/ORB_EKF/orb_ekf/orb_vo_node.py, src/ORB_EKF/config/orb_vo.yaml, src/ORB_EKF/config/ekf.yaml` |
| Khaled | `Navigation_Module` | `e5e9469` | `navigation subsystem design and Nav2 integration artifacts` |
