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

This section should document the core technical contribution of the custom module(s) using formal notation, equations, and direct traceability to implementation.

### 2.1 Problem Formulation

<!-- TODO: Define the robot state, object state, observations, and control/action variables. -->

Describe the inputs, outputs, assumptions, and objective of the algorithm.

### 2.2 Formal Algorithm Description

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

### 2.3 Algorithm Pseudocode

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

### 2.4 Code Traceability

<!-- TODO: Add direct GitHub links to the exact source files and line numbers implementing the logic above. -->

| Custom Module | Purpose | Repository Link |
| :--- | :--- | :--- |
| `pose_estimator` | Converts segmented target cloud into pose estimates | `TODO` |
| `confidence_evaluator` | Computes confidence from a recent estimate history | `TODO` |
| `nbv_planner` | Selects the next best view candidate | `TODO` |
| `orchestrator` | Coordinates confidence checks and replanning | `TODO` |
| `visual_odometry_node` (ORB-SLAM3) | Stereo visual odometry and camera-pose tracking | `https://github.com/mohammadnsr1/MobileRobots_Active_Perception/tree/main/src/ORB_EKF/orb_ekf/orb_vo_node.py` |
| `ekf_node` configuration | VO + wheel odometry fusion settings | `https://github.com/mohammadnsr1/MobileRobots_Active_Perception/tree/main/src/ORB_EKF/config/ekf.yaml` |

### 2.5 Individual Localization Section (Vikas Narang): ORB-SLAM3 + EKF

This section documents the localization module contribution based on ORB-SLAM3 stereo visual odometry integrated with EKF fusion.

#### 2.5.1 Camera Used in ORB-SLAM Pipeline

The ORB-SLAM3 node uses the TurtleBot4 OAK-D stereo camera pair as its primary localization sensor:

- Left camera image topic: `/oakd/left/image_raw`
- Right camera image topic: `/oakd/right/image_raw`
- Left camera calibration topic: `/oakd/left/camera_info`
- Right camera calibration topic: `/oakd/right/camera_info`

Stereo intrinsics and baseline are consumed through calibration/configuration parameters (`fx`, `fy`, `cx`, `cy`, image size, and baseline term such as `Camera.bf`) so that feature correspondences can be triangulated at metric scale.

#### 2.5.2 How ORB-SLAM3 Works in This Project

At each synchronized stereo frame pair, ORB-SLAM3 performs the following processing stages:

1. Extract ORB keypoints and descriptors from left/right images.
2. Match descriptors to build stereo correspondences and temporal feature tracks.
3. Estimate camera motion from geometric constraints and tracked features.
4. Triangulate sparse 3D map points from stereo disparity.
5. Refine pose/map consistency through internal local optimization.
6. Publish local visual odometry as ROS 2 pose/odometry outputs for downstream fusion.

The localization output is relative to startup and can drift over long operation. To stabilize deployment for navigation, the VO estimate is fused with wheel odometry in EKF (`robot_localization`), providing a smoother and more robust `odom` estimate for planning/control.

#### 2.5.3 ORB-SLAM3 Features and Parameters Used

The ORB-SLAM3 configuration relies on the standard ORB feature extractor and stereo parameters:

- `ORBextractor.nFeatures`: target number of features extracted per frame.
- `ORBextractor.scaleFactor`: image pyramid scaling between levels.
- `ORBextractor.nLevels`: number of pyramid levels for multi-scale matching.
- `ORBextractor.iniThFAST`: initial FAST threshold for keypoint detection.
- `ORBextractor.minThFAST`: fallback FAST threshold in low-texture regions.
- `Camera.fps`: expected image frame rate for timing consistency.
- `ThDepth`/stereo depth threshold terms: constrain valid stereo depth range.

Practically, these parameters control the accuracy/robustness trade-off: higher feature counts and tuned FAST thresholds improve tracking in texture-limited indoor scenes but increase compute load.

---

## 3. Benchmarking & Results

This section must present empirical evidence from the final mission and evaluate how well the system performed, even if the final demonstration was only partially successful.

### 3.1 Experimental Setup

<!-- TODO: Describe hardware, software version, environment, trial conditions, and what counts as success. -->

Document the robot configuration, sensing stack, environment conditions, object type(s), and any reset procedure used between trials.

### 3.2 Accuracy: Ground Truth vs. Estimated Pose

<!-- TODO: Insert time-series or spatial comparison plots here. -->

<div align="center">
  <img src="{{ '/assets/images/report3_accuracy_placeholder.png' | relative_url }}"
       alt="Accuracy plot placeholder"
       width="85%">
</div>

**Figure 3.1.** Placeholder for ground-truth versus estimated pose comparison.

Discuss the estimation trend, steady-state behavior, and major failure cases visible in the plot.

### 3.3 Error Analysis

<!-- TODO: Add cross-track error, translational/rotational error, covariance growth, or equivalent uncertainty analysis. -->

| Metric | Definition | Result |
| :--- | :--- | :--- |
| Mean position error | `TODO` | `TODO` |
| Max position error | `TODO` | `TODO` |
| Mean yaw error | `TODO` | `TODO` |
| Cross-track error (CTE) | `TODO` | `TODO` |
| Covariance / uncertainty trend | `TODO` | `TODO` |

Explain what the error metrics reveal about the strongest and weakest parts of the system.

### 3.4 Success Rate Across Trials

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

### 4.1 Privacy

<!-- TODO: Discuss what sensor data is collected, retained, displayed, or shared. -->

Address image or point-cloud data handling, any blurring/anonymization strategy, and whether the system stores personally identifying information.

### 4.2 Safety

<!-- TODO: Discuss motion risk, collision risk, stopping behavior, and kinetic energy management. -->

Evaluate the hazards introduced by autonomous motion, hardware limitations, and failure handling during the final mission.

### 4.3 Bias and Limitations

<!-- TODO: Discuss sensing bias such as material sensitivity, lighting dependence, occlusion, or geometry assumptions. -->

Analyze how the chosen sensors or algorithms may underperform for specific objects, surfaces, or environments.

### 4.4 Ethical Framework Analysis

<!-- TODO: Explicitly discuss utilitarian and justice-based viewpoints as requested. -->

Interpret the design tradeoffs through at least utilitarian and justice-oriented lenses, and note how future iterations should reduce harm and exclusion.

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

### 6.1 Authorship Notes

<!-- TODO: Clarify who authored which custom logic and who handled infrastructure/integration. -->

Add a short note explaining how work was divided and how the commit history supports the claimed authorship.

Localization authorship note (Vikas Narang): implemented and documented the ORB-SLAM3 stereo visual-odometry integration and EKF fusion configuration, including camera-topic interfaces, ORB configuration workflow, VO publication pipeline, and localization handoff to navigation modules.

### 6.2 Audit Readiness Checklist

* [ ] Every custom module named in the report has a direct GitHub link
* [ ] Benchmarking includes evidence from at least 10 trials
* [ ] Ground-truth vs. estimated performance plots are included
* [ ] Error metrics are reported and interpreted
* [ ] Ethical statement covers privacy, safety, and bias
* [ ] Each team member has presentable, auditable technical ownership

---

## 7. Presentation Support Notes

<!-- Optional section kept here to help present directly from the report during the peer audit. -->

### 7.1 Aim & Motivation

Summarize the project motivation and the real-world problem being addressed.

### 7.2 System Design

Reference the ROS 2 graph, hardware setup, and module interactions from Reports 1 and 2.

### 7.3 Custom Logic Deep Dive

Point reviewers to the algorithm section above and the linked source files.

### 7.4 Results & Ethical Defense

Use the benchmarking plots, trial table, and ethical impact statement as the basis for the presentation discussion.
