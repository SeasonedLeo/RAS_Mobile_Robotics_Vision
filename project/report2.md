---
layout: default
title: "Report 2"
parent: Project
nav_order: 2
---

# Report 2 — Mid-Point Technical Proof
{: .no_toc }

This report documents the transition from system design to implementation for the project. It provides the formal technical structure for Milestone 2 while reserving all project-specific results, measurements, and evidence for later completion.

---

## Table of Contents

{: .no_toc .text-delta }

1. TOC
{:toc}

---

## 1. Kinematics

This section must formally describe the robot motion model using LaTeX equations, explicitly mapping control inputs to state updates.

### 1.1 Robot Motion Model

<!-- TODO: State the robot configuration variables and define the coordinate frame convention. -->
<!-- TODO: Derive the motion model appropriate for the robot base used in this project. -->

**Placeholder guidance:** Define the state vector, reference frames, and the kinematic model used for implementation.

$$
\text{TODO: Insert robot motion model equation(s), e.g., } \dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})
$$

**Short explanation placeholder:** Briefly explain what each state variable and model term represents, and why this model is appropriate for the platform.

### 1.2 Control Inputs

<!-- TODO: Identify the commanded inputs used by the controller or middleware interface. -->
<!-- TODO: Validate that the chosen inputs match the deployed ROS 2 control interface. -->

**Placeholder guidance:** Specify the control inputs used by the robot and how they correspond to commanded motion.

$$
\text{TODO: Insert control input definition, e.g., } \mathbf{u} = [u_1, u_2]^\top
$$

**Short explanation placeholder:** Clarify whether the implementation uses linear/angular velocity, wheel velocities, or another control representation.

### 1.3 State Update Equations

<!-- TODO: Derive the discrete-time state update equations used in software. -->
<!-- TODO: Confirm the update step matches the controller or estimator execution rate. -->

**Placeholder guidance:** Show how the selected control inputs propagate the robot state over one time step.

$$
\text{TODO: Insert discrete-time update equation(s), e.g., } \mathbf{x}_{k+1} = f(\mathbf{x}_k, \mathbf{u}_k, \Delta t)
$$

**Short explanation placeholder:** Describe how the continuous or nominal model is converted into the update form used in code.

---

## 2. System Architecture

This section describes the functional modules that compose the active perception pipeline. Each module is responsible for a specific stage of object detection, pose estimation, viewpoint planning, and navigation. The system is organized as a ROS 2 network where perception nodes stream observations, decision modules operate on demand, and an orchestrator manages the active perception loop.



### 2.1.1 Mermaid Diagram

### 2.1.1 Data Flow Diagram (Perception → Estimation → Planning → Actuation)

```mermaid
flowchart LR

  subgraph Perception
    RGBD[RGBD Camera]
    LIDAR[LiDAR]
    IMU[IMU]
  end

  subgraph ObjectPerception
    PCP[Point Cloud]
    OPE[Object Pose]
  end

  subgraph RobotLocalization
    VO[Visual Odometry]
    EKF[EKF]
  end
  subgraph Planning
    CONF[Confidence Evaluation]
    NBV[Next Best View]
    NAV2[Nav2 Global Planner]
    RC[Reactive Controller]
  end

  subgraph Actuation
    DDC[Diff Drive Controller]
    MHI[Motor Hardware Interface]
  end

  %% Perception → Object perception
  RGBD --> PCP
  PCP --> OPE

  %% Perception → Localization
  RGBD --> VO
  IMU --> EKF
  VO --> EKF
  

  %% Estimation to planning
  OPE --> CONF
  CONF --> NBV
  EKF --> NBV

  %% Planning to actuation
  NBV --> NAV2
  NAV2 --> RC
  LIDAR --> RC
  RC --> DDC
  DDC --> MHI

  %% Styles
  style Perception fill:#ffe6e6,stroke:#333,stroke-width:2px
  style ObjectPerception fill:#fff2cc,stroke:#333,stroke-width:2px
  style RobotLocalization fill:#fff2cc,stroke:#333,stroke-width:2px
  style Planning fill:#e6e6ff,stroke:#333,stroke-width:2px
  style Actuation fill:#d9f2d9,stroke:#333,stroke-width:2px

```
**Short explanation placeholder:** Summarize the major data pathways and identify the most important interfaces in the system.

#### 2.1.2 rqt_graph Export

<!-- TODO: Insert the exported rqt_graph image or screenshot here. -->
<!-- TODO: Ensure topic and node labels are readable in the final submission. -->

**Placeholder guidance:** Add the current `rqt_graph` export that reflects the implemented nodes and communication graph.

`[TODO: Insert rqt_graph image or screenshot here]`

**Short explanation placeholder:** Briefly describe what the exported graph confirms about the live ROS 2 computation graph.

#### 2.1.3 Annotated Communication Notes

<!-- TODO: Add annotated notes if the raw rqt_graph requires clarification. -->
<!-- TODO: Highlight any remappings, multiplexed topics, or hidden infrastructure nodes. -->

**Placeholder guidance:** Use this subsection if the raw computational graph needs clarification for reviewers.

**Short explanation placeholder:** Explain any non-obvious communication paths, launch-time remappings, or auxiliary library nodes.

### 2.2 Module Descriptions

This subsection must explain every node shown in the computational map.

### 2.2.1 Module Declaration Table



| Module | Type (Custom/Library) | Inputs | Outputs | Key Parameters | Status | Source File | Notes/Changes from M1 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `[Module Name]` | `Custom` or `Library` | `[input topics/services/actions]` | `[output topics/services/actions]` | `[param_a, param_b]` | `[implemented / in progress / planned]` | `[path/to/source_file](../path/to/source_file)` | `[brief change summary]` |
| `[Module Name]` | `Custom` or `Library` | `[input topics/services/actions]` | `[output topics/services/actions]` | `[param_a, param_b]` | `[implemented / in progress / planned]` | `[path/to/source_file](../path/to/source_file)` | `[brief change summary]` |

### 2.2.2 Custom Node Logic Flow

### 1- cylinder_finder

**Purpose:** Detect cylindrical objects from RGB-D data and publish a segmented point cloud corresponding to the target object.  

**Inputs:** RGB-D point cloud `/robot_10/oakd/points`  

**Outputs:** Segmented object point cloud, detection flag  

**Internal logic flow:** Subscribe to point cloud → filter noise → segment cylindrical geometry → extract object cloud → publish segmentation and detection status  

**Key parameters:** `depth_range`, `segmentation_threshold`, `min_cluster_size`, `target_radius_tolerance`  

**Source file link:** [TODO: Add source file path](../path/to/cylinder_finder)  

**Status:** Implemented  


### 2- pose_estimator

**Purpose:** Compute object pose from segmented point cloud and transform it from the camera optical frame to the `odom` frame.  

**Inputs:** Segmented object point cloud, TF `camera_optical_frame → base_link → odom`  

**Outputs:** Object pose in camera frame, object pose in `odom` frame  

**Internal logic flow:** Receive cloud → compute centroid → apply PCA → estimate yaw → construct pose in camera frame → transform to `odom` → publish pose  

**Key parameters:** `pca_eigenvalue_threshold`, `minimum_point_count`, `pose_smoothing_window_size`  

**Source file link:** [../src/pose_estimator.py](../src/pose_estimator.py)  

**Status:** Implemented  


### 3- confidence_evaluator

**Purpose:** Evaluate the consistency of repeated pose estimates and compute a confidence score used to determine whether another viewpoint is required.  

**Inputs:** Service request containing a batch or window of object poses in `odom` frame  

**Outputs:** Service response containing confidence score and NBV-required flag  

**Internal logic flow:** Receive a set of recent pose estimates → compute consistency metrics over position and orientation → evaluate convergence against thresholds → compute confidence score → return confidence score and NBV decision  

**Key parameters:** `window_size`, `confidence_threshold`, `position_variance_threshold`, `orientation_variance_threshold`  

**Source file link:** [../src/confidence_evaluator.py](../src/confidence_evaluator.py)  

**Status:** Planned  


### 4- nbv_planner

**Purpose:** Generate the next best viewpoint using utility-based sampling around the estimated object pose.  

**Inputs:** Service request containing object pose in `odom`, confidence score, and robot pose in `odom`  

**Outputs:** Service response containing next goal pose in `odom`  

**Internal logic flow:** Sample candidate viewpoints around object → evaluate each candidate using utility function → select highest-utility viewpoint → convert selected candidate to goal pose in `odom` → return goal pose  

**Key parameters:** `sampling_radius`, `num_candidates`, `utility_weights`  

**Source file link:** [../src/nbv_planner.py](../src/nbv_planner.py)  

**Status:** Planned  


### 5- orchestrator

**Purpose:** Coordinate the active perception loop by collecting pose estimates, calling confidence evaluation, requesting a new viewpoint when needed, and sending navigation goals to Nav2.  

**Inputs:** Object poses from `pose_estimator`, navigation status from Nav2  

**Outputs:** Service request to `confidence_evaluator`, service request to `nbv_planner`, navigation goal to Nav2  

**Internal logic flow:** Subscribe to pose estimates from `pose_estimator` → store a fixed window of recent poses → call `confidence_evaluator` service to assess consistency and compute confidence → if confidence is above threshold and NBV flag is false, terminate loop → otherwise call `nbv_planner` service to request a new goal in `odom` → send goal to Nav2 → wait for navigation result → repeat observation and evaluation loop until convergence  

**Key parameters:** `pose_window_size`, `confidence_threshold`, `max_iterations`, `reobserve_delay`  

**Source file link:** [../src/orchestrator.py](../src/orchestrator.py)  

**Status:** Planned   

#### 2.2.3 Library Node Configuration

<!-- TODO: Document any tuned third-party nodes or launch-time parameters. -->
<!-- TODO: Include parameter values only after they have been verified in experiments. -->

**Placeholder guidance:** Document tuned parameters for library-managed components such as update frequency, inflation radius, controller frequency, costmap settings, sensor frame configuration, or filter gains.

**Short explanation placeholder:** Explain which library parameters were tuned, why they mattered, and how configuration changed relative to the Milestone 1 plan.

---

## 3. Experimental Analysis & Validation

This section validates behavior under realistic conditions and documents robustness.

### 3.1 Noise & Uncertainty Analysis

#### 3.1.1 Hardware Noise / Sensor Calibration

<!-- TODO: Summarize measured or observed hardware noise characteristics. -->
<!-- TODO: Document any calibration procedure performed before experiments. -->

- Sensor offsets: [TODO: Record observed biases, frame offsets, or calibration corrections.]
- Variance/noise profiles: [TODO: Summarize measurement variance, drift, or instability.]
- Calibration observations: [TODO: Note what improved or remained problematic after calibration.]
- Tuning changes made due to noise: [TODO: Document filtering, thresholds, or parameter updates.]


### 3.2 Run-Time Issues

**Placeholder guidance:** Document behaviors observed during implementation runs, integration tests, or milestone demonstrations.

#### 3.2.1 Failure Cases Observed

<!-- TODO: List representative failure cases encountered during testing. -->

**Short explanation placeholder:** Describe the most important failure modes observed during execution without overstating conclusions.

#### 3.2.2 Recovery / Mitigation Logic

<!-- TODO: Explain the logic used to detect, recover from, or reduce failures. -->

**Short explanation placeholder:** Summarize the implemented safeguards, fallback behavior, or operator interventions used during testing.

#### 3.2.3 Remaining Limitations

<!-- TODO: Document current limitations that remain unresolved at Milestone 2. -->

**Short explanation placeholder:** Identify the known technical gaps that still affect robustness, performance, or completeness.

### 3.3 Milestone Video

**Placeholder guidance:** Add one or more embedded or directly linked YouTube/Vimeo videos showing a core sub-task or milestone-relevant technical capability.

- Video 1: [embed/link here]
- Video 2: [embed/link here]

<!-- TODO: Ensure each video clearly demonstrates a core sub-task required for Milestone 2. -->

---

## 4. Project Management

This section documents feedback integration and individual contributions.

### 4.1 Instructor Feedback Integration

<!-- TODO: Copy each Milestone 1 feedback item or question into this table and map it to an action. -->

| Milestone 1 Feedback / Question | Action Taken | Technical Change Implemented | Current Status |
| :--- | :--- | :--- | :--- |
| `[feedback item]` | `[TODO: describe response]` | `[TODO: describe technical change]` | `[open / in progress / resolved]` |
| `[feedback item]` | `[TODO: describe response]` | `[TODO: describe technical change]` | `[open / in progress / resolved]` |

### 4.2 Individual Contribution

<!-- TODO: Insert direct links to commits, pull requests, and authored files where possible. -->

| Team Member | Primary Technical Role | Key Git Commits / PRs | Specific File(s) Authorship | Notes |
| :--- | :--- | :--- | :--- | :--- |
| `[Name]` | `[TODO: role]` | `[commit/PR link]` | `[file paths]` | `[TODO: notes]` |
| `[Name]` | `[TODO: role]` | `[commit/PR link]` | `[file paths]` | `[TODO: notes]` |

---

## 5. Conclusion / Current Status

This closing section should summarize current implementation progress and clearly identify what remains before the final milestone.

### 5.1 What Is Working

<!-- TODO: Summarize the subsystems that are currently functional and demonstrated. -->

**Short explanation placeholder:** Briefly state which components are operational as of this milestone checkpoint.

### 5.2 What Is Still Pending

<!-- TODO: List the highest-priority incomplete items that remain before Milestone 3. -->

**Short explanation placeholder:** Identify the major unfinished items, validation gaps, or integrations still in progress.

### 5.3 Next Steps Toward Milestone 3

<!-- TODO: Convert remaining technical work into concrete next steps for the final milestone. -->

**Short explanation placeholder:** Outline the immediate engineering priorities, validation work, and deliverables leading into Milestone 3.
