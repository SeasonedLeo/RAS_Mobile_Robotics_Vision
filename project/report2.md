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


### 2.1 Detailed Computational Map

**Placeholder guidance:** Provide the updated end-to-end communication structure for the implemented ROS 2 system, including nodes, topics, services, and actions.

#### 2.1.1 Mermaid Diagram

<!-- TODO: Replace this placeholder diagram with the updated system Mermaid diagram. -->
<!-- TODO: Label topics as /topic_name [msg_type]. -->
<!-- TODO: Label services/actions as /name [srv_type] or /name [action_type]. -->

```mermaid
flowchart LR
  A[TODO: Node A] -->|/topic_name [msg_type]| B[TODO: Node B]
  B -->|/service_name [srv_type]| C[TODO: Service or Action Client]
  C -->|/action_name [action_type]| D[TODO: Action Server]
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

#### 2.2.1 Module Declaration Table



| Module | Type (Custom/Library) | Inputs | Outputs | Key Parameters | Status | Source File | Notes/Changes from M1 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `[Module Name]` | `Custom` or `Library` | `[input topics/services/actions]` | `[output topics/services/actions]` | `[param_a, param_b]` | `[implemented / in progress / planned]` | `[path/to/source_file](../path/to/source_file)` | `[brief change summary]` |
| `[Module Name]` | `Custom` or `Library` | `[input topics/services/actions]` | `[output topics/services/actions]` | `[param_a, param_b]` | `[implemented / in progress / planned]` | `[path/to/source_file](../path/to/source_file)` | `[brief change summary]` |

#### 2.2.2 Custom Node Logic Flow

##### Node: cylinder_finder

**Purpose:** Detect cylindrical objects from RGB-D data and publish a segmented point cloud corresponding to the target object.

**Inputs:**
- Subscribed topic: RGB-D point cloud `/oadk/points/`

**Outputs:**
- Published topic: segmented object point cloud
- Published topic: detection flag

**Internal logic flow:**
1. Subscribe to the incoming RGB-D point cloud.
2. Apply filtering to remove background noise.
3. Segment cylindrical geometry from the scene.
4. Extract the object-specific point cloud.
5. Publish the segmented cloud.
6. Publish the detection status.

**Key parameters:** `depth_range`, `segmentation_threshold`, `min_cluster_size`, `target_radius_tolerance`

**Source file link:** [TODO: Add source file path](../path/to/cylinder_finder)

**Status:** Implemented

##### Node: pose_estimator

**Purpose:** Compute object pose from the segmented point cloud and transform it from the camera optical frame to the `odom` frame.

**Inputs:**
- Subscribed topic: segmented object point cloud
- TF transform: `camera_optical_frame -> base_link -> odom`

**Outputs:**
- Published topic: object pose in camera optical frame
- Published topic: object pose in `odom` frame

**Internal logic flow:**
1. Receive the segmented object point cloud.
2. Compute the centroid of the object points.
3. Apply PCA to estimate the dominant axis.
4. Compute yaw from the principal direction.
5. Construct the pose in the camera optical frame.
6. Transform the pose to the `odom` frame using TF.
7. Publish the pose estimates.

**Key parameters:**
- `pca_eigenvalue_threshold`
- `minimum_point_count`
- `pose_smoothing_window_size`

**Source file link:** [../src/pose_estimator.py](../src/pose_estimator.py)

**Status:** Implemented

##### Node: pose_evaluator

**Purpose:** Evaluate the reliability of pose estimates over time and compute a confidence score.

**Inputs:**
- Subscribed topic: object pose in `odom` frame

**Outputs:**
- Service response: confidence score
- Service response: NBV required flag

**Internal logic flow:**
1. Store incoming pose estimates in a sliding window.
2. Compute variance in position and orientation.
3. Evaluate convergence criteria.
4. Compute a confidence score.
5. Return the evaluation result via service.

**Key parameters:**
- `sliding_window_size`
- `confidence_threshold`
- `variance_threshold`

**Source file link:** [../src/pose_evaluator.py](../src/pose_evaluator.py)

**Status:** Planned

##### Node: nbv_planner

**Purpose:** Generate the next best viewpoint using a utility-based sampling strategy around the object.

**Inputs:**
- Service request: object pose in `odom` frame
- Service request: confidence score
- TF: robot pose in `odom` frame

**Outputs:**
- Service response: next goal pose in `odom` frame

**Internal logic flow:**
1. Generate candidate viewpoints on a circle around the object.
2. Evaluate candidates using a utility function.
3. Select the best candidate.
4. Convert the selected candidate to a navigation goal pose.
5. Return the goal pose.

**Key parameters:**
- `sampling_radius`
- `number_of_candidate_viewpoints`
- `utility_weights`

**Source file link:** [../src/nbv_planner.py](../src/nbv_planner.py)

**Status:** Planned

##### Node: orchestrator

**Purpose:** Manage the active perception loop and coordinate perception, evaluation, and navigation.

**Inputs:**
- Service response: confidence score
- Navigation status from Nav2
- Subscribed topic: object pose

**Outputs:**
- Service call: pose evaluation
- Service call: NBV planner
- Action goal: Nav2 navigation

**Internal logic flow:**
1. Wait for the initial pose estimate.
2. Request pose evaluation.
3. If confidence is low, request NBV planning.
4. Send the goal to Nav2.
5. Wait for navigation completion.
6. Trigger re-observation.
7. Repeat until the confidence threshold is reached.

**Key parameters:**
- `confidence_threshold`
- `maximum_iterations`
- `re_observation_delay`

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
