# Obstacle Avoidance Tuning Guide for IDA

This document explains how to tune the obstacle avoidance parameters found in `config.py` and `IDA1.py` to adjust the robot's sensitivity and reaction to obstacles.

## 1. Key Configuration Parameters (`config.py`)

These parameters define the global behavior of the robot's navigation and safety systems.

### A. Emergency & Safety Distances

*   **`LIDAR_ACIL_DURMA_M`** (Default: `1.0` - `1.5` meters)
    *   **Description:** The distance at which the robot triggers an "Emergency Stop" or "Reactive Escape" maneuver.
    *   **Tuning:**
        *   **Increase (e.g., 2.0m):** Makes the robot "shy". It will stop or turn away from obstacles much earlier. Good for high speeds but may cause false alarms in narrow channels.
        *   **Decrease (e.g., 0.8m):** Allows the robot to get closer to obstacles before panicking. Necessary for tight maneuvers (like Task 2 or Task 5) but increases collision risk.

*   **`INFLATION_MARGIN_M`** (Default: `0.25` meters)
    *   **Description:** The "buffer zone" added around every obstacle in the costmap. The path planner (A*) treats this buffer as a wall.
    *   **Tuning:**
        *   **Increase (e.g., 0.40m):** Forces the path planner to keep a wider berth from obstacles. Safer navigation but might fail to find a path in narrow gaps.
        *   **Decrease (e.g., 0.10m):** Allows the planner to route through tight spaces. However, the physical robot might clip the obstacle if it drifts slightly.

### B. Map & Planner Behavior

*   **`A_STAR_HEURISTIC_WEIGHT`** (Default: `2.5` - `3.0`)
    *   **Description:** Determines how "greedy" the A* algorithm is.
    *   **Tuning:**
        *   **Higher (e.g., 5.0):** The planner prioritizes getting to the goal as fast as possible, potentially hugging obstacles closely. Faster computation.
        *   **Lower (e.g., 1.0):** The planner explores more safe paths but takes longer to compute.

*   **`LIDAR_FREE_GAIN`** (25) & **`LIDAR_OCCUPIED_GAIN`** (80)
    *   **Description:** How quickly the map updates based on Lidar data.
    *   **Tuning:**
        *   **Higher Occupied Gain:** Obstacles appear instantly on the map (very reactive).
        *   **Higher Free Gain:** Moving obstacles (or ghost noise) are cleared from the map faster.

### C. Reactive Maneuver Intensity

*   **`ESCAPE_PWM`** (Default: `250` - `300`)
    *   **Description:** The motor power used during an emergency escape turn.
    *   **Tuning:**
        *   **Higher:** Violent, fast turns. Good for avoiding fast-moving collisions but can destabilize the heading.
        *   **Lower:** Softer turns.

*   **`MAX_PWM_CHANGE`** (Default: `60` - `120`)
    *   **Description:** Limits how fast the motor speed can change per loop iteration.
    *   **Tuning:**
        *   **Higher:** Snappy response but jerky movement.
        *   **Lower:** Smooth, boat-like movement. Reduces current spikes but increases reaction time.

## 2. Tuning Scenarios

### Scenario 1: Robot is too scared and stops too far from the gate.
*   **Action:** Decrease `LIDAR_ACIL_DURMA_M` (e.g., to 1.0m) and `INFLATION_MARGIN_M` (e.g., to 0.15m).

### Scenario 2: Robot hits the buoys while turning.
*   **Action:** Increase `INFLATION_MARGIN_M` (e.g., to 0.35m) to force a wider turn radius in the planner.

### Scenario 3: Robot reacts too late to obstacles.
*   **Action:** Increase `LIDAR_ACIL_DURMA_M` and ensure `LIDAR_OCCUPIED_GAIN` is high enough (e.g., 80-100) to register walls instantly.

### Scenario 4: Robot zig-zags wildly in narrow corridors.
*   **Action:** Decrease `LIDAR_KORIDOR_KP` (in Vision mode) or reduce `planner_bias` (in GPS mode) to allow smoother paths. Increase `MAX_PWM_CHANGE` slightly if it's reacting too slowly to corrections.

## 3. Code Locations for Advanced Tuning

*   **Reactive Avoidance Logic:** `IDA1.py` (Search for `if center_danger`)
    *   Here you can adjust the logic for `left_d > right_d` to favor one side or change the escape sequence.

*   **Path Planner Config:** `IDA1.py` (Search for `planner.get_path_plan`)
    *   Here you can adjust `cone_deg` (Field of View for path search) and `planner_bias` (Penalty for deviating from straight line).
