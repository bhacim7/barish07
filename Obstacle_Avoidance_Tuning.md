# Obstacle Avoidance Tuning Guide

This document explains the key parameters and logic controlling obstacle avoidance in the IDA1.py system. Tuning these values correctly is crucial for balancing between safety (avoiding crashes) and performance (smooth navigation).

## 1. Reactive Avoidance Parameters (Lidar Reflex)
These parameters control the immediate "reflex" actions when an object is detected too close, bypassing the path planner.

### `LIDAR_ACIL_DURMA_M` (config.py)
*   **Description:** The critical distance threshold for the front sector (-15 to +15 degrees). If an object is detected closer than this value, the robot triggers an emergency stop and escape maneuver.
*   **Default:** `1.5` meters (or `1.0` in VISION mode).
*   **Tuning:**
    *   **Increase (e.g., 2.0m):** Safer, but the robot might stop too often in narrow passages or false positives.
    *   **Decrease (e.g., 0.8m):** Allows tighter maneuvering, but increases collision risk if the robot is moving fast.

### `ESCAPE_PWM` (config.py)
*   **Description:** The motor power (PWM offset from 1500) used during the "Shock Brake" and "Reverse" maneuvers.
*   **Default:** `300` (Resulting in ~1200 or ~1800 PWM).
*   **Tuning:**
    *   **Increase:** Stronger braking and faster reverse. Good for fast approaches.
    *   **Decrease:** Smoother reaction. Use if the robot "jumps" too violently during avoidance.

### Escape Maneuver Logic (IDA1.py -> `execute_reactive_avoidance`)
*   **Shock Brake:** Reverses motors for `0.1s` to kill momentum.
*   **Reverse:** Moves backward for `0.4s`.
*   **Turn:** Performs a tank turn (spot turn) towards the clearer side (Left vs Right Lidar distance).
*   **Tuning:** These durations are currently hardcoded in `IDA1.py`. Modify the `time.sleep(0.4)` call in the avoidance block to change the reverse distance.

## 2. Map-Based Avoidance (Path Planner)
These parameters affect how the robot sees the world and plans paths around obstacles.

### `INFLATION_MARGIN_M` (IDA1.py)
*   **Description:** The safety buffer added around every detected obstacle on the costmap.
*   **Default:** `0.05` meters (Configurable in `IDA1.py` or `config.py`).
*   **Tuning:**
    *   **Increase:** The robot will stay further away from walls/buoys. Vital for preventing side-swipes.
    *   **Decrease:** Allows the robot to pass through narrower gaps (e.g., the Task 5 dock entrance).

### `LIDAR_FREE_GAIN` / `LIDAR_OCCUPIED_GAIN` (config.py / IDA1.py)
*   **Description:** These control how quickly the map updates. The map is probabilistic (0-255).
    *   `FREE_GAIN`: How much "whiter" (safer) a pixel becomes when Lidar sees empty space.
    *   `OCCUPIED_GAIN`: How much "blacker" (blocked) a pixel becomes when Lidar sees an obstacle.
*   **Default:** Free: `25`, Occupied: `80`.
*   **Tuning:**
    *   **Higher Occupied Gain:** Map fills up faster. Good for thin obstacles (poles).
    *   **Higher Free Gain:** Moving objects (boats, debris) are cleared from the map faster after they move.

### `MAP_DECAY_AMOUNT` (config.py)
*   **Description:** How much the entire map "fades" (forgets obstacles) every loop.
*   **Default:** `5`.
*   **Tuning:**
    *   **Increase:** Old obstacles disappear quickly. Good for dynamic environments but risks forgetting a stationary wall.
    *   **Decrease:** Better memory of static obstacles.

## 3. Path Following (Pure Pursuit)
These parameters control how aggressively the robot steers to follow the safe path calculated by A*.

### `pure_pursuit_control` Gains (planner.py)
*   **`LOOKAHEAD_DIST` (Min/Max L):** The distance to the target point on the path.
    *   **Small L:** Robot cuts corners and oscillates (weaves) on straight lines.
    *   **Large L:** Robot moves smoothly but cuts corners significantly (understeers).
*   **`CURVATURE_GAIN`:** Slows the robot down in sharp turns.
    *   **Increase:** Robot slows down more for turns, improving safety.

## 4. Troubleshooting Common Issues

*   **Issue:** Robot stops and reverses constantly in open water.
    *   **Fix:** Check `LIDAR_ACIL_DURMA_M`. It might be too high. Check for Lidar noise (reflections). Increase `LIDAR_FREE_GAIN`.

*   **Issue:** Robot hits obstacles while turning.
    *   **Fix:** Increase `INFLATION_MARGIN_M`. Decrease `pure_pursuit_control` speed or increase `CURVATURE_GAIN`.

*   **Issue:** Robot refuses to enter a narrow gap (e.g., Docking).
    *   **Fix:** Decrease `INFLATION_MARGIN_M` temporarily or use "Blind/Lidar Only" mode for that specific task.
