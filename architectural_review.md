# Architectural Review: Autonomous Vehicle Command & Control

## 1. Path Planning (A* & Optimization)

### Current Implementation Status
The path planning logic relies on `planner.py` for the core algorithm and `dalgaDeneme.py` for high-level goal selection.

*   **Algorithm:** Standard A* (`a_star_search`) is implemented on a downsampled grid (approx. 10cm resolution).
*   **Navigation to Start Point:** The system does **not** plan a global A* path to the initial GPS coordinates if they are outside the local costmap.
    *   In `dalgaDeneme.py` -> `select_mission_target`, the system calculates a "Projected Target" approximately 1.5 meters ahead of the robot in the direction of the GPS bearing (`GPS_LOOKAHEAD`).
    *   A* is then used to find a collision-free path to this local target. This effectively functions as a **Local Planner** with a global heading bias.
*   **Smoothing:** A smoothing function **already exists**.
    *   Function: `planner.smooth_path(path_world)`
    *   Method: Gradient Descent optimization (adjusting points towards the original path for data fidelity and towards neighbors for smoothness).
    *   It is currently called at the end of `get_path_plan`.

### Assessment of Proposed Improvements
**Question:** Do you recommend implementing a `_smooth_path` method?
**Answer:** The method is already implemented and active. If you are observing "zig-zag" or jerky movements, it is not due to the lack of a smoothing function, but likely due to:
1.  **Parameter Tuning:** The `weight_smooth` (0.1) might be too low to smooth out the 10cm grid artifacts effectively.
2.  **Lookahead Distance:** A short Pure Pursuit lookahead distance on a discretized path will aggressively track grid corners.
3.  **Grid Resolution:** The downscaling (SCALE = 0.125) might be creating a coarse map.

**Recommendation:** Do not re-implement. Instead, tune the `weight_smooth` parameter in `planner.py` (increase slightly) or increase the `weight_data` parameter to balance it.

## 2. Trajectory Tracking (Pure Pursuit) & Control

### Current Implementation Status
The `pure_pursuit_control` function in `planner.py` uses a "Bang-Bang" (Threshold-based) adaptive approach:
*   **Lookahead:** Hard-switches between `1.5m` and `0.8m` based on whether the heading error exceeds 20 degrees.
*   **Speed Control:**
    *   Base Speed + 80 PWM.
    *   If error > 10°: Speed reduced by 50%.
    *   If error > 30°: Speed boost removed (0%).

### Assessment of Proposed Improvements
**Question:** Are Dynamic Lookahead and Continuous Speed Profiling worth the engineering effort?
**Verdict:** **YES, Highly Recommended.**

The current implementation introduces discontinuities (sudden jumps) in control values because of the `if/else` thresholds. This causes the robot to jerk when crossing the 10° or 20° error boundaries.

**Proposed Upgrades:**
1.  **Dynamic Lookahead:** $L = k \cdot v_{current}$. This naturally extends the lookahead at high speeds (preventing oscillation) and shortens it at low speeds (improving cornering), without arbitrary angle thresholds.
2.  **Continuous Speed Profiling:** $v_{target} = v_{max} / (1 + k \cdot |\kappa|)$. Using path curvature ($\kappa$) to continuously modulate speed allows for fluid deceleration entering turns and smooth acceleration exiting them.

**Recommendation:** Replace the current discrete logic with these continuous functions to significantly improve trajectory tracking smoothness.

## 3. Mapping, Localization & Sensor Fusion

### Robustness Analysis
The mapping logic in `dalgaDeneme.py` (`mapping_update_lidar`) uses a probabilistic update model, but the gains are heavily unbalanced.
*   **Clearing (Free Space):** `FREE_GAIN = 2` (Adds to whiteness)
*   **Marking (Obstacle):** `OCCUPIED_GAIN = 80` (Subtracts to darkness)

**Issue:** It takes approximately **40 clear scans** (80 / 2) to remove a single "obstacle" detection.
*   If a dynamic obstacle (or a wave false positive) appears for just one frame, it creates a "Ghost Obstacle" that persists for a long time, potentially blocking the path even after the area is clear.

**Recommendation:** Increase `FREE_GAIN` (e.g., to 10 or 20). This will make the map more responsive to dynamic changes and clear "ghosts" faster, while still filtering out random sparse noise.
