# Task 5: Autonomous Docking Strategy (Geometric-Based)

## Overview
This document outlines the navigation logic for Task 5 (Docking). The goal is to autonomously identify a docking bay entrance (a gap in the walls), enter it, park, and exit back to the start point, relying solely on geometric data (LiDAR) without visual markers.

## State Machine Flow

The logic introduces new states to `IDA1.py`:

1.  **`TASK5_APPROACH`** (Existing): Navigate to the starting GPS coordinate.
2.  **`TASK5_SCAN`** (New): Rotate 360° (optional) or process a full 360° LiDAR scan to find the "Best Gap".
3.  **`TASK5_ALIGN`** (New): Rotate the robot to face the detected gap center.
4.  **`TASK5_ENTER`** (Refined): Navigate into the gap using Reactive Gap Centering.
5.  **`TASK5_PARK`** (Refined): Stop and hold position inside the dock.
6.  **`TASK5_EXIT`** (Refined): Reverse out of the dock and return to the approach point.

---

## 1. Gap Detection Algorithm (Pseudocode)

**Objective:** Find the widest "open" segment in the LiDAR scan that accommodates the robot.

**Inputs:**
*   `lidar_data`: List of `(quality, angle, distance)` tuples.
*   `robot_width`: Width of the robot in meters (e.g., 1.0m).
*   `safety_margin`: Extra clearance needed (e.g., 0.5m).
*   `min_gap_width` = `robot_width` + `safety_margin`.
*   `max_gap_width` = 5.0m (To avoid selecting open sea as a "gap").

**Algorithm:**

```python
def find_best_docking_gap(lidar_data):
    # 1. Pre-process Data
    # Convert polar coordinates to Cartesian (x, y) relative to robot
    # Filter out noise (very close points < 0.1m or very far > 10m)
    points = []
    for quality, angle, dist in lidar_data:
        if 0.1 < dist < 10.0:
            rad = math.radians(angle)
            x = dist * math.cos(rad)
            y = dist * math.sin(rad)
            points.append({'angle': angle, 'dist': dist, 'x': x, 'y': y})

    # Sort by angle (0 to 360)
    points.sort(key=lambda p: p['angle'])

    if not points:
        return None

    gaps = []

    # 2. Iterate through points to find discontinuities
    for i in range(len(points) - 1):
        p1 = points[i]
        p2 = points[i+1]

        # Calculate distance between consecutive points
        # If distance is large, it means there is a "jump" (Gap)
        # Note: This is a simplified check. A true gap is a region of "missing" points
        # or points at infinity.

        # Better Approach for Wall-Gap-Wall:
        # We look for a sequence of "Far" points between "Close" points.
        # But LiDAR often returns 0 or max_dist for open space.

        # Geometric Distance Check:
        gap_dist = math.sqrt((p1['x'] - p2['x'])**2 + (p1['y'] - p2['y'])**2)

        if gap_dist > min_gap_width and gap_dist < max_gap_width:
            # Candidate Gap Found between angle p1 and p2
            mid_angle = (p1['angle'] + p2['angle']) / 2
            mid_dist = (p1['dist'] + p2['dist']) / 2

            # Additional Check: Is there "depth" behind this gap?
            # (We assume open space means depth)

            gaps.append({
                'center_angle': mid_angle,
                'width': gap_dist,
                'distance': mid_dist
            })

    # 3. Select the Best Gap
    # Criteria: Closest to current heading (0 degrees) AND widely enough
    best_gap = None
    min_deviation = infinity

    for gap in gaps:
        # Calculate deviation from robot's current forward heading (0 deg)
        # Normalize angle difference to [-180, 180]
        dev = abs(normalize_angle(gap['center_angle'] - 0))

        # Prioritize gaps in front of us (within +/- 90 degrees)
        if dev < 90:
            if dev < min_deviation:
                min_deviation = dev
                best_gap = gap

    return best_gap # Returns {'center_angle', 'distance', ...}
```

---

## 2. Navigation Logic (State Machine)

**Global Variables:**
*   `dock_target_heading`: The heading (bearing) of the identified gap.
*   `dock_state_timer`: General purpose timer.

### State: `TASK5_SCAN`
**Trigger:** Arrival at `TASK5_APPROACH`.
**Logic:**
1.  **Stop** the robot.
2.  Perform a **360° Spot Turn** (optional, but robust).
    *   Set `left_motor = -turn_speed`, `right_motor = +turn_speed`.
    *   Accumulate LiDAR data into a persistent map if needed, or just scan once if LiDAR is 360°.
3.  Call `find_best_docking_gap(local_lidar_scan)`.
4.  **If Gap Found:**
    *   Calculate `dock_target_heading` = `current_magnetic_heading` + `gap.center_angle`.
    *   Transition to `TASK5_ALIGN`.
5.  **If No Gap:**
    *   Rotate slightly (e.g., 45°) and retry.
    *   Timeout: If no gap after 3 attempts, abort or try blindly forward.

### State: `TASK5_ALIGN`
**Trigger:** Gap found in `TASK5_SCAN`.
**Logic:**
1.  Calculate `heading_error` = `dock_target_heading` - `current_magnetic_heading`.
2.  **Spot Turn** to minimize error.
3.  **Condition:** If `abs(heading_error) < 5 degrees`, Transition to `TASK5_ENTER`.

### State: `TASK5_ENTER`
**Trigger:** Aligned with gap.
**Logic:**
1.  **Move Forward** (Positive PWM).
2.  **Reactive Gap Centering (PID):**
    *   Read LiDAR sectors: `Left (-90 to -30)` and `Right (30 to 90)`.
    *   Calculate `error` = `min_dist_right` - `min_dist_left`.
    *   `turn_correction` = `Kp * error`.
    *   `left_motor` = `base_speed + turn_correction`.
    *   `right_motor` = `base_speed - turn_correction`.
3.  **Stop Condition (Parked):**
    *   Check `front_sector (-15 to 15 degrees)`.
    *   If `min_dist_front < 1.0m` (Close to back wall/buoy), **STOP**.
    *   Transition to `TASK5_PARK`.

### State: `TASK5_PARK`
**Trigger:** Reached inside of dock.
**Logic:**
1.  **Stop Motors** (PWM 1500).
2.  Start `dock_state_timer`.
3.  **Wait** for 5 seconds (Simulate parking/docking operation).
4.  Transition to `TASK5_EXIT`.

### State: `TASK5_EXIT`
**Trigger:** Parking timer expired.
**Logic:**
1.  **Reverse** (Negative PWM).
    *   `left_motor` = `1500 - reverse_speed`.
    *   `right_motor` = `1500 - reverse_speed`.
2.  **Guidance:**
    *   Use the same `Right - Left` error, but invert correction for reversing?
    *   Simpler: Just reverse straight for X seconds or until `rear_clearance > 3.0m`.
3.  **Return to Start:**
    *   Once clear of the dock, turn towards `T5_DOCK_APPROACH` GPS coordinate.
    *   Navigate to it.
4.  **Completion:**
    *   Once at `T5_DOCK_APPROACH`, Transition to `TASK1_RETURN` (Home).

---

## 3. Implementation Details for `IDA1.py`

*   **Helper Function:** Add `normalize_angle` to `navigasyon.py` or `utilities.py` if not exists.
*   **LiDAR Parsing:** Ensure `local_lidar_scan` is fresh. The existing `process_lidar_sectors` function is useful for `TASK5_ENTER`, but `TASK5_SCAN` needs the raw list.
*   **Motor Control:** Reuse `controller.set_servo`.
*   **Failsafes:**
    *   If `front_dist` is suddenly very close during `ENTER`, trigger Emergency Stop.
    *   If Gap is lost during `ALIGN`, re-scan.
