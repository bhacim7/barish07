# Navigation Failsafe Documentation

This document explains the specific log messages and behaviors triggered when the Autonomous System cannot find a valid path (A* failure) or loses navigation capability.

## Failsafe Logic Flow

When the `planner` returns no path (`current_path` is empty), the system enters a timed failsafe state machine.

### 1. `[FAILSAFE] BLIND GPS DRIVE (Err: 15.1)`

*   **Trigger Condition:**
    *   A* Planner failed to find a path.
    *   Time since failure < **3.0 seconds** (Config: `BLIND_DRIVE_SECONDS`).
    *   A valid target bearing (GPS or Local) is available.
    *   **LIDAR Safety Check:** The front center distance (`center_d`) is greater than **0.7 meters** (Config: `BLIND_DRIVE_SAFE_DIST`).
*   **Meaning:** "I can't see a path on the map (maybe due to noise), but my nose is clear and I know where the GPS point is. I will drive blindly towards it for a few seconds."
*   **Action:** Uses a simple P-controller to steer towards the target bearing without obstacle avoidance (except the 0.7m cutoff).

---

### 2. `[PLANNER] Yol Yok -> MICRO ESCAPE (Minik Adımlar)`

*   **Trigger Condition:**
    *   A* Planner has failed for more than **5.0 seconds** (Blind Drive Time + 2s wait).
    *   Time since failure < **8.0 seconds**.
*   **Meaning:** "I have been stuck for a while. I will try to inch forward slowly to change my lidar perspective and hopefully clear the map noise."
*   **Action:**
    *   Moves forward at very low speed (`CREEP_SPEED`).
    *   Applies "Corridor centering" logic: pushes away from nearby walls (Left/Right) to avoid scraping.
    *   **Safety:** Stops immediately if `center_danger` (wall directly ahead) is detected.

---

### 3. `[ACİL] PLANNER ÇÖKTÜ -> YERİNDE DÖNÜŞ (FAIL SAFE)`

*   **Trigger Condition:**
    *   A* Planner has failed for more than **8.0 seconds**.
*   **Meaning:** "I am completely stuck. Moving forward is impossible or hasn't worked. I need to drastically change my orientation to find a new path."
*   **Action:**
    *   Stops forward motion.
    *   Executes a rotation (Spot Turn) to the left to scan the environment and potentially unblock the costmap.
    *   This is the final fallback before the system effectively gives up on the current trajectory.
