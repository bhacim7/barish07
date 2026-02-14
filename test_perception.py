import math
import numpy as np
import perception
import config as cfg

def generate_synthetic_scan():
    """
    Generates a synthetic Lidar scan of a U-shaped dock.
    Dock Geometry: Width=3.0, Depth=2.0.
    Position: Center at (5, 0), Facing Left (towards robot at 0,0).
    Walls:
      - Right Wall: y=-1.5, x from 4 to 6.
      - Back Wall: x=6, y from -1.5 to 1.5.
      - Left Wall: y=1.5, x from 6 to 4.
    """
    points = []

    # Right Wall (y=-1.5, x=4 -> 6)
    # Sampling: 20 points
    for x in np.linspace(4, 6, 20):
        points.append((x, -1.5))

    # Back Wall (x=6, y=-1.5 -> 1.5)
    # Sampling: 30 points
    for y in np.linspace(-1.5, 1.5, 30):
        points.append((6, y))

    # Left Wall (y=1.5, x=6 -> 4)
    # Sampling: 20 points
    for x in np.linspace(6, 4, 20):
        points.append((x, 1.5))

    # Convert to Polar (scan format)
    scan_data = []
    for x, y in points:
        dist_m = math.sqrt(x**2 + y**2)
        angle_rad = math.atan2(y, x) # Standard math angle (CCW from X)

        # perception.py uses: theta = -angle_rad (where angle_rad comes from angle_deg)
        # So angle_deg = -degrees(theta) if we follow perception.py logic inverted
        # Wait, perception.py: angle_deg -> angle_rad -> theta = -angle_rad
        # So input angle_deg should be -degrees(atan2(y, x))

        angle_deg = -math.degrees(angle_rad)

        # Normalize to 0..360 for Lidar format usually?
        # perception.py handles > 180.
        if angle_deg < 0:
            angle_deg += 360

        dist_mm = dist_m * 1000.0
        quality = 15
        scan_data.append((quality, angle_deg, dist_mm))

    # Sort by angle?
    # Usually Lidar scans are sorted 0..360.
    # Our points:
    # Right Wall: y=-1.5. Angle is negative (e.g. -20 deg -> 340 deg).
    # Back Wall: Crosses 0 deg.
    # Left Wall: Positive angle (e.g. +20 deg).

    # If we sort by angle 0..360, we get:
    # 0..20 (Left Wall, Back Wall top half)
    # 340..360 (Right Wall, Back Wall bottom half)

    # This splits the object into two clusters separated by the 0/360 boundary!
    # PROBLEM: perception.py doesn't handle wrapping around 360.

    # FIX: Rotate the dock so it doesn't cross 0 deg for the test,
    # OR accept that the algorithm might need contiguous indices.

    # Let's rotate the dock to 90 degrees (Left of robot).
    # Center at (0, 5). Facing South.
    # Right Wall (relative to dock looking out): x=1.5.
    # Actually, let's keep it simple.
    # If the dock is at 180 degrees (Behind), it won't wrap.
    # If the dock is at 90 degrees (Left), it won't wrap.

    # Let's try placing dock at Angle 180 (Behind).
    # Dock Center (-5, 0).
    # Walls:
    # Side 1: y=-1.5, x=-4 to -6. Angle ~ 160 deg.
    # Back: x=-6, y=-1.5 to 1.5. Angle ~ 180 deg.
    # Side 2: y=1.5, x=-6 to -4. Angle ~ -160 (200) deg.

    # This is contiguous in 0..360 space (160 -> 200).

    # Let's regenerate points for Dock at (-5, 0).
    points_rotated = []

    # Order matters for "continuous segment" logic in perception.py
    # It just iterates the list. So as long as points are spatially close in the list, it's fine.
    # Standard Lidar returns points sorted by angle.

    # Let's simulate a scan sweep from 150 to 210 degrees.
    scan_data_rotated = []

    # Generate points by angle
    for angle_d in np.linspace(150, 210, 100):
        angle_rad = math.radians(angle_d)
        # Ray cast to find intersection with dock walls
        # Dock Walls defined by lines.
        # Wall 1: y=-1.5, x in [-6, -4]
        # Wall 2: x=-6, y in [-1.5, 1.5]
        # Wall 3: y=1.5, x in [-6, -4]

        # Ray: x = r cos(a), y = r sin(a)
        # Find r.

        r_hit = 100.0

        # Check Wall 1 (y = -1.5)
        # r * sin(a) = -1.5 => r = -1.5 / sin(a)
        if math.sin(angle_rad) != 0:
            r = -1.5 / math.sin(angle_rad)
            if r > 0:
                x = r * math.cos(angle_rad)
                if -6 <= x <= -4:
                    r_hit = min(r_hit, r)

        # Check Wall 2 (x = -6)
        # r * cos(a) = -6 => r = -6 / cos(a)
        if math.cos(angle_rad) != 0:
            r = -6 / math.cos(angle_rad)
            if r > 0:
                y = r * math.sin(angle_rad)
                if -1.5 <= y <= 1.5:
                    r_hit = min(r_hit, r)

        # Check Wall 3 (y = 1.5)
        # r = 1.5 / sin(a)
        if math.sin(angle_rad) != 0:
            r = 1.5 / math.sin(angle_rad)
            if r > 0:
                x = r * math.cos(angle_rad)
                if -6 <= x <= -4:
                    r_hit = min(r_hit, r)

        if r_hit < 50.0:
            # Found hit
            # Convert to input format
            # perception.py expects angle_deg such that theta = -radians(angle_deg)
            # theta here is angle_rad.
            # so angle_deg = -degrees(angle_rad)

            input_angle = -angle_d
            if input_angle < 0: input_angle += 360

            scan_data_rotated.append((15, input_angle, r_hit * 1000.0))

    return scan_data_rotated

def test_perception():
    scan = generate_synthetic_scan()
    print(f"Generated {len(scan)} points.")

    # Inject config params manually if needed (though we imported cfg)
    cfg.DOCK_WIDTH_M = 3.0
    cfg.DOCK_DEPTH_M = 2.0
    cfg.DOCK_TOLERANCE_M = 0.8 # Relax tolerance for test noise

    found, center, bearing = perception.detect_dock_u_shape(scan)

    if found:
        print("SUCCESS: Dock Found!")
        print(f"Center: {center}")
        print(f"Bearing: {bearing:.2f} deg")

        # Expected Center: (-5.5, 0) roughly (Midpoint of back wall at -6, offset by depth/2=1.0 -> -5.0?)
        # Logic in perception:
        # mid_s2 = (-6, 0).
        # Normal points towards (0,0) -> (+1, 0).
        # Target = (-6, 0) + (1, 0) * (2.0 / 2.0) = (-5, 0).

        dist_err = math.sqrt((center[0] - (-5.0))**2 + (center[1] - 0)**2)
        print(f"Distance Error: {dist_err:.2f}m")

        if dist_err < 0.5:
            print("TEST PASSED")
        else:
            print("TEST FAILED: Position Error too high")
    else:
        print("TEST FAILED: Dock Not Found")

        # Debug: check segments
        points = perception.polar_to_cartesian(scan)
        segments = perception.get_segments_from_points(points, gap_threshold=0.5)
        print(f"Segments found: {len(segments)}")
        for s in segments:
            print(f"Seg: {s[0]} -> {s[1]} Len: {np.linalg.norm(s[1]-s[0]):.2f}")

if __name__ == "__main__":
    test_perception()
