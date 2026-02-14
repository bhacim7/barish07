import math
import numpy as np
import config as cfg

def polar_to_cartesian(scan_data, max_dist=10.0):
    """
    Converts Lidar scan data [(quality, angle_deg, dist_mm), ...] to Cartesian points [(x, y), ...].
    Robot Frame: X=Forward, Y=Left.
    Assumes Lidar Angle 0 is Forward.
    Adjust angle convention to match IDA1.py:
    IDA1.py uses: global_angle = robot_yaw - angle_rad.
    This implies Lidar angle increases Clockwise? Or simply inverted.
    We will strictly use standard math:
    If Lidar 0 is Front, and 90 is Right (CW), then Standard Angle = -90.
    """
    points = []
    for quality, angle_deg, dist_mm in scan_data:
        dist_m = dist_mm / 1000.0

        if dist_m <= 0.05 or dist_m > max_dist:
            continue

        # Normalize angle to -180..180
        if angle_deg > 180:
            angle_deg -= 360

        # Convert to Radians
        # Assumption based on IDA1.py logic: Angle is likely CW or requires inversion
        # To get standard X=Forward, Y=Left frame from typical Lidar (CW):
        # angle_rad = -math.radians(angle_deg)

        # Let's stick to the raw projection used in IDA1 mapping, but local.
        # obstacle_x = dist * cos(-angle)
        # obstacle_y = dist * sin(-angle)

        angle_rad = math.radians(angle_deg)

        # NOTE: If Lidar turns CW, we negate angle to get CCW (Standard Math)
        # Using -angle_rad to be consistent with IDA1.py's `global_angle = robot_yaw - angle`
        theta = -angle_rad

        x = dist_m * math.cos(theta)
        y = dist_m * math.sin(theta)
        points.append((x, y))

    return points

def point_line_distance(point, start, end):
    """Calculates perpendicular distance from point to line segment (start-end)."""
    if np.array_equal(start, end):
        return np.linalg.norm(point - start)

    numerator = np.abs(np.cross(end - start, start - point))
    denominator = np.linalg.norm(end - start)
    return numerator / denominator

def ramer_douglas_peucker(points, epsilon):
    """
    Simplifies a curve of points into a polyline.
    points: List of (x,y) arrays.
    epsilon: Maximum distance threshold.
    """
    if len(points) < 3:
        return [points[0], points[-1]] if len(points) > 1 else points

    dmax = 0
    index = 0
    end = len(points) - 1

    start_pt = np.array(points[0])
    end_pt = np.array(points[-1])

    for i in range(1, end):
        d = point_line_distance(np.array(points[i]), start_pt, end_pt)
        if d > dmax:
            index = i
            dmax = d

    if dmax > epsilon:
        # Recursive call
        rec_results1 = ramer_douglas_peucker(points[:index+1], epsilon)
        rec_results2 = ramer_douglas_peucker(points[index:], epsilon)

        # Build the result list
        return rec_results1[:-1] + rec_results2
    else:
        return [points[0], points[-1]]

def get_segments_from_points(points, gap_threshold=0.5, line_epsilon=0.1):
    """
    Groups points into continuous clusters (based on gap_threshold)
    and then fits lines to those clusters using RDP.
    """
    if not points:
        return []

    clusters = []
    current_cluster = [points[0]]

    for i in range(1, len(points)):
        p1 = np.array(points[i-1])
        p2 = np.array(points[i])
        dist = np.linalg.norm(p1 - p2)

        if dist < gap_threshold:
            current_cluster.append(points[i])
        else:
            if len(current_cluster) > 2:
                clusters.append(current_cluster)
            current_cluster = [points[i]]

    if len(current_cluster) > 2:
        clusters.append(current_cluster)

    # Fit lines to clusters
    segments = []
    for cluster in clusters:
        simplified = ramer_douglas_peucker(cluster, line_epsilon)
        # Convert simplified points to segments (start, end)
        for j in range(len(simplified) - 1):
            segments.append((np.array(simplified[j]), np.array(simplified[j+1])))

    return segments

def normalize_angle(rad):
    return (rad + math.pi) % (2 * math.pi) - math.pi

def detect_dock_u_shape(scan_data):
    """
    Detects a U-shaped dock entrance in the Lidar scan.
    Returns: (found (bool), center_local (x,y), bearing_deg (relative))
    """
    # 1. Convert to Cartesian
    raw_points = polar_to_cartesian(scan_data)
    if not raw_points:
        return False, None, None

    # 2. Extract Segments
    # Use config values or defaults
    tolerance = getattr(cfg, 'DOCK_TOLERANCE_M', 0.5)
    dock_w = getattr(cfg, 'DOCK_WIDTH_M', 3.0)
    dock_d = getattr(cfg, 'DOCK_DEPTH_M', 2.0)

    segments = get_segments_from_points(raw_points, gap_threshold=0.8, line_epsilon=0.15)

    if len(segments) < 3:
        return False, None, None

    # 3. Search for U-Shape (Side - Back - Side)
    # Ideally: S1 (Side) -> S2 (Back) -> S3 (Side)
    # The angle between S1-S2 and S2-S3 should be roughly 90 deg.

    for i in range(len(segments) - 2):
        s1_start, s1_end = segments[i]
        s2_start, s2_end = segments[i+1]
        s3_start, s3_end = segments[i+2]

        # Check connectivity (Points should be close)
        gap1 = np.linalg.norm(s1_end - s2_start)
        gap2 = np.linalg.norm(s2_end - s3_start)

        if gap1 > 1.0 or gap2 > 1.0:
            continue

        # Vector calc
        v1 = s1_end - s1_start
        v2 = s2_end - s2_start
        v3 = s3_end - s3_start

        l1 = np.linalg.norm(v1)
        l2 = np.linalg.norm(v2)
        l3 = np.linalg.norm(v3)

        # Check Lengths
        # Back wall (S2) should be roughly dock_w
        if abs(l2 - dock_w) > tolerance:
            continue

        # Side walls (S1, S3) should be significant (at least 0.5m)
        if l1 < 0.5 or l3 < 0.5:
            continue

        # Check Angles (Dot Product)
        # v1.v2 should be near 0 (Perpendicular)
        # v2.v3 should be near 0

        # Normalize vectors
        nv1 = v1 / l1
        nv2 = v2 / l2
        nv3 = v3 / l3

        dot1 = np.dot(nv1, nv2) # Cos(angle)
        dot2 = np.dot(nv2, nv3)

        # Threshold for perpendicularity (cos(90) = 0). Allow deviation (+- 30 deg -> cos(60)=0.5)
        # We need to be careful about direction.
        # A U-shape implies turns in the SAME winding order (Left-Left or Right-Right).
        # Cross product 2D (determinant) helps check winding.

        cross1 = np.cross(nv1, nv2) # z-component
        cross2 = np.cross(nv2, nv3)

        # Both turns should be roughly 90 degrees in the same direction
        # If cross1 and cross2 have same sign, they turn the same way.
        if cross1 * cross2 < 0:
            continue # Zig-zag, not U-shape

        if abs(dot1) > 0.5 or abs(dot2) > 0.5: # Angle must be between 60 and 120
            continue

        # GEOMETRY CONFIRMED: Found a U-Shape candidate.

        # Check "Recess" property:
        # The center of S2 (Back) should be further away than the line connecting S1_start and S3_end?
        # Or simpler: The midpoint of S2 is the dock "sweet spot".

        mid_s2 = (s2_start + s2_end) / 2.0

        # Calculate Approach Vector (Normal to S2, pointing OUT of the dock)
        # Normal is (-dy, dx) or (dy, -dx)
        # We want the one pointing towards the robot (0,0) usually, OR pointing away from the back wall.
        # Vector v2 is along the back wall.
        dx, dy = v2
        normal = np.array([-dy, dx])

        # Determine correct direction for normal (should point towards the opening)
        # Center of opening is roughly between s1_start and s3_end
        opening_center = (s1_start + s3_end) / 2.0
        vec_to_opening = opening_center - mid_s2

        if np.dot(normal, vec_to_opening) < 0:
            normal = -normal

        normal = normal / np.linalg.norm(normal)

        # Target Point: Slightly in front of the back wall (e.g. 1.0m) to ensure we enter fully
        # or exactly the center of the gap.
        # User requested: "Identify the docking station entrance"
        # Let's return the CENTER of the entrance (between S1 and S3) and the bearing to it.

        entrance_point = (s1_start + s3_end) / 2.0 # Approximation using start of walls
        # Better: Midpoint of S2 + Normal * Depth/2?
        # Actually, let's target 1.5 meter in front of the back wall.

        target_local = mid_s2 + (normal * (dock_d / 2.0))

        bearing_rad = math.atan2(target_local[1], target_local[0])
        bearing_deg = math.degrees(bearing_rad)

        # Convert bearing to 0..360 if needed, but relative -180..180 is fine for control.

        # Debug info
        # print(f"Dock Found! Back Wall Len: {l2:.2f}, Width Err: {abs(l2-dock_w):.2f}")

        return True, (target_local[0], target_local[1]), bearing_deg

    return False, None, None
