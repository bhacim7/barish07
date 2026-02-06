import math

class AlphaFilter:
    """
    # Initialize the magnetic heading filter
    magnetic_filter = AlphaFilter(alpha=0.1)  # Lower alpha means more smoothing
    """
    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.filtered_x = None
        self.filtered_y = None

    def update(self, new_heading):
        # Convert angle to unit circle representation
        new_x = math.cos(math.radians(new_heading))
        new_y = math.sin(math.radians(new_heading))

        if self.filtered_x is None or self.filtered_y is None:
            self.filtered_x = new_x
            self.filtered_y = new_y
        else:
            # Apply low-pass filter in Cartesian coordinates
            self.filtered_x = self.alpha * new_x + (1 - self.alpha) * self.filtered_x
            self.filtered_y = self.alpha * new_y + (1 - self.alpha) * self.filtered_y

        # Convert back to degrees
        filtered_heading = math.degrees(math.atan2(self.filtered_y, self.filtered_x))
        if filtered_heading < 0:
            filtered_heading += 360  # Ensure range is 0-360°

        return filtered_heading

class KalmanFilter:
    """
    # Initialize Kalman filter for magnetic heading
    magnetic_filter = KalmanFilter(process_variance=1e-3, measurement_variance=1e-1)
    """
    def __init__(self, process_variance=1e-3, measurement_variance=1e-1):
        self.process_variance = process_variance  # Process noise covariance
        self.measurement_variance = measurement_variance  # Measurement noise covariance
        self.x_estimate = None  # Filtered cos(heading)
        self.y_estimate = None  # Filtered sin(heading)
        self.error_covariance = 1.0  # Initial uncertainty

    def update(self, angle):
        # Convert angle to unit circle representation
        x_meas = math.cos(math.radians(angle))
        y_meas = math.sin(math.radians(angle))

        if self.x_estimate is None or self.y_estimate is None:
            self.x_estimate, self.y_estimate = x_meas, y_meas  # Initialize filter
            return angle

        # Prediction step
        predicted_x = self.x_estimate
        predicted_y = self.y_estimate
        predicted_error_covariance = self.error_covariance + self.process_variance

        # Kalman gain calculation
        kalman_gain = predicted_error_covariance / (predicted_error_covariance + self.measurement_variance)

        # Update step
        self.x_estimate += kalman_gain * (x_meas - predicted_x)
        self.y_estimate += kalman_gain * (y_meas - predicted_y)
        self.error_covariance = (1 - kalman_gain) * predicted_error_covariance

        # Normalize the vector to maintain unit circle representation
        norm = math.sqrt(self.x_estimate ** 2 + self.y_estimate ** 2)
        self.x_estimate /= norm
        self.y_estimate /= norm

        # Convert back to angle
        filtered_angle = math.degrees(math.atan2(self.y_estimate, self.x_estimate))
        return filtered_angle % 360  # Ensure it stays in the 0-360° range