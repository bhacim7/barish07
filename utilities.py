import math
import threading, queue, os, datetime, cv2
import time


def nfloat(x):
    try:
        if x is None:
            return None
        xf = float(x)
        return xf if math.isfinite(xf) else None
    except Exception:
        return None


def nint(x):
    try:
        if x is None:
            return None
        xf = float(x)
        if not math.isfinite(xf):
            return None
        return int(round(xf))
    except Exception:
        return None


def _init_video_writer(width, height, fps_hint=20):
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    # ZED FPS’i makul aralığa sıkıştır
    try:
        fps = int(round(fps_hint))
    except:
        fps = 20
    fps = max(5, min(60, fps))
    ts = time.strftime("%Y%m%d_%H%M%S")
    name = f"manual_mode_{ts}.mp4"
    vw = cv2.VideoWriter(name, fourcc, fps, (width, height))
    return vw, name


def _close_video_writer():
    global video_writer, video_filename
    if video_writer is not None:
        video_writer.release()
        print(f"[VIDEO] Kaydedildi: {video_filename}")
        video_writer = None
        video_filename = None


# --- Fusion mantığı: açılarla çalışırken düzgün wrap/ortalamaya dikkat et ---
def wrap_to_pi(angle):
    """angle in radians -> [-pi, pi]"""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def angle_diff(a, b):
    """smallest signed difference a-b in radians"""
    return wrap_to_pi(a - b)


def angle_weighted_mean(a, b, w):  # w = weight for b (0..1)
    # compute mean on unit circle to avoid wrap issues
    xa, ya = math.cos(a), math.sin(a)
    xb, yb = math.cos(b), math.sin(b)
    x = (1 - w) * xa + w * xb
    y = (1 - w) * ya + w * yb
    return math.atan2(y, x)


class EmergencyShutdown(Exception):
    """Yer kontrolden acil kapatma isteği geldiğinde fırlatılır."""
    pass


class AsyncVideoWriter:
    def __init__(self, path, fps=20.0, max_queue=120):
        self.path = path
        self.fps = float(fps)
        self.q = queue.Queue(maxsize=max_queue)
        self.stop_event = threading.Event()
        self.t = threading.Thread(target=self._run, daemon=True)
        self.writer = None
        self.size = None  # (W,H)
        self.fourcc_tried = False

    def start(self):
        self.t.start()

    def _open_writer(self, frame_bgr):
        h, w = frame_bgr.shape[:2]
        self.size = (w, h)

        # Codec fallback zinciri (sırasıyla dene)
        fourcc_list = [
            ("mp4v", ".mp4"),
            ("XVID", ".avi"),
            ("avc1", ".mp4"),  # OpenCV derlemen x264 destekliyse
        ]
        # Dosya uzantısını codec’e göre düzelt (gerekirse)
        base, ext = os.path.splitext(self.path)
        for cc, want_ext in fourcc_list:
            out_path = base + want_ext
            fourcc = cv2.VideoWriter_fourcc(*cc)
            wtr = cv2.VideoWriter(out_path, fourcc, self.fps, self.size)
            if wtr.isOpened():
                self.writer = wtr
                self.path = out_path
                print(f"[INFO] VideoWriter opened: {out_path} (FOURCC={cc}, {self.size}, {self.fps}fps)")
                return True
            else:
                wtr.release()
        print("[ERROR] Hiçbir FOURCC ile VideoWriter açılamadı.")
        return False

    def _to_bgr(self, frame):
        # RGBA/BGRA → BGR / GRAY → BGR dönüştür
        if frame.ndim == 2:  # gri
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if frame.shape[2] == 4:  # BGRA/RGBA
            # ZED: BGRA geliyor → BGR
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        return frame  # zaten BGR

    def _run(self):
        while not self.stop_event.is_set():
            try:
                frame = self.q.get(timeout=0.2)
            except queue.Empty:
                continue
            if frame is None:
                break

            frame = self._to_bgr(frame)
            if self.writer is None:
                if not self._open_writer(frame):
                    # writer açılamazsa kuyruğu tüket ve çık
                    while not self.q.empty():
                        _ = self.q.get_nowait()
                    return

            # Boyut uyuşmuyorsa resize et
            if (frame.shape[1], frame.shape[0]) != self.size:
                frame = cv2.resize(frame, self.size, interpolation=cv2.INTER_LINEAR)

            try:
                self.writer.write(frame)
            except Exception as e:
                print(f"[WARN] video write error: {e}")

        # Kuyrukta kalanları boşalt
        while not self.q.empty():
            frame = self.q.get_nowait()
            if frame is None:
                break
            frame = self._to_bgr(frame)
            if (frame.shape[1], frame.shape[0]) != self.size:
                frame = cv2.resize(frame, self.size)
            try:
                if self.writer is not None:
                    self.writer.write(frame)
            except:
                pass

        if self.writer is not None:
            self.writer.release()
            print(f"[INFO] VideoWriter closed: {self.path}")

    def enqueue(self, frame):
        # Non-blocking, doluysa eskiyi bırak
        if self.stop_event.is_set():
            return
        try:
            self.q.put_nowait(frame)
        except queue.Full:
            try:
                _ = self.q.get_nowait()
            except queue.Empty:
                pass
            try:
                self.q.put_nowait(frame)
            except:
                pass

    def stop(self):
        self.stop_event.set()
        try:
            self.q.put_nowait(None)
        except:
            pass
        self.t.join(timeout=3.0)


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
