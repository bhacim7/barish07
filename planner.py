import math
import heapq
import cv2
import numpy as np


class PathPlanner:
    def __init__(self):
        self.prev_error = 0.0

    # =============================================================================
    #  A* (A-STAR) ALGORİTMASI ÇEKİRDEĞİ
    # =============================================================================

    def heuristic(self, a, b, weight=1.5):
        # Euclidean Mesafe (Kuş uçuşu) * Ağırlık
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) * weight


    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]  # Tersten çevir (Başlangıç -> Bitiş)


    def a_star_search(self, grid, start, goal):
        """
        grid: 0=Engel, 255=Yol (Küçültülmüş Harita)
        start, goal: (x, y) tuple - Grid koordinatları
        """
        h, w = grid.shape

        # 8 Yönlü Hareket (Sağ, Sol, Yukarı, Aşağı + Çaprazlar)
        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        # Maliyetler: Düz=1.0, Çapraz=1.414
        costs = [1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414]

        open_set = []
        # (F_Score, (x, y))
        heapq.heappush(open_set, (0, start))

        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            # En düşük F skoruna sahip kareyi seç
            current = heapq.heappop(open_set)[1]

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for i, (dx, dy) in enumerate(neighbors):
                neighbor = (current[0] + dx, current[1] + dy)

                # Harita sınır kontrolü
                if 0 <= neighbor[0] < w and 0 <= neighbor[1] < h:
                    # Engel Kontrolü (0 = Engel)
                    # Not: Grid üzerinde 0 tamamen engeldir.
                    if grid[neighbor[1], neighbor[0]] == 0:
                        continue

                    tentative_g_score = g_score[current] + costs[i]

                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f = tentative_g_score + self.heuristic(neighbor, goal)
                        f_score[neighbor] = f
                        heapq.heappush(open_set, (f, neighbor))

        return None  # Yol bulunamadı


    # =============================================================================
    #  ANA PLANLAYICI FONKSİYONU (PATRON İÇİN ARAYÜZ)
    # =============================================================================

    def _smooth_path(self, path_world, weight_data=0.5, weight_smooth=0.1, tolerance=0.00001):
        """
        Basit bir yol yumuşatma algoritması (Gradient Descent benzeri).
        Grid köşeli olduğu için robot zikzak çizebilir, bu fonksiyon yolu yumuşatır.
        """
        if not path_world:
            return path_world

        new_path = list(path_world)  # Derin kopya
        change = tolerance
        while change >= tolerance:
            change = 0.0
            for i in range(1, len(path_world) - 1):
                for j in range(2): # x ve y için
                    aux = new_path[i][j]
                    new_path[i] = list(new_path[i]) # Tuple -> List
                    new_path[i][j] += weight_data * (path_world[i][j] - new_path[i][j]) + \
                                      weight_smooth * (new_path[i - 1][j] + new_path[i + 1][j] - 2.0 * new_path[i][j])
                    new_path[i] = tuple(new_path[i]) # List -> Tuple
                    change += abs(aux - new_path[i][j])

        return new_path

    def find_nearest_free_point(self, grid, point, search_radius=5):
        """
        Eğer nokta engelse (0), çevresindeki en yakın serbest noktayı (255) bulur.
        point: (x, y) grid koordinatı
        """
        h, w = grid.shape
        px, py = point

        # Zaten boşsa direkt dön
        if 0 <= px < w and 0 <= py < h and grid[py, px] != 0:
            return point

        # Spiral şeklinde ara
        for r in range(1, search_radius + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    # Sadece çemberin kenarlarına bak (içine zaten baktık)
                    if abs(dx) != r and abs(dy) != r:
                        continue

                    nx, ny = px + dx, py + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        if grid[ny, nx] != 0:
                            return (nx, ny)
        return None

    def get_path_plan(self, start_world, goal_world, high_res_map, costmap_center_m, costmap_res, costmap_size):
        """
        Bu fonksiyon ana koddan (deneme.py) çağrılır.
        1. Haritayı küçültür (Downsampling).
        2. A* algoritmasını çalıştırır.
        3. Bulunan yolu metre cinsinden dünya koordinatına çevirir.
        """
        if high_res_map is None: return None

        # 1. KÜÇÜLTME (Downsampling) - Performans ve Güvenlik
        # Engel Kaybını Önleme: Önce Erosion yap (Siyahları genişlet)
        # Böylece küçültürken 1 piksel olan engel kaybolmaz.
        kernel = np.ones((3, 3), np.uint8)
        eroded_map = cv2.erode(high_res_map, kernel, iterations=1)

        # 800x800 haritayı 100x100 (1/8) oranında küçültüyoruz.
        SCALE = 0.125
        low_res_grid = cv2.resize(eroded_map, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_NEAREST)

        # 2. Koordinat Dönüşümü (Dünya -> Grid Pikselleri)
        cw, ch = costmap_size[0] // 2, costmap_size[1] // 2

        # Yerel world_to_pixel fonksiyonu (Bağımlılığı kesmek için)
        def to_grid_pixel(wx, wy):
            # Önce High-Res Pikseli bul
            dx_m = wx - costmap_center_m[0]
            dy_m = wy - costmap_center_m[1]
            px_hi = int(cw + (dx_m / costmap_res))
            py_hi = int(ch - (dy_m / costmap_res))

            # Sonra Low-Res (Grid) Pikseline çevir
            px_lo = int(px_hi * SCALE)
            py_lo = int(py_hi * SCALE)
            return px_lo, py_lo

        start_grid = to_grid_pixel(start_world[0], start_world[1])
        goal_grid = to_grid_pixel(goal_world[0], goal_world[1])

        # Grid sınır kontrolü (Taşmaları önle)
        h_grid, w_grid = low_res_grid.shape
        start_grid = (min(max(0, start_grid[0]), w_grid - 1), min(max(0, start_grid[1]), h_grid - 1))
        goal_grid = (min(max(0, goal_grid[0]), w_grid - 1), min(max(0, goal_grid[1]), h_grid - 1))

        # A* Çalıştırmadan Önce: Başlangıç veya Hedef duvarın içinde mi?
        # Kurtarma Modu: En yakın boş noktayı bul.
        if low_res_grid[start_grid[1], start_grid[0]] == 0:
            # print("[PLAN] Robot duvarın içinde! Kurtarma noktası aranıyor...")
            start_grid = self.find_nearest_free_point(low_res_grid, start_grid)
            if start_grid is None:
                 return None

        if low_res_grid[goal_grid[1], goal_grid[0]] == 0:
            # print("[PLAN] Hedef duvarın içinde! Yakın nokta aranıyor...")
            goal_grid = self.find_nearest_free_point(low_res_grid, goal_grid)
            if goal_grid is None:
                return None

        # 3. A* ÇALIŞTIR
        path_grid = self.a_star_search(low_res_grid, start_grid, goal_grid)

        if path_grid is None: return None

        # 4. YOLU GERİ ÇEVİR (Grid -> Dünya Metre)
        final_path_world = []

        for p in path_grid:
            # Grid -> High Res Px
            hx = p[0] / SCALE
            hy = p[1] / SCALE

            # High Res Px -> Dünya Metre (Ters işlem)
            wx = (hx - cw) * costmap_res + costmap_center_m[0]
            wy = (ch - hy) * costmap_res + costmap_center_m[1]

            final_path_world.append((wx, wy))

        # 5. YOL YUMUŞATMA (Smoothing)
        final_path_world = self._smooth_path(final_path_world)

        return final_path_world


    # =============================================================================
    #  PURE PURSUIT (MOTOR KONTROL)
    # =============================================================================

    def pure_pursuit_control(self, robot_x, robot_y, robot_yaw, path, base_speed=1500, max_pwm_change=60, current_speed_mps=None):
        """
        Verilen yolu takip etmek için gereken motor PWM değerlerini hesaplar.
        PID Kontrolü eklenmiştir.
        """
        if not path or len(path) < 2:
            return 1500, 1500, None, 0.0

        # 1. Lookahead (Tavşan) Noktasını Bul

        # --- DYNAMIC LOOKAHEAD (HIZA BAĞLI) ---
        if current_speed_mps is not None:
             # Hız arttıkça daha ileriye bak
             # Örnek: Dururken 1.0m, 2 m/s hızda 2.0m
             LOOKAHEAD_DIST = 1.0 + (0.5 * abs(current_speed_mps))
             LOOKAHEAD_DIST = max(0.8, min(3.0, LOOKAHEAD_DIST))
        else:
             # Eskisi gibi sabit/açısal
             LOOKAHEAD_DIST = 1.5

             # Robotun burnu ile yolun gidişatı arasında ne kadar fark var?
             check_idx = min(len(path) - 1, 5)  # 5 nokta ilerisine bak
             ref_p = path[check_idx]
             desired_angle = math.atan2(ref_p[1] - robot_y, ref_p[0] - robot_x)
             angle_diff = abs(desired_angle - robot_yaw)
             angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
             angle_diff = abs(math.degrees(angle_diff))

             if angle_diff > 20:
                 LOOKAHEAD_DIST = 0.8

        # ----------------------------------------------------
        target_point = path[-1]  # Varsayılan: Yolun sonu

        # Yolu tersten tara (veya baştan), robottan 1.2m uzaktaki ilk noktayı bul
        for p in path:
            dist = math.sqrt((p[0] - robot_x) ** 2 + (p[1] - robot_y) ** 2)
            if dist > LOOKAHEAD_DIST:
                target_point = p
                break

        # 2. Açıyı Hesapla
        target_angle = math.atan2(target_point[1] - robot_y, target_point[0] - robot_x)

        # Hata Açısı (Robotun burnu ile hedef arasındaki fark)
        alpha = target_angle - robot_yaw
        # -PI ile +PI arasına normalize et
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi

        # 3. PWM Hesapla (PID Kontrol)
        # Hata: alpha (Radyan cinsinden açı farkı)

        TURN_KP = 250.0  # Oransal (Biraz artırdık)
        TURN_KD = 50.0   # Türev (Titremeyi önler)

        # Türev hesabı (Hata değişimi)
        error_diff = alpha - self.prev_error

        turn_cmd = (alpha * TURN_KP) + (error_diff * TURN_KD)
        turn_cmd = np.clip(turn_cmd, -200, 200)

        # --- CONTINUOUS SPEED PROFILING (EĞRİLİĞE BAĞLI HIZ) ---
        # Eğrilik (Curvature) k = 2 * sin(alpha) / L
        # alpha: Hedef açısı hatası, L: Lookahead distance

        curvature = 2.0 * math.sin(alpha) / LOOKAHEAD_DIST

        # Base ek hız (Gaz)
        MAX_ADDED_SPEED = 100.0
        CURVATURE_GAIN = 1.0 # Ayarlanabilir katsayı

        # Eğrilik arttıkça hızı düşür (Sürekli fonksiyon)
        current_speed_pwm = MAX_ADDED_SPEED / (1.0 + CURVATURE_GAIN * abs(curvature))

        sol_pwm = int(base_speed + current_speed_pwm - turn_cmd)
        sag_pwm = int(base_speed + current_speed_pwm + turn_cmd)

        self.prev_error = alpha # Update prev_error

        return sol_pwm, sag_pwm, target_point, alpha
