import pyzed.sl as sl
import cv2
import numpy as np
import math
import time
import os
import select, sys
import tty
import termios
import threading
import queue
import datetime
from colorama import Fore, Back, Style, init
import serial
from ultralytics import YOLO
import supervision as sv
import json

import motorPower
import utilities as utils
import config as cfg
import navigasyon as nav
import kamera
from kamera import TimestampHandler
import telem
from telem import TelemetrySender, CommandReceiver
from headingFilter import KalmanFilter  # todo: heading filtresi testi yap
from MainSystem2 import USVController
import planner

import torch

import threading
from rplidar import RPLidar
import socket, struct, pickle
import pyaudio  # Ses kartına erişim için
import numpy as np  # FFT matematiksel işlemleri için


path_lost_time = None
# --- TASK 6 (SESLİ KOMUT) İÇİN GLOBAL DEĞİŞKENLER ---
task6_interrupt_request = None  # Eğer ses duyulursa burası 3 veya 5 olacak.
task6_detected_freq = 0  # Raporlama için duyulan frekans (örn: 600Hz)
task6_timestamp = 0  # Sinyalin alındığı zaman

task5_dock_timer = 0

# Buoy (Şamandıra) Filtreleme Ayarları
BUOY_MIN_RATIO = 0.005  # %2'den azsa gürültüdür (Su parlaması)
BUOY_MAX_RATIO = 0.35  # %35'ten fazlaysa duvardır/büyük engeldir
DEBUG_BUOY = True  # Konsola detaylı şamandıra verisi basılsın mı?

# Mesafe ve Tolerans Limitleri
BUOY_MAX_DIST = 10.0  # 10 metreden uzaktaki nesneleri Lidar ile tarama
GATE_COLOR_TOLERANCE = 5.0  # Kapı geçişinde renkler arası maks mesafe farkı

# --- YENİ: NAVİGASYON VE ROBOT FİZİĞİ ---
ROBOT_RADIUS_M = 0.45  # Robotun yarıçapı (metre) - Gövde genişliği/2 + biraz pay
INFLATION_MARGIN_M = 0.05  # Ekstra güvenlik payı (Duvara ne kadar yaklaşsın?)
# Toplam Şişirme = 0.60m -> Haritada 6 piksellik gri duvar örülecek.

# Sadece Görüntü Aktarma icin (yarısmada komut satırına alınacak)
if cfg.STREAM == True:
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect(("192.168.1.25", 5000))
    except:
        pass

# Görüntü İşleme
# torch.backends.cudnn.benchmark = True  # sabit input boyutları için hız
# --- GÜNCELLEME: YENİ DATASET VE ORIN NANO OPTİMİZASYONU ---
model_path = "/home/yarkin/roboboatIDA/roboboat/weights/best.engine"

if not os.path.exists(model_path):
    print(f"[UYARI] {model_path} bulunamadı, yolov8n kullanılıyor.")
    # Eğer engine yoksa standart modeli yükle (Test için)
    model = YOLO("yolov11n.pt")
else:
    model = YOLO(model_path)

try:
    model.to('cuda').half()  # FP16 Açık (Orin Nano için şart)
    # model.fuse()  # varsa Conv+BN birleştirir
    print("[INFO] Model FP16 modunda yüklendi.")
except Exception as e:
    print(f"Model GPU hatası: {e}")

bounding_box_annotator = sv.RoundBoxAnnotator()
label_annotator = sv.LabelAnnotator()

# OrangeCube
controller = USVController("/dev/ttyACM0", baud=57600)
print("Arming vehicle...")
# controller.arm_vehicle()
print("Vehicle armed!")
print("Setting mode...")
controller.set_mode("MANUAL")
print("Mode set!")

LIDAR_PORT_NAME = '/dev/ttyUSB0'  # NOT: Bu portu kontrol edin. OrangeCube /dev/ttyACM0'da
lidar_g = None  # Global lidar nesnesi
is_running_g = True  # Thread'i durdurmak için global bayrak
latest_lidar_scan_g = []  # En son taramayı tutan paylaşımlı değişken
lidar_lock_g = threading.Lock()
latest_lidar_timestamp_g = 0

# Telemetry
telemetry = TelemetrySender(cfg.SERIAL_PORT, cfg.SERIAL_BAUD)
cmd_queue = queue.Queue()
cmd_rx = CommandReceiver(telemetry, cmd_queue)
cmd_rx.start()
# Asenkron TX'i bir kez oluştur
tx = telem.TelemetryTx(telemetry, max_hz=10)
# Bazı implementasyonlar start() ister, varsa aç:
if hasattr(tx, "start"):
    tx.start()

# Güvenli başlangıç değerleri (NameError önlemek için)
magnetic_heading = None
magnetic_heading_state = None

# =============================================================================
#  MAPPING / SLAM SİSTEMİ (GLOBAL AYARLAR ve FONKSİYONLAR)
# =============================================================================

# Harita Ayarları
COSTMAP_SIZE_PX = (800, 800)  # 1000x1000 piksellik bir harita (Yeterli)
COSTMAP_RES_M_PER_PX = 0.10  # 1 piksel = 10 cm (Hassas harita)
COSTMAP_BG_BGR = (200, 230, 255)  # Arka plan rengi (Açık Mavi - Su gibi)
COSTMAP_OBSTACLE_BGR = (0, 0, 0)  # Engel rengi (Siyah)
COSTMAP_ROBOT_BGR = (0, 0, 255)  # Robot izi rengi (Kırmızı)

# Global Değişkenler
costmap_img = None  # Harita resmi burada tutulacak
costmap_center_m = (0, 0)  # Haritanın tam ortası Dünya'da hangi (X,Y) metresine denk geliyor?
costmap_ready = False  # Harita hazır mı?


def mapping_init(start_x=0, start_y=0):
    """
    Haritayı 'Olasılıksal Gri Tonlama' modunda başlatır.
    127: Bilinmiyor (Gri)
    0:   Boş (Beyaz)
    255: Dolu (Siyah)
    """
    global costmap_img, costmap_center_m, costmap_ready

    h, w = COSTMAP_SIZE_PX

    # TEK KANALLI (Grayscale) harita oluştur.
    # Başlangıç değeri 127 (Gri - Bilinmeyen Alan)
    costmap_img = np.full((h, w), 127, dtype=np.uint8)

    costmap_center_m = (start_x, start_y)
    costmap_ready = True
    print(f"[MAPPING] Olasılıksal Harita Başlatıldı! (Gri Modu)")


def world_to_pixel(world_x, world_y):
    """
    Dünya koordinatını (Metre X, Y) -> Harita Pikseline (px, py) çevirir.
    """
    if not costmap_ready: return None

    # Harita merkez koordinatları (Piksel cinsinden)
    cw, ch = COSTMAP_SIZE_PX[0] // 2, COSTMAP_SIZE_PX[1] // 2

    # Merkezden farkı bul (Metre)
    dx_m = world_x - costmap_center_m[0]
    dy_m = world_y - costmap_center_m[1]

    # Metreyi Piksele çevir ve merkeze ekle
    # NOT: Görüntü işlemede Y ekseni aşağı doğru artar, ama Dünyada yukarı (Kuzey/İleri) artar.
    # Bu yüzden dy'yi çıkarıyoruz (Ters çevirme).
    px = int(cw + (dx_m / COSTMAP_RES_M_PER_PX))
    py = int(ch - (dy_m / COSTMAP_RES_M_PER_PX))

    # Harita sınırları içinde mi?
    h, w = COSTMAP_SIZE_PX
    if 0 <= px < w and 0 <= py < h:
        return (px, py)
    return None


# Hız hesabı için global değişkenler
prev_yaw = 0
prev_time = 0


def mapping_update_lidar(robot_x, robot_y, robot_yaw_rad, lidar_points):
    """
    OPTIMIZE EDİLMİŞ VERSİYON:
    1. Veri Seyreltme: Her 3 noktadan sadece 1'ini işler (Hız artışı).
    2. Açı Filtresi ve Olasılıksal Güncelleme korundu.
    """
    global costmap_img
    if not costmap_ready or costmap_img is None: return

    p_robot = world_to_pixel(robot_x, robot_y)
    if not p_robot: return

    # Maskeleri oluştur (Hala gerekli ama artık daha az çizim yapacağız)
    empty_mask = np.zeros_like(costmap_img)
    occupied_mask = np.zeros_like(costmap_img)

    FREE_GAIN = getattr(cfg, 'LIDAR_FREE_GAIN', 25)
    OCCUPIED_GAIN = getattr(cfg, 'LIDAR_OCCUPIED_GAIN', 80)

    # --- OPTİMİZASYON: Adım atlayarak döngü kur (3'er 3'er atla) ---
    # Bu sayede 600 nokta yerine 200 nokta işleriz.
    for i, point in enumerate(lidar_points):

        # point yapısı: (quality, angle, distance)
        angle_deg = point[1]
        dist_mm = point[2]
        dist_m = dist_mm / 1000.0

        if dist_m < 0.1 or dist_m > 20.0: continue  # Menzil dışını at

        if dist_m > 5.0:
            pass  # İşlemeye devam et (Seyreltme yok)
        else:
            if i % 3 != 0: continue  # Atla

        # Açı Filtresi (Arka tarafı görme)
        if 135 < angle_deg < 225:
            continue

        angle_rad = math.radians(angle_deg)
        global_angle = robot_yaw_rad - angle_rad

        obstacle_x = robot_x + (dist_m * math.cos(global_angle))
        obstacle_y = robot_y + (dist_m * math.sin(global_angle))
        p_obs = world_to_pixel(obstacle_x, obstacle_y)

        if p_obs:
            try:
                # Ray Casting (Boşlukları temizle)
                cv2.line(empty_mask, p_robot, p_obs, FREE_GAIN, 1)
                # Hit Point (Duvarları koy)
                # Optimizasyon: Circle yerine tek piksel erişimi daha hızlıdır ama
                # görsellik için circle kalsın, zaten döngüyü azalttık.
                # şamandıra büyüklüğü için 2 yi artır
                cv2.circle(occupied_mask, p_obs, 2, OCCUPIED_GAIN, -1)
            except:
                pass

    # Matematiksel Birleştirme
    # 1. 'Boşluk Puanlarını' EKLE (Rengi 255'e/Beyaza doğru aç)
    # empty_mask robotun önünü aydınlatır.
    costmap_img = cv2.add(costmap_img, empty_mask)  # Beyazlaştır
    # 2. 'Doluluk Puanlarını' ÇIKAR (Rengi 0'a/Siyaha doğru karart)
    # occupied_mask duvarları koyulaştırır.
    costmap_img = cv2.subtract(costmap_img, occupied_mask)  # Siyahlaştır


#  LANDMARK (ŞAMANDIRA) HAFIZA SİSTEMİ (YENİ EKLENECEK)
# =============================================================================

# Hafızadaki şamandıraları tutacak liste
# Yapısı: { "id": int, "color": "RED"/"GREEN", "x": float, "y": float, "confidence": int, "last_seen": timestamp }
landmarks_memory = []
landmark_id_counter = 0


def update_landmark_memory(color_type, obj_world_x, obj_world_y):
    """
    Tespit edilen şamandırayı hafızaya ekler veya günceller.
    Lidar onayı almış cisimler buraya gelir.
    """
    global landmarks_memory, landmark_id_counter

    current_time = time.time()
    MATCH_THRESHOLD = 2.0  # 1 metre içindeki cisimler "aynı" kabul edilir

    found = False
    for lm in landmarks_memory:
        # Mesafe hesabı (Öklid)
        dist = math.sqrt((lm["x"] - obj_world_x) ** 2 + (lm["y"] - obj_world_y) ** 2)

        if dist < MATCH_THRESHOLD:
            # Zaten bildiğimiz bir şamandıra, konumunu güncelle (Ortalamasını al)
            # Yeni konumun %20'sini, eski konumun %80'ini al (Stabilite için)
            lm["x"] = (lm["x"] * 0.5) + (obj_world_x * 0.5)
            lm["y"] = (lm["y"] * 0.5) + (obj_world_y * 0.5)
            lm["last_seen"] = current_time
            lm["confidence"] += 1

            # Eğer rengi değiştiyse (Kamera hatası olabilir), en çok görülen renge karar verilir
            if lm["color"] != color_type:
                # Renk çatışması durumunda güveni biraz düşür
                lm["confidence"] = max(1, lm["confidence"] - 2)
                if lm["confidence"] < 5:  # Çok düştüyse rengi değiştir
                    lm["color"] = color_type

            found = True
            break

    if not found:
        # Yeni bir şamandıra keşfedildi!
        landmark_id_counter += 1
        new_lm = {
            "id": landmark_id_counter,
            "color": color_type,
            "x": obj_world_x,
            "y": obj_world_y,
            "confidence": 1,  # İlk tespit
            "last_seen": current_time
        }
        landmarks_memory.append(new_lm)
        # Debug için yazdırabilirsin
        # print(f"[HAFIZA] YENİ {color_type} ŞAMANDIRA (ID: {landmark_id_counter}) X:{obj_world_x:.1f} Y:{obj_world_y:.1f}")


def lidar_thread_func():
    """
    LIDAR okuma thread'i - ULTRA ROBUST VERSİYON
    Hata durumunda cihazı tamamen kapatıp sıfırdan açar (Hard Reset).
    """
    global latest_lidar_scan_g, lidar_lock_g, is_running_g, lidar_g, latest_lidar_timestamp_g

    print("[LIDAR THREAD] Başlatılıyor...", flush=True)

    while is_running_g:
        # 1. BAĞLANTI YOKSA BAĞLAN
        if lidar_g is None:
            if not setup_lidar():
                # Bağlanamazsa 2 saniye bekle tekrar dene
                time.sleep(2)
                continue

        # 2. VERİ OKUMA DÖNGÜSÜ
        try:
            lidar_g.start_motor()
            # Motorun kendine gelmesi için yarım saniye şans tanı
            time.sleep(0.5)

            # iter_scans döngüsü - Veri aktığı sürece buradayız
            # max_buf_meas=5000 tamponu şişirmemek için, min_len=5 gürültüyü azaltmak için
            for scan in lidar_g.iter_scans(max_buf_meas=5000, min_len=5):
                if not is_running_g:
                    break

                valid_points = []
                for quality, angle, distance in scan:
                    if distance > 0:
                        valid_points.append((quality, angle, distance))

                if valid_points:
                    with lidar_lock_g:
                        latest_lidar_scan_g = valid_points
                        latest_lidar_timestamp_g = time.time()

        except Exception as e:
            # HATA ALINDIĞINDA:
            print(f"[LIDAR HATA] Bağlantı koptu veya veri bozuk: {e}", flush=True)
            print("[LIDAR] Cihaz resetleniyor...", flush=True)

            # 3. GÜVENLİ KAPATMA VE RESET
            try:
                if lidar_g:
                    lidar_g.stop_motor()
                    lidar_g.disconnect()
            except:
                pass

            # Nesneyi yok et ki 'if lidar_g is None' bloğu tekrar çalışsın
            lidar_g = None

            # CPU'yu ve portu dinlendir
            time.sleep(1.0)


def process_lidar_sectors(scan_data, max_dist=3.0):
    """
    Lidar verisini Sol, Merkez ve Sağ olarak analiz eder.
    Dönüş: (center_blocked, left_dist, center_dist, right_dist)
    """
    left_min_dist = float('inf')
    center_min_dist = float('inf')
    right_min_dist = float('inf')

    center_blocked = False

    if not scan_data:
        return False, float('inf'), float('inf'), float('inf')

    for quality, angle, distance_mm in scan_data:
        dist_m = distance_mm / 1000.0

        # Gürültü filtresi (0.15m altı ve max_dist üstü yok say)
        if dist_m < 0.03 or dist_m > max_dist:
            continue

        # Açıları normalize et (-180 ile +180 arası)
        norm_angle = angle
        if angle > 180:
            norm_angle = angle - 360

        # 1. MERKEZ (TEHLİKE) BÖLGESİ (-15 ile +15 derece)
        # Burası çarpma riski olan alan.
        if -15 <= norm_angle <= 15:
            if dist_m < center_min_dist: center_min_dist = dist_m

            # YENİSİ: Sabit 1.5 yerine config değeri (1.2m)
            lidar_limit = getattr(cfg, 'LIDAR_ACIL_DURMA_M', 1.5)
            if dist_m < lidar_limit: center_blocked = True

        # 2. SAĞ KORİDOR (+15 ile +60 derece)
        elif 15 < norm_angle <= 60:
            if dist_m < right_min_dist:
                right_min_dist = dist_m

        # 3. SOL KORİDOR (-60 ile -15 derece)
        elif -60 <= norm_angle < -15:
            if dist_m < left_min_dist:
                left_min_dist = dist_m

    return center_blocked, left_min_dist, center_min_dist, right_min_dist


#  PATH PLANNING YARDIMCI FONKSİYONLARI (YENİ)
# =============================================================================

def get_hybrid_point(robot_x, robot_y, robot_yaw, aci_farki, step_dist=2.0):
    target_angle = robot_yaw - math.radians(aci_farki)
    tx = robot_x + (step_dist * math.cos(target_angle))
    ty = robot_y + (step_dist * math.sin(target_angle))
    return tx, ty

def get_inflated_nav_map(raw_costmap, ignore_green=False, ignore_yellow=False):  # <--- PARAMETRE EKLENDİ
    """
    A* için haritayı hazırlar.
    ignore_green: True ise Yeşil şamandıraları engel olarak işaretlemez (Task 2 Circling için).
    ignore_yellow: True ise Sarı şamandıraları engel olarak işaretlemez (Task 3 Circling için).
    """
    if raw_costmap is None: return None, None

    nav_map = raw_costmap.copy()

    global landmarks_memory, robot_x, robot_y
    cur_rx = robot_x if 'robot_x' in globals() else 0
    cur_ry = robot_y if 'robot_y' in globals() else 0

    red_objs = []
    green_objs = []
    black_obstacles = []

    if 'landmarks_memory' in globals():
        for lm in landmarks_memory:
            if lm["confidence"] < 5: continue
            dist = math.sqrt((lm["x"] - cur_rx) ** 2 + (lm["y"] - cur_ry) ** 2)
            if dist > 20.0: continue

            px_py = world_to_pixel(lm["x"], lm["y"])
            if px_py:
                if lm["color"] == "RED":
                    red_objs.append((dist, px_py))
                elif lm["color"] == "GREEN":
                    if not ignore_green:
                        green_objs.append((dist, px_py))
                elif lm["color"] == "YELLOW":
                    # EĞER ignore_yellow=True İSE SARIYI DUVAR YAPMA
                    if not ignore_yellow:
                        # Sarı şamandıra nokta engeldir
                        black_obstacles.append(px_py)
                elif lm["color"] == "BLACK":
                    black_obstacles.append(px_py)

    # Duvar Örme
    def draw_smart_fence(objects_list, img_map):
        if len(objects_list) < 2: return
        objects_list.sort(key=lambda x: x[0])
        points = [item[1] for item in objects_list]
        for i in range(len(points) - 1):
            p1 = points[i]
            p2 = points[i + 1]
            dist_px = math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
            MAX_GAP_PX = int(8.0 / COSTMAP_RES_M_PER_PX)
            if dist_px < MAX_GAP_PX:
                cv2.line(img_map, p1, p2, 0, 4)

    draw_smart_fence(red_objs, nav_map)
    draw_smart_fence(green_objs, nav_map)

    # Siyah (ve engel durumundaki Sarı) Nokta Engeller
    for obs_px in black_obstacles:
        cv2.circle(nav_map, obs_px, 10, 0, -1)

    # Şişirme
    obstacles_mask = (nav_map < 100).astype(np.uint8) * 255
    inflation_m = getattr(cfg, 'INFLATION_MARGIN_M', 0.25)
    kernel_size = (int((ROBOT_RADIUS_M + inflation_m) / COSTMAP_RES_M_PER_PX) * 2) + 1
    inflated_obstacles = cv2.dilate(obstacles_mask, np.ones((kernel_size, kernel_size), np.uint8), iterations=1)

    nav_map[:] = 255
    nav_map[inflated_obstacles > 0] = 0

    return nav_map, inflated_obstacles

def setup_lidar():
    """LIDAR'a bağlanır ve global nesneyi ayarlar."""
    global lidar_g
    try:
        print(f"{LIDAR_PORT_NAME} portu üzerinden LIDAR'a bağlanılıyor...")
        # --- DEĞİŞİKLİK: RPLidar S3 için Baudrate 1.000.000 ---
        # Eğer config'de yoksa varsayılan olarak 1M kullan
        baud = getattr(cfg, 'LIDAR_BAUDRATE', 1000000)
        lidar_g = RPLidar(LIDAR_PORT_NAME, baudrate=baud, timeout=3)
        # ------------------------------------------------------
        print(" LIDAR Bağlantı başarılı.")
        health = lidar_g.get_health()
        print(f" LIDAR Cihaz Sağlığı: {health}")
        if health[0] != 'Good':
            print(f"[UYARI] Lidar sağlık: {health}")
        return True  # Başarılı
    except Exception as e:
        print(f"!!! LIDAR'a bağlanılamadı: {e}", file=sys.stderr)
        return False  # Başarısız

def select_mission_target(robot_x, robot_y, landmarks, robot_yaw, nav_map=None, gps_target_angle_err=None):
    """
    VISON MANTIĞI ENTEGRELİ GPS SÜRÜŞÜ

    Mantık Hiyerarşisi:
    1. KAMERA (Vision): Şamandıra görüyorsam GPS'i EZERİM. Vision modundaki gibi kapı/yan geçiş yaparım.
    2. GPS (Pusula): Şamandıra görmüyorsam, GPS açısına göre sanal bir hedef koyarım.
    3. A* (Planlama): Seçilen hedef neresi olursa olsun, haritadaki engellerden kaçarak rota çizerim.
    """

    # -------------------------------------------------------------------------
    # 1. VISION TARAMASI (Parkur İçi Mantığı)
    # -------------------------------------------------------------------------
    reds = []
    greens = []

    if landmarks:
        for lm in landmarks:
            # Sadece güvenilir ve YAKIN (Parkur içindeki) şamandıraları al
            if lm["confidence"] < 1: continue
            dist = math.sqrt((lm["x"] - robot_x) ** 2 + (lm["y"] - robot_y) ** 2)

            # Vision modundaki gibi sadece yakın çevreyi (10m) dikkate al
            if dist > 10.0: continue

            if lm["color"] == "RED":
                reds.append((dist, lm["x"], lm["y"]))
            elif lm["color"] == "GREEN":
                greens.append((dist, lm["x"], lm["y"]))

    reds.sort(key=lambda k: k[0])
    greens.sort(key=lambda k: k[0])

    visual_target_x = None
    visual_target_y = None
    target_type = "NONE"

    # --- AYNI VISION MODUNDAKİ SENARYOLAR ---

    # A) ÇİFT KAPI (Kırmızı + Yeşil) -> Ortasına Git
    if reds and greens:
        visual_target_x = (reds[0][1] + greens[0][1]) / 2.0
        visual_target_y = (reds[0][2] + greens[0][2]) / 2.0
        target_type = "VISUAL_GATE"

    # B) TEK KIRMIZI -> Sağından geç (+2.5m Yanal Kayma)
    elif reds:
        # Şamandıranın koordinatını alıp yanal kaydırma yapıyoruz
        # Basit matematik: Şamandıraya gitme, yanına git.
        visual_target_x = reds[0][1]
        visual_target_y = reds[0][2] - 1.6  # Y ekseninde kaydır (Basit yaklaşım)
        target_type = "VISUAL_RED_SIDE"

    # C) TEK YEŞİL -> Solundan geç (+2.5m Yanal Kayma)
    elif greens:
        visual_target_x = greens[0][1]
        visual_target_y = greens[0][2] + 1.6
        target_type = "VISUAL_GREEN_SIDE"

    # -------------------------------------------------------------------------
    # 2. HEDEF BELİRLEME (Karar Anı)
    # -------------------------------------------------------------------------

    final_x = None
    final_y = None

    # DURUM 1: KAMERA TESPİT YAPTI -> GPS'İ UNUT, PARKURU TAKİP ET
    if visual_target_x is not None:
        # Hedefi robotun baktığı yönde 3 metre ileriye (arkaya) ötele
        # Böylece kapının tam ortasında durmaz, içinden geçer.
        PROJECTION_M = 3.0
        final_x = visual_target_x + (PROJECTION_M * math.cos(robot_yaw))
        final_y = visual_target_y + (PROJECTION_M * math.sin(robot_yaw))

    # DURUM 2: KAMERA TESPİT YAPAMADI -> GPS ROTA MODU
    else:
        # Burada "Gap Finder" yok. A* zaten haritayı bildiği için,
        # biz sadece "GPS yönünde ileri git" diyeceğiz.
        # Eğer önümüzde engel varsa A* kendisi yol bulacak.

        base_angle = robot_yaw
        if gps_target_angle_err is not None:
            # GPS açısını hedefe ekle
            base_angle = robot_yaw + math.radians(-gps_target_angle_err)
            target_type = "GPS_GUIDED"
        else:
            # GPS de yoksa Kör Sürüş
            target_type = "BLIND_FORWARD"

        GPS_LOOKAHEAD = 1.5  # 5 metre ileriye hedef koy

        final_x = robot_x + (GPS_LOOKAHEAD * math.cos(base_angle))
        final_y = robot_y + (GPS_LOOKAHEAD * math.sin(base_angle))

    # -------------------------------------------------------------------------
    # 3. SMART PROJECTION (DUVAR/ENGEL İÇİNDEN ÇIKARMA)
    # Hedef (GPS veya Vision) yanlışlıkla bir şamandıranın içine denk gelirse
    # A* hesaplayamaz. Hedefi robota doğru biraz geri çekiyoruz.
    # -------------------------------------------------------------------------
    if nav_map is not None and final_x is not None:
        p_check = world_to_pixel(final_x, final_y)
        if p_check:
            px, py = p_check
            h_map, w_map = nav_map.shape
            if 0 <= px < w_map and 0 <= py < h_map:
                # NavMap'te 0 = Engel (Siyah), 255 = Yol (Beyaz)
                if nav_map[py, px] == 0:
                    # Hedef engel üstünde! Geri çek.
                    for pullback in [1.0, 2.0, 3.0]:
                        dx = final_x - robot_x
                        dy = final_y - robot_y
                        length = math.sqrt(dx * dx + dy * dy)
                        if length > pullback:
                            ratio = (length - pullback) / length
                            tx = robot_x + dx * ratio
                            ty = robot_y + dy * ratio

                            # Yeni nokta temiz mi?
                            pc = world_to_pixel(tx, ty)
                            if pc and nav_map[pc[1], pc[0]] > 0:
                                final_x = tx
                                final_y = ty
                                break

    return final_x, final_y, target_type





def pre_flight_check(zed_cam, controller_obj):
    """Saha testinden önce sistem sağlığını kontrol eder."""
    print("\n" + "=" * 60)
    print("[PRE-FLIGHT CHECK] System Diagnostics Started...")
    print("=" * 60)

    all_ok = True

    # 1. CAMERA CHECK
    try:
        if zed_cam.is_opened():
            print("   ✅ Camera: ONLINE")
        else:
            print("   ❌ Camera: OFFLINE or Disconnected!")
            all_ok = False
    except Exception as e:
        print(f"   ❌ Camera Error: {e}")
        all_ok = False

    # 2. LIDAR CHECK
    if lidar_g is not None:
        try:
            health = lidar_g.get_health()
            print(f"   ✅ LIDAR: Connected (Status: {health[0]})")
        except:
            print("   ⚠️ LIDAR: Connected but no data stream (Thread will retry)")
    else:
        print("   ⚠️ LIDAR: Not initialized yet (Will retry in Main)")

    # 3. MAPPING CHECK
    if costmap_ready and costmap_img is not None:
        print(f"   ✅ Mapping: Ready ({COSTMAP_SIZE_PX} px)")
    else:
        print("   ❌ Mapping: Failed to initialize!")
        all_ok = False

    # 4. CONFIG CHECK
    print(f"   ℹ️  Operation Mode: {cfg.NAV_MODE}")
    print(f"   ℹ️  Buoy Filters: Min={BUOY_MIN_RATIO}, Max={BUOY_MAX_RATIO}")

    print("=" * 60)
    if all_ok:
        print("✅ SYSTEM READY FOR MISSION!")
    else:
        print("❌ CRITICAL ERRORS DETECTED! ABORT!")
    print("=" * 60 + "\n")

    return all_ok


def task6_audio_listener_thread():
    """
    Arka planda sürekli çalışarak mikrofonu dinler.
    Ayarları config.py dosyasından (cfg.T6_...) çeker.
    1 Düdük -> task6_interrupt_request = 3 yapar.
    2 Düdük -> task6_interrupt_request = 5 yapar.
    """
    global task6_interrupt_request, task6_detected_freq, task6_timestamp, is_running_g

    print("[TASK 6] Ses Dinleme Modülü Başlatılıyor...")

    p = pyaudio.PyAudio()
    stream = None

    try:
        # Mikrofonu Aç (Ayarlar Config'den)
        stream = p.open(format=pyaudio.paInt16,  # Bu sabit kalabilir
                        channels=cfg.T6_AUDIO_CHANNELS,
                        rate=cfg.T6_AUDIO_RATE,
                        input=True,
                        frames_per_buffer=cfg.T6_AUDIO_CHUNK)
    except Exception as e:
        print(f"[HATA] Mikrofon açılamadı! Task 6 devre dışı. Hata: {e}")
        return

    # Durum Makinesi Değişkenleri
    state = "LISTENING"
    first_blast_end_time = 0
    cooldown_start_time = 0
    BLAST_TIMEOUT = 2.0  # Düdük arası maks bekleme (Sabit kural)
    COOLDOWN_PERIOD = 3.0  # Hata önleme beklemesi

    temp_detected_freq = None

    while is_running_g:  # Ana program çalıştığı sürece dön
        try:
            # Ses verisini oku (Config'den gelen Chunk boyutuyla)
            raw_data = stream.read(cfg.T6_AUDIO_CHUNK, exception_on_overflow=False)
            data_np = np.frombuffer(raw_data, dtype=np.int16)

            # FFT Analizi (Frekans Bulma)
            fft_data = np.fft.fft(data_np * np.hanning(len(data_np)))
            magnitudes = np.abs(fft_data)[:len(fft_data) // 2]
            # Frekans eksenini oluştururken Config'deki Rate'i kullan
            frequencies = np.fft.fftfreq(len(data_np), 1.0 / cfg.T6_AUDIO_RATE)[:len(data_np) // 2]

            sound_present = None  # Frekans bulundu mu?

            # Hedef frekansları tara (Config'deki Listeden)
            for freq_name, (min_f, max_f, threshold) in cfg.T6_TARGET_FREQS.items():
                # İlgili frekans aralığındaki indeksleri bul
                indices = np.where((frequencies >= min_f) & (frequencies <= max_f))
                if len(indices[0]) > 0:
                    max_mag = np.max(magnitudes[indices[0]])
                    # Eşik değerini kontrol et
                    if max_mag > threshold:
                        sound_present = freq_name
                        # print(f"[DEBUG] Ses Algılandı: {freq_name} Güç: {max_mag:.0f}")
                        break

            current_time = time.time()

            # --- DURUM MAKİNESİ (STATE MACHINE) ---
            # (Burası değişmedi, mantık aynı)

            if state == "LISTENING":
                if sound_present is not None:
                    state = "SOUND_DETECTED"
                    temp_detected_freq = sound_present
                    print(f"[TASK 6] 1. Sinyal Algılandı ({temp_detected_freq})...")

            elif state == "SOUND_DETECTED":
                if sound_present is None:
                    state = "WAITING_FOR_NEXT"
                    first_blast_end_time = current_time
                    print("[TASK 6] Sessizlik... 2. sinyal bekleniyor.")

            elif state == "WAITING_FOR_NEXT":
                if sound_present is not None:
                    # 2. SES GELDİ -> TASK 5
                    state = "COOLDOWN"
                    cooldown_start_time = current_time

                    print(f"\n>>> [ALARM] ÇİFT SİNYAL! ({temp_detected_freq}) -> TASK 5 <<<\n")
                    task6_interrupt_request = 5
                    task6_detected_freq = temp_detected_freq
                    task6_timestamp = current_time

                elif (current_time - first_blast_end_time) > BLAST_TIMEOUT:
                    # ZAMAN DOLDU -> TASK 3
                    state = "COOLDOWN"
                    cooldown_start_time = current_time

                    print(f"\n>>> [ALARM] TEK SİNYAL! ({temp_detected_freq}) -> TASK 3 <<<\n")
                    task6_interrupt_request = 3
                    task6_detected_freq = temp_detected_freq
                    task6_timestamp = current_time

            elif state == "COOLDOWN":
                if (current_time - cooldown_start_time) > COOLDOWN_PERIOD:
                    state = "LISTENING"
                    temp_detected_freq = None

        except Exception as e:
            pass

    # Temizlik
    if stream:
        stream.stop_stream()
        stream.close()
    p.terminate()


def main():
    global width, manual_mode, magnetic_heading, mission_started, latest_lidar_scan_g, is_running_g, lidar_g
    global task6_interrupt_request, task6_detected_freq, task6_timestamp
    global task5_dock_timer
    global path_lost_time
    global costmap_img, costmap_ready

    # --- KRİTİK BAŞLANGIÇ DEĞERLERİ (CRASH ÖNLEME) ---
    magnetic_heading = 0.0
    last_pixel_error = 0.0
    last_vision_time = time.time()  # Şimdiki zamanla başlat
    vision_lock_active = False
    acil_durum_aktif_mi = False
    last_pwm_correction = 0

    print("Initializing Camera...")
    zed = kamera.initialize_camera()

    # --- GÜÇLENDİRİLMİŞ KONUM TAKİBİ (DÜZELTİLDİ) ---
    print("Initializing ROBUST Positional Tracking...")
    tracking_parameters = sl.PositionalTrackingParameters()

    # 1. IMU Füzyonunu AÇ (Bu şart)
    tracking_parameters.enable_imu_fusion = True

    # 2. [DEĞİŞİKLİK] Zemini Referans ALMA (Denizde/Havuzda Zemin Oynaktır)
    # Bunu False yapıyoruz. Çünkü tekne yalpaladığında ZED "Yokuş çıkıyorum" sanıp haritayı bozabilir.
    tracking_parameters.set_floor_as_origin = False

    # 3. [DEĞİŞİKLİK] Alan Hafızasını KAPAT (Area Memory)
    # Su yüzeyi sürekli değiştiği için ZED'in eski gördüğü yeri tanıması imkansızdır.
    # Yanlış eşleştirme yapıp haritayı zıplatmaması için bunu kapatıyoruz.
    tracking_parameters.enable_area_memory = False

    # 4. [EKSTRA] Pose Smoothing (Yumuşatma) Açılabilir
    tracking_parameters.enable_pose_smoothing = True

    # Takibi başlat
    err = zed.enable_positional_tracking(tracking_parameters)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"[HATA] Konum takibi başlatılamadı: {err}")

    mapping_init(start_x=0, start_y=0)
    # ---------------------------
    if not pre_flight_check(zed, controller):
        print("!!! UYARI: Pre-Flight Check başarısız oldu. Yine de devam ediliyor (Riskli)...")
        time.sleep(3)  # Kullanıcıya okuma süresi ver

    print("Camera initialized!")

    relay = None
    try:

        # YENİ (Tek Pin):
        relay = motorPower.SingleMotorRelay(cfg.MOTOR_RELAY_PIN)
        relay.power_on()  # Gücü ver

        esc_delay = getattr(cfg, 'ESC_INIT_DELAY', 3.0)
        print(f"ESC stabilizasyonu için {esc_delay} saniye bekleniyor...")
        time.sleep(esc_delay)
        print("Motorlar hazır!")

    except Exception as e:
        print(f"[UYARI] Röle başlatılamadı: {e}")
        relay = None

    # --- YENİ LIDAR BAŞLATMA ---
    if not setup_lidar():
        print("[UYARI] LIDAR başlatılamadı, LIDAR olmadan devam ediliyor.")
    else:
        print("LIDAR okuma thread'i başlatılıyor...")
        lidar_thread = threading.Thread(target=lidar_thread_func)
        lidar_thread.daemon = True
        lidar_thread.start()
    # --- BİTTİ ---

    # --- BURAYA EKLE: TASK 6 SES DİNLEME THREAD BAŞLAT ---
    # Robot başlar başlamaz arka planda mikrofonu dinlemeye başlar.
    try:
        audio_thread = threading.Thread(target=task6_audio_listener_thread)
        audio_thread.daemon = True  # Ana program kapanınca bu da kapansın
        audio_thread.start()
        print("[INIT] Task 6 Ses Dinleyici Arka Planda Çalışıyor.")
    except Exception as e:
        print(f"[UYARI] Ses thread başlatılamadı: {e}")

    camera_info = zed.get_camera_information()
    width = camera_info.camera_configuration.resolution.width
    height = camera_info.camera_configuration.resolution.height
    print(width, height)
    # Görüntüde merkez noktasını hesapla
    center_x = width // 2
    center_y = height // 2

    ts_handler = TimestampHandler()  # Used to store the sensors timestamp to know if the sensors_data is a new one or not

    sensors_data = sl.SensorsData()  # Sensör verisi al
    image = sl.Mat()  # Görüntü ve derinlik verilerini almak için Mat nesneleri oluştur
    depth = sl.Mat()

    # --- DEĞİŞİKLİK 1: Kalman Filtresi döngü dışında tanımlanıyor ---
    # Filtre nesnesini bir kere oluşturuyoruz, her döngüde sıfırlanmıyor.
    magnetic_filter = KalmanFilter(process_variance=1e-3, measurement_variance=1e-1)
    # ---------------------------------------------------------------

    adviced_course = 0
    aci_farki = 0
    hedefe_mesafe = 1000
    TOPLAM_HATA = 0.0

    _waypoints = []  # global waypoint listesi (kullanılıyor)
    _active_wp_index = 0  # hangi waypointteyiz
    _waypoints_created = False

    # --- YENİ LIDAR GÖSTERGE DEĞİŞKENLERİ ---
    lidar_min_dist_mm = 0.0
    lidar_avg_dist_mm = 0.0
    lidar_num_points = 0
    # --- BİTTİ ---

    # video kayıt
    # video kayıt
    writer = None  # Önce boş tanımla

    # Eğer Config'de RECORD_VIDEO True ise (veya hiç yoksa) kaydı başlat
    if getattr(cfg, 'RECORD_VIDEO', True):
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        video_path = f"idaKayit/kayit_{ts}.mp4"
        if not os.path.exists("idaKayit"):
            os.makedirs("idaKayit")

        # VideoWriter thread
        writer = utils.AsyncVideoWriter(video_path, fps=20.0, max_queue=120)
        writer.start()
        print(f"[INFO] Video kaydı başladı: {video_path}")
    else:
        print("[INFO] Video kaydı KAPALI (Config).")

    last_pixel_error = 0.0
    mod_durumu = "BASLANGIC"
    nav_mode = getattr(cfg, 'NAV_MODE', 'GPS')  # Config'de yoksa varsayılan GPS
    print(f"\n======================\n AKTİF MOD: {nav_mode}\n======================\n")
    # print(f"[MODEL] Class İsimleri: {model.names}")

    mevcut_gorev = cfg.MEVCUT_GOREV

    manual_mode = False
    mission_started = True

    # --- FREN KONTROL DEĞİŞKENİ ---
    acil_durum_aktif_mi = False
    # --- YENİ EKLENECEK KISIM (VISION MEMORY) ---
    last_vision_time = 0  # Son ne zaman geçerli bir balon gördük?
    last_pwm_correction = 0  # Son uyguladığımız düzeltme neydi?
    vision_lock_active = False  # Şu an hafızadan mı gidiyoruz?

    # --- YENİ PLANLAMA DEĞİŞKENLERİ ---
    current_path = []  # Hesaplanan rota burada tutulacak
    plan_timer = 0  # A* algoritmasını yavaşlatmak için sayaç
    prev_heading_error = 0.0

    # Hybrid Nav Değişkenleri
    hybrid_local_target = None
    hybrid_target_reached = False

    try:
        while True:

            # ---- 1. Komut kuyruğunu boşalt ve gelen komutları uygula ----
            try:
                while True:
                    cmd = cmd_queue.get_nowait()
                    my_id = 1
                    incoming_id = cmd.get("target_id")
                    if incoming_id is not None:
                        if incoming_id != my_id:
                            # Bu mesaj başka bir tekneye (örn: Tekne 2) gitmiş, biz ilgilenmiyoruz.
                            print(f"[CMD IGNORED] Hedef ID: {incoming_id} != Benim ID: {my_id}")
                            continue  # Döngünün başına dön, sonraki mesajı kontrol et
                    print("[CMD RX]", cmd)
                    # EMERGENCY_STOP yakala
                    if isinstance(cmd, dict) and cmd.get("cmd") == "emergency_stop":
                        print("\n[INFO] Yer kontrolden ACİL KAPATMA alındı, kapanıyor...")
                        raise utils.EmergencyShutdown()
                        raise KeyboardInterrupt
                    manual_mode, mission_started = telem.handle_command(cmd, controller, cfg, manual_mode,
                                                                        mission_started)
            except queue.Empty:
                pass

            # TASK 6: SESLİ KOMUT KESMESİ (INTERRUPT) - BURAYA EKLİYORUZ
            # =========================================================================
            # Robot ne yaparsa yapsın, eğer arka plandaki kulak (thread) bir şey duyarsa
            # burası devreye girer ve görevi zorla değiştirir.

            if task6_interrupt_request is not None:
                # Ses thread'inden bir emir geldi!
                detected_task = task6_interrupt_request
                freq_info = task6_detected_freq

                print(f"{Fore.RED}\n!!! ACİL DURUM KESMESİ (SES ALGILANDI) !!!{Style.RESET_ALL}")
                print(f"SESLİ KOMUT: {freq_info} -> HEDEF GÖREV: TASK {detected_task}")

                # 1. GÖREVİ DEĞİŞTİR
                if detected_task == 3:
                    mevcut_gorev = "TASK3_APPROACH"
                    print(f"{Fore.YELLOW}>> ROTA GÜNCELLENDİ: TASK 3 (ACİL DURUM NOKTASI){Style.RESET_ALL}")

                elif detected_task == 5:
                    mevcut_gorev = "TASK5_APPROACH"
                    print(f"{Fore.YELLOW}>> ROTA GÜNCELLENDİ: TASK 5 (MARİNAYA DÖNÜŞ){Style.RESET_ALL}")

                # 2. ESKİ ROTA VE HEDEFLERİ SİL (RESET)
                # Robotun eski hedefe gitmeye çalışmasını engellemek için hafızayı temizle.
                current_path = []  # A* rotasını sil
                target_lat = None  # GPS hedefini sil

                # Emiri uyguladık, bayrağı indir ki sürekli buraya girmesin.
                task6_interrupt_request = None

                # Robotun kafası karışmasın diye kısa bir bekleme ve döngü başı
                time.sleep(0.1)
                continue  # continue diyerek aşağıdakileri (Lidar, Kamera vs) atla, yeni görevle baştan başla.

            # -----------------------------------------------------------------------------------------
            # 2. ORTAK VERİ ALMA: GPS (HATA DÜZELTME: EN BAŞA ALINDI)
            # -----------------------------------------------------------------------------------------
            # Bu satır artık mod fark etmeksizin her döngüde çalışacak.
            ida_enlem, ida_boylam = controller.get_current_position()

            # [EK GÜVENLİK] GPS verisi yoksa (None ise) telemetri hatası vermemesi için kontrol edilebilir
            # ancak utils.nfloat zaten None kontrolü yapıyor, o yüzden kod kırılmaz.

            # -----------------------------------------------------------------------------------------
            # 3. ORTAK VERİ ALMA: LIDAR
            # -----------------------------------------------------------------------------------------
            pp_target = None
            local_lidar_scan = []
            lidar_is_fresh = False

            with lidar_lock_g:
                if latest_lidar_scan_g:
                    local_lidar_scan = list(latest_lidar_scan_g)
                    if (time.time() - latest_lidar_timestamp_g) < 0.5:  # 0.5 sn tolerans
                        lidar_is_fresh = True

            if not lidar_is_fresh:
                local_lidar_scan = []

            # Lidar Sektör Analizi

            # 1. Robotun Havuzdaki Konumunu (Odometry) Oku
            zed_pose = sl.Pose()
            robot_x, robot_y, robot_yaw = 0, 0, 0

            state = zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
            if state == sl.POSITIONAL_TRACKING_STATE.OK:
                # Konum (Metre)
                t_vec = zed_pose.get_translation().get()

                # --- KOORDİNAT DÜZELTMESİ (AXIS SWAP - 90 DERECE DÖNÜŞ) ---
                # Sorun: Robot ileri gidince haritada yan (veya yukarı gidince sağa) kayıyor.
                # Çözüm: İleri gitme verisini (t_vec[1]) diğer eksene alıyoruz.

                # DENEME: X ve Y'nin yerini değiştiriyoruz.
                # Robot ileri gittiğinde (t_vec[1] arttığında), haritada X ekseninde ilerlesin.
                robot_x = t_vec[1]

                # Robot yan gittiğinde (t_vec[0]), haritada Y ekseninde ilerlesin.
                # (Yön ters gelirse başına eksi koyarız: -t_vec[0])
                robot_y = -t_vec[0]

                # --- QUATERNION YAW HESABI ---
                ox = zed_pose.get_orientation().get()[0]
                oy = zed_pose.get_orientation().get()[1]
                oz = zed_pose.get_orientation().get()[2]
                ow = zed_pose.get_orientation().get()[3]

                siny_cosp = 2 * (ow * oz + ox * oy)
                cosy_cosp = 1 - 2 * (oy * oy + oz * oz)
                robot_yaw = math.atan2(siny_cosp, cosy_cosp)
                # Debug (Değerleri kontrol et)
                # print(f"[NAV] MapX: {robot_x:.2f} | MapY: {robot_y:.2f} | Yaw: {math.degrees(robot_yaw):.1f}")

                # 1. Roll (Yalpalama / Sağa-Sola Yatma) Hesabı
                sinr_cosp = 2 * (ow * ox + oy * oz)
                cosr_cosp = 1 - 2 * (ox * ox + oy * oy)
                robot_roll = math.atan2(sinr_cosp, cosr_cosp)
                robot_roll_deg = math.degrees(robot_roll)

                # 2. Pitch (Yunuslama / Öne-Arkaya Batma) Hesabı
                sinp = 2 * (ow * oy - oz * ox)
                if abs(sinp) >= 1:
                    robot_pitch = math.copysign(math.pi / 2, sinp)
                else:
                    robot_pitch = math.asin(sinp)
                robot_pitch_deg = math.degrees(robot_pitch)

                # 3. Dalga Filtresi (Wave Stability Check)
                # Tekne 7 dereceden fazla yatıyorsa lidar verisini haritaya işleme!
                MAX_TILT_ANGLE = getattr(cfg, 'MAX_TILT_ANGLE', 5.0)
                wave_stable = True

                if abs(robot_roll_deg) > MAX_TILT_ANGLE or abs(robot_pitch_deg) > MAX_TILT_ANGLE:
                    wave_stable = False
                    # Debug için istersen aç:
                    # print(f"[UYARI] Yuksek Egim! Roll:{robot_roll_deg:.1f} Pitch:{robot_pitch_deg:.1f}")

            # 2. Haritayı Lidar Verisiyle Güncelle
            if 'last_mapped_time' not in locals():
                last_mapped_time = 0

            # --- DEĞİŞİKLİK: 'and wave_stable' EKLENDİ ---
            # Sadece veri tazeyse VE tekne dengedeyse haritayı güncelle
            if lidar_is_fresh and local_lidar_scan and wave_stable:
                if (latest_lidar_timestamp_g > last_mapped_time):
                    mapping_update_lidar(robot_x, robot_y, robot_yaw, local_lidar_scan)
                    last_mapped_time = latest_lidar_timestamp_g

            # --- MAP DECAY (AGGRESSIVE CLEARING) ---
            # Every loop, brighten the map slightly to remove ghost obstacles
            if costmap_ready and costmap_img is not None:
                 decay_amount = getattr(cfg, 'MAP_DECAY_AMOUNT', 0)
                 if decay_amount > 0:
                     costmap_img = cv2.add(costmap_img, decay_amount)

            # (Güvenlik Önlemi) Tekne çok yatıksa (Dalga), lidar verisini boşalt.
            # Böylece "process_lidar_sectors" fonksiyonu önümüzde engel var sanıp ani fren yapmaz.
            if not wave_stable:
                local_lidar_scan = []

            lidar_max_d = getattr(cfg, 'LIDAR_MAX_DIST', 10.0)
            center_danger, left_d, center_d, right_d = process_lidar_sectors(local_lidar_scan, max_dist=lidar_max_d)

            # -----------------------------------------------------------------------------------------
            # 4. ORTAK VERİ ALMA: KAMERA (DÜZELTME: BURADA ALIYORUZ)
            # -----------------------------------------------------------------------------------------
            frame_hazir = False
            output_frame = None
            detections = None

            if zed.grab() == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image, sl.VIEW.LEFT)
                zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
                frame = cv2.cvtColor(image.get_data(), cv2.COLOR_BGRA2BGR)
                frame_hazir = True

                # YOLO Modelini Çalıştır
                conf_val = getattr(cfg, 'YOLO_CONFIDENCE', 0.50)
                results = model(frame, conf=conf_val, verbose=False)[0]
                detections = sv.Detections.from_ultralytics(results)

                #  SENSOR FUSION & LANDMARK MAPPING (YENİ EKLENECEK)
                # -----------------------------------------------------------------

                # Sadece harita hazırsa ve tespit varsa çalışır
                if detections and costmap_ready and costmap_img is not None:
                    # ZED 2i Yatay Görüş Açısı (Yaklaşık 110 derece)
                    hfov_deg = getattr(cfg, 'CAM_HFOV', 110.0)
                    hfov_rad = math.radians(hfov_deg)

                    coords = detections.xyxy.tolist()
                    cids = detections.class_id.tolist()

                    for i, cid in enumerate(cids):
                        color_label = "UNKNOWN"
                        if cid in [0, 3, 5]:
                            color_label = "RED"
                        elif cid in [1, 4, 12]:
                            color_label = "GREEN"
                        elif cid == 10:
                            color_label = "BLACK"
                        elif cid == 9:
                            color_label = "YELLOW"
                        else:
                            continue
                        # 1. Kameradaki Konumu Bul
                        x1, y1, x2, y2 = map(int, coords[i])
                        cx = int((x1 + x2) / 2)

                        # --- DÜZELTME: MERKEZ YERİNE ALT %15'E BAK ---
                        # Şamandıranın suyla birleştiği nokta en doğru mesafeyi verir.
                        # Kutunun yüksekliği: (y2 - y1)
                        box_h = y2 - y1
                        target_cy = int(y2 - (box_h * 0.15))

                        # Güvenlik: Ekran dışına taşmasın
                        cy = max(0, min(target_cy, height - 1))
                        # ---------------------------------------------

                        # Derinlik Oku (ZED'den)
                        err, dist_m = depth.get_value(cx, cy)

                        # Güvenlik: Geçersiz derinlikse yoksay
                        if np.isnan(dist_m) or np.isinf(dist_m) or dist_m <= 0.1:
                            continue

                        if dist_m > 15.0: continue  # Çok uzaktakilere güvenme

                        # 2. Dünya Koordinatını Hesapla (TRİGONOMETRİ)
                        # Kameranın merkezinden ne kadar sağda/solda? (-0.5 sol, +0.5 sağ)
                        pixel_offset = (cx - (width / 2)) / width

                        # ZED kamerasında sağ taraf negatif açıdır (Genelde)
                        angle_offset = -pixel_offset * hfov_rad

                        # Cismin Dünyadaki mutlak açısı (Robot Yaw + Bakış Açısı)
                        obj_global_angle = robot_yaw + angle_offset

                        # Cismin Dünyadaki koordinatı (Robot Konumu + Vektör)
                        obj_world_x = robot_x + (dist_m * math.cos(obj_global_angle))
                        obj_world_y = robot_y + (dist_m * math.sin(obj_global_angle))

                        # KRİTİK EKLENTİ: SANAL ENGEL ENJEKSİYONU (Lidar Körlüğüne Son)
                        # -----------------------------------------------------------
                        # Kamera bir şey gördüyse, Lidar görmese bile haritaya engel koy.
                        # Böylece A* algoritması "burası dolu" diyip rota değiştirecek.

                        p_virtual = world_to_pixel(obj_world_x, obj_world_y)
                        if p_virtual:
                            # 0.6 metre yarıçapında SİYAH DAİRE (Engel)
                            # Harita 1px = 0.1m olduğu için -> 0.6m = 6 piksel yarıçap
                            disk_radius_px = 6

                            # Haritaya (costmap_img) doğrudan çiziyoruz (0 = Siyah/Engel)
                            cv2.circle(costmap_img, p_virtual, disk_radius_px, 0, -1)

                        # 3. LIDAR VALIDATION (BUOY DENSITY CHECK - FINAL)
                        is_verified_by_lidar = False
                        best_verified_x = obj_world_x
                        best_verified_y = obj_world_y

                        # Dinamik Arama Aralığı (Kamera mesafesinin +- 1 metresi)
                        search_start = max(0.5, dist_m - 1.0)
                        search_end = dist_m + 1.0
                        step_size = 0.10

                        for d_scan in np.arange(search_start, search_end, step_size):
                            # Menzil Limiti Kontrolü (Global Ayar)
                            if d_scan > BUOY_MAX_DIST: break

                            t_x = robot_x + (d_scan * math.cos(obj_global_angle))
                            t_y = robot_y + (d_scan * math.sin(obj_global_angle))

                            p_check = world_to_pixel(t_x, t_y)

                            if p_check:
                                px, py = p_check
                                # Harita sınır kontrolü
                                if 0 <= px < COSTMAP_SIZE_PX[0] and 0 <= py < COSTMAP_SIZE_PX[1]:
                                    val = costmap_img[py, px]

                                    # Eşik Değeri (125 - Gri Tonu)
                                    if val < 125:
                                        # Alan Kontrolü (ROI - Region of Interest)
                                        y_min, y_max = max(0, py - 5), min(COSTMAP_SIZE_PX[1], py + 5)
                                        x_min, x_max = max(0, px - 5), min(COSTMAP_SIZE_PX[0], px + 5)
                                        roi = costmap_img[y_min:y_max, x_min:x_max]

                                        # Yoğunluk (Density) Hesabı
                                        obstacle_ratio = np.mean(roi < 100)

                                        # --- YENİ İSİMLERLE KONTROL ---
                                        # Şamandıra mı, Duvar mı?
                                        if BUOY_MIN_RATIO < obstacle_ratio < BUOY_MAX_RATIO:
                                            is_verified_by_lidar = True
                                            best_verified_x = t_x
                                            best_verified_y = t_y

                                            # Debug Log (Sadece ayar açıksa yazar)
                                            if DEBUG_BUOY:
                                                print(
                                                    f"[BUOY CONFIRMED] {color_label} Dist:{d_scan:.1f}m Ratio:{obstacle_ratio:.2f}")
                                            break

                                            # Doğrulandıysa koordinatı güncelle ve hafızaya at
                        if is_verified_by_lidar:
                            obj_world_x = best_verified_x
                            obj_world_y = best_verified_y
                            update_landmark_memory(color_label, obj_world_x, obj_world_y)

                # Çizimleri yap (Yayın için)
                if getattr(cfg, 'STREAM', False) or getattr(cfg, 'RECORD_VIDEO', False):
                    frame = bounding_box_annotator.annotate(scene=frame, detections=detections)
                    frame = label_annotator.annotate(scene=frame, detections=detections)

                current_fps = round(zed.get_current_fps())

                # Çerçeveyi hazırla (Resize)
                if width < 1000:
                    output_frame = frame
                else:
                    output_frame = cv2.resize(frame, (960, 540))

            # Değişkenleri varsayılan olarak ayarla
            mod_durumu = "BEKLEME"

            # -----------------------------------------------------------------------------------------
            # MOD 1: VISION (KAPALI ALAN)
            # -----------------------------------------------------------------------------------------
            if nav_mode == "VISION":

                # A) ACİL DURUM (LIDAR - REFLEKS KAÇIŞ VE YÖN DEĞİŞTİRME)
                if center_danger:
                    mod_durumu = "ACIL (KACIS MANEVRASI)"
                    print(f"[ACİL] ENGEL! Mesafe: {center_d:.2f}m")

                    # --- 1. ADIM: ŞOK FREN (MOMENTUMU ÖLDÜR) ---
                    if not acil_durum_aktif_mi:
                        print(" -> [KORUMA] ŞOK FREN!")
                        # Ters güç vererek zınk diye durdur
                        shock_pwm = 250
                        controller.set_servo(cfg.SOL_MOTOR, 1500 - shock_pwm)
                        controller.set_servo(cfg.SAG_MOTOR, 1500 - shock_pwm)
                        time.sleep(0.1)  # Çok kısa fren
                        acil_durum_aktif_mi = True

                        # Vision hafızasını sil (Çünkü yönümüzü zorla değiştireceğiz)
                        last_vision_time = 0
                        vision_lock_active = False

                    escape_gucu = getattr(cfg, 'ESCAPE_PWM', 300)

                    # --- 2. ADIM: KISA GERİ SEKME (REFLEKS) ---
                    # Sürekli geri gitmek yerine sadece anlık bir kaçış yapıyoruz.
                    print(f" -> [MANEVRA] Anlık Geri Sekme...")

                    # Sol motor dengesizliği varsa buradaki katsayıyı kullan, yoksa sil.
                    pwm_sol = 1500 - escape_gucu
                    pwm_sag = 1500 - escape_gucu

                    controller.set_servo(cfg.SOL_MOTOR, int(pwm_sol))
                    controller.set_servo(cfg.SAG_MOTOR, int(pwm_sag))

                    # ÖNEMLİ: Sadece 0.4 saniye geri git.
                    # Bu sayede robot parkurun başına kadar geri gitmez, sadece açılır.
                    time.sleep(0.4)

                    # --- 3. ADIM: KURTULMA MANEVRASI (YÖN DEĞİŞTİR) ---
                    # Geri geldik, şimdi burnumuzu engelden çevirmeliyiz.
                    turn_power = 200  # Dönüş sertliği

                    # Lidar verisine bak: Hangi taraf daha boş?
                    if left_d > right_d:
                        print(f" -> [MANEVRA] Sol Boş ({left_d:.1f}m) -> SOLA TANK DÖNÜŞÜ")
                        # Sol Geri, Sağ İleri (Olduğu yerde sola dön)
                        controller.set_servo(cfg.SOL_MOTOR, 1500 - turn_power)
                        controller.set_servo(cfg.SAG_MOTOR, 1500 + turn_power)
                    else:
                        print(f" -> [MANEVRA] Sağ Boş ({right_d:.1f}m) -> SAĞA TANK DÖNÜŞÜ")
                        # Sol İleri, Sağ Geri (Olduğu yerde sağa dön)
                        controller.set_servo(cfg.SOL_MOTOR, 1500 + turn_power)
                        controller.set_servo(cfg.SAG_MOTOR, 1500 - turn_power)

                    # Dönüşün gerçekleşmesi için kısa bir süre tanı
                    time.sleep(0.3)
                # B) NORMAL SEYİR (KAMERA) - ZED 3D DERİNLİK MODU
                elif frame_hazir:
                    acil_durum_aktif_mi = False  # Tehlike geçti

                    # Listeleri (Merkez_X, Mesafe_Metre, Alan) formatında tutacağız
                    red_objects = []
                    green_objects = []

                    if detections:
                        coords = detections.xyxy.tolist()
                        cids = detections.class_id.tolist()
                        for i, cid in enumerate(cids):
                            x1, y1, x2, y2 = map(int, coords[i])
                            cx = int((x1 + x2) / 2)
                            cy = int((y1 + y2) / 2)

                            # --- 1. ZED KAMERADAN MESAFE OKUMA ---
                            # Kutunun merkezindeki pikselin derinliğini al
                            # Koordinatları sınırla (Safety Clamp)
                            cx = max(0, min(cx, width - 1))
                            cy = max(0, min(cy, height - 1))

                            # Şimdi güvenle oku
                            err, dist_m = depth.get_value(cx, cy)

                            # Güvenlik Kontrolü: Derinlik okunamazsa (NaN/Inf) Alanı kullan
                            # Alan ne kadar büyükse, cisim o kadar yakındır (Ters orantı mantığı)
                            area = (x2 - x1) * (y2 - y1)

                            final_dist = 99.0  # Varsayılan: Çok uzak

                            if not np.isnan(dist_m) and not np.isinf(dist_m) and dist_m > 0.1:
                                final_dist = dist_m
                            elif area > (width * height * 0.1):
                                # Derinlik okunamadı ama cisim ekranın %10'unu kaplıyor -> ÇOK YAKIN!
                                final_dist = 0.5

                            if cid in [0, 3, 5]:  # Kırmızı (Kule, İşaretçi, Normal)
                                red_objects.append((cx, final_dist, area))
                            elif cid in [1, 4, 12]:  # Yeşil (Kule, İşaretçi, Normal)
                                green_objects.append((cx, final_dist, area))

                    target_x = center_x
                    visual_lock = False

                    # En yakın (Tehlikeli) objeleri seç
                    best_red = min(red_objects, key=lambda x: x[1]) if red_objects else None
                    best_green = min(green_objects, key=lambda x: x[1]) if green_objects else None

                    # --- 2. AKILLI KARAR MEKANİZMASI (1.5m vs 4m Sorunu Çözümü) ---

                    if best_red and best_green:
                        r_dist = best_red[1]  # Kırmızı Mesafe
                        g_dist = best_green[1]  # Yeşil Mesafe

                        dist_diff = abs(r_dist - g_dist)

                        # EĞER: Biri diğerinden 1.5 metre daha yakınsa, UZAKTAKİNİ UNUT.
                        # Senin örneğinde: |4.0 - 1.5| = 2.5m fark var -> Kırmızı iptal edilir.
                        if dist_diff > 1.5:
                            if r_dist < g_dist:
                                best_green = None  # Kırmızı çok daha yakın, Yeşili yoksay
                                print(f" -> [ZED] Kırmızı ({r_dist:.1f}m) referans alındı. Yeşil ({g_dist:.1f}m) uzak.")
                            else:
                                best_red = None  # Yeşil çok daha yakın, Kırmızıyı yoksay
                                print(f" -> [ZED] Yeşil ({g_dist:.1f}m) referans alındı. Kırmızı ({r_dist:.1f}m) uzak.")

                    # --- 3. HEDEF BELİRLEME ---

                    # Güvenlik Açıklığı (Topa ne kadar uzaktan geçsin?)
                    # 0.35 demek ekran genişliğinin %35'i kadar açığından geç demek.
                    SAFE_OFFSET = width * 0.4

                    if best_red and best_green:
                        # İkisi de yakın mesafede, ORTALA
                        target_x = (best_red[0] + best_green[0]) / 2
                        mod_durumu = "ORTALA"
                        visual_lock = True

                    elif best_red:
                        # Sadece Kırmızı (veya yakındaki Kırmızı)
                        # Kırmızıyı SOLUNDA tutmak için SAĞA git (Kırmızı X + Offset)
                        target_x = best_red[0] + SAFE_OFFSET
                        mod_durumu = "SOL REF (Kirmizi Solda)"
                        visual_lock = True

                    elif best_green:
                        # Sadece Yeşil (veya yakındaki Yeşil)
                        # Yeşili SAĞINDA tutmak için SOLA git (Yeşil X - Offset)
                        target_x = best_green[0] - SAFE_OFFSET
                        mod_durumu = "SAG REF (Yesil Sagda)"
                        visual_lock = True

                    # --- GÜNCELLEME: VISION LOCK (JITTER ÖNLEME) ---
                    # 1. Durum: Bu karede balon gördük (visual_lock zaten True geldi)
                    if visual_lock:
                        last_vision_time = time.time()  # Zamanı güncelle
                        vision_lock_active = False


                    # 2. Durum: Bu karede balon YOK, hafızaya bakalım
                    else:
                        # Son görüşümüzden bu yana 0.5 saniyeden az geçtiyse
                        if (time.time() - last_vision_time) < 0.5:
                            visual_lock = True  # Sanal kilit! Görüyor gibi davran.
                            vision_lock_active = True  # Hafıza modu aktif
                            mod_durumu = "VISION MEMORY"
                        else:
                            # 0.5 saniyeyi geçtiyse artık Lidar'a düş
                            mod_durumu = "LIDAR KOR"

                    # --- 4. PID VE MOTOR SÜRME ---
                    pwm_correction = 0

                    if visual_lock:
                        if not vision_lock_active:
                            # CANLI GÖRÜYORSAK: Yeni PWM hesapla
                            err = target_x - center_x
                            kp = getattr(cfg, 'Kp_PIXEL', 0.3)
                            kd = getattr(cfg, 'Kd_PIXEL', 0.1)
                            P = err * kp
                            D = (err - last_pixel_error) * kd
                            pwm_correction = P + D
                            last_pixel_error = err

                            # Bu düzeltmeyi hafızaya at
                            last_pwm_correction = pwm_correction
                        else:
                            # HAFIZA MODUNDAYSAK: En son hesaplanan PWM'i kullan (Hesap yapma)
                            pwm_correction = last_pwm_correction

                    else:
                        # Lidar Koridor Yedek (Hiçbir şey yoksa)
                        if math.isinf(left_d) and math.isinf(right_d):
                            dist_diff = 0.0
                        else:
                            r_safe = 10.0 if math.isinf(right_d) else right_d
                            l_safe = 10.0 if math.isinf(left_d) else left_d
                            dist_diff = (r_safe - l_safe)
                        lkp = getattr(cfg, 'LIDAR_KORIDOR_KP', 30.0)
                        pwm_correction = np.clip(dist_diff * lkp, -150, 150)

                    # Yumuşatma
                    max_change = getattr(cfg, 'MAX_PWM_CHANGE', 60)
                    pwm_correction = np.clip(pwm_correction, -max_change, max_change)

                    # Motorlara Güç Ver
                    if mission_started and not manual_mode and not current_path:  # <--- BURASI DEĞİŞTİ
                        throttle = getattr(cfg, 'CRUISE_PWM', 80)
                        sol = int(np.clip(cfg.BASE_PWM + throttle + pwm_correction, 1100, 1900))
                        sag = int(np.clip(cfg.BASE_PWM + throttle - pwm_correction, 1100, 1900))
                        controller.set_servo(cfg.SOL_MOTOR, sol)
                        controller.set_servo(cfg.SAG_MOTOR, sag)
                    else:
                        controller.set_servo(cfg.SOL_MOTOR, 1500)
                        controller.set_servo(cfg.SAG_MOTOR, 1500)



            # -----------------------------------------------------------------------------------------
            # MOD 2: GPS (AÇIK ALAN)
            # -----------------------------------------------------------------------------------------
            elif nav_mode == "GPS":

                # GPS konumu zaten en başta (ortak alanda) alındı: ida_enlem, ida_boylam

                # GPS Sensör Verilerini Güncelle (Kamera zaten yukarıda okundu)
                if frame_hazir:
                    if zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT):
                        if ts_handler.is_new(sensors_data.get_imu_data()):
                            # 1. ZED Heading (Base)
                            raw_heading = sensors_data.get_magnetometer_data().magnetic_heading
                            zed_heading = magnetic_filter.update(raw_heading)
                            zed_heading = (zed_heading + 6) % 360

                            # 2. FC Heading
                            fc_heading = controller.get_heading()

                            # 3. Kaynak Seçimi (Config)
                            heading_source = getattr(cfg, 'HEADING_SOURCE', 'ZED')

                            if heading_source == 'FC' and fc_heading is not None:
                                magnetic_heading = fc_heading
                            elif heading_source == 'FUSED' and fc_heading is not None:
                                # Açısal Ortalama
                                diff = nav.signed_angle_difference(zed_heading, fc_heading)
                                magnetic_heading = (zed_heading + (diff * 0.5)) % 360
                            else:
                                magnetic_heading = zed_heading

                            heading_dogruluk = sensors_data.get_magnetometer_data().magnetic_heading_accuracy
                            if magnetic_heading is None:
                                pass

                                # Derinlik değerlerini ekrana yazma (Senin Kodun)
                    # output_frame yerine orijinal 'frame' üzerine basıp sonra tekrar resize edebiliriz
                    # ya da koordinatları uydururuz. Kolaylık olsun diye output_frame üzerine basacağız.
                    # Ancak tespit koordinatları 1280x720'ye göre. output_frame 960x540.
                    # Bu yüzden orijinal frame'e basıp tekrar resize etmek en doğrusu.
                    if detections:
                        coordinates = detections.xyxy.tolist()
                        for box in coordinates:
                            x1, y1, x2, y2 = map(int, box)
                            depth_val = depth.get_value(int((x2 + x1) / 2), int((y1 + y2) / 2))[1]
                            if not np.isnan(depth_val):
                                text = f"{depth_val:.2f} m"
                                cv2.putText(frame, text, (x2 - 60, y1 + 20), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 0, 255),
                                            2)

                        # frame üzerine yazı bastık, output_frame'i güncelle
                        if width < 1000:
                            output_frame = frame
                        else:
                            output_frame = cv2.resize(frame, (960, 540))

                target_lat = None
                target_lon = None

                # --- GÖREV 1: KANAL GEÇİŞİ (STATE MACHINE) ---
                if mevcut_gorev == "TASK1_APPROACH":
                    mevcut_gorev = "TASK1_STATE_ENTER"

                if mevcut_gorev in ["TASK1_STATE_ENTER", "TASK1_STATE_MID", "TASK1_STATE_EXIT"]:
                    # 1. Hedef Belirleme
                    if mevcut_gorev == "TASK1_STATE_ENTER":
                        target_lat = cfg.T1_GATE_ENTER_LAT
                        target_lon = cfg.T1_GATE_ENTER_LON
                    elif mevcut_gorev == "TASK1_STATE_MID":
                        target_lat = cfg.T1_GATE_MID_LAT
                        target_lon = cfg.T1_GATE_MID_LON
                    else:
                        target_lat = cfg.T1_GATE_EXIT_LAT
                        target_lon = cfg.T1_GATE_EXIT_LON

                    # 2. Geçiş Kontrolü (2 Metre)
                    dist_to_wp = nav.haversine(ida_enlem, ida_boylam, target_lat, target_lon)
                    if dist_to_wp < 2.0:
                        print(f"{Fore.GREEN}[TASK1] Waypoint {mevcut_gorev} Reached!{Style.RESET_ALL}")
                        if mevcut_gorev == "TASK1_STATE_ENTER":
                            mevcut_gorev = "TASK1_STATE_MID"
                        elif mevcut_gorev == "TASK1_STATE_MID":
                            mevcut_gorev = "TASK1_STATE_EXIT"
                        else:
                            mevcut_gorev = "TASK2_START"
                            print(f"{Fore.GREEN}[TASK1] Tamamlandı -> Task 2 Başlıyor (TASK2_START){Style.RESET_ALL}")

                # --- GÖREV 2: DEBRIS (ENGEL SAHASI) - STATE MACHINE ---
                # STATES: TASK2_START -> GO_TO_MID -> GO_TO_END -> SEARCH_PATTERN -> GREEN_MARKER_FOUND -> RETURN_HOME
                # ---------------------------------------------------------------------

                if 'task2_breadcrumbs' not in globals():
                    global task2_breadcrumbs
                    task2_breadcrumbs = []

                if 'task2_green_verify_count' not in globals():
                    global task2_green_verify_count
                    task2_green_verify_count = 0

                if 'task2_circle_center_x' not in globals():
                    global task2_circle_center_x
                    global task2_circle_center_y
                    task2_circle_center_x = 0
                    task2_circle_center_y = 0

                if mevcut_gorev == "TASK2_START":
                    # 1. T2_ZONE_ENTRY'ye git (Spot Turn Logic ile)
                    target_lat = cfg.T2_ZONE_ENTRY_LAT
                    target_lon = cfg.T2_ZONE_ENTRY_LON

                    # Transition: 2m yaklaşınca
                    if nav.haversine(ida_enlem, ida_boylam, target_lat, target_lon) < 2.0:
                        print(f"{Fore.GREEN}[TASK2] ENTRY REACHED -> GO_TO_MID{Style.RESET_ALL}")
                        mevcut_gorev = "TASK2_GO_TO_MID"

                elif mevcut_gorev == "TASK2_GO_TO_MID":
                    # 2. T2_ZONE_MID'e git (A* / Pure Pursuit)
                    target_lat = cfg.T2_ZONE_MID_LAT
                    target_lon = cfg.T2_ZONE_MID_LON

                    # Breadcrumbs (3 metrede bir)
                    if len(task2_breadcrumbs) == 0:
                        task2_breadcrumbs.append((ida_enlem, ida_boylam))
                    else:
                        last_lat, last_lon = task2_breadcrumbs[-1]
                        if nav.haversine(ida_enlem, ida_boylam, last_lat, last_lon) > 3.0:
                             task2_breadcrumbs.append((ida_enlem, ida_boylam))
                             # print(f"[TASK2] Breadcrumb Added: {len(task2_breadcrumbs)}")

                    if nav.haversine(ida_enlem, ida_boylam, target_lat, target_lon) < 2.0:
                        print(f"{Fore.GREEN}[TASK2] MID REACHED -> GO_TO_END{Style.RESET_ALL}")
                        mevcut_gorev = "TASK2_GO_TO_END"

                elif mevcut_gorev == "TASK2_GO_TO_END":
                    # 3. T2_ZONE_END'e git (A* / Pure Pursuit)
                    target_lat = cfg.T2_ZONE_END_LAT
                    target_lon = cfg.T2_ZONE_END_LON

                    # Breadcrumbs
                    if len(task2_breadcrumbs) > 0:
                         last_lat, last_lon = task2_breadcrumbs[-1]
                         if nav.haversine(ida_enlem, ida_boylam, last_lat, last_lon) > 3.0:
                              task2_breadcrumbs.append((ida_enlem, ida_boylam))
                              # print(f"[TASK2] Breadcrumb Added: {len(task2_breadcrumbs)}")

                    if nav.haversine(ida_enlem, ida_boylam, target_lat, target_lon) < 2.0:
                        print(f"{Fore.GREEN}[TASK2] END REACHED -> SEARCH_PATTERN{Style.RESET_ALL}")
                        mevcut_gorev = "TASK2_SEARCH_PATTERN"
                        # Search Pattern değişkenlerini sıfırla
                        task2_search_phase = 0
                        task2_search_laps = 0
                        task2_search_center_x = robot_x
                        task2_search_center_y = robot_y

                elif mevcut_gorev == "TASK2_SEARCH_PATTERN":
                    # Search for Green Marker (Class ID 4) - Real-time Detection
                    # 2 Laps (8 phases) around task2_search_center_x,y

                    found_green_live = False
                    found_green_x = 0
                    found_green_y = 0

                    if detections:
                        cids = detections.class_id.tolist()
                        coords = detections.xyxy.tolist()

                        for i, cid in enumerate(cids):
                            # Strict verification: Only ID 4 (Green Marker) as per request
                            if cid == 4:
                                # Calculate position
                                x1, y1, x2, y2 = map(int, coords[i])
                                cx = int((x1 + x2) / 2)
                                box_h = y2 - y1
                                target_cy = int(y2 - (box_h * 0.15))
                                cy = max(0, min(target_cy, height - 1))

                                err, dist_m = depth.get_value(cx, cy)

                                # Strict Distance Filter: < 5.0m to avoid noise
                                if not np.isnan(dist_m) and not np.isinf(dist_m) and 0.1 < dist_m < 5.0:
                                    # Calculate world coordinates
                                    hfov_rad = math.radians(getattr(cfg, 'CAM_HFOV', 110.0))
                                    pixel_offset = (cx - (width / 2)) / width
                                    angle_offset = -pixel_offset * hfov_rad
                                    obj_global_angle = robot_yaw + angle_offset

                                    found_green_x = robot_x + (dist_m * math.cos(obj_global_angle))
                                    found_green_y = robot_y + (dist_m * math.sin(obj_global_angle))

                                    found_green_live = True
                                    # Plot on map (update its position)
                                    update_landmark_memory("GREEN", found_green_x, found_green_y)
                                    break

                    if found_green_live:
                        task2_green_verify_count += 1
                        print(f"[TASK2] Verify Green: {task2_green_verify_count}/5")

                        if task2_green_verify_count >= 5:
                            print(f"{Fore.GREEN}[TASK2] GREEN MARKER CONFIRMED! -> CIRCLING OBJECT{Style.RESET_ALL}")
                            mevcut_gorev = "TASK2_GREEN_MARKER_FOUND"

                            # LOCK COORDINATES ONCE
                            task2_circle_center_x = found_green_x
                            task2_circle_center_y = found_green_y
                            print(f"[TASK2] Locked Center: ({task2_circle_center_x:.1f}, {task2_circle_center_y:.1f})")

                            # Smart Start: Calculate the closest phase relative to the LOCKED center
                            rel_x = robot_x - task2_circle_center_x
                            rel_y = robot_y - task2_circle_center_y
                            robot_angle_rad = math.atan2(rel_y, rel_x)

                            # Normalize angle to 0..2pi
                            if robot_angle_rad < 0: robot_angle_rad += 2 * math.pi

                            # Find closest phase (0=0, 1=pi/2, 2=pi, 3=3pi/2)
                            closest_phase = int(round(robot_angle_rad / (math.pi / 2.0))) % 4

                            # Start targeting the NEXT phase
                            task2_search_phase = closest_phase + 1
                            task2_circle_target_phase = task2_search_phase + 8

                            print(f"[TASK2] Robot Angle: {math.degrees(robot_angle_rad):.1f}deg -> Start Phase: {task2_search_phase}")

                            task2_search_laps = 0
                            task2_green_verify_count = 0
                    else:
                        # Reset if continuity is broken
                        if task2_green_verify_count > 0:
                            print(f"[TASK2] Verification Lost! Resetting counter.")
                        task2_green_verify_count = 0

                        # Continue Pattern
                        R = getattr(cfg, 'TASK2_SEARCH_DIAMETER', 2.0) / 2.0
                        # 4 points per lap. 2 laps = 8 points.
                        # Phase 0-3: Lap 1, Phase 4-7: Lap 2

                        if task2_search_phase >= 8:
                            print(f"{Fore.RED}[TASK2] MARKER NOT FOUND (TIMEOUT) -> RETURN HOME{Style.RESET_ALL}")
                            mevcut_gorev = "TASK2_RETURN_HOME"
                        else:
                            # Calculate target point on circle
                            # Angles: 0, 90, 180, 270 (deg) -> 0, pi/2, pi, 3pi/2
                            phase_mod = task2_search_phase % 4
                            angle_rad = phase_mod * (math.pi / 2.0)

                            # Target point
                            override_target_x = task2_search_center_x + (R * math.cos(angle_rad))
                            override_target_y = task2_search_center_y + (R * math.sin(angle_rad))

                            # Check arrival
                            dist_to_wp = math.sqrt((override_target_x - robot_x)**2 + (override_target_y - robot_y)**2)
                            if dist_to_wp < 1.0:
                                print(f"[TASK2] Search Phase {task2_search_phase} Reached.")
                                task2_search_phase += 1

                elif mevcut_gorev == "TASK2_GREEN_MARKER_FOUND":
                     # Circle the object (2m radius, 2 laps)
                     R = getattr(cfg, 'TASK2_SEARCH_DIAMETER', 2.0) / 2.0
                     if 'task2_circle_target_phase' not in locals(): task2_circle_target_phase = task2_search_phase + 8

                     if task2_search_phase >= task2_circle_target_phase:
                         print(f"{Fore.GREEN}[TASK2] OBJECT CIRCLED -> RETURN HOME{Style.RESET_ALL}")
                         mevcut_gorev = "TASK2_RETURN_HOME"
                     else:
                         # 4 points per lap (0, 90, 180, 270 degrees)
                         phase_mod = task2_search_phase % 4
                         angle_rad = phase_mod * (math.pi / 2.0)

                         # Calculate Target Point (Using LOCKED center)
                         override_target_x = task2_circle_center_x + (R * math.cos(angle_rad))
                         override_target_y = task2_circle_center_y + (R * math.sin(angle_rad))

                         # Distance Check
                         dist_to_wp = math.sqrt((override_target_x - robot_x)**2 + (override_target_y - robot_y)**2)

                         # Debug Print (Required for diagnosis)
                         print(f"[TASK2] Circling Phase {task2_search_phase} | Center:({task2_circle_center_x:.1f}, {task2_circle_center_y:.1f}) | Target:({override_target_x:.1f}, {override_target_y:.1f}) | Dist: {dist_to_wp:.2f}m")

                         if dist_to_wp < 1.5: # Increased tolerance to 1.5m to avoid stalling
                             print(f"[TASK2] Object Circle Phase {task2_search_phase} Reached (Dist: {dist_to_wp:.2f}m).")
                             task2_search_phase += 1

                elif mevcut_gorev == "TASK2_RETURN_HOME":
                    # Retrace breadcrumbs
                    if len(task2_breadcrumbs) > 0:
                        target_lat, target_lon = task2_breadcrumbs[-1]

                        # Check arrival
                        if nav.haversine(ida_enlem, ida_boylam, target_lat, target_lon) < 2.0:
                            print(f"[TASK2] Breadcrumb Reached. Remaining: {len(task2_breadcrumbs)-1}")
                            task2_breadcrumbs.pop()
                    else:
                        # Reached start or list empty
                        print(f"{Fore.GREEN}[TASK2] RETURN HOME COMPLETE -> NEXT TASK (TASK3){Style.RESET_ALL}")
                        mevcut_gorev = "TASK3_APPROACH"

                # ---------------------------------------------------------------------
                # TASK 3: SPEED CHALLENGE (TAM OTOMATİK)
                # ---------------------------------------------------------------------

                # Global Değişkenler
                if 'task3_breadcrumbs' not in globals():
                    global task3_breadcrumbs
                    task3_breadcrumbs = []

                if 'task3_turn_direction' not in globals():
                    global task3_turn_direction
                    task3_turn_direction = "right"  # Varsayılan: Sağdan

                if 'task3_retry_count' not in globals():
                    global task3_retry_count
                    task3_retry_count = 0

                if 'task3_gate_found' not in globals():
                    global task3_gate_found
                    task3_gate_found = False

                # ---------------------------------------------------------------------
                # NEW TASK 3 LOGIC
                # ---------------------------------------------------------------------

                elif mevcut_gorev == "TASK3_APPROACH":
                    # Backward compatibility for Config default
                    mevcut_gorev = "TASK3_START"

                elif mevcut_gorev == "TASK3_START":
                    target_lat = cfg.T3_GATE_SEARCH_LAT
                    target_lon = cfg.T3_GATE_SEARCH_LON

                    # Spot Turn Logic utilizes the generic spot turn block in motor control section
                    # Just setting target is enough for the generic logic to kick in if heading error is large.

                    dist_to_wp = nav.haversine(ida_enlem, ida_boylam, target_lat, target_lon)
                    if dist_to_wp < 2.0:
                        print(f"{Fore.GREEN}[TASK3] GATE SEARCH REACHED -> GATE APPROACH{Style.RESET_ALL}")
                        task3_success = False
                        mevcut_gorev = "TASK3_GATE_APPROACH"
                        task3_gate_found = False

                elif mevcut_gorev == "TASK3_GATE_APPROACH":
                    target_lat = cfg.T3_YELLOW_APPROACH_LAT
                    target_lon = cfg.T3_YELLOW_APPROACH_LON

                    # Breadcrumbs (every 3m)
                    if len(task3_breadcrumbs) == 0:
                        task3_breadcrumbs.append((ida_enlem, ida_boylam))
                    else:
                        last_lat, last_lon = task3_breadcrumbs[-1]
                        if nav.haversine(ida_enlem, ida_boylam, last_lat, last_lon) > 3.0:
                            task3_breadcrumbs.append((ida_enlem, ida_boylam))

                    # --- GATE CHECK & TURN DIRECTION ---
                    # 1. Turn Direction (Camera)
                    if detections:
                        cids = detections.class_id.tolist()
                        if any(c in [0, 3, 5] for c in cids): task3_turn_direction = "left"
                        if any(c in [1, 4, 12] for c in cids): task3_turn_direction = "right"

                        # 2. Gate Check (Simultaneous Camera)
                        has_red = any(c in [0, 3, 5] for c in cids)
                        has_green = any(c in [1, 4, 12] for c in cids)
                        if has_red and has_green:
                            task3_gate_found = True
                            print(f"{Fore.GREEN}[TASK3] GATE CONFIRMED (CAMERA){Style.RESET_ALL}")

                    # 3. Gate Check (Map Landmarks)
                    if not task3_gate_found and landmarks_memory:
                        r_found = False
                        g_found = False
                        for lm in landmarks_memory:
                            d = math.sqrt((lm["x"] - robot_x)**2 + (lm["y"] - robot_y)**2)
                            if d < 10.0:
                                if lm["color"] == "RED": r_found = True
                                if lm["color"] == "GREEN": g_found = True
                        if r_found and g_found:
                             task3_gate_found = True
                             print(f"{Fore.GREEN}[TASK3] GATE CONFIRMED (MAP){Style.RESET_ALL}")

                    # Check Arrival
                    dist_to_wp = nav.haversine(ida_enlem, ida_boylam, target_lat, target_lon)
                    if dist_to_wp < 2.0:
                        if task3_gate_found:
                            print(f"{Fore.GREEN}[TASK3] YELLOW APPROACH REACHED (GATE FOUND) -> SEARCH PATTERN{Style.RESET_ALL}")
                            mevcut_gorev = "TASK3_SEARCH_PATTERN"
                            task3_search_phase = 0
                            task3_search_center_x = robot_x
                            task3_search_center_y = robot_y
                            task3_search_laps = 0
                        else:
                            # FAIL -> RETRY LOOP 1
                            task3_retry_count += 1
                            if task3_retry_count <= 3:
                                print(f"{Fore.RED}[TASK3] GATE NOT FOUND (RETRY {task3_retry_count}/3) -> RETURN HOME{Style.RESET_ALL}")
                                mevcut_gorev = "TASK3_RETURN_HOME"
                            else:
                                print(f"{Fore.RED}[TASK3] GATE NOT FOUND (MAX RETRIES) -> ABORT{Style.RESET_ALL}")
                                mevcut_gorev = "TASK5_APPROACH"

                elif mevcut_gorev == "TASK3_SEARCH_PATTERN":
                    # Circular search (2m diameter, 2 laps)
                    yellow_obj = None
                    if landmarks_memory:
                         for lm in landmarks_memory:
                             if lm["color"] == "YELLOW":
                                 yellow_obj = lm
                                 break

                    if yellow_obj:
                         print(f"{Fore.GREEN}[TASK3] YELLOW OBJECT FOUND -> CIRCLING{Style.RESET_ALL}")
                         mevcut_gorev = "TASK3_YELLOW_FOUND"
                         task3_yellow_obj_x = yellow_obj["x"]
                         task3_yellow_obj_y = yellow_obj["y"]
                         task3_circle_phase = 0
                         # 2 Laps = 8 Phases
                         task3_circle_target_phase = 8
                    else:
                         R = getattr(cfg, 'TASK3_SEARCH_DIAMETER', 2.0) / 2.0
                         phases_per_lap = 4
                         total_phases = phases_per_lap * 2

                         if task3_search_phase >= total_phases:
                              task3_retry_count += 1
                              if task3_retry_count <= 3:
                                   print(f"{Fore.RED}[TASK3] YELLOW NOT FOUND (RETRY {task3_retry_count}) -> RETURN HOME{Style.RESET_ALL}")
                                   mevcut_gorev = "TASK3_RETURN_HOME"
                              else:
                                   print(f"{Fore.RED}[TASK3] YELLOW NOT FOUND (MAX RETRIES) -> RETURN HOME (ABORT){Style.RESET_ALL}")
                                   mevcut_gorev = "TASK3_RETURN_HOME"
                         else:
                              phase_mod = task3_search_phase % phases_per_lap
                              angle_rad = phase_mod * (math.pi / 2.0)
                              override_target_x = task3_search_center_x + (R * math.cos(angle_rad))
                              override_target_y = task3_search_center_y + (R * math.sin(angle_rad))

                              if math.sqrt((override_target_x - robot_x)**2 + (override_target_y - robot_y)**2) < 1.0:
                                   task3_search_phase += 1

                elif mevcut_gorev == "TASK3_YELLOW_FOUND":
                     # Circle 2 laps (8 phases)
                     R = getattr(cfg, 'TASK3_SEARCH_DIAMETER', 2.0) / 2.0

                     if 'task3_circle_target_phase' not in locals():
                         task3_circle_target_phase = 8

                     if task3_circle_phase >= task3_circle_target_phase:
                          print(f"{Fore.GREEN}[TASK3] CIRCLING COMPLETE -> RETURN HOME{Style.RESET_ALL}")
                          mevcut_gorev = "TASK3_RETURN_HOME"
                          task3_success = True
                     else:
                          offsets = []
                          if task3_turn_direction == "right":
                               # CW points: (R,0), (0,-R), (-R,0), (0,R)?
                               # Right Turn around object means keeping object to Right?
                               # Or moving Right?
                               # Task 2 code used: 0, 90, 180, 270.
                               # Let's stick to standard phases.
                               offsets = [(R, 0), (0, R), (-R, 0), (0, -R)]
                          else:
                               offsets = [(R, 0), (0, -R), (-R, 0), (0, R)]

                          off_x, off_y = offsets[task3_circle_phase % 4]
                          override_target_x = task3_yellow_obj_x + off_x
                          override_target_y = task3_yellow_obj_y + off_y

                          if math.sqrt((override_target_x - robot_x)**2 + (override_target_y - robot_y)**2) < 1.0:
                               task3_circle_phase += 1

                elif mevcut_gorev == "TASK3_RETURN_HOME":
                     # Reverse breadcrumbs
                     if len(task3_breadcrumbs) > 0:
                          target_lat, target_lon = task3_breadcrumbs[-1]
                          if nav.haversine(ida_enlem, ida_boylam, target_lat, target_lon) < 2.0:
                               task3_breadcrumbs.pop()
                     else:
                          if 'task3_success' in locals() and task3_success:
                               print(f"{Fore.GREEN}[TASK3] HOME REACHED (SUCCESS) -> NEXT TASK (TASK5){Style.RESET_ALL}")
                               mevcut_gorev = "TASK5_APPROACH"
                          elif task3_retry_count <= 3:
                               print(f"{Fore.YELLOW}[TASK3] HOME REACHED (RETRY) -> RESTARTING{Style.RESET_ALL}")
                               mevcut_gorev = "TASK3_START"
                               task3_breadcrumbs = []
                          else:
                               print(f"{Fore.RED}[TASK3] HOME REACHED (FAIL) -> NEXT TASK (TASK5){Style.RESET_ALL}")
                               mevcut_gorev = "TASK5_APPROACH"

                # TASK 5: DOCKING (SAĞ/SOL BOŞLUK TARAMALI)
                # ---------------------------------------------------------------------

                # Global Değişkenler
                if 'task5_dock_side' not in globals():
                    global task5_dock_side
                    task5_dock_side = "RIGHT"  # Varsayılan (Bulunca değişecek)

                # AŞAMA 1: MARİNA GİRİŞİNE GİT
                elif mevcut_gorev == "TASK5_APPROACH":
                    target_lat = cfg.T5_DOCK_APPROACH_LAT
                    target_lon = cfg.T5_DOCK_APPROACH_LON

                    if nav.haversine(ida_enlem, ida_boylam, target_lat, target_lon) < 4.0:
                        print(
                            f"{Fore.YELLOW}[GÖREV] Marina Girişine Varıldı -> KORİDORA GİRİLİYOR{Style.RESET_ALL}")
                        mevcut_gorev = "TASK5_ENTER"
                        # İçeri girerken GPS'i unutacağız, tamamen Lidar/Kör sürüş
                        target_lat = None

                        # AŞAMA 2: KORİDORDA İLERLE VE BOŞLUK ARA (SAĞ VEYA SOL)
                elif mevcut_gorev == "TASK5_ENTER":
                    target_lat = None  # GPS Yok

                    # --- BOŞLUK TESPİTİ (GAP DETECTION) ---
                    # Lidar verisinden SAĞ (30-110) ve SOL (-110 ile -30) sektörlere bak.
                    # Duvarlar genelde 1-1.5m mesafededir.
                    # Eğer mesafe > 2.0m olursa, orada boşluk var demektir.

                    gap_threshold = 1.5  # İstenilen Limit

                    avg_right = 0
                    avg_left = 0

                    if local_lidar_scan:
                        # Sağ Sektör (Genelde 90 derece civarı)
                        right_dists = [p[2] for p in local_lidar_scan if 70 < p[1] < 110 and p[2] > 0]
                        # Sol Sektör (Genelde -90 derece civarı -> 250-290 derece gibi gelebilir veya eksi)
                        # process_lidar_sectors fonksiyonunda açılar -180..180 formatına dönüyordu, onu kullanalım:
                        # Veya basitçe ham veriden:
                        left_dists = []
                        for p in local_lidar_scan:
                            angle = p[1]
                            dist = p[2]
                            # Açıyı normalize et (-180, 180)
                            if angle > 180: angle -= 360

                            if -110 < angle < -70 and dist > 0:
                                left_dists.append(dist)

                        # Ortalamaları Al
                        if right_dists: avg_right = sum(right_dists) / len(right_dists)
                        if left_dists: avg_left = sum(left_dists) / len(left_dists)

                        # --- KARAR MEKANİZMASI ---
                        if avg_right > gap_threshold:
                            print(
                                f"{Fore.CYAN}[GÖREV] SAĞDA BOŞLUK ({avg_right:.1f}m) -> PARK (SAĞ){Style.RESET_ALL}")
                            task5_dock_side = "RIGHT"
                            mevcut_gorev = "TASK5_DOCK"
                            task5_dock_timer = 0

                        elif avg_left > gap_threshold:
                            print(
                                f"{Fore.CYAN}[GÖREV] SOLDA BOŞLUK ({avg_left:.1f}m) -> PARK (SOL){Style.RESET_ALL}")
                            task5_dock_side = "LEFT"
                            mevcut_gorev = "TASK5_DOCK"
                            task5_dock_timer = 0

                # AŞAMA 3: İÇERİ GİR (KÖR DALIŞ)
                elif mevcut_gorev == "TASK5_DOCK":
                    target_lat = None
                    # Manevra Sürüş Bloğunda yapılacak (Timer ile)

                # AŞAMA 4: ÇIKIŞ
                elif mevcut_gorev == "TASK5_EXIT":
                    # Hedef: Task 5 Başlangıç Noktası (Çıkış Kapısı)
                    target_lat = cfg.T5_DOCK_APPROACH_LAT
                    target_lon = cfg.T5_DOCK_APPROACH_LON

                    # Eğer manevra bittiyse ve hedefe yaklaştıysak DUR
                    # (Timer 100'ü geçtiyse sürüş moduna geçilmiş demektir)
                    if 'task5_dock_timer' in globals() and task5_dock_timer > 80:
                        if nav.haversine(ida_enlem, ida_boylam, target_lat, target_lon) < 4.0:
                            print(f"{Fore.GREEN}[GÖREV] MARİNADAN ÇIKILDI -> GÖREVLER TAMAMLANDI.{Style.RESET_ALL}")
                            mission_started = False  # Robotu Durdur
                            controller.set_servo(cfg.SOL_MOTOR, 1500)
                            controller.set_servo(cfg.SAG_MOTOR, 1500)

                # --- GENEL NAVİGASYON HESABI ---
                # Hangi görevde olursak olalım, target_lat belirlendiyse hesap yap.
                if target_lat is not None:
                    adviced_course = nav.calculate_bearing(ida_enlem, ida_boylam, target_lat, target_lon)
                    aci_farki = nav.signed_angle_difference(magnetic_heading, adviced_course)
                    hedefe_mesafe = nav.haversine(ida_enlem, ida_boylam, target_lat, target_lon)

            # -----------------------------------------------------------------------------------------
            # 5. YAYIN VE KAYIT (HER DURUMDA ÇALIŞIR - EN SONA ALINDI)
            # -----------------------------------------------------------------------------------------
            if frame_hazir and output_frame is not None:
                stream_frame = cv2.resize(output_frame, (640, 360))

                # --- HARİTA GÖSTERİMİ VE PATH PLANNING TESTİ ---
                # HİBRİT SÜRÜŞ SİSTEMİ (MOTOR KONTROLÜ BURADA)
                # =========================================================================
                try:
                    # 1. Haritayı Hazırla (Senin Lidar Mapping kodun burada çalışıyor!)
                    if costmap_ready and costmap_img is not None:

                        # 1. HARİTAYI AL
                        # Task 2 dönerken Yeşili, Task 3 dönerken Sarıyı yoksay
                        ignore_green_buoys = (mevcut_gorev == "TASK2_GREEN_MARKER_FOUND")
                        ignore_yellow_buoys = (mevcut_gorev in ["TASK3_SEARCH_PATTERN", "TASK3_YELLOW_FOUND"])

                        nav_map, inflated_mask = get_inflated_nav_map(costmap_img,
                                                                      ignore_green=ignore_green_buoys,
                                                                      ignore_yellow=ignore_yellow_buoys)

                        # 2. HEDEFİ BELİRLE
                        tx_world, ty_world = None, None

                        # DURUM A: Manuel Override (Task 2 veya Task 3 Circling)
                        if (
                                mevcut_gorev in ["TASK2_SEARCH_PATTERN", "TASK2_GREEN_MARKER_FOUND", "TASK3_SEARCH_PATTERN", "TASK3_YELLOW_FOUND"]) and 'override_target_x' in locals():
                            tx_world = override_target_x
                            ty_world = override_target_y

                            # --- FAILSAFE DÜZELTMESİ (A* ÇÖKERSE KÖR SÜRÜŞ İÇİN) ---
                            # Eğer A* rota bulamazsa ve Blind Drive'a düşersek, robotun eski GPS hedefine değil,
                            # şu anki aktif daire noktasına (override_target) gitmesini sağlamalıyız.
                            # Bunun için 'aci_farki' (gps_angle) değişkenini YEREL AÇI ile eziyoruz.

                            # 1. Hedef Açısı (Radyan -> Derece)
                            local_target_angle = math.atan2(ty_world - robot_y, tx_world - robot_x)
                            local_target_deg = math.degrees(local_target_angle)

                            # 2. Robot Açısı (Radyan -> Derece)
                            # NOT: ZED Yaw açısı ile hedef açısı aynı referans düzlemindedir (Start Point).
                            robot_yaw_deg = math.degrees(robot_yaw)

                            # 3. Açı Farkını Hesapla ve EZE (Override)
                            aci_farki = nav.signed_angle_difference(robot_yaw_deg, local_target_deg)
                            gps_angle = aci_farki

                        # DURUM B: Standart GPS / Vision
                        else:
                            gps_angle = aci_farki if 'aci_farki' in locals() else None

                            # --- HYBRID NAV LOGIC (STEP-SCAN-STEP) ---
                            if mevcut_gorev in ["TASK2_GO_TO_MID", "TASK2_GO_TO_END", "TASK3_GATE_APPROACH"]:
                                # 1. Mesafe Kontrolü (Hedeve vardık mı?)
                                need_new_target = True
                                if hybrid_local_target:
                                    d_local = math.sqrt((hybrid_local_target[0] - robot_x)**2 + (hybrid_local_target[1] - robot_y)**2)
                                    if d_local > 0.5:
                                        need_new_target = False  # Hala gidiyoruz

                                # 2. Yeni Hedef Belirleme (Vardıysak veya ilk kez)
                                if need_new_target and 'aci_farki' in locals():
                                    step_dist = getattr(cfg, 'HYBRID_STEP_DIST', 2.0)
                                    # GPS Hedefine doğru 2 metre ileri nokta koy
                                    h_tx, h_ty = get_hybrid_point(robot_x, robot_y, robot_yaw, aci_farki, step_dist)

                                    # Harita dışına taşmayı önle (Basit clamp)
                                    # (Gerekirse eklenebilir ama A* zaten hata verir)
                                    hybrid_local_target = (h_tx, h_ty)

                                if hybrid_local_target:
                                    tx_world, ty_world = hybrid_local_target
                                else:
                                    # Fallback
                                    tx_world, ty_world, _ = select_mission_target(
                                        robot_x, robot_y, landmarks_memory, robot_yaw, nav_map,
                                        gps_target_angle_err=gps_angle
                                    )
                            else:
                                # NORMAL MOD (Task 1, Task 2 Search, Task 3 Search vb.)
                                hybrid_local_target = None # Reset
                                tx_world, ty_world, _ = select_mission_target(
                                    robot_x, robot_y, landmarks_memory, robot_yaw, nav_map,
                                    gps_target_angle_err=gps_angle
                                )

                        # 3. A* ROTA PLANLAMA
                        plan_timer += 1
                        if plan_timer > 4:
                            plan_timer = 0
                            if tx_world is not None:
                                # Determine Bias based on Task 2 Requirements
                                planner_bias = 0.0
                                if mevcut_gorev in ["TASK2_GO_TO_MID", "TASK2_GO_TO_END", "TASK2_RETURN_HOME"]:
                                    planner_bias = 0.5  # High penalty for deviating from the straight line

                                new_path = planner.get_path_plan(
                                    (robot_x, robot_y), (tx_world, ty_world), nav_map,
                                    costmap_center_m, COSTMAP_RES_M_PER_PX, COSTMAP_SIZE_PX,
                                    bias_to_goal_line=planner_bias,
                                    heuristic_weight=getattr(cfg, 'A_STAR_HEURISTIC_WEIGHT', 2.5),
                                    cone_deg=45.0
                                )
                                if new_path:
                                    current_path = new_path
                                else:
                                    current_path = None

                        # 3. SÜRÜŞ KARARI (PLAN A vs PLAN B)
                        if mission_started and not manual_mode:

                            # --- ÖZEL DURUM: TASK 5 (MARİNA İÇİ - LIDAR SÜRÜŞÜ) ---
                            if mevcut_gorev == "TASK5_ENTER":
                                # GPS YOK, PLANNER YOK -> LIDAR KORİDORU
                                # Amaç: İki duvarın ortasından yavaşça git (Duvarlara sürtme)

                                # Hata Hesabı: (Sağ Mesafe - Sol Mesafe)
                                # Eğer bir taraf sonsuz (inf) ise o tarafı dikkate alma
                                r_val = right_d if not math.isinf(right_d) else 2.0
                                l_val = left_d if not math.isinf(left_d) else 2.0

                                err = r_val - l_val

                                # Basit P Kontrolcü
                                kp_dock = 50
                                rot = np.clip(err * kp_dock, -100, 100)

                                # Yavaş İleri + Düzeltme
                                FWD = 1580  # Yavaş ve dikkatli hız
                                controller.set_servo(cfg.SOL_MOTOR, int(FWD + rot))
                                controller.set_servo(cfg.SAG_MOTOR, int(FWD - rot))


                            elif mevcut_gorev == "TASK5_DOCK":
                                # KÖR PARK MANEVRASI (Zamanlayıcı ile)
                                task5_dock_timer += 1

                                # Yöne Göre Motor Güçlerini Ayarla
                                # Varsayılan: SAĞA DÖN (Sol motor güçlü, Sağ motor zayıf)
                                turn_pwm_sol = 1650
                                turn_pwm_sag = 1350

                                if task5_dock_side == "LEFT":  # SOLA DÖN (Sağ motor güçlü, Sol motor zayıf)
                                    turn_pwm_sol = 1350
                                    turn_pwm_sag = 1650

                                if task5_dock_timer < 25:  # İlk 2.5 saniye -> DÖN
                                    print(f"[DOCK] {task5_dock_side} Tarafına Dönülüyor...")
                                    controller.set_servo(cfg.SOL_MOTOR, turn_pwm_sol)
                                    controller.set_servo(cfg.SAG_MOTOR, turn_pwm_sag)

                                elif task5_dock_timer < 65:  # Sonraki 4 saniye -> DÜZ GİR
                                    print("[DOCK] İçeri Giriliyor...")
                                    controller.set_servo(cfg.SOL_MOTOR, 1600)
                                    controller.set_servo(cfg.SAG_MOTOR, 1600)

                                else:
                                    # DUR VE BİTİR
                                    print("[DOCK] PARK TAMAMLANDI - DUR!")
                                    controller.set_servo(cfg.SOL_MOTOR, 1500)
                                    controller.set_servo(cfg.SAG_MOTOR, 1500)
                                    mevcut_gorev = "TASK5_EXIT"
                                    task5_dock_timer = 0


                            elif mevcut_gorev == "TASK5_EXIT":
                                # MARİNADAN ÇIKIŞ MANEVRASI
                                task5_dock_timer += 1
                                # A) PARK YERİNDEN GERİ ÇIK (0 - 4.5 sn / 45 döngü)
                                if task5_dock_timer < 45:
                                    print("[DOCK] Geri Çıkılıyor...")
                                    controller.set_servo(cfg.SOL_MOTOR, 1400)
                                    controller.set_servo(cfg.SAG_MOTOR, 1400)

                                # B) ÇIKIŞA DOĞRU DÖN (4.5 - 7.5 sn / 30 döngü)
                                elif task5_dock_timer < 75:
                                    # Sağdaki parka girdiysek, çıkış SAĞIMIZDA kalır.
                                    # Soldaki parka girdiysek, çıkış SOLUMUZDA kalır.
                                    # (Geri çıktığımızda burnumuz hala parka bakıyor, çıkış yanımızda kalıyor)
                                    print(f"[DOCK] Çıkış Yönüne ({task5_dock_side}) Dönülüyor...")
                                    turn_pwm_sol = 1650
                                    turn_pwm_sag = 1350

                                    if task5_dock_side == "LEFT":  # Sol parka girdiysek Sola Dön
                                        turn_pwm_sol = 1350
                                        turn_pwm_sag = 1650

                                    # (Default RIGHT için zaten ayarlı)
                                    controller.set_servo(cfg.SOL_MOTOR, turn_pwm_sol)
                                    controller.set_servo(cfg.SAG_MOTOR, turn_pwm_sag)

                                # C) KORİDORDA İLERLE (START NOKTASINA KADAR)
                                else:
                                    print("[DOCK] Başlangıç Noktasına Sürülüyor...")
                                    # TASK5_ENTER mantığını aynen kullan (Lidar ile Ortala)
                                    # Hata Hesabı: (Sağ Mesafe - Sol Mesafe)
                                    r_val = right_d if not math.isinf(right_d) else 2.0
                                    l_val = left_d if not math.isinf(left_d) else 2.0
                                    err = r_val - l_val
                                    kp_dock = 50
                                    rot = np.clip(err * kp_dock, -100, 100)
                                    FWD = 1580  # Yavaş İleri
                                    controller.set_servo(cfg.SOL_MOTOR, int(FWD + rot))
                                    controller.set_servo(cfg.SAG_MOTOR, int(FWD - rot))

                            # --- TASK 1 & TASK 2 START CUSTOM CONTROL (Heading/Spot Turn) ---
                            # 1. Hizalama Gerektiren Durum Analizi
                            should_force_alignment = False

                            # A) Daima Hizalama Yapanlar (A* Kullanmaz, Sadece Heading/Pusula Sürüşü)
                            # TASK2_START burada kalsın ki start noktasındaki stabil davranışı korusun.
                            if mevcut_gorev in ["TASK1_STATE_ENTER", "TASK1_STATE_MID", "TASK1_STATE_EXIT",
                                                "TASK2_START", "TASK3_START"]:
                                should_force_alignment = True

                            # B) HİBRİT MOD (Önce Dön, Sonra A* Kullan) - Sizin istediğiniz kısım burası
                            # TASK2 Mid ve End aşamalarında, eğer kafa çok dönükse önce düzeltecek.
                            elif mevcut_gorev in ["TASK2_GO_TO_MID", "TASK2_GO_TO_END", "TASK3_GATE_APPROACH"]:
                                threshold = getattr(cfg, 'HYBRID_HEADING_THRESHOLD', 30.0)
                                # Eğer açı farkı eşikten büyükse A*'ı bekleme, önce dön.
                                if abs(aci_farki) > threshold:
                                    should_force_alignment = True
                                else:
                                    # Açı düzeldi, artık A* (current_path) bloğuna düşebiliriz.
                                    should_force_alignment = False

                            # -------------------------------------------------------------------------
                            # BLOĞU UYGULA
                            # -------------------------------------------------------------------------

                            # DURUM 1: Zorla Hizalama (Spot Turn & Heading Hold)
                            if should_force_alignment:
                                # 1. Vision Scan ve Bearing Hesabı (Mevcut kodunuzdaki gibi)
                                visual_bearing = None
                                best_red = None
                                best_green = None

                                if detections and mevcut_gorev.startswith("TASK1"):
                                    coords = detections.xyxy.tolist()
                                    cids = detections.class_id.tolist()
                                    reds = []
                                    greens = []

                                    for i, cid in enumerate(cids):
                                        x1, y1, x2, y2 = map(int, coords[i])
                                        cx = int((x1 + x2) / 2)
                                        cy = int((y1 + y2) / 2)

                                        # Depth check
                                        err, dist_m = depth.get_value(cx, cy)
                                        if np.isnan(dist_m) or np.isinf(dist_m) or dist_m > 10.0:
                                            continue

                                        if cid in [0, 3, 5]: reds.append((dist_m, cx))
                                        elif cid in [1, 4, 12]: greens.append((dist_m, cx))

                                    if reds: best_red = min(reds, key=lambda x: x[0])
                                    if greens: best_green = min(greens, key=lambda x: x[0])

                                    if best_red and best_green:
                                        # Check if they form a gate (similar distance)
                                        if abs(best_red[0] - best_green[0]) < 2.5:
                                            gate_cx = (best_red[1] + best_green[1]) / 2

                                            # Calculate Bearing
                                            hfov_rad = math.radians(getattr(cfg, 'CAM_HFOV', 110.0))
                                            pixel_offset = (gate_cx - (width / 2)) / width
                                            angle_offset = pixel_offset * hfov_rad
                                            visual_bearing = magnetic_heading + math.degrees(angle_offset)

                                # 2. Select Target Bearing
                                final_bearing = 0
                                if visual_bearing is not None:
                                    final_bearing = visual_bearing
                                else:
                                    if target_lat is not None:
                                        final_bearing = nav.calculate_bearing(ida_enlem, ida_boylam, target_lat, target_lon)
                                    else:
                                        final_bearing = magnetic_heading

                                # 3. Calculate Error & Control
                                heading_err = nav.signed_angle_difference(magnetic_heading, final_bearing)

                                # Spot Turn Check
                                threshold = getattr(cfg, 'SPOT_TURN_THRESHOLD', 45.0)
                                spot_pwm = getattr(cfg, 'SPOT_TURN_PWM', 200)

                                if abs(heading_err) > threshold:
                                    # Spot Turn
                                    if heading_err > 0: # Target Right
                                        controller.set_servo(cfg.SOL_MOTOR, 1500 + spot_pwm)
                                        controller.set_servo(cfg.SAG_MOTOR, 1500 - spot_pwm)
                                    else: # Target Left
                                        controller.set_servo(cfg.SOL_MOTOR, 1500 - spot_pwm)
                                        controller.set_servo(cfg.SAG_MOTOR, 1500 + spot_pwm)
                                else:
                                    # Forward P-Control
                                    kp = 1.0
                                    corr = heading_err * kp
                                    fwd = cfg.BASE_PWM + 100

                                    sol = int(np.clip(fwd + corr, 1100, 1900))
                                    sag = int(np.clip(fwd - corr, 1100, 1900))
                                    controller.set_servo(cfg.SOL_MOTOR, sol)
                                    controller.set_servo(cfg.SAG_MOTOR, sag)

                            # --- STANDART A* SÜRÜŞÜ (TASK 1, 2, 3 - PLANNER VARSA) ---
                            elif current_path:

                                # --- YENİ: REAKTİF ENGEL KAÇINMA (LIDAR) ---
                                # Eğer önümüzde aniden bir engel belirirse (center_danger),
                                # A* rotasını iptal et ve anında dur. Bir sonraki döngüde yeniden planlama yapılacak.
                                # Lidar verisi: center_danger, left_d, center_d, right_d (Yukarıda hesaplandı)

                                # Sadece GPS modunda ve kör sürüşte değilsek (Task 5 hariç)
                                if nav_mode == "GPS" and mevcut_gorev not in ["TASK5_ENTER", "TASK5_DOCK", "TASK5_EXIT"]:
                                    # --- REACTIVE LAYER (SAFETY) ---
                                    if center_danger:
                                        print(f"{Back.RED}[AVOID] ENGEL TESPİT EDİLDİ ({center_d:.2f}m)! ROTA İPTAL -> YENİDEN HESAPLANIYOR...{Style.RESET_ALL}")
                                        controller.set_servo(cfg.SOL_MOTOR, 1500)
                                        controller.set_servo(cfg.SAG_MOTOR, 1500)
                                        current_path = None  # Rotayı sil -> Replan tetiklenir
                                        continue  # Döngüyü kır, aşağıya gitme
                                # -------------------------------------------

                                path_lost_time = None

                                # HIZ AYARI: Task 3'te ("ENTER" veya "RETURN" aşamalarında) daha hızlı git
                                current_base_pwm = cfg.BASE_PWM
                                if mevcut_gorev in ["TASK3_ENTER", "TASK3_RETURN"]:
                                    current_base_pwm += getattr(cfg, "T3_SPEED_PWM", 100)

                                # Get current speed for Dynamic Pure Pursuit
                                cur_spd = controller.get_horizontal_speed()
                                if cur_spd is None: cur_spd = 0.0

                                pp_sol, pp_sag, raw_target, current_error, pruned_path = planner.pure_pursuit_control(
                                    robot_x, robot_y, robot_yaw, current_path,
                                    current_speed=cur_spd,
                                    base_speed=current_base_pwm,
                                    prev_error=prev_heading_error
                                )
                                current_path = pruned_path
                                prev_heading_error = current_error

                                # --- TARGET SMOOTHING (HEDEF YUMUŞATMA) ---
                                # Hedef aniden zıplamasın, %70 eski hedefi koru.
                                if 'prev_pp_target' not in globals():
                                    global prev_pp_target
                                    prev_pp_target = None

                                if raw_target is not None:
                                    if prev_pp_target is None:
                                        pp_target = raw_target
                                    else:
                                        # FORMÜL: 0.7 * Eski + 0.3 * Yeni
                                        new_x = 0.7 * prev_pp_target[0] + 0.3 * raw_target[0]
                                        new_y = 0.7 * prev_pp_target[1] + 0.3 * raw_target[1]
                                        pp_target = (new_x, new_y)

                                    prev_pp_target = pp_target
                                else:
                                    pp_target = None
                                    prev_pp_target = None


                                controller.set_servo(cfg.SOL_MOTOR, pp_sol)
                                controller.set_servo(cfg.SAG_MOTOR, pp_sag)
                                acil_durum_aktif_mi = False


                            # --- PLAN B: AKILLI BEKLEME & MİNİK KAÇIŞ MODU (Yol Bulunamadı!) ---
                            else:
                                # 1. ZAMANLAYICIYI KONTROL ET

                                current_now = time.time()

                                # Yol ilk defa kaybolduysa sayacı başlat ve motorları durdur (NEFES AL)
                                if path_lost_time is None:
                                    path_lost_time = current_now
                                    print(f"{Back.YELLOW}[PLANNER] Yol Kayıp -> Veri Bekleniyor (WAIT STATE)...{Style.RESET_ALL}")
                                    # Anında dur
                                    controller.set_servo(cfg.SOL_MOTOR, 1500)
                                    controller.set_servo(cfg.SAG_MOTOR, 1500)

                                # Ne kadar süredir bekliyoruz?
                                wait_duration = current_now - path_lost_time

                                # Lidar verisini al (Her aşamada lazım)
                                center_danger, left_d, center_d, right_d = process_lidar_sectors(local_lidar_scan, max_dist=10.0)

                                # --- FAZ 0: BLIND GPS FALLBACK (CRITICAL FIX) ---
                                # If A* fails but we have a valid GPS target, just GO!
                                blind_drive_dur = getattr(cfg, 'BLIND_DRIVE_SECONDS', 3.0)
                                blind_safe_dist = getattr(cfg, 'BLIND_DRIVE_SAFE_DIST', 0.7)
                                if wait_duration < blind_drive_dur and gps_angle is not None and center_d > blind_safe_dist:
                                    print(f"{Back.RED}[FAILSAFE] BLIND GPS DRIVE (Err: {gps_angle:.1f}){Style.RESET_ALL}")

                                    # Simple P-Control
                                    kp_blind = 1.0
                                    turn_val = np.clip(gps_angle * kp_blind, -150, 150)
                                    base_blind = cfg.BASE_PWM + 100

                                    controller.set_servo(cfg.SOL_MOTOR, int(base_blind + turn_val))
                                    controller.set_servo(cfg.SAG_MOTOR, int(base_blind - turn_val))

                                # ---------------------------------------------------------
                                # FAZ 1: BEKLE VE GÖZLEM YAP (Blind süresinden sonra)
                                # ---------------------------------------------------------
                                elif wait_duration < (blind_drive_dur + 2.0):
                                    # Hiçbir şey yapma. Sensörlerin gürültüyü temizlemesine ve
                                    # kameranın yeni bir kare yakalamasına izin ver.
                                    pass

                                # ---------------------------------------------------------
                                # FAZ 2: MICRO ESCAPE / MİNİK KAÇIŞ
                                # ---------------------------------------------------------
                                elif wait_duration < (blind_drive_dur + 5.0):
                                    print(f"{Fore.RED}[PLANNER] Yol Yok -> MICRO ESCAPE (Minik Adımlar){Style.RESET_ALL}")

                                    # Yine de önümüz doluysa (Duvar dibindeysek) GİTME!
                                    if center_danger:
                                        print(f"[ACİL] ESCAPE ENGELLENDİ! Ön Dolu ({center_d:.2f}m)")
                                        controller.set_servo(cfg.SOL_MOTOR, 1500)
                                        controller.set_servo(cfg.SAG_MOTOR, 1500)
                                    else:
                                        # ÇOK YAVAŞ İLERLE (Costmap hücresinden çıkmak için)
                                        CREEP_SPEED = 40  # Eskiden 60 idi, şimdi daha temkinli

                                        # Yön düzeltme (Koridor mantığı - Eskisi gibi ama daha yumuşak)
                                        lidar_correction = 0
                                        PUSH = 80 # İtme kuvvetini azalttık
                                        if left_d < 2.0:
                                            lidar_correction = +PUSH # Sağa it
                                        elif right_d < 2.0:
                                            lidar_correction = -PUSH # Sola it

                                        # Hedefe Dönme Girişimi (Varsa)
                                        target_corr = 0
                                        if gps_angle is not None:
                                            target_corr = cfg.KpACI * gps_angle * 0.5 # Etkisini azalttık

                                        total_corr = (lidar_correction * 0.8) + (target_corr * 0.2)

                                        # Hafifçe motorlara güç ver
                                        controller.set_servo(cfg.SOL_MOTOR, int(cfg.BASE_PWM + CREEP_SPEED + total_corr))
                                        controller.set_servo(cfg.SAG_MOTOR, int(cfg.BASE_PWM + CREEP_SPEED - total_corr))

                                # ---------------------------------------------------------
                                # FAZ 3: FAIL SAFE / YERİNDE DÖNÜŞ (5.0+ Saniye)
                                # ---------------------------------------------------------
                                else:
                                    print(f"{Back.MAGENTA}[ACİL] PLANNER ÇÖKTÜ -> YERİNDE DÖNÜŞ (FAIL SAFE){Style.RESET_ALL}")
                                    # Artık ileri gitmek manasız, olduğu yerde dönerek yeni açı/yol ara
                                    # Sola doğru yavaş dönüş
                                    controller.set_servo(cfg.SOL_MOTOR, 1580)
                                    controller.set_servo(cfg.SAG_MOTOR, 1420)

                                # Konsola bilgi bas (Debug)
                                # print(f"[AUTO] Path Len:{len(current_path)} Sol:{pp_sol} Sağ:{pp_sag}")

                            # --- GÖRSELLEŞTİRME (DEBUG) ---
                            vis_map = costmap_img.copy()

                            # a) Güvenlik Alanlarını Gri Yap
                            vis_map[inflated_mask > 0] = 170
                            # b) Gerçek Engelleri Siyah Yap (Üste yaz)
                            real_obstacles = (costmap_img < 100)
                            vis_map[real_obstacles] = 0

                            # c) Rota Çizgisi (Yeşil Yılan) - YENİ
                            if current_path:
                                path_pixels = []
                                for p in current_path:
                                    px = world_to_pixel(p[0], p[1])
                                    if px: path_pixels.append(px)

                                if len(path_pixels) > 1:
                                    cv2.polylines(vis_map, [np.array(path_pixels)], False, (200, 200, 200), 2)

                            # d) Pure Pursuit Tavşanı (Mavi Daire) - YENİ
                            if pp_target:
                                pp_pix = world_to_pixel(pp_target[0], pp_target[1])
                                if pp_pix:
                                    cv2.circle(vis_map, pp_pix, 6, (255, 0, 0), -1)

                            # e) Hedef Nokta (X)
                            if tx_world is not None:
                                t_pix = world_to_pixel(tx_world, ty_world)
                                if t_pix:
                                    cv2.drawMarker(vis_map, t_pix, 50, cv2.MARKER_CROSS, 20, 2)

                            # ... (Buradan sonrası mevcut zoom ve ekrana basma kodunla aynı devam eder) ...
                            c_px = world_to_pixel(robot_x, robot_y)
                            if c_px is None:
                                cx, cy = 500, 500
                            else:
                                cx, cy = c_px

                            # Kırpma (Zoom)
                            disp_size = 220
                            radius = disp_size // 2
                            h_map_orig, w_map_orig = vis_map.shape[:2]

                            y1 = max(0, cy - radius)
                            y2 = min(h_map_orig, cy + radius)
                            x1 = max(0, cx - radius)
                            x2 = min(w_map_orig, cx + radius)

                            map_roi = vis_map[y1:y2, x1:x2]  # costmap_img yerine vis_map

                            if map_roi.size > 0:
                                # 1. Gri haritayı BGR (Renkli) formata çevir
                                map_display = cv2.cvtColor(map_roi, cv2.COLOR_GRAY2BGR)

                                # 2. Resmi Boyutlandır (Zoom penceresini ayarla)
                                map_display = cv2.resize(map_display, (disp_size, disp_size))

                                # -----------------------------------------------------------
                                # YENİ EKLENEN KISIM: HAFIZADAKİ ŞAMANDIRALARI ÇİZ
                                # -----------------------------------------------------------
                                # ROI (Kesilen parça) boyutu ile Ekran boyutu arasındaki oran
                                # x2-x1: Kesilen parçanın orijinal genişliği
                                roi_w = x2 - x1
                                roi_h = y2 - y1

                                if roi_w > 0 and roi_h > 0:
                                    scale_x = disp_size / roi_w
                                    scale_y = disp_size / roi_h

                                    for lm in landmarks_memory:
                                        # Sadece güvenilir olanları çiz
                                        if lm["confidence"] >= 1:
                                            g_px = world_to_pixel(lm["x"], lm["y"])  # Global pikseli al

                                            if g_px:
                                                # Lokal ROI koordinatına çevir:
                                                # (Global_Piksel - Kesim_Baslangici) * Ölçek
                                                roi_x = int((g_px[0] - x1) * scale_x)
                                                roi_y = int((g_px[1] - y1) * scale_y)

                                                # Eğer bu şamandıra şu anki mini harita ekranına sığıyorsa çiz
                                                if 0 <= roi_x < disp_size and 0 <= roi_y < disp_size:
                                                    l_color = (0, 255, 0) if lm["color"] == "GREEN" else (0, 0, 255)

                                                    # Şamandıra Çemberi
                                                    cv2.circle(map_display, (roi_x, roi_y), 6, l_color, 2)
                                                    # ID Numarası (Beyaz renkli)
                                                    cv2.putText(map_display, str(lm["id"]), (roi_x + 5, roi_y + 5),
                                                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                                # -----------------------------------------------------------

                                # Robotun Merkezi ve Oku
                                center_disp = (disp_size // 2, disp_size // 2)
                                arrow_len = 20
                                end_x = int(center_disp[0] + arrow_len * math.cos(robot_yaw))
                                end_y = int(center_disp[1] - arrow_len * math.sin(robot_yaw))

                                cv2.arrowedLine(map_display, center_disp, (end_x, end_y), (0, 255, 0), 2, tipLength=0.3)
                                cv2.circle(map_display, center_disp, 3, (0, 0, 255), -1)
                                cv2.rectangle(map_display, (0, 0), (disp_size - 1, disp_size - 1), (0, 0, 255), 2)

                                # Ana görüntüye yapıştır
                                h_main, w_main, _ = stream_frame.shape
                                h_m, w_m, _ = map_display.shape
                                y_off = h_main - h_m - 10
                                x_off = w_main - w_m - 10
                                stream_frame[y_off:y_off + h_m, x_off:x_off + w_m] = map_display

                except Exception as e:
                    print(f"Harita gömme hatası: {e}")

                # YENİ: YEREL MONİTÖR GÖSTERİMİ
                # =================================================================
                if getattr(cfg, 'SHOW_LOCAL_WINDOW', False):
                    cv2.imshow("RoboBoat Monitor", stream_frame)
                    # 'q' tuşuna basılırsa kapat (Opsiyonel manuel çıkış)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        raise KeyboardInterrupt

                # --- 1. SOL TARAFTAKİ ORTAK BİLGİLER ---
                cv2.putText(stream_frame, f"MOD: {nav_mode} ({mod_durumu})", (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                if 'current_fps' in locals():
                    cv2.putText(stream_frame, f"FPS: {current_fps}", (10, 45),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                cv2.putText(stream_frame, f"sol:{utils.nint(controller.get_servo_pwm(cfg.SOL_MOTOR))}", (10, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.putText(stream_frame, f"sag:{utils.nint(controller.get_servo_pwm(cfg.SAG_MOTOR))}", (10, 95),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                # --- 2. LIDAR BİLGİLERİ (SADECE ÖN TARAF FİLTRESİ) ---
                lidar_count = 0
                lidar_min = 0
                lidar_avg = 0

                if local_lidar_scan:
                    front_dists = []
                    for p in local_lidar_scan:
                        # p[1]=Angle, p[2]=Distance
                        angle = p[1]
                        dist = p[2]
                        if dist > 0:
                            # Açıyı -180 ile 180 arasına çek
                            if angle > 180: angle -= 360

                            # BURASI ÖNEMLİ: Sadece Ön Tarafı (-90 ile +90) listeye ekle
                            if -90 < angle < 90:
                                front_dists.append(dist)

                    # İstatistikleri sadece ön taraf verisiyle hesapla
                    if front_dists:
                        lidar_count = len(front_dists)
                        lidar_min = min(front_dists)
                        lidar_avg = np.mean(front_dists)

                cv2.putText(stream_frame, f"LIDAR Nokta(On): {lidar_count}", (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 0), 2)
                cv2.putText(stream_frame, f"LIDAR Min(On): {lidar_min:.0f}", (10, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 0), 2)

                # --- 3. SAĞ TARAF (SADECE GPS MODUNDA) ---
                if nav_mode == "GPS":
                    x_pos = 350

                    dist_str = f"H.Msf: {hedefe_mesafe:.1f}m" if 'hedefe_mesafe' in locals() and hedefe_mesafe is not None else "H.Msf: -"
                    cv2.putText(stream_frame, dist_str, (x_pos, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

                    adv_str = f"Rota: {adviced_course:.0f}" if 'adviced_course' in locals() else "Rota: -"
                    cv2.putText(stream_frame, adv_str, (x_pos, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 255), 1)

                    head_str = f"Head: {magnetic_heading:.0f}" if 'magnetic_heading' in locals() and magnetic_heading is not None else "Head: -"
                    cv2.putText(stream_frame, head_str, (x_pos, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

                    fix_str = f"GPS: {controller.get_gps_fix_type_verbose()}"
                    cv2.putText(stream_frame, fix_str, (x_pos, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                    lat_str = f"La:{ida_enlem:.5f}" if ida_enlem is not None else "La:-"
                    lon_str = f"Lo:{ida_boylam:.5f}" if ida_boylam is not None else "Lo:-"
                    cv2.putText(stream_frame, lat_str, (x_pos, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.putText(stream_frame, lon_str, (x_pos, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                # --- 4. KAYIT VE YAYIN ---
                if writer:
                    # Orijinal büyük kareyi kaydet
                    writer.enqueue(output_frame)

                if getattr(cfg, 'STREAM', False):
                    try:
                        # OPTİMİZASYON 2: Kaliteyi %15 yap (Veri tasarrufu)
                        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 45]

                        # Yayına KÜÇÜK VE YAZILI (stream_frame) kareyi gönder
                        ret, buffer = cv2.imencode('.jpg', stream_frame, encode_param)

                        if ret:
                            data = pickle.dumps(buffer)
                            size_pack = struct.pack("!Q", len(data))
                            client_socket.sendall(size_pack + data)
                    except Exception:
                        pass
            # 5. Telemetri Gönderimi (Senin kodun)
            if 'current_fps' in locals():
                # 9 Noktalı Yeni Yapı
                gps_points = {
                    "id": 1,
                    "GPS1": {"lat": float(getattr(cfg, "T1_GATE_ENTER_LAT", 0.0)),
                             "lon": float(getattr(cfg, "T1_GATE_ENTER_LON", 0.0))},
                    "GPS2": {"lat": float(getattr(cfg, "T1_GATE_MID_LAT", 0.0)),
                             "lon": float(getattr(cfg, "T1_GATE_MID_LON", 0.0))},
                    "GPS3": {"lat": float(getattr(cfg, "T1_GATE_EXIT_LAT", 0.0)),
                             "lon": float(getattr(cfg, "T1_GATE_EXIT_LON", 0.0))},

                    "GPS4": {"lat": float(getattr(cfg, "T2_ZONE_ENTRY_LAT", 0.0)),
                             "lon": float(getattr(cfg, "T2_ZONE_ENTRY_LON", 0.0))},
                    "GPS5": {"lat": float(getattr(cfg, "T2_ZONE_MID_LAT", 0.0)),
                             "lon": float(getattr(cfg, "T2_ZONE_MID_LON", 0.0))},
                    "GPS6": {"lat": float(getattr(cfg, "T2_ZONE_END_LAT", 0.0)),
                             "lon": float(getattr(cfg, "T2_ZONE_END_LON", 0.0))},

                    "GPS7": {"lat": float(getattr(cfg, "T3_GATE_SEARCH_LAT", 0.0)),
                             "lon": float(getattr(cfg, "T3_GATE_SEARCH_LON", 0.0))},
                    "GPS8": {"lat": float(getattr(cfg, "T3_YELLOW_APPROACH_LAT", 0.0)),
                             "lon": float(getattr(cfg, "T3_YELLOW_APPROACH_LON", 0.0))},

                    "GPS9": {"lat": float(getattr(cfg, "T5_DOCK_APPROACH_LAT", 0.0)),
                             "lon": float(getattr(cfg, "T5_DOCK_APPROACH_LON", 0.0))},
                }

                hs = controller.get_horizontal_speed()
                payload = {
                    "t_ms": datetime.datetime.now().strftime('%H:%M:%S'),

                    # 2) SOL İTİCİ İTKİ İSTEĞİ (PWM)
                    "SOL_İTİCİ_İTKİ_İSTEĞİ_PWM": utils.nint(controller.get_servo_pwm(cfg.SOL_MOTOR)),

                    # 3) SAĞ İTİCİ İTKİ İSTEĞİ (PWM)
                    "SAĞ_İTİCİ_İTKİ_İSTEĞİ_PWM": utils.nint(controller.get_servo_pwm(cfg.SAG_MOTOR)),

                    # 4) İDA GERÇEK HIZ (m/s)
                    "İDA_GERÇEK_HIZ_mps": utils.nfloat(hs),

                    # 5) GERÇEK HEADING (°)
                    "GERÇEK_HEADING_deg": (
                        f"{magnetic_heading:.0f}" if 'magnetic_heading' in locals() and magnetic_heading is not None else None),

                    # 6) HEDEF HEADING (°)
                    "HEDEF_HEADING_deg": utils.nint(adviced_course) if 'adviced_course' in locals() else 0,

                    # 7) HEADING SAĞLIĞI
                    "HEADING_SAĞLIĞI": magnetic_heading_state if 'magnetic_heading_state' in locals() else "UNKNOWN",

                    # 8) SONRAKİ GÖREV NOKTASI
                    "SONRAKİ_GÖREV_NOKTASI": mevcut_gorev,

                    # 9) MEVCUT KONUM (lat/lon ayrı anahtarlarla)
                    "MEVCUT_KONUM": {"lat": utils.nfloat(ida_enlem), "lon": utils.nfloat(ida_boylam)},

                    # 10) KALAN MESAFE (m)
                    "KALAN_MESAFE_m": utils.nint(hedefe_mesafe) if 'hedefe_mesafe' in locals() else 0,

                    # 11) MANUEL MOD
                    "MANUEL_MOD": bool(manual_mode),

                    "FPS": utils.nfloat(round(zed.get_current_fps())) if 'zed' in locals() else 0,
                }
                payload["GÖREV_NOKTALARI"] = gps_points

                tx.send(payload)  # asenkron çalışıyor
                # controller.set_servo(1,1500)



    except (KeyboardInterrupt, utils.EmergencyShutdown):
        print("[INFO] Ctrl+C alındı, kapanıyor...")
        controller.set_servo(cfg.SOL_MOTOR, 1500)
        controller.set_servo(cfg.SAG_MOTOR, 1500)
    finally:

        if relay:
            print("[INFO] Motor güçleri kesiliyor...")
            relay.cleanup()
        cv2.destroyAllWindows()

        # --- YENİ LIDAR KAPATMA ---
        print("[INFO] LIDAR thread'i ve bağlantısı kapatılıyor...")
        is_running_g = False  # Arka plan thread'ine durma sinyali gönder
        if lidar_g:
            lidar_g.stop()
            lidar_g.stop_motor()
            lidar_g.disconnect()
        # --- BİTTİ ---
        # --- kaynakları daima kapat ---
        try:
            if writer:  # Writer varsa durdur
                writer.stop()
        except Exception as e:
            print(f"[WARN] writer stop: {e}")

        try:
            cmd_rx.stop()
        except Exception:
            pass

        try:
            telemetry.close()
        except Exception:
            pass

        try:
            controller.disarm_vehicle()
        except Exception:
            pass

        try:
            if os.environ.get("DISPLAY"):
                cv2.destroyAllWindows()
        except Exception:
            pass

        try:
            zed.close()
        except Exception:
            pass

        print("[INFO] Temiz çıkış tamamlandı.")


if __name__ == "__main__":
    main()