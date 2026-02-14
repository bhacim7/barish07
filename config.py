import os

# =============================================================================
# ÇALIŞMA MODU SEÇİMİ
# =============================================================================
# "GPS":    Eski kodlar çalışır. Açık alan.
# "VISION": Kapalı alan. Sadece Lidar ve Kamera.
NAV_MODE = "GPS"

# =============================================================================
# ORTAK DONANIM AYARLARI (Her iki modda geçerli)
# =============================================================================
SERIAL_PORT = os.getenv("TELEM_PORT", "/dev/ttyUSB0")
SERIAL_BAUD = int(os.getenv("TELEM_BAUD", "57600"))

SOL_MOTOR=1
SAG_MOTOR=3


BASE_PWM = 1500
MIN_PWM_LIMIT = 1100
MAX_PWM_LIMIT = 1900

GPIO_MODE = "BOARD"

# Motor Röle Pinleri
MOTOR_RELAY_PIN = 15

# ESC Başlatma Bekleme Süresi (Saniye)
# Röle açıldıktan sonra ESC'lerin dıt-dıt sesini bitirmesi için beklenir.
ESC_INIT_DELAY = 3.0

# Heading Kaynağı Seçimi
# "ZED": Sadece ZED2i Manyetometresi (Mevcut Sistem)
# "FC": Sadece Uçuş Kontrol Kartı (Orange Cube / Pixhawk)
# "FUSED": İkisinin Ortalaması (Basit Füzyon)
HEADING_SOURCE = "ZED"


# =============================================================================
# MODA ÖZEL AYARLAR
# =============================================================================

if NAV_MODE == "VISION":
    # --- VISION (KAPALI ALAN) AYARLARI ---
    print(">> SISTEM VISION MODUNDA BASLATILIYOR")

    # Lidar ve Güvenlik
    STREAM = True
    RECORD_VIDEO = False
    LIDAR_BAUDRATE = 1000000  # RPLIDAR S3 Hızı
    LIDAR_MAX_DIST = 10.0  # Tarama menzili
    LIDAR_ACIL_DURMA_M = 1.0  # (YENİ) Bu mesafede engel varsa ACİL KAÇIŞ yap
    LIDAR_KORIDOR_KP = 30.0  # Kamera körse Lidar ile ortalama katsayısı

    # Kamera Navigasyon (PID)
    Kp_PIXEL = 0.3
    Kd_PIXEL = 0.1

    # Motor ve Hız Kontrolü
    CRUISE_PWM = 150  # Sabit hız
    ESCAPE_PWM = 300  # Kaçış şiddeti
    MAX_PWM_CHANGE = 120  # (YENİ) Ani hız değişim limiti (Yumuşatma için)

else:
    # --- GPS (AÇIK ALAN) AYARLARI ---
    print(">> SISTEM GPS MODUNDA BASLATILIYOR")

    # Sistem Ayarları
    STREAM = True  # Yer istasyonuna görüntü basılsın mı?
    RECORD_VIDEO = False  # SD karta kayıt yapılsın mı?
    SHOW_LOCAL_WINDOW = False  # True: Monitörde pencere aç, False: Arka planda çalış
    YOLO_CONFIDENCE = 0.40  # Hava durumuna göre düşürüp artırabilirsin
    CAM_HFOV = 110.0  # ZED 2i Yatay Görüş Açısı (Derece)

    # Task Toggle Flags
    ENABLE_TASK3 = True
    ENABLE_TASK5 = True

    MEVCUT_GOREV = "TASK5_APPROACH"

    # Sürüş Katsayıları
    KpACI = 2.5  # Sadece A* çalışmazsa devreye giren 'Plan B' için kullanılır.

    # Task 3 için Ekstra Hız (Base PWM üzerine eklenir)
    T3_SPEED_PWM = 100
     # FAILSAFE SETTINGS
    MAP_DECAY_AMOUNT = 5
    BLIND_DRIVE_SECONDS = 3.0
    BLIND_DRIVE_SAFE_DIST = 0.7
    MAX_TILT_ANGLE = 5.0

    # A* Tuning
    A_STAR_HEURISTIC_WEIGHT = 2.5
    LIDAR_FREE_GAIN = 25
    LIDAR_OCCUPIED_GAIN = 80

    # Hybrid Navigation Logic
    HYBRID_STEP_DIST = 2.0  # Meters
    HYBRID_HEADING_THRESHOLD = 30.0 # Degrees
    TASK2_SEARCH_DIAMETER = 4.0
    TASK3_SEARCH_DIAMETER = 2.0

    # --- YARIŞMA KOORDİNATLARI (RoboBoat 2026) ---

    # TASK 1: Evacuation Route (Kanal Geçişi)
    T1_GATE_ENTER_LAT = 40.8091428
    T1_GATE_ENTER_LON = 29.2619132
    T1_GATE_EXIT_LAT = 40.8089363
    T1_GATE_EXIT_LON = 29.2619269
    T1_GATE_MID_LAT = 40.8090249
    T1_GATE_MID_LON =  29.2618888

    # Task 1 Navigation Settings
    SPOT_TURN_THRESHOLD = 15.0  # Degrees
    SPOT_TURN_PWM = 130

    # TASK 2: Debris Clearance (Engel ve Işık Sahası)
    # T2 Başlangıcı (Referans)
    T2_ZONE_ENTRY_LAT = 40.8091428
    T2_ZONE_ENTRY_LON = 29.2619132
    # TASK 2 (ADDED)
    T2_ZONE_MID_LAT = 40.8090249
    T2_ZONE_MID_LON = 29.2618888
    # T2 Bitişi (Hedef Burası)
    T2_ZONE_END_LAT = 40.8089363
    T2_ZONE_END_LON = 29.2619269

    # TASK 3: Speed Challenge (Sürat Kapısı) - REFACTORED
    # Yeni 5 Noktali Sistem (Placeholders - Update with real GPS)
    T3_START_LAT = 0.0
    T3_START_LON = 0.0

    T3_MID_LAT = 0.0
    T3_MID_LON = 0.0

    T3_RIGHT_LAT = 0.0
    T3_RIGHT_LON = 0.0

    T3_END_LAT = 0.0
    T3_END_LON = 0.0

    T3_LEFT_LAT = 0.0
    T3_LEFT_LON = 0.0

    # TASK 5: Docking (Park Etme)
    T5_DOCK_APPROACH_LAT = 40.8092732
    T5_DOCK_APPROACH_LON = 29.2626729

    # Dock Geometry (U-Shape Detection)
    DOCK_WIDTH_M = 3.0  # Width between the side pontoons
    DOCK_DEPTH_M = 2.0  # Depth of the slip (recess)
    DOCK_TOLERANCE_M = 0.5  # Tolerance for width/depth matching

    # TASK 6: HARBOR ALERT (SESLİ KOMUT)
    T6_AUDIO_CHUNK = 2048
    T6_AUDIO_RATE = 44100
    T6_AUDIO_CHANNELS = 1
    T6_TARGET_FREQS = {
        '600Hz': (570, 630, 6000000),
        '800Hz': (760, 840, 2000000),
        '1000Hz': (950, 1050, 6000000),
        '1200Hz': (1140, 1260, 6000000),
    }
       
