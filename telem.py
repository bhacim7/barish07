import serial
import json
import math
import numpy as np
import threading
import queue
import time
import enum

def _json_default(o):
    # Enum ise .name döndür (örn: HEADING_STATE.GOOD -> "GOOD")
    if isinstance(o, enum.Enum):
        return o.name
    # datetime gibi objeler gelirse örnek:
    # if isinstance(o, datetime.datetime):
    #     return o.isoformat()
    # Diğer tüm tipleri güvenli şekilde stringe çevir
    return str(o)

class TelemetrySender:
    def __init__(self, port: str, baud: int):
        self.port = port
        self.baud = baud
        self.ser = None
        self.enabled = False
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.enabled = True
            print(f"[TELEM] Connected {self.port} @ {self.baud}")
        except Exception as e:
            print(f"[TELEM] Serial open failed: {e}. Telemetry disabled.")

    @staticmethod
    def _clean_num(x):
        try:
            if x is None:
                return None
            xf = float(x)
            return xf if math.isfinite(xf) else None
        except Exception:
            return None

    def send(self, payload: dict):
        if not self.enabled:
            return
        try:
            # sayıları temizle (NaN/inf -> None) ki JSON düzgün olsun
            clean = {k: (self._clean_num(v) if isinstance(v, (int, float, np.floating, np.integer)) else v)
                     for k, v in payload.items()}
            line = json.dumps(clean, ensure_ascii=False, default=_json_default) + "\n"
            self.ser.write(line.encode("utf-8"))
        except Exception as e:
            print(f"[TELEM] Send failed: {e}")

    def close(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass

class CommandReceiver(threading.Thread):
    def __init__(self, telemetry, out_queue: queue.Queue):
        super().__init__(daemon=True)
        self.telemetry = telemetry
        self.ser = getattr(telemetry, "ser", None)
        self.out_queue = out_queue
        self._stop = threading.Event()

    def run(self):
        if not self.ser:
            print("[CMD] Telemetry serial not available; command receiver disabled.")
            return
        while not self._stop.is_set():
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    time.sleep(0.01); continue
                # Sadece komut JSON'larını kabul et
                try:
                    obj = json.loads(line)
                    if isinstance(obj, dict) and "cmd" in obj:
                        self.out_queue.put(obj)
                except Exception:
                    # Telemetry JSON'ları gelirse (teorik olarak echo) yok say
                    pass
            except Exception:
                time.sleep(0.05)

    def stop(self):
        self._stop.set()

class TelemetryTx:
    def __init__(self, telemetry, max_hz=10, queue_size=256):
        self.telemetry = telemetry
        self.q = queue.Queue(maxsize=queue_size)
        self.max_hz = max_hz
        self.stop = threading.Event()
        self.t = threading.Thread(target=self._worker, daemon=True)
        self.t.start()

    def send(self, payload):
        """Ana döngüyü bloklamadan kuyruğa at.
        Kuyruk doluysa en eskisini at, yenisini ekle (son durum önemli)."""
        try:
            self.q.put_nowait(payload)
        except queue.Full:
            try:
                self.q.get_nowait()
            except queue.Empty:
                pass
            finally:
                # tekrar dene; doluysa bırak (ana döngü beklemesin)
                try: self.q.put_nowait(payload)
                except queue.Full: pass

    def _worker(self):
        period = 1.0 / max(1, self.max_hz)
        next_t = time.perf_counter()
        while not self.stop.is_set():
            # Bu periyotta birikenleri tek pakete indir (coalesce: en son geçerli)
            last = None
            try:
                # min bekleme ile bir öğe al
                last = self.q.get(timeout=period)
                # kuyruğu boşalt, sadece en son kalacak
                while True:
                    last = self.q.get_nowait()
            except queue.Empty:
                pass

            if last is not None:
                try:
                    # JSON pahalıysa burada orjson ya da önceden encode kullanabilirsin
                    self.telemetry.send(last)
                except Exception as e:
                    # ağ kesilirse ana döngü etkilenmesin
                    print(f"[TELEM] send fail: {e}")
                    pass

            # sabit yayın hızı
            now = time.perf_counter()
            sleep_t = next_t - now
            if sleep_t > 0:
                time.sleep(sleep_t)
            next_t += period

    def close(self, timeout=1.0):
        self.stop.set()
        self.t.join(timeout=timeout)



def handle_command(cmd: dict, controller, cfg, manual_mode: bool, mission_started: bool):
        ctype = cmd.get("cmd")

        if ctype == "set_manual":
            # value: true/false
            val = bool(cmd.get("value"))
            manual_mode = val
            if val is False:
                mission_started = True
            print(f"[CMD] manual_mode -> {manual_mode} (mission_started={mission_started})")

            # --- 2. GPS NOKTASI GÜNCELLEME (9 NOKTALI SİSTEM) ---

        elif ctype == "set_gps":

            try:

                idx = int(cmd.get("index"))

                lat = float(cmd.get("lat"))

                lon = float(cmd.get("lon"))

                # 9 Noktalı Yeni Harita

                if idx == 1:

                    cfg.T1_GATE_ENTER_LAT, cfg.T1_GATE_ENTER_LON = lat, lon

                    print(f"[CMD] GPS1 (T1 ENTER) Güncellendi: {lat}, {lon}")


                elif idx == 2:

                    cfg.T1_GATE_MID_LAT, cfg.T1_GATE_MID_LON = lat, lon

                    print(f"[CMD] GPS2 (T1 MID) Güncellendi: {lat}, {lon}")


                elif idx == 3:

                    cfg.T1_GATE_EXIT_LAT, cfg.T1_GATE_EXIT_LON = lat, lon

                    print(f"[CMD] GPS3 (T1 EXIT) Güncellendi: {lat}, {lon}")


                elif idx == 4:

                    cfg.T2_ZONE_ENTRY_LAT, cfg.T2_ZONE_ENTRY_LON = lat, lon

                    print(f"[CMD] GPS4 (T2 ENTRY) Güncellendi: {lat}, {lon}")


                elif idx == 5:

                    cfg.T2_ZONE_MID_LAT, cfg.T2_ZONE_MID_LON = lat, lon

                    print(f"[CMD] GPS5 (T2 MID) Güncellendi: {lat}, {lon}")


                elif idx == 6:

                    cfg.T2_ZONE_END_LAT, cfg.T2_ZONE_END_LON = lat, lon

                    print(f"[CMD] GPS6 (T2 END) Güncellendi: {lat}, {lon}")


                elif idx == 7:

                    cfg.T3_GATE_SEARCH_LAT, cfg.T3_GATE_SEARCH_LON = lat, lon

                    print(f"[CMD] GPS7 (T3 SEARCH) Güncellendi: {lat}, {lon}")


                elif idx == 8:

                    cfg.T3_YELLOW_APPROACH_LAT, cfg.T3_YELLOW_APPROACH_LON = lat, lon

                    print(f"[CMD] GPS8 (T3 YELLOW) Güncellendi: {lat}, {lon}")


                elif idx == 9:

                    cfg.T5_DOCK_APPROACH_LAT, cfg.T5_DOCK_APPROACH_LON = lat, lon

                    print(f"[CMD] GPS9 (T5 DOCK) Güncellendi: {lat}, {lon}")


                else:

                    print(f"[CMD] Geçersiz GPS İndeksi: {idx}")


            except Exception as e:

                print(f"[CMD] set_gps hatası: {e}")

        elif ctype == "manual_pwm":
            # left, right: PWM values (only if manual_mode True)
            if manual_mode:
                try:
                    left = int(cmd.get("left"))
                    right = int(cmd.get("right"))
                    controller.set_servo(cfg.SOL_MOTOR, left)
                    controller.set_servo(cfg.SAG_MOTOR, right)
                    print(f"[CMD] manual PWM -> L:{left} R:{right}")
                except Exception as e:
                    print(f"[CMD] manual_pwm error: {e}")
            else:
                print("[CMD] manual_pwm ignored (manual_mode=False)")

        elif ctype == "get_gps":
            # İsteğe bağlı: GCS mevcut GPS noktalarını görmek isterse
            # Bir sonraki payload'a GÖREV_NOKTALARI alanını ekleyerek yanıt vereceğiz
            pass

        else:
            print(f"[CMD] Unknown command: {ctype}")

        return manual_mode, mission_started
