import Jetson.GPIO as GPIO
import time
import atexit
import config as cfg

class SingleMotorRelay:
    def __init__(self, pin):
        self.pin = pin
        self._setup()
        atexit.register(self.cleanup)

    def _setup(self):
        """GPIO kurulumunu yapar."""
        try:
            mode = GPIO.getmode()
            if mode is None:
                if getattr(cfg, 'GPIO_MODE', 'BOARD') == 'BCM':
                    GPIO.setmode(GPIO.BCM)
                else:
                    GPIO.setmode(GPIO.BOARD)

            # Pini çıkış yap ve başlangıçta KAPALI (LOW) tut
            GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)

        except Exception as e:
            print(f"[HATA] Role kurulum hatasi: {e}")

    def power_on(self):
        """Röleyi açar (Motorlara güç gider)."""
        try:
            GPIO.output(self.pin, GPIO.HIGH)
            # print("[GUC] Motorlar AKTIF.")
        except Exception as e:
            print(f"[HATA] Role acma hatasi: {e}")

    def power_off(self):
        """Röleyi kapatır (Güç kesilir)."""
        try:
            GPIO.output(self.pin, GPIO.LOW)
            print(f"[GUC] Motorlar KAPATILDI.")
        except Exception as e:
            print(f"[HATA] Role kapatma hatasi: {e}")

    def cleanup(self):
        self.power_off()
        try:
            GPIO.cleanup(self.pin)
        except:
            pass