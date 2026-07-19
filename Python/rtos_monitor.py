import csv
import json
import math
import queue
import random
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


SERIAL_BAUD_RATE = 115200
SIMULATION_PERIOD_SECONDS = 0.1
FLASH_FILE = Path("flash_log.csv")


class RtosMonitor(tk.Tk):

    def __init__(self):
        super().__init__()

        self.title("STM32 RTOS Monitor")
        self.geometry("900x570")
        self.resizable(False, False)

        self.data_queue = queue.Queue()
        self.status_queue = queue.Queue()

        self.stop_event = threading.Event()
        self.serial_connection = None
        self.worker_thread = None

        self.mode_var = tk.StringVar(value="simulation")
        self.port_var = tk.StringVar()
        self.flash_enabled_var = tk.BooleanVar(value=True)

        self.sensor_var = tk.StringVar(value="Bekleniyor")
        self.x_var = tk.StringVar(value="0")
        self.y_var = tk.StringVar(value="0")
        self.z_var = tk.StringVar(value="0")
        self.motion_var = tk.StringVar(value="Yok")
        self.green_var = tk.StringVar(value="Kapalı")
        self.time_var = tk.StringVar(value="0 ms")
        self.record_count_var = tk.StringVar(value="0")
        self.status_var = tk.StringVar(value="Hazır")

        self.record_count = self.find_record_count()

        self.create_widgets()
        self.refresh_ports()

        self.after(50, self.process_queues)
        self.protocol("WM_DELETE_WINDOW", self.close_application)

    def create_widgets(self):

        source_frame = ttk.LabelFrame(
            self,
            text="Veri kaynağı",
            padding=10
        )
        source_frame.pack(
            fill="x",
            padx=10,
            pady=10
        )

        ttk.Radiobutton(
            source_frame,
            text="Simülasyon",
            variable=self.mode_var,
            value="simulation"
        ).grid(row=0, column=0, padx=5)

        ttk.Radiobutton(
            source_frame,
            text="STM32 seri port",
            variable=self.mode_var,
            value="serial"
        ).grid(row=0, column=1, padx=5)

        ttk.Label(
            source_frame,
            text="Port:"
        ).grid(row=0, column=2, padx=(20, 5))

        self.port_combobox = ttk.Combobox(
            source_frame,
            textvariable=self.port_var,
            width=15,
            state="readonly"
        )
        self.port_combobox.grid(row=0, column=3, padx=5)

        ttk.Button(
            source_frame,
            text="Portları yenile",
            command=self.refresh_ports
        ).grid(row=0, column=4, padx=5)

        ttk.Button(
            source_frame,
            text="Başlat",
            command=self.start_source
        ).grid(row=0, column=5, padx=(20, 5))

        ttk.Button(
            source_frame,
            text="Durdur",
            command=self.stop_source
        ).grid(row=0, column=6, padx=5)

        body_frame = ttk.Frame(self)
        body_frame.pack(
            fill="both",
            expand=True,
            padx=10
        )

        data_frame = ttk.LabelFrame(
            body_frame,
            text="STM32 verileri",
            padding=15
        )
        data_frame.pack(
            side="left",
            fill="y",
            padx=(0, 10)
        )

        rows = [
            ("Sensör", self.sensor_var),
            ("X ivmesi", self.x_var),
            ("Y ivmesi", self.y_var),
            ("Z ivmesi", self.z_var),
            ("Hareket", self.motion_var),
            ("Yeşil LED", self.green_var),
            ("STM32 zamanı", self.time_var),
            ("Flash kayıt sayısı", self.record_count_var)
        ]

        for row_number, (name, variable) in enumerate(rows):

            ttk.Label(
                data_frame,
                text=name + ":"
            ).grid(
                row=row_number,
                column=0,
                sticky="w",
                pady=8
            )

            ttk.Label(
                data_frame,
                textvariable=variable,
                font=("Arial", 11, "bold")
            ).grid(
                row=row_number,
                column=1,
                sticky="w",
                padx=15,
                pady=8
            )

        ttk.Checkbutton(
            data_frame,
            text="Verileri flash_log.csv dosyasına kaydet",
            variable=self.flash_enabled_var
        ).grid(
            row=len(rows),
            column=0,
            columnspan=2,
            sticky="w",
            pady=(20, 5)
        )

        ttk.Button(
            data_frame,
            text="Flash kayıtlarını temizle",
            command=self.clear_flash_file
        ).grid(
            row=len(rows) + 1,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=5
        )

        oled_frame = ttk.LabelFrame(
            body_frame,
            text="128 × 64 OLED önizleme",
            padding=15
        )
        oled_frame.pack(
            side="right",
            fill="both",
            expand=True
        )

        self.oled_canvas = tk.Canvas(
            oled_frame,
            width=550,
            height=320,
            background="black"
        )
        self.oled_canvas.pack(
            fill="both",
            expand=True
        )

        self.draw_oled({
            "time": 0,
            "x": 0,
            "y": 0,
            "z": 0,
            "motion": 0,
            "sensor_ok": 0,
            "green": 0
        })

        status_label = ttk.Label(
            self,
            textvariable=self.status_var,
            relief="sunken",
            anchor="w",
            padding=5
        )
        status_label.pack(
            fill="x",
            padx=10,
            pady=10
        )

        self.record_count_var.set(str(self.record_count))

    def refresh_ports(self):

        if serial is None:
            self.port_combobox["values"] = []
            self.port_var.set("")
            self.status_var.set(
                "pyserial kurulu değil. Simülasyon modu kullanılabilir."
            )
            return

        ports = [
            item.device
            for item in serial.tools.list_ports.comports()
        ]

        self.port_combobox["values"] = ports

        if ports:
            self.port_var.set(ports[0])
        else:
            self.port_var.set("")

    def start_source(self):

        self.stop_source(show_status=False)
        self.stop_event.clear()

        if self.mode_var.get() == "simulation":

            self.worker_thread = threading.Thread(
                target=self.simulation_worker,
                daemon=True
            )

            self.worker_thread.start()
            self.status_var.set("Simülasyon çalışıyor.")

        else:

            if serial is None:
                messagebox.showerror(
                    "Hata",
                    "pyserial kurulu değil.\n\n"
                    "Komut satırına şunu yaz:\n"
                    "pip install pyserial"
                )
                return

            port = self.port_var.get()

            if not port:
                messagebox.showwarning(
                    "Port seçilmedi",
                    "Bir COM port seç."
                )
                return

            try:
                self.serial_connection = serial.Serial(
                    port=port,
                    baudrate=SERIAL_BAUD_RATE,
                    timeout=0.2
                )

            except serial.SerialException as error:

                messagebox.showerror(
                    "Seri port açılamadı",
                    str(error)
                )
                return

            self.worker_thread = threading.Thread(
                target=self.serial_worker,
                daemon=True
            )

            self.worker_thread.start()

            self.status_var.set(
                f"{port} açıldı, baud rate: {SERIAL_BAUD_RATE}"
            )

    def stop_source(self, show_status=True):

        self.stop_event.set()

        if self.serial_connection is not None:

            try:
                self.serial_connection.close()
            except serial.SerialException:
                pass

            self.serial_connection = None

        if show_status:
            self.status_var.set("Veri kaynağı durduruldu.")

    def simulation_worker(self):

        start_time = time.monotonic()

        while not self.stop_event.is_set():

            elapsed = time.monotonic() - start_time

            x = int(
                20 * math.sin(elapsed)
                + random.randint(-4, 4)
            )

            y = int(
                15 * math.cos(elapsed * 0.7)
                + random.randint(-4, 4)
            )

            z = int(
                52
                + 7 * math.sin(elapsed * 0.4)
                + random.randint(-2, 2)
            )

            motion = 0

            # Yaklaşık her 6 saniyede bir hareket oluştur.
            if 4.5 < elapsed % 6.0 < 5.5:

                motion = 1

                x = random.choice([-1, 1]) * random.randint(65, 95)
                y = random.choice([-1, 1]) * random.randint(40, 75)

            data = {
                "time": int(elapsed * 1000),
                "x": max(-128, min(127, x)),
                "y": max(-128, min(127, y)),
                "z": max(-128, min(127, z)),
                "motion": motion,
                "sensor_ok": 1,
                "green": int(elapsed / 3) % 2
            }

            self.data_queue.put(data)

            self.stop_event.wait(
                SIMULATION_PERIOD_SECONDS
            )

    def serial_worker(self):

        while not self.stop_event.is_set():

            try:
                raw_line = self.serial_connection.readline()

                if not raw_line:
                    continue

                text = raw_line.decode(
                    "utf-8",
                    errors="strict"
                ).strip()

                if not text:
                    continue

                data = json.loads(text)

                required_fields = (
                    "time",
                    "x",
                    "y",
                    "z",
                    "motion",
                    "sensor_ok",
                    "green"
                )

                for field in required_fields:

                    if field not in data:
                        raise ValueError(
                            f"Eksik alan: {field}"
                        )

                self.data_queue.put(data)

            except UnicodeDecodeError:

                self.status_queue.put(
                    "UART verisi UTF-8 formatında değil."
                )

            except json.JSONDecodeError:

                self.status_queue.put(
                    "Geçersiz JSON paketi alındı."
                )

            except ValueError as error:

                self.status_queue.put(str(error))

            except serial.SerialException as error:

                self.status_queue.put(
                    f"Seri port hatası: {error}"
                )

                break

    def process_queues(self):

        latest_data = None

        while True:

            try:
                latest_data = self.data_queue.get_nowait()
            except queue.Empty:
                break

        if latest_data is not None:
            self.update_display(latest_data)

        while True:

            try:
                message = self.status_queue.get_nowait()
                self.status_var.set(message)
            except queue.Empty:
                break

        self.after(50, self.process_queues)

    def update_display(self, data):

        self.sensor_var.set(
            "OK" if int(data["sensor_ok"]) else "HATA"
        )

        self.x_var.set(str(data["x"]))
        self.y_var.set(str(data["y"]))
        self.z_var.set(str(data["z"]))

        self.motion_var.set(
            "ALGILANDI"
            if int(data["motion"])
            else "YOK"
        )

        self.green_var.set(
            "AÇIK"
            if int(data["green"])
            else "KAPALI"
        )

        self.time_var.set(
            f'{data["time"]} ms'
        )

        self.draw_oled(data)

        if self.flash_enabled_var.get():
            self.write_flash_file(data)

    def draw_oled(self, data):

        self.oled_canvas.delete("all")

        lines = [
            "RTOS SENSOR",
            f'X: {int(data["x"]):4d}',
            f'Y: {int(data["y"]):4d}',
            f'Z: {int(data["z"]):4d}',
            "MOTION: YES"
            if int(data["motion"])
            else "MOTION: NO",
            "SENSOR: OK"
            if int(data["sensor_ok"])
            else "SENSOR: ERROR"
        ]

        y_position = 25

        for line in lines:

            self.oled_canvas.create_text(
                25,
                y_position,
                text=line,
                fill="white",
                anchor="nw",
                font=("Courier New", 18, "bold")
            )

            y_position += 45

    def find_record_count(self):

        if not FLASH_FILE.exists():
            return 0

        try:
            with FLASH_FILE.open(
                "r",
                newline="",
                encoding="utf-8"
            ) as file:

                rows = list(csv.DictReader(file))

            return len(rows)

        except OSError:
            return 0

    def write_flash_file(self, data):

        file_exists = (
            FLASH_FILE.exists()
            and FLASH_FILE.stat().st_size > 0
        )

        self.record_count += 1

        row = {
            "sequence": self.record_count,
            "time": data["time"],
            "x": data["x"],
            "y": data["y"],
            "z": data["z"],
            "motion": data["motion"],
            "sensor_ok": data["sensor_ok"],
            "green": data["green"],
            "computer_time": time.strftime(
                "%Y-%m-%d %H:%M:%S"
            )
        }

        try:
            with FLASH_FILE.open(
                "a",
                newline="",
                encoding="utf-8"
            ) as file:

                writer = csv.DictWriter(
                    file,
                    fieldnames=row.keys()
                )

                if not file_exists:
                    writer.writeheader()

                writer.writerow(row)

            self.record_count_var.set(
                str(self.record_count)
            )

        except OSError as error:

            self.flash_enabled_var.set(False)

            self.status_var.set(
                f"Flash dosyası yazılamadı: {error}"
            )

    def clear_flash_file(self):

        result = messagebox.askyesno(
            "Flash kayıtlarını temizle",
            "flash_log.csv dosyası silinsin mi?"
        )

        if not result:
            return

        try:
            if FLASH_FILE.exists():
                FLASH_FILE.unlink()

            self.record_count = 0
            self.record_count_var.set("0")

            self.status_var.set(
                "Flash kayıtları temizlendi."
            )

        except OSError as error:

            messagebox.showerror(
                "Dosya silinemedi",
                str(error)
            )

    def close_application(self):

        self.stop_source(show_status=False)
        self.destroy()


if __name__ == "__main__":

    application = RtosMonitor()
    application.mainloop()