import json
import threading
import time

import serial
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

# ---------------- CONFIG ----------------
SERIAL_PORT = "COM7"      # change if your ESP is on a different COM port
BAUD_RATE = 115200
# ----------------------------------------


app = FastAPI()

# Allow any origin for now (fine for local dev / demo)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# This will store the most recent reading from the ESP
latest_reading = {
    "freq_hz": None,
    "freq_mhz": None,
    "pos_mm": None,
    "timestamp": None,
}


def serial_worker():
    """
    Background thread:
    - Opens the serial port to the ESP32
    - Continuously reads lines
    - Parses JSON lines like:
      {"freq_hz": 6168752.174, "freq_mhz": 6.16875, "pos_mm": 52.431}
    - Updates global latest_reading
    """
    global latest_reading

    while True:
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print(f"[serial_worker] Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
        except Exception as e:
            print(f"[serial_worker] Could not open {SERIAL_PORT}: {e}")
            time.sleep(2)
            continue

        # Read loop
        while True:
            try:
                line_bytes = ser.readline()
                if not line_bytes:
                    continue

                line = line_bytes.decode(errors="ignore").strip()
                if not line:
                    continue

                # For debugging:
                # print(f"[serial_worker] RAW LINE: {line}")

                # We expect pure JSON lines from your firmware.
                # If ESP_LOGI messages are also printed, they'll be ignored.
                try:
                    data = json.loads(line)
                except json.JSONDecodeError:
                    # Not a JSON line (likely ESP_LOG), ignore it.
                    continue

                # Validate expected keys exist
                if not all(k in data for k in ("freq_hz", "freq_mhz", "pos_mm")):
                    continue

                data["timestamp"] = time.time()
                latest_reading = data

            except Exception as e:
                print(f"[serial_worker] Error during read loop: {e}")
                break  # break inner loop, close and reopen serial

        try:
            ser.close()
        except Exception:
            pass

        print("[serial_worker] Serial connection lost. Reconnecting...")
        time.sleep(1)


@app.on_event("startup")
def startup_event():
    """
    When FastAPI starts, spin up the serial worker thread.
    """
    t = threading.Thread(target=serial_worker, daemon=True)
    t.start()
    print("[startup] Serial worker started")


@app.get("/api/latest")
def get_latest():
    """
    Return the most recent reading from the sensor.
    Example response:
    {
      "freq_hz": 6168752.174,
      "freq_mhz": 6.16875,
      "pos_mm": 52.431,
      "timestamp": 1733158000.123
    }
    """
    return latest_reading
