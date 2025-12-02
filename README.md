📡 EM Position Sensor — ESP32 + Inductive Sensing + Live UI Dashboard

This project implements a contactless electromagnetic (EM) position sensor using an LDC1612 inductance-to-digital converter, an ESP32 microcontroller, and a Python backend + Streamlit dashboard for live visualization.

The system detects the position of a metal target by tracking changes in the resonant frequency of a sensing coil. Frequency readings are streamed from the ESP32 to a PC, processed by a backend service, and visualized in real time.

🧠 How the System Works

A copper coil forms an LC resonant tank circuit connected to the LDC1612.
As a metal plate moves above the coil:

The magnetic field changes

The effective inductance L changes

The resonant frequency shifts according to:

𝑓
=
1
2
𝜋
𝐿
𝐶
f=
2π
LC
	​

1
	​


These frequency shifts are predictable and monotonic across the 0–100 mm sensing range.

A lookup table (LUT) generated during calibration maps:
→ resonant frequency → physical position.

The ESP32 continuously reads LDC1612 measurements and prints:

{
  "freq_hz": 6168752.174,
  "freq_mhz": 6.16875,
  "pos_mm": 52.431
}


The Python backend listens to the serial port, exposes the latest reading at:

GET /api/latest


The Streamlit dashboard then displays:

Real-time frequency

Live position

Position indicator bar

Explanation of sensing principles

🗂 Project Structure
Sensors/
│
├── backend/
│   ├── main.py        # FastAPI backend, serial reader, REST API
│   ├── ui.py          # Streamlit dashboard (live visualization)
│
├── src/
│   └── main.c         # ESP32 firmware for LDC1612
│
├── include/           # ESP-IDF headers
├── lib/               # Additional libraries if needed
│
├── platformio.ini     # PlatformIO project configuration
├── CMakeLists.txt     # Build system files
└── .gitignore         # Clean repo (ignores .pio, pycache, etc.)

🚀 Getting Started
1️⃣ Install Required Tools
ESP32 Firmware

PlatformIO + VS Code

ESP32 FireBeetle board support

Backend & UI

Python 3.10+

Install dependencies:

pip install fastapi uvicorn streamlit requests pyserial

🔌 2️⃣ Flash the ESP32 Firmware

Open the project in VS Code → PlatformIO.

Connect ESP32 → Select correct COM port.

Build and upload:

PlatformIO: Upload


Open Serial Monitor (115200 baud) to confirm output:

Measured Resonant Frequency: 6.16896 MHz
Target position: 52.44 mm

🖥 3️⃣ Start the Backend Server

From the backend/ folder:

uvicorn main:app --reload


Expected output:

Uvicorn running on http://127.0.0.1:8000
[serial_worker] Connected to COM7


Check latest measurement:

http://127.0.0.1:8000/api/latest

📊 4️⃣ Launch the Streamlit Dashboard

From backend/:

streamlit run ui.py


You will see the live dashboard with:

Frequency (MHz)

Position (mm)

Movement bar

Auto-updating explanations

🔧 Calibration Process (Overview)

To ensure accurate position detection:

Move metal plate to known positions (0, 2.5, 5, 7.5 ... 100 mm)

Record frequency at each point

Store results in the lookup table inside ESP32 firmware

The code performs linear interpolation between nearest calibration points

This enables smooth, continuous position estimation.

📘 Technical Summary
The LDC1612 measures inductance by:

Driving the LC tank

Measuring how long it takes for oscillations to decay

Converting this into a 28-bit digital value

Computing frequency internally

Why inductive sensing?

✔ Contactless — no friction or wear
✔ Works in dust, oil, moisture, or vibration
✔ Very stable over time
✔ High update rate (kHz possible)
✔ Excellent for industrial and robotics applications

🛠 Future Improvements

Wireless data streaming (ESP32 → WiFi → backend)

Real-time calibration curve graph

Temperature compensation

Multi-coil or multi-axis sensing

Machine-learning-based position estimation

📄 License

Open source for academic and research use.

🙌 Acknowledgements

This project was developed as part of an engineering sensor coursework module, involving:

Embedded systems

Inductive sensing physics

Calibration methodology

Backend/web development

Real-time dashboards
