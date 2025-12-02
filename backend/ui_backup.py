# import time
# import requests
# import streamlit as st

# API_URL = "http://127.0.0.1:8000/api/latest"

# st.set_page_config(page_title="EM Position Sensor", layout="wide")

# st.title("🔍 EM Sensor Live Dashboard")

# placeholder = st.empty()

# # Streamlit-friendly loop
# while True:
#     try:
#         r = requests.get(API_URL, timeout=1)
#         data = r.json()
#     except Exception:
#         data = None

#     with placeholder.container():

#         if data and data.get("freq_mhz") is not None:
#             pos = data["pos_mm"]
#             freq = data["freq_mhz"]

#             col1, col2 = st.columns(2)
#             col1.metric("Resonant Frequency (MHz)", f"{freq:.4f}")
#             col2.metric("Estimated Position (mm)", f"{pos:.2f}")

#             st.write("### Position Along Sensor (0–100 mm)")
#             st.progress(min(max(pos / 100.0, 0.0), 1.0))

#             st.caption(f"Last update: {time.ctime(data['timestamp'])}")

#         else:
#             st.warning("Waiting for sensor data…")

#     # Streamlit-friendly sleep
#     time.sleep(0.5)
