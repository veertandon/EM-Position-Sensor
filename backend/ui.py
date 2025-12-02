import time
import requests
import streamlit as st

API_URL = "http://127.0.0.1:8000/api/latest"

st.set_page_config(page_title="EM Position Sensor", layout="wide")

st.title("🔍 EM Sensor Live Dashboard")

placeholder = st.empty()

# Streamlit-friendly loop
while True:
    try:
        r = requests.get(API_URL, timeout=1)
        data = r.json()
    except Exception:
        data = None

    with placeholder.container():

        if data and data.get("freq_mhz") is not None:
            pos = data["pos_mm"]
            freq = data["freq_mhz"]

            # Top metrics
            col1, col2 = st.columns(2)
            col1.metric("Resonant Frequency (MHz)", f"{freq:.4f}")
            col2.metric("Estimated Position (mm)", f"{pos:.2f}")

            # Position bar
            st.write("### Position Along Sensor (0–100 mm)")
            st.progress(min(max(pos / 100.0, 0.0), 1.0))

            st.caption(f"Last update: {time.ctime(data['timestamp'])}")

        else:
            st.warning("Waiting for sensor data…")

        # ---------- Explanation section (rendered once per refresh) ----------
        st.markdown("---")
        st.subheader("📘 How This Electromagnetic Position Sensor Works")

        st.info(
            "Our sensor measures position by tracking shifts in the resonant "
            "frequency of an LC circuit.\n\n"
            "A copper coil generates a magnetic field. As the metal plate moves "
            "along the sensor, it disturbs this field and changes the coil’s "
            "**inductance**. Because the resonant frequency depends on both "
            "inductance (L) and capacitance (C), any change in L produces a "
            "measurable change in frequency."
        )

        st.latex(r"f = \frac{1}{2\pi\sqrt{LC}}")

        with st.expander("🔬 Technical Details"):
            st.markdown(
                "- Movement of the metal target modifies magnetic flux paths,\n"
                "  effectively changing the inductance of the sensing coil.\n"
                "- The frequency of the resonant tank shifts in a predictable, "
                "monotonic way with position.\n"
                "- During calibration, we measure frequency at known plate "
                "positions and store these in a lookup table.\n"
                "- At runtime, the system interpolates between calibration "
                "points to convert live frequency measurements into a precise "
                "position estimate.\n\n"
                "This gives a **contactless, robust** measurement principle that "
                "is well-suited to harsh environments where optical or "
                "mechanical sensors would struggle."
            )

    time.sleep(0.5)
