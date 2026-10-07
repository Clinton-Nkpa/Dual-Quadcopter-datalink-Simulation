"""Read-only Streamlit dashboard for quadcopter MAVLink telemetry.

Run with:
    streamlit run telemetry_dashboard.py

The dashboard can use generated demo data or listen to a MAVLink UDP/serial
connection. It displays telemetry only; it does not send flight commands.
"""

from __future__ import annotations

import math
import time
from collections import deque
from typing import Any

import streamlit as st
from pymavlink import mavutil


st.set_page_config(
    page_title="Quadcopter Telemetry",
    page_icon="🛸",
    layout="wide",
)

st.title("🛸 Quadcopter telemetry")
st.caption("Read-only flight and link monitor · MAVLink or simulated demo data")

DEFAULT_TELEMETRY: dict[str, Any] = {
    "mode": None,
    "armed": None,
    "battery_v": None,
    "battery_a": None,
    "battery_remaining": None,
    "altitude_m": None,
    "groundspeed_m_s": None,
    "heading_deg": None,
    "roll_deg": None,
    "pitch_deg": None,
    "gps_fix": None,
    "satellites": None,
    "latitude": None,
    "longitude": None,
    "last_heartbeat": None,
    "last_message": None,
    "status_text": None,
}


def initialise_state() -> None:
    if "telemetry" not in st.session_state:
        st.session_state.telemetry = DEFAULT_TELEMETRY.copy()
    if "history" not in st.session_state:
        st.session_state.history = deque(maxlen=120)
    if "mav_connection" not in st.session_state:
        st.session_state.mav_connection = None
    if "demo_started" not in st.session_state:
        st.session_state.demo_started = time.time()
    if "telemetry_error" not in st.session_state:
        st.session_state.telemetry_error = None


initialise_state()

with st.sidebar:
    st.header("Connection")
    source = st.radio("Telemetry source", ["Demo", "MAVLink"], horizontal=True)
    endpoint = st.text_input(
        "MAVLink endpoint",
        value="udpin:0.0.0.0:14550",
        help="UDP listener example: udpin:0.0.0.0:14550 · Serial example: /dev/ttyUSB0",
        disabled=source == "Demo",
    )
    baud = st.selectbox(
        "Serial baud rate",
        [57600, 115200, 921600],
        index=0,
        disabled=source == "Demo" or endpoint.startswith(("udp:", "udpin:", "tcp:", "tcpin:")),
    )

    if source == "MAVLink":
        if st.session_state.mav_connection is None:
            if st.button("Connect", type="primary", use_container_width=True):
                try:
                    st.session_state.mav_connection = mavutil.mavlink_connection(
                        endpoint.strip(),
                        baud=int(baud),
                        autoreconnect=True,
                    )
                    st.session_state.telemetry = DEFAULT_TELEMETRY.copy()
                    st.session_state.history.clear()
                    st.session_state.telemetry_error = None
                    st.rerun()
                except Exception as exc:
                    st.session_state.telemetry_error = str(exc)
        else:
            st.success("MAVLink listener open")
            if st.button("Disconnect", use_container_width=True):
                try:
                    st.session_state.mav_connection.close()
                except Exception:
                    pass
                st.session_state.mav_connection = None
                st.session_state.telemetry = DEFAULT_TELEMETRY.copy()
                st.session_state.history.clear()
                st.rerun()
    else:
        st.info("Demo mode is active. No vehicle connection is required.")

    st.divider()
    st.caption("MAVLink connection examples")
    st.code("udpin:0.0.0.0:14550\n/dev/ttyUSB0", language="text")
    st.caption("For serial, select the vehicle's baud rate above.")


def set_if_valid(key: str, value: Any, invalid: Any = None, scale: float = 1.0) -> None:
    if value is None or value == invalid:
        return
    try:
        st.session_state.telemetry[key] = float(value) / scale
    except (TypeError, ValueError, OverflowError):
        return


def handle_mavlink_message(message: Any) -> None:
    telemetry = st.session_state.telemetry
    kind = message.get_type()

    if kind == "BAD_DATA":
        return

    telemetry["last_message"] = time.time()

    if kind == "HEARTBEAT":
        telemetry["last_heartbeat"] = time.time()
        telemetry["mode"] = mavutil.mode_string_v10(message)
        telemetry["armed"] = bool(
            message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
    elif kind == "SYS_STATUS":
        set_if_valid("battery_v", message.voltage_battery, invalid=65535, scale=1000)
        set_if_valid("battery_a", message.current_battery, invalid=-1, scale=100)
        set_if_valid("battery_remaining", message.battery_remaining, invalid=-1)
    elif kind == "ATTITUDE":
        set_if_valid("roll_deg", math.degrees(message.roll))
        set_if_valid("pitch_deg", math.degrees(message.pitch))
        set_if_valid("heading_deg", math.degrees(message.yaw) % 360)
    elif kind == "GLOBAL_POSITION_INT":
        set_if_valid("altitude_m", message.relative_alt, scale=1000)
        set_if_valid("latitude", message.lat, scale=10_000_000)
        set_if_valid("longitude", message.lon, scale=10_000_000)
    elif kind == "VFR_HUD":
        set_if_valid("groundspeed_m_s", message.groundspeed)
        if telemetry["altitude_m"] is None:
            set_if_valid("altitude_m", message.alt)
        set_if_valid("heading_deg", message.heading)
    elif kind == "GPS_RAW_INT":
        fix_names = {
            0: "No GPS",
            1: "No fix",
            2: "2D fix",
            3: "3D fix",
            4: "DGPS",
            5: "RTK float",
            6: "RTK fixed",
        }
        telemetry["gps_fix"] = fix_names.get(message.fix_type, "Unknown")
        set_if_valid("satellites", message.satellites_visible, invalid=255)
    elif kind == "STATUSTEXT":
        text = message.text
        if isinstance(text, bytes):
            text = text.decode("utf-8", errors="replace")
        telemetry["status_text"] = str(text).strip("\\x00 ")


def drain_mavlink_messages() -> None:
    connection = st.session_state.mav_connection
    if connection is None:
        return
    # Bound work on each UI refresh so a burst cannot freeze the dashboard.
    for _ in range(250):
        message = connection.recv_match(blocking=False)
        if message is None:
            break
        handle_mavlink_message(message)


def update_demo_telemetry() -> None:
    elapsed = time.time() - st.session_state.demo_started
    telemetry = st.session_state.telemetry
    telemetry.update(
        {
            "mode": "LOITER",
            "armed": True,
            "battery_v": 16.4 - 0.002 * elapsed,
            "battery_a": 4.2 + 0.8 * math.sin(elapsed * 0.7),
            "battery_remaining": max(0, 87 - int(elapsed / 12)),
            "altitude_m": 2.4 + 0.35 * math.sin(elapsed * 0.55),
            "groundspeed_m_s": 0.6 + 0.25 * math.sin(elapsed * 0.8),
            "heading_deg": (210 + elapsed * 3) % 360,
            "roll_deg": 4 * math.sin(elapsed * 0.8),
            "pitch_deg": 3 * math.cos(elapsed * 0.65),
            "gps_fix": "3D fix",
            "satellites": 12,
            "latitude": 53.7182,
            "longitude": -6.3471,
            "last_heartbeat": time.time(),
            "last_message": time.time(),
            "status_text": "Demo telemetry stream",
        }
    )


def display_value(value: Any, unit: str = "", precision: int = 1) -> str:
    if value is None:
        return "—"
    if isinstance(value, bool):
        return "Yes" if value else "No"
    if isinstance(value, (int, float)):
        return f"{value:.{precision}f}{unit}"
    return f"{value}{unit}"


def record_history() -> None:
    telemetry = st.session_state.telemetry
    if telemetry["last_message"] is None:
        return
    row = {
        "time": time.strftime("%H:%M:%S"),
        "altitude": telemetry["altitude_m"],
        "speed": telemetry["groundspeed_m_s"],
        "battery": telemetry["battery_remaining"],
    }
    history = st.session_state.history
    if not history or history[-1]["time"] != row["time"]:
        history.append(row)


@st.fragment(run_every=1)
def telemetry_panel() -> None:
    if source == "Demo":
        update_demo_telemetry()
    elif st.session_state.mav_connection is not None:
        try:
            drain_mavlink_messages()
            st.session_state.telemetry_error = None
        except Exception as exc:
            st.session_state.telemetry_error = str(exc)

    telemetry = st.session_state.telemetry
    now = time.time()
    heartbeat = telemetry["last_heartbeat"]
    online = source == "Demo" or (
        heartbeat is not None and now - heartbeat < 3.0
    )

    if source == "Demo":
        st.success("DEMO STREAM · simulated values update once per second")
    elif st.session_state.mav_connection is None:
        st.warning("DISCONNECTED · configure the endpoint and select Connect")
    elif online:
        age = now - heartbeat
        st.success(f"VEHICLE ONLINE · heartbeat {age:.1f}s ago")
    else:
        st.error("NO HEARTBEAT · check the vehicle, UDP forwarding, or serial link")

    if st.session_state.telemetry_error:
        st.error(f"MAVLink read error: {st.session_state.telemetry_error}")

    armed = telemetry["armed"]
    status_text = "ARMED" if armed is True else "DISARMED" if armed is False else "UNKNOWN"
    st.markdown(f"**Flight state:** {status_text} &nbsp;&nbsp; **Mode:** {telemetry['mode'] or '—'}")

    metric_cols = st.columns(4)
    metric_cols[0].metric("Relative altitude", display_value(telemetry["altitude_m"], " m", 2))
    metric_cols[1].metric("Ground speed", display_value(telemetry["groundspeed_m_s"], " m/s", 2))
    metric_cols[2].metric("Battery", display_value(telemetry["battery_v"], " V", 2))
    metric_cols[3].metric("Battery remaining", display_value(telemetry["battery_remaining"], "%", 0))

    detail_cols = st.columns(4)
    detail_cols[0].metric("Heading", display_value(telemetry["heading_deg"], "°", 0))
    detail_cols[1].metric("Roll", display_value(telemetry["roll_deg"], "°", 1))
    detail_cols[2].metric("Pitch", display_value(telemetry["pitch_deg"], "°", 1))
    gps = telemetry["gps_fix"] or "—"
    if telemetry["satellites"] is not None:
        gps += f" · {int(telemetry['satellites'])} sats"
    detail_cols[3].metric("GPS", gps)

    if telemetry["battery_a"] is not None:
        st.caption(f"Battery current: {telemetry['battery_a']:.2f} A")
    if telemetry["status_text"]:
        st.info(f"Vehicle status: {telemetry['status_text']}")

    record_history()
    history = list(st.session_state.history)
    if history:
        chart_cols = st.columns(2)
        chart_cols[0].caption("Relative altitude (m)")
        chart_cols[0].line_chart(
            {"Altitude (m)": [row["altitude"] for row in history]},
            height=180,
        )
        chart_cols[1].caption("Ground speed (m/s)")
        chart_cols[1].line_chart(
            {"Ground speed (m/s)": [row["speed"] for row in history]},
            height=180,
        )
    else:
        st.caption("Waiting for telemetry. Charts will appear when data arrives.")

    with st.expander("Raw telemetry"):
        st.json(
            {
                key: value
                for key, value in telemetry.items()
                if key not in {"last_heartbeat", "last_message"}
            }
        )


if source == "Demo" and st.session_state.mav_connection is not None:
    try:
        st.session_state.mav_connection.close()
    except Exception:
        pass
    st.session_state.mav_connection = None
    st.session_state.telemetry = DEFAULT_TELEMETRY.copy()
    st.session_state.history.clear()

telemetry_panel()

st.caption(
    "Monitor only: this dashboard reads MAVLink telemetry and does not send "
    "arming, mode, or control commands."
)
