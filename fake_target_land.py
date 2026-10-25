#!/usr/bin/env python3
"""
STAGE 1 - Fake target pe closed-loop precision landing (ArduPilot SITL)

Ye script wo karta hai jo QGC nahi kar sakta:
har 0.1 second pe drone ki position padhta hai, target se galti (error)
nikalta hai, aur us hisaab se velocity command bhejta hai. Jab drone target
ke upar centre ho jaata hai, tab dheere-dheere neeche utarta hai.

Baad mein (Stage 3) "fake target" ki jagah webcam + ArUco ka output lagega.

Chalane ka tareeka:
  Terminal 1:  cd ~/ardupilot && sim_vehicle.py -v ArduCopter
  Terminal 2:  python3 fake_target_land.py
               python3 fake_target_land.py --move            (target ghoomta rahega)
               python3 fake_target_land.py --move --ff       (feedforward ON)
               python3 fake_target_land.py --move --log p.csv (log ka naam)

Safety: Ctrl+C dabane pe drone LAND karega. Centre na ho paaye ya
flight bahut lambi ho jaaye to RTL.

Connection: default tcp:127.0.0.1:5762 (SITL ka doosra port), taaki
MAVProxy aur QGC wala connection disturb na ho.
"""

import argparse
import math
import time

from pymavlink import mavutil

# ---- Tuning (yahin se behaviour badlo) ----
KP = 0.5            # error (m) -> speed (m/s). Bada = tez, par overshoot ka risk
VMAX = 3.0          # horizontal speed limit (m/s)
START_TOL = 0.5     # error isse kam ho to utarna SHURU karo (m)
STOP_TOL = 1.0      # error isse zyada ho jaaye tabhi utarna ROKO (m)  -> hysteresis
DESCENT_SPEED = 0.6 # neeche utarne ki speed (m/s)
SLOW_ALT = 2.0      # isse neeche aakar dheere utro
SLOW_DESCENT = 0.3  # (m/s)
LAND_ALT = 0.5      # is height tak tracking chalu, phir LAND mode (m)
CENTER_TIMEOUT = 45 # itne second mein centre nahi hua to RTL (safety)
MAX_FLIGHT = 180    # poori flight ki upper limit, phir RTL (safety)

# velocity-only command: position, acceleration, yaw, yaw-rate ignore
VEL_ONLY_MASK = 0b0000110111000111
MODES = {}


def log_statustext(msg):
    if msg is not None and msg.get_type() == "STATUSTEXT":
        print(f"  [drone] {msg.text}")


def recv(m, msg_type, timeout=1.0):
    """msg_type aane tak messages padho; beech mein STATUSTEXT print karo."""
    end = time.time() + timeout
    types = [msg_type, "STATUSTEXT"] if isinstance(msg_type, str) else list(msg_type) + ["STATUSTEXT"]
    while time.time() < end:
        msg = m.recv_match(type=types, blocking=True, timeout=max(0.01, end - time.time()))
        if msg is None:
            return None
        if msg.get_type() == "STATUSTEXT":
            log_statustext(msg)
            continue
        if msg.get_srcSystem() != m.target_system:
            continue
        return msg
    return None


def request_rate(m, msg_id, hz):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)


def is_armed(hb):
    return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def wait_vehicle(m, timeout=30):
    """Sirf asli drone ki heartbeat pakdo (QGC/MAVProxy jaise GCS ki nahi)."""
    end = time.time() + timeout
    while time.time() < end:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb is None:
            continue
        if (hb.get_srcSystem() > 0
                and hb.type != mavutil.mavlink.MAV_TYPE_GCS
                and hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID):
            m.target_system = hb.get_srcSystem()
            m.target_component = hb.get_srcComponent()
            return mavutil.mode_mapping_byname(hb.type)
    raise RuntimeError("Drone ki heartbeat nahi mili. SITL chal raha hai?")


def set_mode(m, name, timeout=10):
    mode_id = MODES[name]
    end = time.time() + timeout
    while time.time() < end:
        m.set_mode(mode_id)
        hb = recv(m, "HEARTBEAT", timeout=1.5)
        if hb and hb.custom_mode == mode_id:
            print(f"Mode: {name}")
            return
    raise RuntimeError(f"Mode {name} set nahi hua")


def arm(m, timeout=90):
    print("Arm kar rahe hain (GPS/EKF ready hone tak retry hoga)...")
    end = time.time() + timeout
    while time.time() < end:
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)
        hb = recv(m, "HEARTBEAT", timeout=2)
        if hb and is_armed(hb):
            print("ARMED")
            return
        time.sleep(1)
    raise RuntimeError("Arm nahi hua. Upar [drone] PreArm messages dekho.")


def takeoff(m, alt, timeout=40):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, alt)
    print(f"Takeoff -> {alt} m")
    t0 = time.time()
    last_print = 0.0
    last_alt, last_change = 0.0, time.time()
    while True:
        if time.time() - t0 > timeout:
            raise RuntimeError("Takeoff timeout. Upar [drone] messages aur QGC dekho.")
        pos = recv(m, "LOCAL_POSITION_NED", timeout=2)
        if pos is None:
            print("  (position message nahi aa raha...)")
            continue
        cur = -pos.z
        if time.time() - last_print > 1.0:
            print(f"  height {cur:5.1f} m")
            last_print = time.time()
        if abs(cur - last_alt) > 0.1:
            last_alt, last_change = cur, time.time()
        # 90% tak pahunch gaya, YA climb 3 second se ruka hua hai (aur 50% se upar hai)
        if cur >= alt * 0.90 or (cur > alt * 0.5 and time.time() - last_change > 3):
            print(f"Takeoff complete: {cur:.1f} m")
            return


def send_velocity(m, vx, vy, vz):
    """NED frame: vx = North, vy = East, vz = Down (positive = neeche)."""
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, VEL_ONLY_MASK,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", default="tcp:127.0.0.1:5762")
    ap.add_argument("--alt", type=float, default=10.0, help="takeoff height (m)")
    ap.add_argument("--north", type=float, default=8.0, help="target kitna North (m)")
    ap.add_argument("--east", type=float, default=5.0, help="target kitna East (m)")
    ap.add_argument("--move", action="store_true", help="target ko circle mein ghumao")
    ap.add_argument("--ff", action="store_true", help="feedforward on (target velocity jodo)")
    ap.add_argument("--log", default=None, help="CSV log file ka naam (default: log_HHMMSS.csv)")
    args = ap.parse_args()

    print(f"Connecting {args.connect} ...")
    m = mavutil.mavlink_connection(args.connect)
    global MODES
    MODES = wait_vehicle(m)
    print(f"Drone mila (system {m.target_system}, component {m.target_component})")

    request_rate(m, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10)
    request_rate(m, mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2)

    # Pichla run Ctrl+C se band hua ho to drone already hawa mein ho sakta hai
    hb = recv(m, "HEARTBEAT", timeout=3)
    pos0 = recv(m, "LOCAL_POSITION_NED", timeout=3)
    set_mode(m, "GUIDED")
    if hb and is_armed(hb) and pos0 and -pos0.z > 2.0:
        print(f"Drone already hawa mein hai ({-pos0.z:.1f} m). Takeoff skip.")
    else:
        arm(m)
        takeoff(m, args.alt)

    log_name = args.log or time.strftime("log_%H%M%S.csv")
    log = open(log_name, "w")
    log.write("t,error,alt,tx,ty,x,y,vx,vy,vz,descending\n")
    print(f"Log: {log_name}")

    try:
        fly(m, args, log)
    except KeyboardInterrupt:
        # Script band ho raha hai -> drone ko hawa mein latka mat chhodo
        print("\nCtrl+C! Drone ko LAND mode de rahe hain...")
        set_mode(m, "LAND")
    finally:
        log.close()

    while True:
        hb = recv(m, "HEARTBEAT", timeout=2)
        if hb and not is_armed(hb):
            print("Landed + DISARMED. Done.")
            break


def fly(m, args, log):
    start = recv(m, "LOCAL_POSITION_NED", timeout=3)
    cx, cy = start.x + args.north, start.y + args.east
    print(f"Target: start se {args.north} m North, {args.east} m East"
          + ("  (moving)" if args.move else "")
          + ("  [feedforward ON]" if args.ff else ""))

    t0 = time.time()
    last_print = 0.0
    descending = False
    ever_centered = False
    while True:
        pos = recv(m, "LOCAL_POSITION_NED", timeout=1)
        if pos is None:
            continue
        t = time.time() - t0

        # ---- SAFETY ----
        if not ever_centered and t > CENTER_TIMEOUT:
            print(f"SAFETY: {CENTER_TIMEOUT}s mein centre nahi hua -> RTL")
            set_mode(m, "RTL")
            return
        if t > MAX_FLIGHT:
            print(f"SAFETY: {MAX_FLIGHT}s flight limit -> RTL")
            set_mode(m, "RTL")
            return

        # ---- target kahan hai (Stage 3 mein ye camera se aayega) ----
        if args.move:
            w = 2 * math.pi / 60.0          # angular speed (rad/s)
            tx = cx + 4.0 * math.cos(w * t)
            ty = cy + 4.0 * math.sin(w * t)
            # target ki velocity = position ka derivative
            tvx = -4.0 * w * math.sin(w * t)
            tvy = 4.0 * w * math.cos(w * t)
        else:
            tx, ty = cx, cy
            tvx, tvy = 0.0, 0.0             # ruka hua target

        # ---- error -> velocity (P + feedforward + speed limit) ----
        ex, ey = tx - pos.x, ty - pos.y
        dist = math.hypot(ex, ey)
        vx, vy = KP * ex, KP * ey
        if args.ff:
            vx += tvx
            vy += tvy
        speed = math.hypot(vx, vy)
        if speed > VMAX:
            vx, vy = vx * VMAX / speed, vy * VMAX / speed

        # ---- hysteresis: shuru aur rokne ki alag seema ----
        if not descending and dist < START_TOL:
            descending = True
            ever_centered = True
        elif descending and dist > STOP_TOL:
            descending = False

        alt = -pos.z
        if descending:
            vz = SLOW_DESCENT if alt < SLOW_ALT else DESCENT_SPEED
        else:
            vz = 0.0

        send_velocity(m, vx, vy, vz)
        log.write(f"{t:.2f},{dist:.3f},{alt:.2f},{tx:.3f},{ty:.3f},"
                  f"{pos.x:.3f},{pos.y:.3f},{vx:.3f},{vy:.3f},{vz:.3f},{int(descending)}\n")

        if t - last_print > 1.0:
            state = "UTAR RAHA" if descending else "CENTRE KAR RAHA"
            print(f"t={t:5.1f}s  error={dist:5.2f} m  height={alt:5.1f} m  {state}")
            last_print = t

        # tracking LAND_ALT tak chalu rehti hai, phir LAND
        if alt < LAND_ALT and descending:
            print(f"Final error {dist:.2f} m at {alt:.1f} m -> LAND mode")
            set_mode(m, "LAND")
            return

if __name__ == "__main__":
    main()
