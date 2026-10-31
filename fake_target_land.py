#!/usr/bin/env python3
"""
STAGE 1 - Closed-loop precision landing on a simulated target (ArduPilot SITL)

Every ~0.1 s the script reads the drone's position, computes the error to the
target, and sends a velocity command to close it. The drone only descends
while it is centred over the target.

Run:
  Terminal 1:  cd ~/ardupilot && sim_vehicle.py -v ArduCopter
  Terminal 2:  python3 fake_target_land.py
               python3 fake_target_land.py --move              (target moves on a circle)
               python3 fake_target_land.py --move --ff         (feedforward on)
               python3 fake_target_land.py --jump 0.8          (target jumps 0.8 m near the ground)
               python3 fake_target_land.py --jump 0.8 --cone   (same, with the landing cone)
               python3 fake_target_land.py --log run.csv       (log file name)

Safety: Ctrl+C makes the drone LAND. If it cannot centre, or the flight is too
long, it switches to RTL.

Connection: default tcp:127.0.0.1:5762 (SITL's second port), so MAVProxy and
QGC keep their own connection.
"""

import argparse
import math
import random
import time

from pymavlink import mavutil

# ---- Tuning ----
KP = 0.5            # error (m) -> speed (m/s). Higher = faster, but overshoots
VMAX = 3.0          # horizontal speed limit (m/s)
START_TOL = 0.5     # start descending when error is below this (m)
STOP_TOL = 1.0      # stop descending when error goes above this (m) -> hysteresis
DESCENT_SPEED = 0.6 # descent speed (m/s)
SLOW_ALT = 2.0      # below this height, descend slowly
SLOW_DESCENT = 0.3  # (m/s)
LAND_ALT = 0.5      # track the target down to this height, then LAND mode (m)
CENTER_TIMEOUT = 45 # not centred within this time (s) -> RTL
MAX_FLIGHT = 180    # whole flight limit (s) -> RTL

# ---- Landing cone (--cone) ----
# The allowed error shrinks with height: generous up high, strict near the ground.
CONE_SLOPE = 0.05   # allowed start error per metre of height (0.5 m at 10 m)
CONE_MIN = 0.15     # never stricter than this (m), or noise would stop every descent
RETRY_ALT = 1.5     # off-target below this height -> climb back up to it
RETRY_CLIMB = 0.5   # climb speed for a retry (m/s)

# ---- Simulated sensor (--sensor) ----
# Without --sensor the controller knows the target position exactly. With it,
# the target is seen the way a downward camera would see it.
SENS_FOV = 30.0      # half-angle of the camera cone (deg): target visible only inside it
SENS_NOISE = 0.5     # angle noise, 1 sigma (deg) -> position noise grows with height
SENS_DELAY = 0.15    # image processing delay (s)
SENS_ALPHA = 0.3     # low-pass filter on the measurement (0..1)
LOST_HOLD = 0.5      # target unseen this long (s) -> hover, stop descending
LOST_CLIMB = 2.0     # target unseen this long (s) -> climb to get it back in view
SEARCH_MAX_ALT = 8.0 # do not climb above this while searching (m)

# velocity-only command: ignore position, acceleration, yaw, yaw rate
VEL_ONLY_MASK = 0b0000110111000111
MODES = {}


def log_statustext(msg):
    if msg is not None and msg.get_type() == "STATUSTEXT":
        print(f"  [drone] {msg.text}")


def recv(m, msg_type, timeout=1.0):
    """Read until msg_type arrives; print any STATUSTEXT on the way."""
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
    """Lock onto the vehicle's heartbeat (ignore ground stations like QGC/MAVProxy)."""
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
    raise RuntimeError("No vehicle heartbeat. Is SITL running?")


def set_mode(m, name, timeout=10):
    mode_id = MODES[name]
    end = time.time() + timeout
    while time.time() < end:
        m.set_mode(mode_id)
        hb = recv(m, "HEARTBEAT", timeout=1.5)
        if hb and hb.custom_mode == mode_id:
            print(f"Mode: {name}")
            return
    raise RuntimeError(f"Could not set mode {name}")


def arm(m, timeout=90):
    print("Arming (retrying until GPS/EKF are ready)...")
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
    raise RuntimeError("Arming failed. Check the [drone] PreArm messages above.")


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
            raise RuntimeError("Takeoff timeout. Check the [drone] messages and QGC.")
        pos = recv(m, "LOCAL_POSITION_NED", timeout=2)
        if pos is None:
            print("  (no position message...)")
            continue
        cur = -pos.z
        if time.time() - last_print > 1.0:
            print(f"  height {cur:5.1f} m")
            last_print = time.time()
        if abs(cur - last_alt) > 0.1:
            last_alt, last_change = cur, time.time()
        # reached 90%, OR the climb has stalled for 3 s above 50%
        if cur >= alt * 0.90 or (cur > alt * 0.5 and time.time() - last_change > 3):
            print(f"Takeoff complete: {cur:.1f} m")
            return


def climb_to(m, alt, timeout=30):
    """GUIDED takeoff often stops short (~8 m). Climb the rest so every run starts at the same height."""
    end = time.time() + timeout
    while time.time() < end:
        pos = recv(m, "LOCAL_POSITION_NED", timeout=1)
        if pos is None:
            continue
        err = alt - (-pos.z)
        if abs(err) < 0.15 and abs(pos.vz) < 0.1:
            print(f"At {alt:.1f} m.")
            return
        vz = max(-1.0, min(1.0, -0.8 * err))     # negative = climb
        send_velocity(m, 0.0, 0.0, vz)
    print("Could not reach the start height exactly, continuing.")


def send_velocity(m, vx, vy, vz):
    """NED frame: vx = North, vy = East, vz = Down (positive = descend)."""
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, VEL_ONLY_MASK,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", default="tcp:127.0.0.1:5762")
    ap.add_argument("--alt", type=float, default=10.0, help="takeoff height (m)")
    ap.add_argument("--north", type=float, default=8.0, help="target offset North (m)")
    ap.add_argument("--east", type=float, default=5.0, help="target offset East (m)")
    ap.add_argument("--move", action="store_true", help="target moves on a circle")
    ap.add_argument("--ff", action="store_true", help="feedforward (add target velocity)")
    ap.add_argument("--cone", action="store_true", help="landing cone: allowed error shrinks with height")
    ap.add_argument("--jump", type=float, default=0.0,
                    help="target jumps this far East (m) when the drone first descends below --jump-alt")
    ap.add_argument("--jump-alt", type=float, default=1.2, help="height for --jump (m)")
    ap.add_argument("--sensor", action="store_true",
                    help="see the target through a simulated camera: field of view, noise, delay")
    ap.add_argument("--seed", type=int, default=1, help="random seed for --sensor noise")
    ap.add_argument("--log", default=None, help="CSV log file name (default: log_HHMMSS.csv)")
    args = ap.parse_args()

    print(f"Connecting {args.connect} ...")
    m = mavutil.mavlink_connection(args.connect)
    global MODES
    MODES = wait_vehicle(m)
    print(f"Vehicle found (system {m.target_system}, component {m.target_component})")

    request_rate(m, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10)
    request_rate(m, mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2)

    # If a previous run was stopped, the drone may still be in the air
    hb = recv(m, "HEARTBEAT", timeout=3)
    pos0 = recv(m, "LOCAL_POSITION_NED", timeout=3)
    set_mode(m, "GUIDED")
    if hb and is_armed(hb) and pos0 and -pos0.z > 2.0:
        print(f"Already airborne ({-pos0.z:.1f} m), skipping takeoff.")
    else:
        if hb and is_armed(hb):
            # armed but on the ground (left over from an earlier run): takeoff
            # is often refused in this state, so start clean
            print("Armed on the ground from an earlier run -> disarming first")
            disarm(m)
        arm(m)
        takeoff(m, args.alt)
    climb_to(m, args.alt)

    log_name = args.log or time.strftime("log_%H%M%S.csv")
    log = open(log_name, "w")
    log.write("t,error,alt,tx,ty,x,y,vx,vy,vz,descending\n")
    print(f"Log: {log_name}")

    landed = False
    try:
        landed = fly(m, args, log)
    except KeyboardInterrupt:
        # never leave the drone hovering when the script stops
        print("\nCtrl+C! Coming straight down...")
        final_descent(m, None)
        landed = True
    finally:
        log.close()

    if not landed:          # RTL safety case
        wait_disarm(m)


def wait_disarm(m, target=None, timeout=120):
    """Wait for touchdown (e.g. after RTL); disarm if it sits on the ground armed."""
    last = None
    still = None
    end = time.time() + timeout
    while time.time() < end:
        msg = recv(m, ["HEARTBEAT", "LOCAL_POSITION_NED"], timeout=2)
        if msg is None:
            continue
        if msg.get_type() == "LOCAL_POSITION_NED":
            last = msg
            if -msg.z < 0.2 and abs(msg.vz) < 0.1:
                still = still or time.time()
                if time.time() - still > 3:
                    print("On the ground but still armed -> disarming (SITL)")
                    disarm(m)
            else:
                still = None
        elif not is_armed(msg):
            if target and last:
                err = math.hypot(target[0] - last.x, target[1] - last.y)
                print(f"Touchdown error: {err:.2f} m")
            print("Landed + DISARMED. Done.")
            return
    print("Timed out waiting for disarm - check QGC.")


def disarm(m):
    """Normal disarm first; if refused, force it (magic 21196). Only safe in SITL."""
    for force in (0, 21196):
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                0, force, 0, 0, 0, 0, 0)
        end = time.time() + 3
        while time.time() < end:
            hb = recv(m, "HEARTBEAT", timeout=1)
            if hb and not is_armed(hb):
                print("DISARMED")
                return True
    print("Disarm failed - disarm from QGC or MAVProxy (disarm force).")
    return False


def final_descent(m, target=None, timeout=60, truth=None):
    """
    Last part of the landing in GUIDED, still tracking the target, then disarm.

    We do NOT use LAND mode here: with PLND_ENABLED=1, LAND runs ArduPilot's
    own precision landing, which would take over from our controller.
    target = (x, y) in local NED, or None to come straight down.
    """
    still = None
    last = None
    end = time.time() + timeout
    while time.time() < end:
        pos = recv(m, "LOCAL_POSITION_NED", timeout=1)
        if pos is None:
            continue
        last = pos
        vx = vy = 0.0
        if target:
            vx, vy = KP * (target[0] - pos.x), KP * (target[1] - pos.y)
            sp = math.hypot(vx, vy)
            if sp > 0.3:                      # gentle near the ground
                vx, vy = vx * 0.3 / sp, vy * 0.3 / sp
        send_velocity(m, vx, vy, SLOW_DESCENT)
        if -pos.z < 0.2 and abs(pos.vz) < 0.1:
            still = still or time.time()
            if time.time() - still > 1.0:
                break
        else:
            still = None
    ref = truth or target
    if ref and last:
        err = math.hypot(ref[0] - last.x, ref[1] - last.y)
        print(f"Touchdown error: {err:.2f} m")
    disarm(m)
    print("Landed + DISARMED. Done.")


def tolerances(alt, cone):
    """(start, stop) descent tolerances in metres for this height."""
    if not cone:
        return START_TOL, STOP_TOL
    start = min(START_TOL, max(CONE_MIN, CONE_SLOPE * alt))
    return start, 2 * start


def fly(m, args, log):
    start = recv(m, "LOCAL_POSITION_NED", timeout=3)
    cx, cy = start.x + args.north, start.y + args.east
    print(f"Target: {args.north} m North, {args.east} m East of start"
          + ("  (moving)" if args.move else "")
          + ("  [feedforward ON]" if args.ff else "")
          + ("  [cone ON]" if args.cone else "")
          + (f"  [jump {args.jump} m at {args.jump_alt} m]" if args.jump else "")
          + (f"  [sensor: FOV {SENS_FOV:.0f} deg, noise {SENS_NOISE} deg, delay {SENS_DELAY} s]"
             if args.sensor else ""))
    random.seed(args.seed)
    hist = []            # (t, true tx, true ty) for the delay
    est = None           # filtered target estimate (what the drone believes)
    last_seen = None
    searching = False

    t0 = time.time()
    last_print = 0.0
    descending = False
    ever_centered = False
    jumped = False
    retrying = False
    while True:
        pos = recv(m, "LOCAL_POSITION_NED", timeout=1)
        if pos is None:
            continue
        t = time.time() - t0

        # ---- SAFETY ----
        if not ever_centered and t > CENTER_TIMEOUT:
            print(f"SAFETY: not centred within {CENTER_TIMEOUT}s -> RTL")
            set_mode(m, "RTL")
            return False
        if t > MAX_FLIGHT:
            print(f"SAFETY: {MAX_FLIGHT}s flight limit -> RTL")
            set_mode(m, "RTL")
            return False

        alt = -pos.z

        # ---- disturbance: target jumps near the ground ----
        if args.jump and not jumped and descending and alt < args.jump_alt:
            cy += args.jump
            jumped = True
            print(f"t={t:5.1f}s  TARGET JUMPED {args.jump} m East at height {alt:.1f} m")

        # ---- where is the target (Stage 3: from the camera) ----
        if args.move:
            w = 2 * math.pi / 60.0          # angular speed (rad/s)
            tx = cx + 4.0 * math.cos(w * t)
            ty = cy + 4.0 * math.sin(w * t)
            # target velocity = derivative of its position
            tvx = -4.0 * w * math.sin(w * t)
            tvy = 4.0 * w * math.cos(w * t)
        else:
            tx, ty = cx, cy
            tvx, tvy = 0.0, 0.0             # target at rest

        true_dist = math.hypot(tx - pos.x, ty - pos.y)

        # ---- simulated camera: what the drone actually knows ----
        if args.sensor:
            hist.append((t, tx, ty))
            while len(hist) > 2 and hist[1][0] <= t - SENS_DELAY:
                hist.pop(0)
            _, dtx, dty = hist[0]                         # target as it was SENS_DELAY ago
            visible = true_dist <= max(alt, 0.05) * math.tan(math.radians(SENS_FOV))
            if visible:
                sigma = max(alt, 0.3) * math.tan(math.radians(SENS_NOISE))
                mx, my = dtx + random.gauss(0, sigma), dty + random.gauss(0, sigma)
                est = (mx, my) if est is None else (est[0] + SENS_ALPHA * (mx - est[0]),
                                                    est[1] + SENS_ALPHA * (my - est[1]))
                last_seen = t
                if searching:
                    print(f"t={t:5.1f}s  target back in view at {alt:.1f} m")
                    searching = False
            lost_for = (t - last_seen) if last_seen is not None else float("inf")
            ctx, cty = est if est is not None else (tx, ty)
            tvx = tvy = 0.0                               # no velocity from the camera
        else:
            lost_for = 0.0
            ctx, cty = tx, ty

        # ---- error -> velocity (P + feedforward + speed limit) ----
        ex, ey = ctx - pos.x, cty - pos.y
        dist = math.hypot(ex, ey)                         # error the drone believes
        vx, vy = KP * ex, KP * ey
        if args.ff:
            vx += tvx
            vy += tvy
        speed = math.hypot(vx, vy)
        if speed > VMAX:
            vx, vy = vx * VMAX / speed, vy * VMAX / speed

        # ---- hysteresis (and landing cone if --cone) ----
        start_tol, stop_tol = tolerances(alt, args.cone)
        if not descending and dist < start_tol and not retrying:
            descending = True
            ever_centered = True
        elif descending and dist > stop_tol:
            descending = False
            if args.cone and alt < RETRY_ALT:
                retrying = True
                print(f"t={t:5.1f}s  RETRY: {dist:.2f} m off at {alt:.1f} m -> climbing to {RETRY_ALT} m")

        if retrying and alt >= RETRY_ALT:
            retrying = False

        if retrying:
            vz = -RETRY_CLIMB            # negative = climb
        elif descending:
            vz = SLOW_DESCENT if alt < SLOW_ALT else DESCENT_SPEED
        else:
            vz = 0.0

        # ---- target not visible: hover, then climb until it is back in view ----
        if args.sensor and lost_for > LOST_HOLD:
            vx = vy = 0.0
            descending = False
            retrying = False
            vz = -RETRY_CLIMB if (lost_for > LOST_CLIMB and alt < SEARCH_MAX_ALT) else 0.0
            if not searching:
                print(f"t={t:5.1f}s  TARGET NOT VISIBLE at {alt:.1f} m -> hover, then climb")
                searching = True

        send_velocity(m, vx, vy, vz)
        log.write(f"{t:.2f},{true_dist:.3f},{alt:.2f},{tx:.3f},{ty:.3f},"
                  f"{pos.x:.3f},{pos.y:.3f},{vx:.3f},{vy:.3f},{vz:.3f},{int(descending)}\n")

        if t - last_print > 1.0:
            state = ("SEARCH" if searching else "RETRY" if retrying
                     else "DESCENDING" if descending else "CENTRING")
            extra = f"  (believed {dist:4.2f} m)" if args.sensor else ""
            print(f"t={t:5.1f}s  error={true_dist:5.2f} m{extra}  height={alt:5.1f} m  {state}")
            last_print = t

        # track the target down to LAND_ALT, then the final descent
        if alt < LAND_ALT and descending:
            print(f"Final error {true_dist:.2f} m at {alt:.1f} m -> final descent")
            final_descent(m, (ctx, cty), truth=(tx, ty))
            return True

if __name__ == "__main__":
    main()
