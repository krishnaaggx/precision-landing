#!/usr/bin/env python3
"""
STAGE 4 - ArduPilot's built-in precision landing, for comparison with ours.

ArduPilot SITL has a simulated landing-target sensor (PLND_TYPE 4) and a
simulated rangefinder. Here the drone takes off, switches to LAND mode, and
ArduPilot's own precision landing steers it onto a simulated pad. The script
measures where it actually touches down.

Same test as Stage 1 so the two can be compared:
    pad 5 m East of start, take off to 10 m, land on the pad,
    optionally the pad jumps sideways near the ground (--jump).

ONE-TIME SETUP (sets parameters, then you restart SITL):
    python3 builtin_precland.py --setup
    -> Ctrl+C sim_vehicle.py in Terminal 1 and start it again (WITHOUT -w,
       which would wipe the parameters), from the same folder as before.

RUN:
    python3 builtin_precland.py --log ap_land.csv
    python3 builtin_precland.py --jump 0.8 --log ap_jump.csv
    python3 plot_logs.py ap_land.csv ap_jump.csv

Note: SIM_PLD_LAT/LON are stored as 32-bit floats. At longitude ~77 that is a
step of ~0.75 m, so the script reads the values back and measures the error
against where the pad really is, not where we asked it to be. The jump is
applied North (latitude, ~0.2 m step) for the same reason.

Needs fake_target_land.py in the same folder.
"""

import argparse
import math
import time

import fake_target_land as ftl
from fake_target_land import mavutil

M_PER_DEG = 111320.0
PAD_EAST = 5.0       # pad offset from the start position (m), same as Stage 1 tests

SETUP_PARAMS = {
    "PLND_ENABLED": 1,
    "PLND_TYPE": 4,          # SITL simulated landing-target sensor
    "SIM_PLD_ENABLE": 1,
    "RNGFND1_TYPE": 1,       # simulated analog rangefinder (precland needs height above ground)
    "RNGFND1_PIN": 0,
    "RNGFND1_SCALING": 12.12,
}
# Older firmware uses *_CM in centimetres, newer firmware uses metres.
RANGE_PARAMS = [("RNGFND1_MIN_CM", 0), ("RNGFND1_MAX_CM", 4000),
                ("RNGFND1_MIN", 0.0), ("RNGFND1_MAX", 40.0)]


def _pid(msg):
    pid = msg.param_id
    if isinstance(pid, bytes):
        pid = pid.decode(errors="ignore")
    return pid.rstrip("\x00")


def get_param(m, name, timeout=3):
    end = time.time() + timeout
    while time.time() < end:
        m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
        msg = ftl.recv(m, "PARAM_VALUE", timeout=1)
        if msg and _pid(msg) == name:
            return msg.param_value
    return None


def set_param(m, name, value, timeout=3):
    end = time.time() + timeout
    while time.time() < end:
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(),
                             float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        msg = ftl.recv(m, "PARAM_VALUE", timeout=1)
        if msg and _pid(msg) == name:
            return msg.param_value
    return None


POS_ONLY_MASK = 0b0000110111111000   # position only


def pad_offset(m, pad_lat, pad_lon):
    """Return (local position msg, north, east) from the drone to the pad."""
    lpos = gpos = None
    while lpos is None or gpos is None:
        msg = ftl.recv(m, ["LOCAL_POSITION_NED", "GLOBAL_POSITION_INT"], timeout=2)
        if msg is None:
            continue
        if msg.get_type() == "LOCAL_POSITION_NED":
            lpos = msg
        else:
            gpos = msg
    lat, lon = gpos.lat / 1e7, gpos.lon / 1e7
    dn = (pad_lat - lat) * M_PER_DEG
    de = (pad_lon - lon) * M_PER_DEG * math.cos(math.radians(lat))
    return lpos, dn, de


def goto_start(m, pad_lat, pad_lon, alt):
    """Fly to PAD_EAST m West of the pad, so every run starts from the same place."""
    lpos, dn, de = pad_offset(m, pad_lat, pad_lon)
    gx, gy = lpos.x + dn, lpos.y + de - PAD_EAST
    print(f"Moving to start point ({math.hypot(dn, de - PAD_EAST):.1f} m away)...")
    end = time.time() + 40
    while time.time() < end:
        m.mav.set_position_target_local_ned_send(
            0, m.target_system, m.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            POS_ONLY_MASK, gx, gy, -alt, 0, 0, 0, 0, 0, 0, 0, 0)
        p = ftl.recv(m, "LOCAL_POSITION_NED", timeout=1)
        if p and math.hypot(gx - p.x, gy - p.y) < 0.2 and abs(-p.z - alt) < 0.3 \
                and math.hypot(p.vx, p.vy) < 0.1:
            print("At start point.")
            return
    print("Could not reach the start point exactly, continuing anyway.")


def connect(addr):
    print(f"Connecting {addr} ...")
    m = mavutil.mavlink_connection(addr)
    ftl.MODES = ftl.wait_vehicle(m)
    print(f"Vehicle found (system {m.target_system})")
    return m


def setup(m):
    pos = None
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_POSITION, 5, 1)
    ftl.request_rate(m, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5)
    end = time.time() + 10
    while pos is None and time.time() < end:
        pos = ftl.recv(m, "GLOBAL_POSITION_INT", timeout=2)
    if pos is None:
        raise SystemExit("No GPS position yet. Wait for 'Ready' in QGC and try again.")
    lat, lon = pos.lat / 1e7, pos.lon / 1e7
    pad_lon = lon + PAD_EAST / (M_PER_DEG * math.cos(math.radians(lat)))

    missing = []
    for name, val in SETUP_PARAMS.items():
        got = set_param(m, name, val)
        print(f"  {name:16s} = {got}" if got is not None else f"  {name:16s} -> not found")
        if got is None:
            missing.append(name)
    for name, val in RANGE_PARAMS:
        got = set_param(m, name, val, timeout=1.5)
        if got is not None:
            print(f"  {name:16s} = {got}")
    got_lat = set_param(m, "SIM_PLD_LAT", lat)
    got_lon = set_param(m, "SIM_PLD_LON", pad_lon)
    print(f"  SIM_PLD_LAT      = {got_lat}")
    print(f"  SIM_PLD_LON      = {got_lon}   (pad ~{PAD_EAST:.0f} m East of current position)")
    if missing:
        print(f"\nNot found yet: {', '.join(missing)}. Some parameters only appear after "
              "PLND_ENABLED=1 and a restart.")
        print("Restart SITL (Ctrl+C in Terminal 1, run sim_vehicle.py again WITHOUT -w), "
              "then run --setup once more.")
    else:
        print("\nDone. Now restart SITL: Ctrl+C in Terminal 1, then run sim_vehicle.py again WITHOUT -w.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", default="tcp:127.0.0.1:5762")
    ap.add_argument("--setup", action="store_true", help="set precland/rangefinder params (once)")
    ap.add_argument("--alt", type=float, default=10.0)
    ap.add_argument("--jump", type=float, default=0.0,
                    help="pad jumps this far North (m) when the drone first descends below --jump-alt")
    ap.add_argument("--jump-alt", type=float, default=1.2)
    ap.add_argument("--max-land", type=float, default=60.0,
                    help="no touchdown within this many seconds of LAND -> count as failed, abort")
    ap.add_argument("--log", default=None)
    args = ap.parse_args()

    m = connect(args.connect)
    if args.setup:
        setup(m)
        return

    # ---- check setup ----
    en, typ = get_param(m, "PLND_ENABLED"), get_param(m, "PLND_TYPE")
    if en != 1 or typ != 4:
        raise SystemExit(f"PLND_ENABLED={en}, PLND_TYPE={typ}. Run with --setup first, then restart SITL.")
    pad_lat0, pad_lon0 = get_param(m, "SIM_PLD_LAT"), get_param(m, "SIM_PLD_LON")
    print(f"Pad at {pad_lat0:.7f}, {pad_lon0:.7f} (as stored)")

    ftl.request_rate(m, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10)
    ftl.request_rate(m, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10)
    ftl.request_rate(m, mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2)

    ftl.set_mode(m, "GUIDED")
    ftl.arm(m)
    ftl.takeoff(m, args.alt)
    goto_start(m, pad_lat0, pad_lon0, args.alt)
    time.sleep(2)   # settle

    log_name = args.log or time.strftime("ap_%H%M%S.csv")
    log = open(log_name, "w")
    log.write("t,error,alt,tx,ty,x,y,vx,vy,vz,descending\n")
    print(f"Log: {log_name}")

    pad_lat, pad_lon = pad_lat0, pad_lon0
    jumped = False
    lpos = gpos = None
    t0 = time.time()
    last_print = 0.0
    err = float("nan")
    ftl.set_mode(m, "LAND")
    print("LAND mode: ArduPilot precision landing is in control now.")
    touch = None            # (time, error) when touchdown was first detected
    still_since = None
    disarm_sent = False
    retry_after_touch = False
    failed = False
    try:
        while True:
            msg = ftl.recv(m, ["LOCAL_POSITION_NED", "GLOBAL_POSITION_INT", "HEARTBEAT"], timeout=2)
            if msg is None:
                continue
            typ = msg.get_type()
            if typ == "HEARTBEAT":
                if not ftl.is_armed(msg):
                    break
                continue
            if typ == "GLOBAL_POSITION_INT":
                gpos = msg
                continue
            lpos = msg
            if gpos is None:
                continue

            t = time.time() - t0
            alt = -lpos.z
            lat, lon = gpos.lat / 1e7, gpos.lon / 1e7
            dn = (pad_lat - lat) * M_PER_DEG
            de = (pad_lon - lon) * M_PER_DEG * math.cos(math.radians(lat))
            err = math.hypot(dn, de)

            if args.jump and not jumped and alt < args.jump_alt:
                want = pad_lat + args.jump / M_PER_DEG
                got = set_param(m, "SIM_PLD_LAT", want, timeout=1)
                if got is not None:
                    real = (got - pad_lat) * M_PER_DEG
                    pad_lat = got
                    print(f"t={t:5.1f}s  PAD JUMPED {real:.2f} m North at height {alt:.1f} m")
                jumped = True

            log.write(f"{t:.2f},{err:.3f},{alt:.2f},{lpos.x + dn:.3f},{lpos.y + de:.3f},"
                      f"{lpos.x:.3f},{lpos.y:.3f},{lpos.vx:.3f},{lpos.vy:.3f},{lpos.vz:.3f},1\n")
            if t - last_print > 1.0:
                print(f"t={t:5.1f}s  error={err:5.2f} m  height={alt:5.1f} m")
                last_print = t

            # ---- our own touchdown detector: on the ground and not moving ----
            on_ground = alt < 0.2 and abs(lpos.vz) < 0.1 and math.hypot(lpos.vx, lpos.vy) < 0.1
            if on_ground:
                still_since = still_since or t
                if touch is None and t - still_since > 1.0:
                    touch = (t, err)
                    print(f"t={t:5.1f}s  TOUCHDOWN detected, error {err:.2f} m")
            else:
                still_since = None
                if touch is not None and alt > 0.5 and not retry_after_touch:
                    retry_after_touch = True
                    print(f"t={t:5.1f}s  WARNING: ArduPilot took off again after touchdown (precland retry)")

            # ArduPilot should disarm by itself ~1-2 s after touchdown.
            # Still armed after 3 s AND still on the ground -> disarm (SITL only).
            # Never disarm in the air.
            if touch is not None and not disarm_sent and on_ground and t - touch[0] > 3:
                print("ArduPilot did not disarm after touchdown -> sending disarm (SITL only)")
                disarm(m)
                disarm_sent = True
            # Took off again after touching down -> stop the experiment, come down safely.
            if retry_after_touch:
                abort_descend(m)
                break
            # Never landed within the time limit -> record a failure, come down safely.
            if touch is None and t > args.max_land:
                failed = True
                print(f"t={t:5.1f}s  FAILED: no touchdown within {args.max_land:.0f} s "
                      f"(last error {err:.2f} m) -> aborting")
                abort_descend(m)
                break
    except KeyboardInterrupt:
        print("\nCtrl+C -> disarm if on the ground, otherwise come straight down")
        if lpos is not None and -lpos.z < 0.3:
            disarm(m)
        else:
            abort_descend(m)
    finally:
        log.close()
        if jumped:
            set_param(m, "SIM_PLD_LAT", pad_lat0)   # put the pad back for the next run

    if failed:
        print("RESULT: FAILED to land within the time limit.")
    if touch:
        print(f"Touchdown error: {touch[1]:.2f} m")
        print(f"Time in LAND until touchdown: {touch[0]:.1f} s")
    else:
        print(f"No touchdown detected. Last error: {err:.2f} m")
    if retry_after_touch:
        print("NOTE: ArduPilot climbed again after touching down - see the [drone] messages above.")
    print("Done.")


def abort_descend(m, speed=0.5, timeout=60):
    """Leave LAND (which would keep running precland) and come straight down in GUIDED."""
    print("Abort: GUIDED, straight down, then disarm.")
    ftl.set_mode(m, "GUIDED")
    end = time.time() + timeout
    still = None
    while time.time() < end:
        ftl.send_velocity(m, 0.0, 0.0, speed)
        p = ftl.recv(m, "LOCAL_POSITION_NED", timeout=1)
        if p is None:
            continue
        if -p.z < 0.2 and abs(p.vz) < 0.1:
            still = still or time.time()
            if time.time() - still > 1.0:
                disarm(m)
                return
        else:
            still = None
    print("Abort descent timed out - check QGC.")


def disarm(m):
    """Normal disarm first; if refused, force it (magic 21196). Only safe in SITL."""
    for force in (0, 21196):
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                0, force, 0, 0, 0, 0, 0)
        end = time.time() + 3
        while time.time() < end:
            hb = ftl.recv(m, "HEARTBEAT", timeout=1)
            if hb and not ftl.is_armed(hb):
                print("DISARMED")
                return True
    print("Disarm failed - disarm from QGC or MAVProxy (disarm force).")
    return False

if __name__ == "__main__":
    main()
