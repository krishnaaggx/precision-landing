#!/usr/bin/env python3
"""
STAGE 3 - A real camera drives a simulated drone.

The webcam detects an ArUco marker in your hand. The marker's offset from the
camera centre becomes the target position for the SITL drone, and the Stage 1
controller flies the drone to it and lands.

    Move the marker right   -> target moves East
    Move the marker up      -> target moves North
    Hold it still, centred  -> drone centres and lands

IMPORTANT LIMITATION (read this):
The webcam does not move with the drone. So this is NOT yet a closed loop
through the camera: the drone moving does not change what the camera sees.
Here the camera only tells the drone WHERE the target is. A true closed loop
needs a camera mounted on the drone (Gazebo or real hardware). This stage
tests the camera -> controller pipeline: detection, noise, delay, and what to
do when the marker is lost.

Run:
    Terminal 1:  sim_vehicle.py -v ArduCopter     (wait for "Ready" in QGC)
    Terminal 2:  python3 aruco_sitl_land.py --size 0.146 --hfov 68.2
                 python3 aruco_sitl_land.py --size 0.146 --hfov 68.2 --ff --log cam_ff.csv
                 python3 aruco_sitl_land.py --gain 10 --hover 40 --log cam_move.csv
                 (--hover 40: follow the marker for 40 s before landing is allowed)

Press 'q' in the camera window (or Ctrl+C in the terminal) to stop. Both make
the drone LAND.

Needs fake_target_land.py and aruco_webcam.py in the same folder.
"""

import argparse
import math
import threading
import time

import cv2
import numpy as np

import aruco_webcam as cam
import fake_target_land as ftl

# ---- Tuning ----
GAIN = 20.0          # metres of target movement per metre of marker movement
ALPHA_POS = 0.3      # low-pass filter on marker position (0..1, lower = smoother, slower)
ALPHA_VEL = 0.1      # low-pass filter on target velocity estimate (for --ff)
LOST_HOLD = 0.5      # marker unseen this long (s) -> hover, stop descending
LOST_RTL = 10.0      # marker unseen this long (s) -> RTL
MAX_FLIGHT = 300.0   # whole flight limit (s) -> RTL


class Shared:
    """Data passed from the camera loop (main thread) to the control thread."""

    def __init__(self):
        self.lock = threading.Lock()
        self.east = 0.0          # filtered marker offset, metres (camera scale)
        self.north = 0.0
        self.v_east = 0.0        # filtered marker velocity, m/s (camera scale)
        self.v_north = 0.0
        self.seen_time = None    # time of last detection
        self.status = "starting"
        self.stop = False
        self.done = False        # control thread finished (landed / RTL)
        self.target = None       # target position when LAND started


def control_loop(m, args, shared, log):
    start = ftl.recv(m, "LOCAL_POSITION_NED", timeout=3)
    cx, cy = start.x, start.y
    t0 = time.time()
    descending = False
    retrying = False
    last_print = 0.0
    try:
        while not shared.stop:
            pos = ftl.recv(m, "LOCAL_POSITION_NED", timeout=1)
            if pos is None:
                continue
            now = time.time()
            t = now - t0
            alt = -pos.z

            if t > MAX_FLIGHT:
                print(f"SAFETY: {MAX_FLIGHT:.0f}s flight limit -> RTL")
                ftl.set_mode(m, "RTL")
                return

            with shared.lock:
                east, north = shared.east, shared.north
                v_east, v_north = shared.v_east, shared.v_north
                seen = shared.seen_time
            age = (now - seen) if seen else float("inf")

            # ---- marker lost ----
            if age > LOST_HOLD:
                descending = False
                ftl.send_velocity(m, 0.0, 0.0, 0.0)
                if age > LOST_RTL:
                    print(f"SAFETY: marker lost for {LOST_RTL:.0f}s -> RTL")
                    ftl.set_mode(m, "RTL")
                    return
                status = f"MARKER LOST {age:4.1f}s - hovering"
                log.write(f"{t:.2f},nan,{alt:.2f},nan,nan,{pos.x:.3f},{pos.y:.3f},0,0,0,0\n")
                with shared.lock:
                    shared.status = status
                if t - last_print > 1.0:
                    print(f"t={t:5.1f}s  height={alt:5.1f} m  {status}")
                    last_print = t
                continue

            # ---- target from camera ----
            tx = cx + args.gain * north
            ty = cy + args.gain * east
            tvx, tvy = args.gain * v_north, args.gain * v_east

            ex, ey = tx - pos.x, ty - pos.y
            dist = math.hypot(ex, ey)
            vx, vy = ftl.KP * ex, ftl.KP * ey
            if args.ff:
                vx += tvx
                vy += tvy
            speed = math.hypot(vx, vy)
            if speed > ftl.VMAX:
                vx, vy = vx * ftl.VMAX / speed, vy * ftl.VMAX / speed

            start_tol, stop_tol = ftl.tolerances(alt, args.cone)
            if not descending and dist < start_tol and t > args.hover and not retrying:
                descending = True
            elif descending and dist > stop_tol:
                descending = False
                if args.cone and alt < ftl.RETRY_ALT:
                    retrying = True
                    print(f"t={t:5.1f}s  RETRY: {dist:.2f} m off at {alt:.1f} m -> climbing to {ftl.RETRY_ALT} m")
            if retrying and alt >= ftl.RETRY_ALT:
                retrying = False
            if retrying:
                vz = -ftl.RETRY_CLIMB
            elif descending:
                vz = ftl.SLOW_DESCENT if alt < ftl.SLOW_ALT else ftl.DESCENT_SPEED
            else:
                vz = 0.0

            ftl.send_velocity(m, vx, vy, vz)
            log.write(f"{t:.2f},{dist:.3f},{alt:.2f},{tx:.3f},{ty:.3f},"
                      f"{pos.x:.3f},{pos.y:.3f},{vx:.3f},{vy:.3f},{vz:.3f},{int(descending)}\n")

            phase = ("RETRY" if retrying else "DESCENDING" if descending
                     else "FOLLOWING" if t <= args.hover else "CENTRING")
            status = phase + f"  err {dist:4.2f} m  h {alt:4.1f} m"
            with shared.lock:
                shared.status = status
            if t - last_print > 1.0:
                print(f"t={t:5.1f}s  error={dist:5.2f} m  height={alt:5.1f} m  {phase}")
                last_print = t

            if alt < ftl.LAND_ALT and descending:
                print(f"Final error {dist:.2f} m at {alt:.1f} m -> LAND")
                ftl.set_mode(m, "LAND")
                shared.target = (tx, ty)
                return
    finally:
        shared.done = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", default="tcp:127.0.0.1:5762")
    ap.add_argument("--size", type=float, default=0.146, help="marker black square width (m)")
    ap.add_argument("--hfov", type=float, default=68.2, help="webcam horizontal FOV (deg)")
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--alt", type=float, default=10.0)
    ap.add_argument("--gain", type=float, default=GAIN)
    ap.add_argument("--ff", action="store_true", help="feedforward from estimated marker velocity")
    ap.add_argument("--cone", action="store_true", help="landing cone: allowed error shrinks with height")
    ap.add_argument("--hover", type=float, default=0.0,
                    help="only follow (no descent) for the first N seconds - use for follow tests")
    ap.add_argument("--log", default=None)
    args = ap.parse_args()

    # ---- camera first: fail early if it does not open ----
    cap = cv2.VideoCapture(args.camera)
    ok, frame = cap.read()
    if not ok:
        raise SystemExit("Camera did not open. Allow Terminal in System Settings > Privacy > Camera.")
    h, w = frame.shape[:2]
    K, dist_coeffs = cam.camera_matrix(w, h, args.hfov)
    obj_pts = cam.marker_object_points(args.size)
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cam.DICT), cv2.aruco.DetectorParameters())

    # ---- drone ----
    print(f"Connecting {args.connect} ...")
    m = ftl.mavutil.mavlink_connection(args.connect)
    ftl.MODES = ftl.wait_vehicle(m)
    print(f"Vehicle found (system {m.target_system})")
    ftl.request_rate(m, ftl.mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10)
    ftl.request_rate(m, ftl.mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2)

    hb = ftl.recv(m, "HEARTBEAT", timeout=3)
    pos0 = ftl.recv(m, "LOCAL_POSITION_NED", timeout=3)
    ftl.set_mode(m, "GUIDED")
    if hb and ftl.is_armed(hb) and pos0 and -pos0.z > 2.0:
        print(f"Already airborne ({-pos0.z:.1f} m), skipping takeoff.")
    else:
        ftl.arm(m)
        ftl.takeoff(m, args.alt)

    log_name = args.log or time.strftime("cam_%H%M%S.csv")
    log = open(log_name, "w")
    log.write("t,error,alt,tx,ty,x,y,vx,vy,vz,descending\n")
    print(f"Log: {log_name}")
    print("Hold the marker in front of the camera. 'q' or Ctrl+C = LAND.")

    shared = Shared()
    ctrl = threading.Thread(target=control_loop, args=(m, args, shared, log), daemon=True)
    ctrl.start()

    # ---- camera loop (must stay on the main thread on macOS) ----
    f_east = f_north = None
    f_ve = f_vn = 0.0
    last_t = None
    user_stop = False
    try:
        while not shared.done:
            ok, frame = cap.read()
            if not ok:
                break
            now = time.time()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            res = cam.detect(detector, gray, obj_pts, K, dist_coeffs, args.id)

            if res:
                corners, rvec, (x, y, z) = res
                cv2.aruco.drawDetectedMarkers(frame, [corners])
                # camera faces you: your right = camera's left (-x), up = -y
                raw_e, raw_n = -x, -y
                if f_east is None:
                    f_east, f_north = raw_e, raw_n
                else:
                    pe, pn = f_east, f_north
                    f_east += ALPHA_POS * (raw_e - f_east)
                    f_north += ALPHA_POS * (raw_n - f_north)
                    dt = now - last_t
                    if dt > 0:
                        f_ve += ALPHA_VEL * ((f_east - pe) / dt - f_ve)
                        f_vn += ALPHA_VEL * ((f_north - pn) / dt - f_vn)
                last_t = now
                with shared.lock:
                    shared.east, shared.north = f_east, f_north
                    shared.v_east, shared.v_north = f_ve, f_vn
                    shared.seen_time = now

            # mirror so moving the marker right moves it right on screen
            view = cv2.flip(frame, 1)
            cv2.drawMarker(view, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 40, 2)
            with shared.lock:
                status = shared.status
            color = (0, 0, 255) if "LOST" in status else (0, 255, 0)
            cv2.putText(view, status, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)
            if f_east is not None:
                cv2.putText(view, f"target E {args.gain * f_east:+5.1f} m  N {args.gain * f_north:+5.1f} m",
                            (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            cv2.imshow("Stage 3 - camera drives SITL", view)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                user_stop = True
                break
    except KeyboardInterrupt:
        user_stop = True

    shared.stop = True
    ctrl.join(timeout=3)
    cap.release()
    cv2.destroyAllWindows()
    log.close()

    if user_stop and not shared.done:
        print("Stopped by user -> LAND")
        ftl.set_mode(m, "LAND")

    ftl.wait_disarm(m, shared.target)


if __name__ == "__main__":
    main()
