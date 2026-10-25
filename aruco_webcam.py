#!/usr/bin/env python3
"""
STAGE 2 - Mac webcam se ArUco marker detect karo aur uski position nikalo

Ye precision landing ki "aankh" hai. Har frame mein:
  - marker dhoondo
  - camera se marker kitna door hai (Z) aur kitna left/right, upar/neeche (X, Y)
  - angle offsets (yahi baad mein ArduPilot ke LANDING_TARGET message mein jaate hain)

Setup (ek baar):
  pip install opencv-contrib-python numpy

Step 1 - marker banao:
  python3 aruco_webcam.py --make-marker
  -> marker_id0.png banega. Print karo, aur printed kaale square ki
     asli width ruler se naapo (jaise 10 cm = 0.10).

Step 2 - chalao:
  python3 aruco_webcam.py --size 0.10
  (Pehli baar macOS Terminal ke liye Camera permission maangega -> Allow.)
  'q' dabake band karo.

Note: camera calibration nahi kiya hai, isliye focal length andaaze se hai
(--hfov). Distance thoda galat ho sakta hai; tape se naapke --hfov adjust karo.
"""

import argparse
import math
import time

import cv2
import numpy as np

DICT = cv2.aruco.DICT_4X4_50


def make_marker(marker_id, px):
    dictionary = cv2.aruco.getPredefinedDictionary(DICT)
    img = cv2.aruco.generateImageMarker(dictionary, marker_id, px)
    # safed border, warna print mein detect karna mushkil hota hai
    img = cv2.copyMakeBorder(img, px // 5, px // 5, px // 5, px // 5,
                             cv2.BORDER_CONSTANT, value=255)
    name = f"marker_id{marker_id}.png"
    cv2.imwrite(name, img)
    print(f"Saved {name}. Print karo aur kaale square ki width naapo.")


def camera_matrix(w, h, hfov_deg):
    f = (w / 2) / math.tan(math.radians(hfov_deg) / 2)
    K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]], dtype=np.float64)
    return K, np.zeros(5)


def marker_object_points(size):
    s = size / 2
    # ArUco corner order: top-left, top-right, bottom-right, bottom-left
    return np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float64)


def detect(detector, gray, obj_pts, K, dist, marker_id):
    """Return (corners, rvec, tvec) ya None."""
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return None
    for c, i in zip(corners, ids.flatten()):
        if marker_id is not None and i != marker_id:
            continue
        img_pts = c.reshape(4, 2).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if ok:
            return c, rvec, tvec.flatten()
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--make-marker", action="store_true")
    ap.add_argument("--id", type=int, default=0, help="marker ID")
    ap.add_argument("--size", type=float, default=0.10, help="printed marker width (m)")
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--hfov", type=float, default=70.0, help="webcam horizontal FOV (degree, andaaza)")
    args = ap.parse_args()

    if args.make_marker:
        make_marker(args.id, 600)
        return

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise SystemExit("Camera nahi khula. System Settings > Privacy > Camera mein Terminal allow karo.")

    ok, frame = cap.read()
    if not ok:
        raise SystemExit("Camera se frame nahi aaya.")
    h, w = frame.shape[:2]
    K, dist = camera_matrix(w, h, args.hfov)
    obj_pts = marker_object_points(args.size)
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(DICT), cv2.aruco.DetectorParameters())

    print(f"Camera {w}x{h}, marker size {args.size} m. 'q' se band karo.")
    last_print = 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        res = detect(detector, gray, obj_pts, K, dist, args.id)

        if res:
            corners, rvec, (x, y, z) = res
            # camera frame: x = right, y = down, z = aage (door)
            angle_x = math.degrees(math.atan2(x, z))
            angle_y = math.degrees(math.atan2(y, z))
            cv2.aruco.drawDetectedMarkers(frame, [corners])
            cv2.drawFrameAxes(frame, K, dist, rvec, np.array([x, y, z]), args.size * 0.5)
            text = f"dist {z*100:5.1f} cm  x {x*100:+5.1f} cm  y {y*100:+5.1f} cm"
            cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"angle x {angle_x:+5.1f} deg  y {angle_y:+5.1f} deg",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            if time.time() - last_print > 0.5:
                print(f"{text}  |  angle_x {angle_x:+5.1f}  angle_y {angle_y:+5.1f}")
                last_print = time.time()
        else:
            cv2.putText(frame, "marker nahi mila", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("ArUco", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
