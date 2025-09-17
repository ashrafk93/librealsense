# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#test:device:jetson D457
#test:device:!jetson D455

import pyrealsense2 as rs
from rspy import log, test
import numpy as np
import cv2
import time

NUM_FRAMES = 100 # Number of frames to check
COLOR_TOLERANCE = 20 # Acceptable per-channel deviation in RGB values
FRAMES_PASS_THRESHOLD =0.8 # Percentage of frames that needs to pass
DEBUG_MODE = False

# A4 size in pixels at 96 DPI
A4_WIDTH = 794
A4_HEIGHT = 1123

# expected colors (insertion order -> mapped row-major to 3x3 grid)
expected_colors = {
    "red":   (139, 48, 57),
    "green": (30, 84, 72),
    "blue":  (8, 67, 103),
    "black": (25, 27, 14),
    "white": (140, 142, 143),
    "gray": (84, 84, 84),
    "purple": (56, 52, 78),
    "orange": (150, 66, 60),
    "yellow": (152, 132, 69),
}
# list of color names in insertion order -> used left->right, top->bottom
color_names = list(expected_colors.keys())

# we are given a 3x3 grid, we split it using 2 vertical and 2 horizontal separators
# we also calculate the center of each grid cell for sampling from it for the test
xs = [A4_WIDTH / 6.0, A4_WIDTH / 2.0, 5.0 * A4_WIDTH / 6.0]
ys = [A4_HEIGHT / 6.0, A4_HEIGHT / 2.0, 5.0 * A4_HEIGHT / 6.0]
centers = [(x, y) for y in ys for x in xs]


def is_color_close(actual, expected, tolerance):
    return all(abs(int(a) - int(e)) <= tolerance for a, e in zip(actual, expected))

def compute_homography(pts):
    pts_sorted = sorted(pts, key=lambda p: (p[1], p[0]))
    top_left, top_right = sorted(pts_sorted[:2], key=lambda p: p[0])
    bottom_left, bottom_right = sorted(pts_sorted[2:], key=lambda p: p[0])

    src = np.array([top_left, top_right, bottom_right, bottom_left], dtype=np.float32)
    dst = np.array([[0,0],[A4_WIDTH-1,0],[A4_WIDTH-1,A4_HEIGHT-1],[0,A4_HEIGHT-1]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(src, dst)
    return M  # we later use M to get our roi


def draw_debug(frame_bgr, a4_page_bgr):
    """
    Simple debug view:
      - left: camera frame
      - right: focused view on the A4 page with grid and color names
    """
    vertical_lines = [A4_WIDTH / 3.0, 2.0 * A4_WIDTH / 3.0]
    horizontal_lines = [A4_HEIGHT / 3.0, 2.0 * A4_HEIGHT / 3.0]
    H, W = a4_page_bgr.shape[:2]

    # draw grid on a4 page image
    for x in vertical_lines:
        cv2.line(a4_page_bgr, (int(x), 0), (int(x), H - 1), (255, 255, 255), 2)
    for y in horizontal_lines:
        cv2.line(a4_page_bgr, (0, int(y)), (W - 1, int(y)), (255, 255, 255), 2)

    # label centers with color names
    for i, (cx, cy) in enumerate(centers):
        cx_i, cy_i = int(round(cx)), int(round(cy))
        lbl = color_names[i] if i < len(color_names) else str(i)
        # white marker with black text for readability
        cv2.circle(a4_page_bgr, (cx_i, cy_i), 10, (255, 255, 255), -1)
        cv2.putText(a4_page_bgr, lbl, (cx_i + 12, cy_i + 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)

    # resize and display side by side
    height = 600
    frame_width = int(frame_bgr.shape[1] * (height / frame_bgr.shape[0]))
    a4_page_width = int(a4_page_bgr.shape[1] * (height / a4_page_bgr.shape[0]))
    left = cv2.resize(frame_bgr, (frame_width, height))
    right = cv2.resize(a4_page_bgr, (a4_page_width, height))
    return np.hstack([left, right])


def detect_a4_page(img, dict_type=cv2.aruco.DICT_4X4_1000, required_ids=(0,1,2,3)):
    """
    Detect ArUco markers and return center of each one
    Returns None if not all required markers are found
    """
    # init aruco detector
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(dict_type)
    try:
        # new API (OpenCV >= 4.7)
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(dictionary, parameters)
        corners, ids, _ = detector.detectMarkers(img)
    except AttributeError:
        # legacy API (OpenCV <= 4.6) - used on some of our machines
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(img, dictionary, parameters=parameters)

    if ids is None or not all(rid in ids for rid in required_ids):
        return None

    ids = ids.flatten()  # flatten to 1D array
    id_to_corner = dict(zip(ids.flatten(), corners))  # map id to corners
    values = [id_to_corner[rid][0].mean(axis=0) for rid in required_ids] # for each required id, get center of marker coords

    return np.array(values, dtype=np.float32)


def find_roi_location(pipeline):
    """
    Returns a matrix that transforms from frame to region of interest
    This matrix will later be used with cv2.warpPerspective()
    """
    # stream until page found
    page_pts = None
    start_time = time.time()
    while page_pts is None and time.time() - start_time < 5:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        img_bgr = np.asanyarray(color_frame.get_data())

        if DEBUG_MODE:
            cv2.imshow("PageDetect - waiting for page", img_bgr)
            cv2.waitKey(1)

        page_pts = detect_a4_page(img_bgr)

    if page_pts is None:
        log.e("Failed to detect page within timeout")
        test.fail()
        raise Exception("Page not found")

    # page found - use it to calculate transformation matrix from frame to region of interest
    M = compute_homography(page_pts)
    cv2.destroyAllWindows()
    return M, page_pts

def run_test(resolution, fps):
    test.start("Basic Color Image Quality Test:", f"{resolution[0]}x{resolution[1]} @ {fps}fps")
    color_match_count = {color: 0 for color in expected_colors.keys()}
    dev, ctx = test.find_first_device_or_exit()
    pipeline = rs.pipeline(ctx)
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, fps)
    pipeline_profile = pipeline.start(cfg)
    for i in range(30):  # skip initial frames
        pipeline.wait_for_frames()
    try:

        # find region of interest (page) and get the transformation matrix
        # page_pts is only used for debug display
        M, page_pts = find_roi_location(pipeline)

        # sampling loop
        for i in range(NUM_FRAMES):
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            img_bgr = np.asanyarray(color_frame.get_data())

            # use M to get the region of interest - our colored grid printed in the lab
            a4_bgr = cv2.warpPerspective(img_bgr, M, (A4_WIDTH, A4_HEIGHT))

            # sample each grid center and compare to expected color by row-major insertion order
            for idx, (x, y) in enumerate(centers):
                color = color_names[idx] if idx < len(color_names) else str(idx)
                expected_rgb = expected_colors[color]
                x = int(round(x))
                y = int(round(y))
                pixel = tuple(int(v) for v in a4_bgr[y, x])[::-1]  # -1 because we stream BGR but expect RGB
                if is_color_close(pixel, expected_rgb, COLOR_TOLERANCE):
                    color_match_count[color] += 1
                else:
                    log.d(f"Frame {i} - {color} at ({x},{y}) sampled: {pixel} too far from expected {expected_rgb}")

            if DEBUG_MODE:
                dbg = draw_debug(img_bgr, a4_bgr)
                cv2.imshow("PageDetect - camera | A4", dbg)
                cv2.waitKey(1)

        # wait for close
        # if DEBUG_MODE:
        #     cv2.waitKey(0)

        # check colors sampled correctly
        min_passes = int(NUM_FRAMES * FRAMES_PASS_THRESHOLD)
        for name, count in color_match_count.items():
            log.i(f"{name.title()} passed in {count}/{NUM_FRAMES} frames")
            test.check(count >= min_passes)

    except Exception as e:
        test.fail()
        raise e
    finally:
        cv2.destroyAllWindows()

    pipeline.stop()
    test.finish()


configurations = [
    ((640,480), 15),
    ((640,480), 30),
    ((640,480), 60),
    ((848,480), 15),
    ((848,480), 30),
    ((848,480), 60),
    ((1280,720), 5),
    ((1280,720), 10),
    ((1280,720), 15),
]
log.d("context is:", test.context)

if "nightly" not in test.context:
    run_test((1280,720), 30)
else:
    for cfg in configurations:
        run_test(*cfg)

test.print_results_and_exit()
