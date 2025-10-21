# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

# test:device D400*
# test:donotrun

import pyrealsense2 as rs
from rspy import log, test
import numpy as np
import cv2
import time
from iq_helper import find_roi_location, get_roi_from_frame

NUM_FRAMES = 100 # Number of frames to check
DEPTH_TOLERANCE = 0.05  # Acceptable deviation from expected depth in meters
FRAMES_PASS_THRESHOLD =0.8 # Percentage of frames that needs to pass
DEBUG_MODE = False

test.start("Basic Depth Image Quality Test")

try:
    dev, ctx = test.find_first_device_or_exit()

    pipeline = rs.pipeline(ctx)
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)  # needed for finding the ArUco markers
    profile = pipeline.start(cfg)
    time.sleep(2)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # find region of interest (page) and get the transformation matrix
    find_roi_location(pipeline, (4,5,6,7), DEBUG_MODE)  # markers in the lab are 4,5,6,7
    depth_passes = {}
    for i in range(NUM_FRAMES):
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        infrared_frame = frames.get_infrared_frame()
        if not depth_frame:
            continue

        depth_image = get_roi_from_frame(depth_frame)

        # Known pixel positions and expected depth values (in meters)
        # Using temporary values until setup in lab is completed
        h, w = depth_image.shape
        depth_points = {
            "cube":  ((h // 2, w // 2), 0.45), # center of page, cube at 0.45m
            "background": ((h // 2, int(w * 0.1)), 0.6), # left edge, background at 0.6m
        }
        if not depth_passes:
            depth_passes = {name: 0 for name in depth_points}
        for point_name, ((x, y), expected_depth) in depth_points.items():
            raw_depth = depth_image[y, x]
            depth_value = raw_depth * depth_scale  # Convert to meters

            if abs(depth_value - expected_depth) <= DEPTH_TOLERANCE:
                depth_passes[point_name] += 1
            else:
                log.d(f"Frame {i} - {point_name} at ({x},{y}): {depth_value:.3f}m ≠ {expected_depth:.3f}m")

        if DEBUG_MODE:
            # display IR image along with transformed view of IR, get_roi_from_frame(infrared_frame)
            infrared_np = np.asanyarray(infrared_frame.get_data())
            w, h = infrared_np.shape
            dbg_resized = cv2.resize(get_roi_from_frame(infrared_frame), (h, w))

            dbg = np.hstack([infrared_np, dbg_resized])
            cv2.imshow("Depth IQ - IR | Depth", dbg)
            cv2.waitKey(1)

    # wait for close
    if DEBUG_MODE:
        cv2.waitKey(0)

    # Check that each point passed the threshold
    min_passes = int(NUM_FRAMES * FRAMES_PASS_THRESHOLD)
    for point_name, count in depth_passes.items():
        log.i(f"{point_name.title()} passed in {count}/{NUM_FRAMES} frames")
        test.check(count >= min_passes)

except Exception as e:
    test.fail()
    raise e
finally:
    cv2.destroyAllWindows()

pipeline.stop()
test.finish()
test.print_results_and_exit()
