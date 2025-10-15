# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

# test:device D400*
# test:donotrun

import pyrealsense2 as rs
from rspy import log, test
import numpy as np
import cv2
import time
from iq_helper import find_roi_location, A4_WIDTH, A4_HEIGHT

NUM_FRAMES = 10 # Number of frames to check
DEPTH_TOLERANCE = 0.05  # Acceptable deviation from expected depth in meters
FRAMES_PASS_THRESHOLD =0.8 # Percentage of frames that needs to pass
DEBUG_MODE = True

test.start("Basic Depth Image Quality Test")

try:
    dev, ctx = test.find_first_device_or_exit()

    pipeline = rs.pipeline(ctx)
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30) # needed for finding the ArUco markers
    profile = pipeline.start(cfg)
    time.sleep(2)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()


    # find region of interest (page) and get the transformation matrix
    M, _ = find_roi_location(pipeline, (4,5,6,7), DEBUG_MODE)  # markers in the lab are 4,5,6,7
    depth_passes = {}
    for i in range(NUM_FRAMES):
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_image = frames.get_color_frame()
        if not depth_frame:
            continue

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_image.get_data())
        colorizer = rs.colorizer()
        colorized_depth_image = np.asanyarray(colorizer.colorize(depth_frame).get_data())

        # use M to get the region of interest - our colored grid printed in the lab
        a3_img = cv2.warpPerspective(depth_image, M, (A4_WIDTH, A4_HEIGHT))

        # Known pixel positions and expected depth values (in meters)
        # Using temporary values until setup in lab is completed
        depth_points = {
            "cube":  ((a3_img.shape[0] // 2, a3_img.shape[1] // 2), 0.45), # center of page, cube at 0.45m
            "background": ((int(a3_img.shape[0] * 0.1), a3_img.shape[1] // 2), 0.6), # left edge, background at 0.6m
        }
        if not depth_passes:
            depth_passes = {name: 0 for name in depth_points}
        for point_name, ((x, y), expected_depth) in depth_points.items():
            raw_depth = a3_img[y, x]
            depth_value = raw_depth * depth_scale  # Convert to meters

            if abs(depth_value - expected_depth) <= DEPTH_TOLERANCE:
                depth_passes[point_name] += 1
            else:
                log.d(f"Frame {i} - {point_name} at ({x},{y}): {depth_value:.3f}m ≠ {expected_depth:.3f}m")

        if DEBUG_MODE:
            # use rs align to align depth to color for better visualization
            align = rs.align(rs.stream.color)
            aligned_frameset = align.process(frames)

            aligned_color = aligned_frameset.get_color_frame()
            aligned_depth = aligned_frameset.get_depth_frame()
            colorized_depth = colorizer.colorize(aligned_depth)

            # Convert to numpy arrays
            depth_image = np.asanyarray(colorized_depth.get_data())
            color_image = np.asanyarray(aligned_color.get_data())

            # To see the depth on top of the color, blend the images
            alpha = 0.5  # transparency factor
            overlay = cv2.addWeighted(depth_image, 1 - alpha, color_image, alpha, 0)

            # crop the image according to the markers found
            #overlay = cv2.warpPerspective(overlay, M, (A4_WIDTH, A4_HEIGHT))
            cv2.imshow('Overlay', overlay)
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
