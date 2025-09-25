# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

# test:device each(D400*)
# test:donotrun:!nightly

import pyrealsense2 as rs
from rspy import test, log
import time

# This test is checking that timestamps of depth, infrared and color frames are consistent

def detect_frame_drops(frames_dict, prev_frame_counters):
    """
    Detect frame drops using hardware frame counters
    
    Args:
        frames_dict: Dictionary with stream names as keys and frames as values
        prev_frame_counters: Dictionary with previous frame counter values
        
    Returns:
        pair: (frame_drop_detected, current_frame_counters)
    """
    frame_drop_detected = False
    current_frame_counters = {}
    
    for stream_name, frame in frames_dict.items():
        if frame.supports_frame_metadata(rs.frame_metadata_value.frame_counter):
            current_counter = frame.get_frame_metadata(rs.frame_metadata_value.frame_counter)
            current_frame_counters[stream_name] = current_counter
            
            # Check for frame drops
            if prev_frame_counters[stream_name] is not None:
                expected_counter = prev_frame_counters[stream_name] + 1
                
                if current_counter == expected_counter:
                    # Normal progression - no drop
                    pass
                elif current_counter > expected_counter:
                    # Frame drop detected
                    dropped_frames = current_counter - expected_counter
                    log.w(f"Frame drop detected on {stream_name}: counter jumped from {prev_frame_counters[stream_name]} to {current_counter} ({dropped_frames} frames dropped)")
                    frame_drop_detected = True
                elif current_counter == prev_frame_counters[stream_name]:
                    # Same counter as previous - likely duplicate frame, not a rollover
                    log.d(f"Duplicate frame counter on {stream_name}: {current_counter} (same as previous)")
                else:
                    # current_counter < expected_counter - could be rollover or error
                    gap = prev_frame_counters[stream_name] - current_counter
                    if gap > 1000:  # Large gap suggests counter rollover
                        log.d(f"Frame counter rollover detected on {stream_name}: {prev_frame_counters[stream_name]} -> {current_counter}")
                    else:
                        log.w(f"Unexpected counter sequence on {stream_name}: {prev_frame_counters[stream_name]} -> {current_counter}")
        else:
            log.d(f"Frame counter metadata not available for {stream_name}")
    
    return frame_drop_detected, current_frame_counters

# Tolerance for gaps between frames
TS_TOLERANCE_MS = 1.5  # in ms
TS_TOLERANCE_MICROSEC = TS_TOLERANCE_MS * 1000  # in microseconds

# Frame drop detection using frame counter
SKIP_FRAMES_AFTER_DROP = 10  # Number of frames to skip after detecting a drop
with test.closure("Verify syncronized frames"):
    device, ctx = test.find_first_device_or_exit()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth)
    cfg.enable_stream(rs.stream.infrared, 1)
    cfg.enable_stream(rs.stream.infrared, 2)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.yuyv, 30)  # setting VGA since it fails with HD resolution

    depth_sensor = device.first_depth_sensor()
    color_sensor = device.first_color_sensor()

    for sensor in [depth_sensor, color_sensor]:  # Enable global timestamp in case it is disabled
        if sensor.supports(rs.option.global_time_enabled):
            if not sensor.get_option(rs.option.global_time_enabled):
                sensor.set_option(rs.option.global_time_enabled, 1)
        else:
            log.f(f"Sensor {sensor.name} does not support global time option")

    pipe = rs.pipeline(ctx)
    pipe.start(cfg)
    time.sleep(5)  # Longer stabilization to prevent initial frame drop issues

    # Initialize frame drop detection using frame counters
    prev_frame_counters = {'depth': None, 'ir1': None, 'ir2': None, 'color': None}
    frames_to_skip = 0
    frame_count = 0
    consecutive_drops = 0

    try:
        for _ in range(100):
            frame_count += 1
            frames = pipe.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            ir1_frame = frames.get_infrared_frame(1)
            ir2_frame = frames.get_infrared_frame(2)
            color_frame = frames.get_color_frame()

            if not (depth_frame and ir1_frame and ir2_frame and color_frame):
                log.e("One or more frames are missing")
                continue

            # Detect frame drops FIRST using frame counter metadata (before any processing)
            frames_dict = {
                'depth': depth_frame,
                'ir1': ir1_frame, 
                'ir2': ir2_frame,
                'color': color_frame
            }
            
            frame_drop_detected, current_frame_counters = detect_frame_drops(frames_dict, prev_frame_counters)
            
            # If frame drop detected, skip this frame entirely (including timestamp checks)
            if frame_drop_detected:
                consecutive_drops += 1
                if consecutive_drops > 20:  # Too many consecutive drops
                    log.f(f"Continuous frame drops detected ({consecutive_drops} consecutive). Hardware issue - unable to achieve stable synchronization.")
                
                frames_to_skip = SKIP_FRAMES_AFTER_DROP
                log.w(f"Frame drop detected at frame {frame_count}, will skip next {frames_to_skip} frames for re-sync")
                prev_frame_counters = current_frame_counters
                continue  # Skip ALL processing for this frame
            
            # Skip frames if we're in post-drop recovery period
            if frames_to_skip > 0:
                frames_to_skip -= 1
                log.d(f"Skipping frame {frame_count} (post-drop recovery, {frames_to_skip} remaining)")
                
                # Reset frame counter tracking after skip period ends
                if frames_to_skip == 0:
                    log.d("Re-sync period complete, resetting frame counter tracking")
                    prev_frame_counters = {'depth': None, 'ir1': None, 'ir2': None, 'color': None}
                
                continue
            
            # Update previous frame counters for next iteration
            prev_frame_counters = current_frame_counters
            consecutive_drops = 0  # Reset counter for successful frames

            # Global timestamps (only process if no frame drops detected)
            depth_ts = depth_frame.timestamp
            ir1_ts = ir1_frame.timestamp
            ir2_ts = ir2_frame.timestamp
            color_ts = color_frame.timestamp

            log.d(f"Depth Global TS: {depth_ts}, IR1 Global TS: {ir1_ts}, IR2 Global TS: {ir2_ts}")
            log.d(f"Color Global TS: {color_ts}")
            
            # Log frame counters for debugging
            counter_info = ", ".join([f"{name}: {counter}" for name, counter in current_frame_counters.items()])
            log.d(f"Frame Counters - {counter_info}")
            
            # Log timing differences for analysis
            depth_ir1_diff = abs(depth_ts - ir1_ts)
            depth_ir2_diff = abs(depth_ts - ir2_ts)
            depth_color_diff = abs(depth_ts - color_ts)
            log.d(f"Frame {frame_count} - Global TS differences: IR1={depth_ir1_diff:.2f}ms, IR2={depth_ir2_diff:.2f}ms, Color={depth_color_diff:.2f}ms")

            # Check global timestamps
            test.check_approx_abs(depth_ts, ir1_ts, TS_TOLERANCE_MS)
            test.check_approx_abs(depth_ts, ir2_ts, TS_TOLERANCE_MS)
            test.check_approx_abs(depth_ts, color_ts, TS_TOLERANCE_MS)

            # Frame metadata timestamps
            if all(frame.supports_frame_metadata(rs.frame_metadata_value.frame_timestamp)
                   for frame in [depth_frame, ir1_frame, ir2_frame, color_frame]):
                # if some frame does not support frame timestamp metadata, skip the check

                depth_frame_ts = depth_frame.get_frame_metadata(rs.frame_metadata_value.frame_timestamp)
                ir1_frame_ts = ir1_frame.get_frame_metadata(rs.frame_metadata_value.frame_timestamp)
                ir2_frame_ts = ir2_frame.get_frame_metadata(rs.frame_metadata_value.frame_timestamp)
                color_frame_ts = color_frame.get_frame_metadata(rs.frame_metadata_value.frame_timestamp)

                log.d(f"Depth Frame TS: {depth_frame_ts}, IR1 Frame TS: {ir1_frame_ts}, IR2 Frame TS: {ir2_frame_ts}")
                log.d(f"Color Frame TS: {color_frame_ts}")

                # Check frame timestamps
                test.check_approx_abs(depth_frame_ts, ir1_frame_ts, TS_TOLERANCE_MICROSEC)
                test.check_approx_abs(depth_frame_ts, ir2_frame_ts, TS_TOLERANCE_MICROSEC)
                test.check_approx_abs(depth_frame_ts, color_frame_ts, TS_TOLERANCE_MICROSEC)
            else:
                log.d("One or more frames do not support frame timestamp metadata")

    finally:
        pipe.stop()

test.print_results_and_exit()
