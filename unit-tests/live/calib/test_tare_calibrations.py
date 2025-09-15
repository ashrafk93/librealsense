# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2023 RealSense, Inc. All Rights Reserved.
                                                          ##
import sys
import time
import pyrealsense2 as rs
from rspy import test, log
from test_calibrations_common import calibration_main, is_mipi_device

#disabled until we stabilize lab
#test:donotrun

def tare_calibration_json(tare_json_file, host_assistance):
    tare_json = None
    if tare_json_file is not None:
        try:
            tare_json = open(tare_json_file).read()
        except:
            tare_json = None
            log.e('Error reading tare_json_file: ', tare_json_file)
    if tare_json is None:
        log.i('Using default parameters for Tare calibration.')
        tare_json = '{\n  '+\
                    '"host assistance": ' + str(int(host_assistance)) + ',\n'+\
                    '"speed": 3,\n'+\
                    '"scan parameter": 0,\n'+\
                    '"step count": 20,\n'+\
                    '"apply preset": 1,\n'+\
                    '"accuracy": 2,\n'+\
                    '"depth": 0,\n'+\
                    '"resize factor": 1\n'+\
                    '}'
    return tare_json


def calculate_target_z():
    number_of_images = 50  # The required number of frames is 10+
    timeout_s = 30
    target_size = [175, 100]

    cfg = rs.config()
    cfg.enable_stream(rs.stream.infrared, 1, 1280, 720, rs.format.y8, 30)

    q = rs.frame_queue(capacity=number_of_images, keep_frames=True)
    q2 = rs.frame_queue(capacity=number_of_images, keep_frames=True)
    q3 = rs.frame_queue(capacity=number_of_images, keep_frames=True)

    counter = 0

    def cb(frame):
        nonlocal counter
        if counter > number_of_images:
            return
        for f in frame.as_frameset():
            q.enqueue(f)
            counter += 1

    ctx = rs.context()
    pipe = rs.pipeline(ctx)
    pp = pipe.start(cfg, cb)
    dev = pp.get_device()

    try:
        stime = time.time()
        while counter < number_of_images:
            time.sleep(0.5)
            if timeout_s < (time.time() - stime):
                raise RuntimeError(f"Failed to capture {number_of_images} frames in {timeout_s} seconds, got only {counter} frames")

        adev = dev.as_auto_calibrated_device()
        log.i('Calculating distance to target...')
        log.i(f'\tTarget Size:\t{target_size}')
        target_z = adev.calculate_target_z(q, q2, q3, target_size[0], target_size[1])
        log.i(f'Calculated distance to target is {target_z}')
    finally:
        pipe.stop()

    return target_z


# Constants for validation
HEALTH_FACTOR_THRESHOLD = 0.25
TARGET_Z_MIN = 600
TARGET_Z_MAX = 1500
_target_z = None

def run_advanced_calibration_test(host_assistance, image_width, image_height, fps, modify_ppy=True):
    """Run advanced OCC calibration test with calibration table modifications.

    Args:
        host_assistance (bool)
        image_width (int)
        image_height (int)
        fps (int)
        modify_ppy (bool): True to modify ppy, False to modify ppx
    Returns:
        auto_calibrated_device (rs.auto_calibrated_device)
    """
    config, pipeline, calib_dev = get_calibration_device(image_width, image_height, fps)

    # Ensure we start from factory calibration (clean baseline) before applying any modifications
    log.i("Restoring factory calibration before test scenario...")
    if not restore_calibration_table(calib_dev):
        log.e("Failed to restore factory calibration")
        test.fail()

    principal_points_result = get_current_rect_params(calib_dev)
    if principal_points_result is not None:
        orig_left_pp, orig_right_pp, orig_offsets = principal_points_result
        log.i(f"  Current principal points (pixel coordinates) - Right: ppx={orig_right_pp[0]:.6f}, ppy={orig_right_pp[1]:.6f}")
    else:
        log.e("Could not read current principal points")
        test.fail()

    # Apply manual raw intrinsic correction
    target_label = 'ppy' if modify_ppy else 'ppx'
    log.i(f"Applying manual raw intrinsic {target_label} correction...")
    modification_success, _modified_table_bytes, modified_ppx, modified_ppy = modify_extrinsic_calibration(
        calib_dev, PIXEL_CORRECTION, modify_ppy=modify_ppy)
    if not modification_success:
        log.e("Failed to modify calibration table")
        test.fail()

    modified_principal_points_result = get_current_rect_params(calib_dev)
    # Verify the modification was applied correctly
    if modified_principal_points_result is not None:
        modified_left_pp, modified_right_pp, modified_offsets = modified_principal_points_result
        log.i(f"  Modified principal points (pixel coordinates) - Right: ppx={modified_right_pp[0]:.6f}, ppy={modified_right_pp[1]:.6f}")

        # Determine expected modified value and original for selected axis
        original_axis_val = orig_right_pp[1] if modify_ppy else orig_right_pp[0]
        reported_modified_axis_val = modified_right_pp[1] if modify_ppy else modified_right_pp[0]
        returned_modified_axis_val = modified_ppy if modify_ppy else modified_ppx
        if abs(reported_modified_axis_val - returned_modified_axis_val) > EPSILON:
            log.e(f"Calibration modification not applied correctly. Expected {target_label}={returned_modified_axis_val:.6f}, got {reported_modified_axis_val:.6f}")
            test.fail()
        else:
            log.i(f"Calibration modification applied correctly. {target_label} changed by {reported_modified_axis_val - original_axis_val:.6f} pixels")
    else:
        log.e("Could not read current principal points after modification")
        test.fail()

    # Run OCC calibration via calibration_main (captures table)
    occ_json = on_chip_calibration_json(None, host_assistance)
    new_calib_bytes = None  # ensure defined even if calibration_main raises
    try:
        health_factor, new_calib_bytes = calibration_main(config, pipeline, calib_dev, True, occ_json, None, return_table=True)
    except Exception as e:
        log.e(f"Calibration_main failed: {e}")
        health_factor = None

    if new_calib_bytes and health_factor is not None and abs(health_factor) < HEALTH_FACTOR_THRESHOLD_AFTER_MODIFICATION:
        log.i("OCC calibration completed (health factor within threshold)")
        write_ok, _ = write_calibration_table_with_crc(calib_dev, new_calib_bytes)
        if not write_ok:
            log.e("Failed to write OCC calibration table to device")
            test.fail()
        # Analyze what corrections OCC made after OCC
        final_principal_points_result = get_current_rect_params(calib_dev)
        if final_principal_points_result is None:
            log.e("Could not read final principal points")
            test.fail()
        else:
            final_left_pp, final_right_pp, final_offsets = final_principal_points_result
            log.i(f"  Final principal points (pixel coordinates) - Right: ppx={final_right_pp[0]:.6f}, ppy={final_right_pp[1]:.6f}")
            # Select axis for comparison
            final_axis_val = final_right_pp[1] if modify_ppy else final_right_pp[0]
            original_axis_val = orig_right_pp[1] if modify_ppy else orig_right_pp[0]
            modified_axis_val = modified_right_pp[1] if modify_ppy else modified_right_pp[0]
            distance_from_original = abs(final_axis_val - original_axis_val)
            distance_from_modified = abs(final_axis_val - modified_axis_val)
            log.i(f"  (Right {target_label}) Distance from original: {distance_from_original:.6f} Distance from modified: {distance_from_modified:.6f}")
            # Success criteria (current expectation): OCC should revert toward original (fail if it stays near modified)
            if distance_from_modified + EPSILON <= distance_from_original or abs(distance_from_modified) == 0:
                log.e(f"OCC preserved the manual {target_label} correction (unexpected per test expectation)")
                test.fail()
            else:
                log.i(f"OCC reverted {target_label} closer to original calibration as expected")
    else:
        log.e("OCC calibration failed or health factor out of threshold")
        test.fail()
return calib_dev


with test.closure("Tare calibration test with host assistance"):
    try:
        host_assistance = True
        if (_target_z is None):
            _target_z = calculate_target_z()
            test.check(_target_z > TARGET_Z_MIN and _target_z < TARGET_Z_MAX)
        
        tare_json = tare_calibration_json(None, host_assistance)
        health_factor = calibration_main(256, 144, 90, False, tare_json, _target_z)

        test.check(abs(health_factor) < HEALTH_FACTOR_THRESHOLD)
    except Exception as e:
        log.e("Tare calibration test with host assistance failed: ", str(e))
        test.fail()

with test.closure("Tare calibration test"):
    try:
        host_assistance = False
        if is_mipi_device():
            log.i("MIPI device - skip the test w/o host assistance")
            test.skip()

        if _target_z is None:
            _target_z = calculate_target_z()
            test.check(_target_z > TARGET_Z_MIN and _target_z < TARGET_Z_MAX)

        tare_json = tare_calibration_json(None, host_assistance)
        health_factor = calibration_main(1280, 720, 30, False, tare_json, _target_z)
        
        test.check(abs(health_factor) < HEALTH_FACTOR_THRESHOLD)
    except Exception as e:
        log.e("Tare calibration test failed: ", str(e))
        test.fail()

# for step 2 -  not in use for now
"""
# This test performs Tare calibration with calibration table backup and modification.
# It demonstrates backing up the calibration table, running the calibration, and restoring the table if needed.
# The test checks that the health factor after calibration is within the allowed threshold.
with test.closure("Tare calibration with table backup and modification"):
    try:
        host_assistance = False
        target_z = calculate_target_z()
        test.check(target_z > TARGET_Z_MIN and target_z < TARGET_Z_MAX)
        tare_json = tare_calibration_json(None, host_assistance)
        
        log.i("Starting Tare calibration with calibration table backup/restore demonstration")
        health_factor = perform_calibration_with_table_backup(host_assistance, False, tare_json, target_z)
        
        if health_factor is not None:
            test.check(abs(health_factor) < HEALTH_FACTOR_THRESHOLD)
            log.i("Tare calibration with table manipulation completed successfully")
        else:
            log.e("Tare calibration with table backup failed")
            test.fail()
            
    except Exception as e:
        log.e("Tare calibration with table backup failed: ", str(e))
        test.fail()
    log.i("Done\n")

test.print_results_and_exit()
change exposuere 8500 (tried with various exposure values)/ host assisatnece true
"""