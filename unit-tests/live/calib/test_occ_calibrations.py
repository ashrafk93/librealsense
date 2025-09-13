# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2023 RealSense, Inc. All Rights Reserved.

import time
import struct
import pyrealsense2 as rs
from rspy import test, log
from test_calibrations_common import (calibration_main, get_calibration_device, get_current_rect_params, 
                                     is_mipi_device, modify_calibration_table, restore_calibration_table,
                                     get_d400_calibration_table, write_calibration_table_with_crc)

# Pixel correction constant for DSCalibrationModifier-style corrections
PIXEL_CORRECTION = -1
EPSILON = 0.05  # tolerance in pixels when comparing distances

#disabled until we stabilize lab
#test:donotrun:disabled

def on_chip_calibration_json(occ_json_file, host_assistance):
    occ_json = None
    if occ_json_file is not None:
        try:
            occ_json = open(occ_json_file).read()
        except:
            occ_json = None
            log.e('Error reading occ_json_file: ', occ_json_file)
        
    if occ_json is None:
        log.i('Using default parameters for on-chip calibration.')
        occ_json = '{\n  '+\
                    '"calib type": 0,\n'+\
                    '"host assistance": ' + str(int(host_assistance)) + ',\n'+\
                    '"keep new value after successful scan": 1,\n'+\
                    '"fl data sampling": 0,\n'+\
                    '"adjust both sides": 0,\n'+\
                    '"fl scan location": 0,\n'+\
                    '"fy scan direction": 0,\n'+\
                    '"white wall mode": 0,\n'+\
                    '"speed": 2,\n'+\
                    '"scan parameter": 0,\n'+\
                    '"apply preset": 0,\n'+\
                    '"scan only": ' + str(int(host_assistance)) + ',\n'+\
                    '"interactive scan": 0,\n'+\
                    '"resize factor": 1\n'+\
                    '}'
    # TODO - host assistance actual value may be different when reading from json
    return occ_json


def modify_extrinsic_calibration(device):
    """Apply a small ppy correction ONLY to the raw right intrinsic principal point (no rectified edits)."""
    try:
        calib_table = get_d400_calibration_table(device)
        if not calib_table:
            return False, "Failed to get calibration table"

        modified_data = bytearray(calib_table)
        header_size = 16
        right_intrinsics_offset = header_size + 36  # skip left 9 floats
        right_intrinsics = list(struct.unpack('<9f', modified_data[right_intrinsics_offset:right_intrinsics_offset+36]))
        height = 800.0
        original_raw_ppy = right_intrinsics[3] * height
        corrected_raw_ppy = original_raw_ppy + PIXEL_CORRECTION
        right_intrinsics[3] = corrected_raw_ppy / height
        modified_data[right_intrinsics_offset:right_intrinsics_offset+36] = struct.pack('<9f', *right_intrinsics)
        log.i(f"  Raw Right ppy original={original_raw_ppy:.6f} modified={corrected_raw_ppy:.6f}")

        return write_calibration_table_with_crc(device, bytes(modified_data))
    except Exception as e:
        log.e(f"Error modifying calibration: {e}")
        return False, str(e)


# Health factor threshold for calibration success
# Note: Higher threshold for tests with intentional calibration modifications
HEALTH_FACTOR_THRESHOLD = 0.25
HEALTH_FACTOR_THRESHOLD_AFTER_MODIFICATION = 0.8

with test.closure("OCC calibration test"):
    try:        
        host_assistance = False
        image_width, image_height, fps = (256, 144, 90)
        config, pipeline, calib_dev = get_calibration_device(image_width, image_height, fps)

        if is_mipi_device():
            log.i("MIPI device - skip the test w/o host assistance")
            test.skip()

        occ_json = on_chip_calibration_json(None, host_assistance)
        time.sleep(1.0)
        health_factor = calibration_main(config, pipeline, calib_dev, True, occ_json, None)
        test.check(abs(health_factor) < HEALTH_FACTOR_THRESHOLD)
    except Exception as e:
        log.e("OCC calibration test failed: ", str(e))
        test.fail()
"""
with test.closure("OCC calibration test with host assistance"):
    try:
        host_assistance = True
        image_width, image_height, fps = (1280, 800, 30)
        config, pipeline, calib_dev = get_calibration_device(image_width, image_height, fps)

        occ_json = on_chip_calibration_json(None, host_assistance)
        time.sleep(1.0)
        health_factor = calibration_main(config, pipeline, calib_dev, True, occ_json, None)
        test.check(abs(health_factor) < HEALTH_FACTOR_THRESHOLD)     
    except Exception as e:
        log.e("OCC calibration test with host assistance failed: ", str(e))
        test.fail()
"""
with test.closure("Advanced OCC calibration test with calibration table modifications"):
    calib_dev = None
    modified_applied = False
    try:
        host_assistance = False
        image_width, image_height, fps = (256, 144, 90)
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
        # Apply manual raw intrinsic ppy correction
        modification_success, _modified_table_bytes = modify_extrinsic_calibration(calib_dev)
        if not modification_success:
            log.e("Failed to modify calibration table")
            test.fail()
        modified_applied = True
        modified_principal_points_result = get_current_rect_params(calib_dev)
        if modified_principal_points_result is not None:
            modified_left_pp, modified_right_pp, modified_offsets = modified_principal_points_result
            log.i(f"  Modified principal points (pixel coordinates) - Right: ppx={modified_right_pp[0]:.6f}, ppy={modified_right_pp[1]:.6f}")
        else:
            log.e("Could not read current principal points after modification")
            test.fail()
        # Run OCC calibration via calibration_main (captures table)
        occ_json = on_chip_calibration_json(None, host_assistance)
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
            final_left_pp, final_right_pp, final_offsets = final_principal_points_result
            log.i(f"  Final principal points (pixel coordinates) - Right: ppx={final_right_pp[0]:.6f}, ppy={final_right_pp[1]:.6f}")
            distance_from_original = abs(final_right_pp[1] - orig_right_pp[1])
            distance_from_modified = abs(final_right_pp[1] - modified_right_pp[1])
            log.i(f"  (Right) Distance from original: {distance_from_original:.6f} (Right) Distance from modified: {distance_from_modified:.6f}")
            # Success criteria (current expectation): OCC should revert toward original (fail if it stays near modified)
            if distance_from_modified + EPSILON <= distance_from_original:
                log.e("OCC preserved the manual correction (unexpected per test expectation)")
                test.fail()
            else:
                log.i("OCC reverted closer to original calibration as expected")
        else:
            log.e("OCC calibration failed or health factor out of threshold")
            test.fail()
    except Exception as e:
        log.e("OCC calibration with principal point modification failed: ", str(e))
        test.fail()
    finally:
        if calib_dev and modified_applied:
            restore_calibration_table(calib_dev)
        log.i("Done\n")

test.print_results_and_exit()


