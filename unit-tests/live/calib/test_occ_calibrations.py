# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2023 RealSense, Inc. All Rights Reserved.

import time
import struct
import pyrealsense2 as rs
from rspy import test, log
from test_calibrations_common import (
    calibration_main,
    get_calibration_device,
    get_current_rect_params,
    modify_extrinsic_calibration,
    restore_calibration_table,
    write_calibration_table_with_crc,
)

# Constants & thresholds (reintroduce after import fix)
PIXEL_CORRECTION = -0.8  # pixel shift to apply to principal point
EPSILON = 0.001         # distance comparison tolerance
HEALTH_FACTOR_THRESHOLD_AFTER_MODIFICATION = 0.8

def on_chip_calibration_json(occ_json_file, host_assistance):
    occ_json = None
    if occ_json_file is not None:
        try:
            occ_json = open(occ_json_file).read()
        except Exception:
            occ_json = None
            log.e('Error reading occ_json_file: ', occ_json_file)
    if occ_json is None:
        log.i('Using default parameters for on-chip calibration.')
        occ_json = '{\n  ' + \
                   '"calib type": 0,\n' + \
                   '"host assistance": ' + str(int(host_assistance)) + ',\n' + \
                   '"keep new value after successful scan": 1,\n' + \
                   '"fl data sampling": 0,\n' + \
                   '"adjust both sides": 0,\n' + \
                   '"fl scan location": 0,\n' + \
                   '"fy scan direction": 0,\n' + \
                   '"white wall mode": 0,\n' + \
                   '"speed": 2,\n' + \
                   '"scan parameter": 0,\n' + \
                   '"apply preset": 0,\n' + \
                   '"scan only": ' + str(int(host_assistance)) + ',\n' + \
                   '"interactive scan": 0,\n' + \
                   '"resize factor": 1\n' + \
                   '}'
    return occ_json

#disabled until we stabilize lab
#test:donotrun:disabled

def run_advanced_occ_calibration_test(host_assistance, image_width, image_height, fps, modify_ppy=True):
    """Run advanced OCC calibration test with calibration table modifications.

        Flow:
            1. OCC (baseline) or Restore factory calibration
            2. Read & log base principal point
            3. Apply controlled raw intrinsic perturbation (ppy or ppx)
            4. Verify perturbation was applied
            5. Run OCC once and capture returned table
            6. Write table, read final principal point, compute correction metrics
    """
    config, pipeline, calib_dev = get_calibration_device(image_width, image_height, fps)
    try:

        # 1. run OCC for the first time to ensure we start from a known state
        occ_json = on_chip_calibration_json(None, host_assistance)
        new_calib_bytes = None
        try:
            health_factor, new_calib_bytes = calibration_main(config, pipeline, calib_dev, True, occ_json, None, return_table=True)
        except Exception as e:
            log.e(f"Calibration_main failed: {e}")
            restore_calibration_table(calib_dev)
            test.fail()

        if not (new_calib_bytes and health_factor is not None and abs(health_factor) < HEALTH_FACTOR_THRESHOLD_AFTER_MODIFICATION):
            log.e(f"OCC calibration failed or health factor out of threshold (hf={health_factor})")
            restore_calibration_table(calib_dev)
            test.fail()
        log.i(f"OCC calibration completed (health factor={health_factor:+.4f})")

        write_ok, _ = write_calibration_table_with_crc(calib_dev, new_calib_bytes)
        if not write_ok:
            log.e("Failed to write OCC calibration table to device")
            restore_calibration_table(calib_dev)
            test.fail()


        # 2. Read base (reference) principal points
        principal_points_result = get_current_rect_params(calib_dev)
        if principal_points_result is None:
            log.e("Could not read current principal points")
            test.fail()
        base_left_pp, base_right_pp, base_offsets = principal_points_result
        log.i(f"  Base principal points (Right) ppx={base_right_pp[0]:.6f} ppy={base_right_pp[1]:.6f}")

        base_axis_val = base_right_pp[1]

        # 3. Apply perturbation
        log.i(f"Applying manual raw intrinsic correction: delta={PIXEL_CORRECTION:+.3f} px")
        modification_success, _modified_table_bytes, modified_ppx, modified_ppy = modify_extrinsic_calibration(
            calib_dev, PIXEL_CORRECTION, modify_ppy=modify_ppy)
        if not modification_success:
            log.e("Failed to modify calibration table")
            test.fail()

        # 4. Verify modification
        modified_principal_points_result = get_current_rect_params(calib_dev)
        if modified_principal_points_result is None:
            log.e("Could not read principal points after modification")
            test.fail()
        mod_left_pp, mod_right_pp, mod_offsets = modified_principal_points_result
        modified_axis_val = mod_right_pp[1]
        returned_modified_axis_val = modified_ppy if modify_ppy else modified_ppx
        if abs(modified_axis_val - returned_modified_axis_val) > EPSILON:
            log.e(f"Modification mismatch for ppy. Expected {returned_modified_axis_val:.6f} got {modified_axis_val:.6f}")
            test.fail()
        applied_delta = modified_axis_val - base_axis_val

        # 5. Run OCC again
        occ_json = on_chip_calibration_json(None, host_assistance)
        new_calib_bytes = None
        try:
            health_factor, new_calib_bytes = calibration_main(config, pipeline, calib_dev, True, occ_json, None, return_table=True)
        except Exception as e:
            log.e(f"Calibration_main failed: {e}")
            health_factor = None

        if not (new_calib_bytes and health_factor is not None and abs(health_factor) < HEALTH_FACTOR_THRESHOLD_AFTER_MODIFICATION):
            log.e(f"OCC calibration failed or health factor out of threshold (hf={health_factor})")
            test.fail()
        log.i(f"OCC calibration completed (health factor={health_factor:+.4f})")

        # 6. Write updated table & evaluate
        write_ok, _ = write_calibration_table_with_crc(calib_dev, new_calib_bytes)
        if not write_ok:
            log.e("Failed to write OCC calibration table to device")
            test.fail()

        final_principal_points_result = get_current_rect_params(calib_dev)
        if final_principal_points_result is None:
            log.e("Could not read final principal points")
            test.fail()
        fin_left_pp, fin_right_pp, fin_offsets = final_principal_points_result
        final_axis_val = fin_right_pp[1]
        log.i(f"  Final principal points (Right) ppx={fin_right_pp[0]:.6f} ppy={fin_right_pp[1]:.6f}")

        # Reversion checks:
        # 1. Final must differ from modified (change happened)
        # 2. Final must be closer to base than to modified (strict revert expectation)
        dist_from_original = abs(final_axis_val - base_axis_val)
        dist_from_modified = abs(final_axis_val - modified_axis_val)
        log.i(f"  ppy distances: from_base={dist_from_original:.6f} from_modified={dist_from_modified:.6f}")

        if abs(final_axis_val - modified_axis_val) <= EPSILON:
            log.e(f"OCC left ppy unchanged (within EPSILON={EPSILON}); failing")
            test.fail()
        elif dist_from_modified + EPSILON <= dist_from_original:
            log.e("OCC did not revert toward base (still closer to modified)")
            test.fail()
        else:
            log.i("OCC reverted ppy toward base successfully")
    finally:
        # Always stop pipeline before returning device so subsequent tests can reset factory calibration
        try:
            pipeline.stop()
        except Exception:
            pass
    return calib_dev

with test.closure("Advanced OCC calibration test with calibration table modifications"):
    calib_dev = None
    try:
        host_assistance = False
        image_width, image_height, fps = (256, 144, 90)
        calib_dev = run_advanced_occ_calibration_test(host_assistance, image_width, image_height, fps, modify_ppy=True)
    except Exception as e:
        log.e("OCC calibration with principal point modification failed: ", str(e))
        restore_calibration_table(calib_dev)
        test.fail()

with test.closure("Advanced OCC calibration test with host assistance"):
    calib_dev = None
    try:
        host_assistance = True
        image_width, image_height, fps = (1280, 720, 30)
        calib_dev = run_advanced_occ_calibration_test(host_assistance, image_width, image_height, fps, modify_ppy=True)
    except Exception as e:
        log.e("OCC calibration with principal point modification failed: ", str(e))
        restore_calibration_table(calib_dev)
        test.fail()        
            

test.print_results_and_exit()

"""
OCC in Host Assistance mode is allowing to run on any resolution selected by the user.

For example see the attached video - running in 848x100 res.
"""