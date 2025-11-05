# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2023 RealSense, Inc. All Rights Reserved.
                                                          ##
import sys
import time
import pyrealsense2 as rs
from rspy import test, log
from test_calibrations_common import (
    calibration_main,
    is_mipi_device,
    get_calibration_device,
    get_current_rect_params,
    modify_extrinsic_calibration,
    restore_calibration_table,
    write_calibration_table_with_crc,
    measure_average_depth,
)

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
TARGET_Z_MAX = 2000
_target_z = None

# Additional constants & thresholds for advanced calibration modification test
PIXEL_CORRECTION = -1.5  # pixel shift to apply to principal point (right IR)
EPSILON = 0.001          # distance comparison tolerance for reversion checks
HEALTH_FACTOR_THRESHOLD_AFTER_MODIFICATION = 0.75  # OCC health factor acceptance for modified run


def run_advanced_tare_calibration_test(host_assistance, image_width, image_height, fps, target_z=None):
    """Run advanced tare calibration test with calibration table modifications.

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

        # 1. run tare for the first time to ensure we start from a known state
        tare_json = tare_calibration_json(None, host_assistance)
        new_calib_bytes = None
        try:
            health_factor, new_calib_bytes = calibration_main(config, pipeline, calib_dev, False, tare_json, target_z, return_table=True)
        except Exception as e:
            log.e(f"Calibration_main failed: {e}")
            restore_calibration_table(calib_dev)
            test.fail()

        if not (new_calib_bytes and health_factor is not None and abs(health_factor) < HEALTH_FACTOR_THRESHOLD_AFTER_MODIFICATION):
            log.e(f"tare calibration failed or health factor out of threshold (hf={health_factor})")
            restore_calibration_table(calib_dev)
            test.fail()
        log.i(f"tare calibration completed (health factor={health_factor:+.4f})")

        write_ok, _ = write_calibration_table_with_crc(calib_dev, new_calib_bytes)
        if not write_ok:
            log.e("Failed to write tare calibration table to device")
            restore_calibration_table(calib_dev)
            test.fail()


        # 2. Read base (reference) principal points
        principal_points_result = get_current_rect_params(calib_dev)
        if principal_points_result is None:
            log.e("Could not read current principal points")
            test.fail()
        base_left_pp, base_right_pp, base_offsets = principal_points_result
        log.i(f"  Base principal points (Right) ppx={base_right_pp[0]:.6f} ppy={base_right_pp[1]:.6f}")

        base_axis_val = base_right_pp[0]

        # (Deferred) Average depth measurement will occur AFTER modification verification
        pre_avg_depth_m = None
        pre_depth_delta_mm = None

        # 3. Apply perturbation
        log.i(f"Applying manual raw intrinsic correction: delta={PIXEL_CORRECTION:+.3f} px")
        modification_success, _modified_table_bytes, modified_ppx, modified_ppy = modify_extrinsic_calibration(
            calib_dev, PIXEL_CORRECTION, False)
        if not modification_success:
            log.e("Failed to modify calibration table")
            test.fail()

        # 4. Verify modification
        modified_principal_points_result = get_current_rect_params(calib_dev)
        if modified_principal_points_result is None:
            log.e("Could not read principal points after modification")
            test.fail()
        mod_left_pp, mod_right_pp, mod_offsets = modified_principal_points_result
        if abs(modified_ppx - mod_right_pp[0]) > EPSILON:
            log.e(f"Modification mismatch for ppx. Expected {modified_ppx:.6f} got {mod_right_pp[0]:.6f}")
            test.fail()

        # 5. Measure average depth AFTER modification (baseline for convergence) and distance to target
        try:
            pre_avg_depth_m = measure_average_depth(width=image_width, height=image_height, fps=fps)
            if pre_avg_depth_m is not None:
                log.i(f"  Average depth after modification (pre-calibration): {pre_avg_depth_m*1000:.1f} mm")
                if target_z is not None:
                    pre_depth_delta_mm = abs(pre_avg_depth_m * 1000.0 - target_z)
                    log.i(f"  Distance to target pre-calibration: {pre_depth_delta_mm:.1f} mm (target={target_z} mm)")
            else:
                log.w("  Average depth pre-calibration unavailable")
        except Exception as e:
            log.w(f"Average depth measurement pre-calibration failed: {e}")

        # 6. Run tare again
        tare_json = tare_calibration_json(None, host_assistance)
        new_calib_bytes = None
        try:
            health_factor, new_calib_bytes = calibration_main(config, pipeline, calib_dev, False, tare_json, target_z, return_table=True)
        except Exception as e:
            log.e(f"Calibration_main failed: {e}")
            health_factor = None

        if not (new_calib_bytes and health_factor is not None and abs(health_factor) < HEALTH_FACTOR_THRESHOLD_AFTER_MODIFICATION):
            log.e(f"tare calibration failed or health factor out of threshold (hf={health_factor})")
            test.fail()
        log.i(f"tare calibration completed (health factor={health_factor:+.4f})")

        # 7. Write updated table & evaluate
        write_ok, _ = write_calibration_table_with_crc(calib_dev, new_calib_bytes)
        if not write_ok:
            log.e("Failed to write tare calibration table to device")
            test.fail()

        final_principal_points_result = get_current_rect_params(calib_dev)
        if final_principal_points_result is None:
            log.e("Could not read final principal points")
            test.fail()
        fin_left_pp, fin_right_pp, fin_offsets = final_principal_points_result
        final_axis_val = fin_right_pp[0]
        log.i(f"  Final principal points (Right) ppx={fin_right_pp[0]:.6f} ppy={fin_right_pp[1]:.6f}")

        # Measure average depth after calibration and compute new distance to target
        post_avg_depth_m = None
        post_depth_delta_mm = None
        try:
            post_avg_depth_m = measure_average_depth(width=image_width, height=image_height, fps=fps)
            if post_avg_depth_m is not None:
                log.i(f"  Average depth after calibration: {post_avg_depth_m*1000:.1f} mm")
                if target_z is not None:
                    post_depth_delta_mm = abs(post_avg_depth_m * 1000.0 - target_z)
                    log.i(f"  Distance to target after calibration: {post_depth_delta_mm:.1f} mm (target={target_z} mm)")
            else:
                log.w("  Average depth after calibration unavailable")
        except Exception as e:
            log.w(f"Average depth measurement after calibration failed: {e}")

        # Reversion checks:
        # 1. Final must differ from modified (change happened)
        # 2. Final must be closer to base than to modified (strict revert expectation)
        dist_from_original = abs(final_axis_val - base_axis_val)
        dist_from_modified = abs(final_axis_val - modified_ppx)
        log.i(f"  ppx distances: from_base={dist_from_original:.6f} from_modified={dist_from_modified:.6f}")

        if abs(final_axis_val - modified_ppx) <= EPSILON:
            log.e(f"Tare left ppx unchanged (within EPSILON={EPSILON}); failing")
            test.fail()
        elif dist_from_modified + EPSILON <= dist_from_original:
            log.e("Tare did not revert toward base (still closer to modified)")
            test.fail()
        else:
            log.i("Tare reverted ppx toward base successfully")

        # Average depth convergence assertion: ensure we got closer (or equal within 1mm tolerance) to target
        if target_z is not None and pre_depth_delta_mm is not None and post_depth_delta_mm is not None:
            if post_depth_delta_mm > pre_depth_delta_mm + 1.0:  # allow small tolerance
                log.e(f"Average depth diverged from target: before={pre_depth_delta_mm:.1f}mm after={post_depth_delta_mm:.1f}mm target={target_z}mm")
                test.fail()
            else:
                improvement = pre_depth_delta_mm - post_depth_delta_mm
                log.i(f"Average depth moved {'closer' if improvement >= 0 else 'not closer'} to target (Δ improvement={improvement:.1f} mm)")
    finally:
        # Always stop pipeline before returning device so subsequent tests can reset factory calibration
        try:
            pipeline.stop()
        except Exception:
            pass
    return calib_dev

with test.closure("Advanced tare calibration test with calibration table modifications"):
    calib_dev = None
    try:
        host_assistance = False
        if (_target_z is None):
            _target_z = calculate_target_z()
            test.check(_target_z > TARGET_Z_MIN and _target_z < TARGET_Z_MAX)
        image_width, image_height, fps = (256, 144, 90)
        calib_dev = run_advanced_tare_calibration_test(host_assistance, image_width, image_height, fps, _target_z)
        restore_calibration_table(calib_dev)
    except Exception as e:
        log.e("Tare calibration with principal point modification failed: ", str(e))
        if calib_dev is not None:
            restore_calibration_table(calib_dev)
        test.fail()

with test.closure("Advanced tare calibration test with host assistance"):
    calib_dev = None
    try:
        host_assistance = True
        if (_target_z is None):
            _target_z = calculate_target_z()
            test.check(_target_z > TARGET_Z_MIN and _target_z < TARGET_Z_MAX)
        image_width, image_height, fps = (1280, 720, 30)
        calib_dev = run_advanced_tare_calibration_test(host_assistance, image_width, image_height, fps, _target_z)
        restore_calibration_table(calib_dev)
    except Exception as e:
        log.e("Tare calibration with principal point modification failed: ", str(e))
        if calib_dev is not None:
            restore_calibration_table(calib_dev)
        test.fail()

test.print_results_and_exit()

# for step 2 -  not in use for now
"""
test.print_results_and_exit()
change exposuere 8500 (tried with various exposure values)/ host assisatnece true
"""