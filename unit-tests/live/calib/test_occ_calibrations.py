# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2023 RealSense, Inc. All Rights Reserved.

import sys
import time
import struct
import zlib
import pyrealsense2 as rs
from rspy import test, log
from test_calibrations_common import (calibration_main, get_calibration_device, get_current_rect_params, 
                                     is_mipi_device, modify_calibration_table, restore_calibration_table,
                                     get_d400_calibration_table, analyze_calibration_table, 
                                     write_calibration_table_with_crc)

# Pixel correction constant for DSCalibrationModifier-style corrections
PIXEL_CORRECTION = -1

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


def analyze_occ_corrections(device, original_calib):
    """Analyze what OCC corrected after calibration"""
    try:
        # Get final calibration
        final_calib = get_d400_calibration_table(device)
        if not final_calib:
            return
        
        # Parse original and final rect_params for 1280x800 (index 8)
        rect_params_offset = 256 + (8 * 16)  # base + (index * 16 bytes)
        
        orig_rect_params = struct.unpack('<4f', original_calib[rect_params_offset:rect_params_offset+16])
        final_rect_params = struct.unpack('<4f', final_calib[rect_params_offset:rect_params_offset+16])
        
        # Get final pixel coordinates from our extraction function
        final_left_ppx, final_left_ppy = get_current_rect_params(device)[0]
        log.i(f"  Current principal points (pixel coordinates) after OCC - Left: ppx={final_left_ppx:.6f}, ppy={final_left_ppy:.6f}")
        
        # Check if OCC preserved our correction
        distance_from_original = abs(final_rect_params[3] - orig_rect_params[3])
        distance_from_modified = abs(final_rect_params[3] - (orig_rect_params[3] + PIXEL_CORRECTION))
        
        print("-I-     Distance from original: {:.6f}".format(distance_from_original))
        print("-I-     Distance from modified: {:.6f}".format(distance_from_modified))
        
        if distance_from_modified > distance_from_original:
            print("-I- ✓ OCC preserved ppy corrections")
            return True
        else:
            print("-I- ⚠ OCC didnt correct ppy")
            return False
            
    except Exception as e:
        print(f"-E- Error analyzing corrections: {e}")
        return False


# Health factor threshold for calibration success
# Note: Higher threshold for tests with intentional calibration modifications
HEALTH_FACTOR_THRESHOLD = 0.8

def _wait_for_device(serial, timeout_s=25.0, poll_interval=0.5):
            """Wait for device with given serial to re-enumerate; return rs.device or None"""
            ctx = rs.context()
            start = time.time()
            while time.time() - start < timeout_s:
                dev_list = ctx.query_devices()
                for d in dev_list:
                    try:
                        if serial is None:
                            # If serial unknown, just return first device
                            return d
                        if d.supports(rs.camera_info.serial_number) and \
                           d.get_info(rs.camera_info.serial_number) == serial:
                            return d
                    except Exception:
                        continue
                time.sleep(poll_interval)
            return None

def _reacquire_auto_calib_device(serial):
    base_dev = _wait_for_device(serial)
    if not base_dev:
        return None
    try:
        new_acd = rs.auto_calibrated_device(base_dev)
        return new_acd
    except Exception as e:
        log.e(f"Failed to create auto_calibrated_device after reset: {e}")
        return None
"""
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
with test.closure("OCC calibration with manual raw right ppy modification"):
    try:
        host_assistance = False
        image_width, image_height, fps = (256, 144, 90)
        config, pipeline, calib_dev = get_calibration_device(image_width, image_height, fps)
        # Ensure we start from factory calibration (clean baseline) before applying any modifications
        log.i("Restoring factory calibration before test scenario...")
        if not restore_calibration_table(calib_dev):
            log.e("Failed to restore factory calibration")
            test.fail()
        else:
            # Give device a moment after restore (factory reset may trigger internal operations)
            time.sleep(2.0)

        # Capture device serial for later re-enumeration after hardware reset
        try:
            device_serial = calib_dev.get_info(rs.camera_info.serial_number)
            log.i(f"Using device serial: {device_serial}")
        except Exception as e:
            log.w(f"Could not read device serial before reset: {e}")
            device_serial = None        
                
        principal_points_result = get_current_rect_params(calib_dev)
        if principal_points_result is not None:
            orig_left_pp, orig_right_pp, orig_offsets = principal_points_result
            log.i(f"  Current principal points (pixel coordinates) - Right: ppx={orig_right_pp[0]:.6f}, ppy={orig_right_pp[1]:.6f}")           
        else:
            log.w("Could not read current principal points")       

    # Apply manual raw intrinsic ppy correction
        modification_success, original_values = modify_extrinsic_calibration(calib_dev)
        if not modification_success:
            log.e("Failed to modify calibration table")
            test.fail()
        else:
            modified_principal_points_result = get_current_rect_params(calib_dev)
            if modified_principal_points_result is not None:
                modified_left_pp, modified_right_pp, modified_offsets = modified_principal_points_result
                log.i(f"  Modified principal points (pixel coordinates) - Right: ppx={modified_right_pp[0]:.6f}, ppy={modified_right_pp[1]:.6f}")
            else:
                log.w("Could not read current principal points after modification")

            # Run OCC calibration with instrumentation to capture returned table
            occ_json = on_chip_calibration_json(None, host_assistance)
            log.i("Starting OCC calibration (raw-only ppy modification test)")

            # Start streaming similar to calibration_main but inline to capture new table
            conf = pipeline.start(config)
            pipeline.wait_for_frames()
            depth_sensor = conf.get_device().first_depth_sensor()
            if depth_sensor.supports(rs.option.emitter_enabled):
                depth_sensor.set_option(rs.option.emitter_enabled, 1)
            if depth_sensor.supports(rs.option.thermal_compensation):
                depth_sensor.set_option(rs.option.thermal_compensation, 0)

            new_calib_bytes = b''
            try:
                log.i("Starting on-chip calibration (instrumented)")
                new_calib_bytes, health_tuple = calib_dev.run_on_chip_calibration(occ_json, lambda p: None, 9000)
                calib_done = len(new_calib_bytes) > 0
                timeout_end = time.time() + 30
                while not calib_done:
                    if time.time() > timeout_end:
                        raise RuntimeError("Calibration timed out")
                    frame_set = pipeline.wait_for_frames()
                    depth_frame = frame_set.get_depth_frame()
                    new_calib_bytes, health_tuple = calib_dev.process_calibration_frame(depth_frame, lambda p: None, 5000)
                    calib_done = len(new_calib_bytes) > 0
                log.i("Calibration completed successfully (instrumented)")
                health_factor = health_tuple[0]
            except Exception as e:
                log.e(f"Instrumented OCC failed: {e}")
                health_factor = None
            finally:
                pipeline.stop()

            if new_calib_bytes:
                if isinstance(new_calib_bytes, list):
                    new_calib_bytes = bytes(new_calib_bytes)
                def crc_of(data):
                    return zlib.crc32(data[16:]) & 0xffffffff if data and len(data) > 16 else 0
                pre_occ_table = get_d400_calibration_table(calib_dev)
                pre_occ_crc = crc_of(pre_occ_table)
                occ_crc = crc_of(new_calib_bytes)
                log.i(f"  CRC pre-OCC(modified) = 0x{pre_occ_crc:08x}; OCC returned table CRC = 0x{occ_crc:08x}")
                try:
                    header_size = 16
                    right_off = header_size + 36
                    rt_intr = struct.unpack('<9f', new_calib_bytes[right_off:right_off+36])
                    occ_ppy = rt_intr[3] * 800.0
                    log.i(f"  OCC returned raw right ppy = {occ_ppy:.6f}")
                except Exception as e:
                    log.w(f"  Could not parse OCC returned table: {e}")
                if occ_crc == pre_occ_crc:
                    log.w("  OCC table identical to modified input (no revert / no change)")
                else:
                    ok, _ = write_calibration_table_with_crc(calib_dev, new_calib_bytes)
                    if ok:
                        log.i("  OCC-produced table written to flash")
                    else:
                        log.e("  Failed to write OCC-produced table to flash")
            else:
                log.w("  OCC returned empty calibration table bytes")
                health_factor = None
            # Reset camera after OCC calibration

            log.i("Issuing hardware reset to device ...")
            try:
                calib_dev.hardware_reset()
            except Exception as e:
                log.e(f"Hardware reset call failed: {e}")
                test.fail()
            # After hardware reset the old handle becomes invalid; wait for re-enumeration
            log.i("Waiting for device to re-enumerate after reset ...")
            time.sleep(3.0)  # initial grace period
            config, pipeline, calib_dev = get_calibration_device(image_width, image_height, fps)
            #reacquired = _reacquire_auto_calib_device(device_serial)
            #if not reacquired:
            #    log.e("Failed to re-enumerate device after hardware reset")
            #    test.fail()
            #else:
            #    calib_dev = reacquired
            #    log.i("✓ Device re-enumerated and auto_calibrated_device reacquired")
                # Extra short delay to allow internal services to stabilize
            time.sleep(1.5)
            if health_factor is not None:
                if abs(health_factor) >= HEALTH_FACTOR_THRESHOLD:
                    log.w(f"Health factor magnitude {abs(health_factor):.3f} exceeds threshold {HEALTH_FACTOR_THRESHOLD}, continuing to evaluate preservation anyway")
                else:
                    log.i("OCC calibration completed (health factor within threshold)")
                # Analyze what corrections OCC made after OCC
                final_principal_points_result = get_current_rect_params(calib_dev)
                if final_principal_points_result is not None:
                    final_left_pp, final_right_pp, final_offsets = final_principal_points_result
                    log.i(f"  Final principal points (pixel coordinates) - Right: ppx={final_right_pp[0]:.6f}, ppy={final_right_pp[1]:.6f}")
                    # (debug CRC logging removed)

                    if principal_points_result is not None and modified_principal_points_result is not None:
                        orig_ppy = orig_right_pp[1]
                        modified_ppy = modified_right_pp[1]
                        final_ppy = final_right_pp[1]

                        distance_from_original = abs(final_ppy - orig_ppy)
                        distance_from_modified = abs(final_ppy - modified_ppy)
                        log.i(f"  (Right) Distance from original: {distance_from_original:.6f}")
                        log.i(f"  (Right) Distance from modified: {distance_from_modified:.6f}")

                        # Success criteria: final closer to modified than original
                        if distance_from_modified <= distance_from_original:
                            log.i("✓ OCC preserved the manual correction")
                            test.fail()
                        else:
                            log.e("⚠ OCC reverted closer to original calibration")
                           
                           
                    else:
                        log.w("Missing comparison data")
                        test.fail()
                else:
                    log.w("Could not read final principal points")
                    test.fail()
                                                
                # restore_calibration_table(calib_dev)
            else:
                log.e("OCC calibration failed: health_factor is None")
                test.fail()
        
    except Exception as e:
        log.e("OCC calibration with principal point modification failed: ", str(e))
        test.fail()
    log.i("Done\n")

test.print_results_and_exit()


