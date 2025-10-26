# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2023 RealSense, Inc. All Rights Reserved.

import sys
import time
import copy
import pyrealsense2 as rs
from rspy import test, log
import struct
import zlib
import ctypes

# Constants for calibration
CALIBRATION_TIMEOUT_SECONDS = 30
OCC_TIMEOUT_MS = 9000
TARE_TIMEOUT_MS = 10000
FRAME_PROCESSING_TIMEOUT_MS = 5000
HARDWARE_RESET_DELAY_SECONDS = 3

# Global variable to store original calibration table
_global_original_calib_table = None

def on_calib_cb(progress):
    """Callback function for calibration progress reporting."""
    pp = int(progress)
    log.d( f"Calibration at {progress}%" )

def get_calibration_device(image_width, image_height, fps):
    """
    Setup and configure the calibration device.
    
    Args:
        image_width (int): Image width
        image_height (int): Image height
        fps (int): Frames per second
        
    Returns:
        tuple: (pipeline, auto_calibrated_device)
    """
    config = rs.config()
    pipeline = rs.pipeline()
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    config.enable_stream(rs.stream.depth, image_width, image_height, rs.format.z16, fps)            

    pipeline_profile = config.resolve(pipeline_wrapper)
    auto_calibrated_device = rs.auto_calibrated_device(pipeline_profile.get_device())
    if not auto_calibrated_device:
        raise RuntimeError("Failed to open auto_calibrated_device for calibration")
    
    return config, pipeline, auto_calibrated_device

def calibration_main(config, pipeline, calib_dev, occ_calib, json_config, ground_truth, return_table=False):
    """
    Main calibration function for both OCC and Tare calibrations.
    
    Args:
        host_assistance (bool): Whether to use host assistance mode
        occ_calib (bool): True for OCC calibration, False for Tare calibration
        json_config (str): JSON configuration string
        ground_truth (float): Ground truth value for Tare calibration (None for OCC)
    
    Returns:
        float: Health factor from calibration
    """

    conf = pipeline.start(config)
    pipeline.wait_for_frames()  # Verify streaming started before calling calibration methods

    depth_sensor = conf.get_device().first_depth_sensor()
    if depth_sensor.supports(rs.option.emitter_enabled):
        depth_sensor.set_option(rs.option.emitter_enabled, 1)
    if depth_sensor.supports(rs.option.thermal_compensation):
        depth_sensor.set_option(rs.option.thermal_compensation, 0)

    new_calib_result = b''
    # Execute calibration based on type
    try:
        if occ_calib:    
            log.i("Starting on-chip calibration")
            new_calib, health = calib_dev.run_on_chip_calibration(json_config, on_calib_cb, OCC_TIMEOUT_MS)
        else:    
            log.i("Starting tare calibration")
            new_calib, health = calib_dev.run_tare_calibration(ground_truth, json_config, on_calib_cb, TARE_TIMEOUT_MS)

        calib_done = len(new_calib) > 0
        timeout_end = time.time() + CALIBRATION_TIMEOUT_SECONDS
        
        while not calib_done:
            if time.time() > timeout_end:
                raise RuntimeError("Calibration timed out after {} seconds".format(CALIBRATION_TIMEOUT_SECONDS))
            frame_set = pipeline.wait_for_frames()
            depth_frame = frame_set.get_depth_frame()
            new_calib, health = calib_dev.process_calibration_frame(depth_frame, on_calib_cb, FRAME_PROCESSING_TIMEOUT_MS)
            calib_done = len(new_calib) > 0
        # Preserve final table
        if isinstance(new_calib, list):
            new_calib_result = bytes(new_calib)
        else:
            new_calib_result = bytes(new_calib) if new_calib else b''
            
        log.i("Calibration completed successfully")
        log.i("Health factor = ", health[0])
        
    except Exception as e:
        log.e("Calibration failed: ", str(e))
        raise
    finally:
        # Stop pipeline
        pipeline.stop()

    if return_table:
        return health[0], new_calib_result
    return health[0]


def measure_depth_fill_rate(ctx=None, width=640, height=480, fps=30, frames=10, timeout_s=12, enable_emitter=True):
    """Measure average depth fill rate (non-zero pixel ratio * 100) over a number of frames.

    Args:
        ctx (rs.context|None): Optional existing context; created if None.
        width, height, fps: Stream configuration.
        frames (int): Max frames to sample (may exit early after 5 frames if stable).
        timeout_s (float): Overall timeout.
        enable_emitter (bool): Enable emitter if supported for consistent fill.

    Returns:
        float | None: Average fill rate percentage (0-100) or None if capture failed.
    """
    try:
        if ctx is None:
            ctx = rs.context()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        pipe = rs.pipeline(ctx)
        profile = pipe.start(cfg)
        # Small warm-up
        try:
            depth_sensor = profile.get_device().first_depth_sensor()
            if enable_emitter and depth_sensor.supports(rs.option.emitter_enabled):
                depth_sensor.set_option(rs.option.emitter_enabled, 1)
            if depth_sensor.supports(rs.option.thermal_compensation):
                depth_sensor.set_option(rs.option.thermal_compensation, 0)
        except Exception:
            pass
        start = time.time()
        collected = 0
        total_fill = 0.0
        import numpy as np
        while collected < frames and (time.time() - start) < timeout_s:
            try:
                fs = pipe.wait_for_frames(3000)
            except Exception:
                continue
            depth = fs.get_depth_frame()
            if not depth:
                continue
            data = np.asanyarray(depth.get_data())  # z16
            total_pixels = data.size
            if total_pixels == 0:
                continue
            non_zero = np.count_nonzero(data)
            fill = (non_zero / total_pixels) * 100.0
            total_fill += fill
            collected += 1
            if collected >= 5 and (time.time() - start) > 1.5:
                break
        pipe.stop()
        if collected == 0:
            return None
        return total_fill / collected
    except Exception as e:
        log.w(f"measure_depth_fill_rate failed: {e}")
        try:
            pipe.stop()
        except Exception:
            pass
        return None


def measure_average_depth(ctx=None, width=640, height=480, fps=30, frames=10, timeout_s=12, enable_emitter=True):
    """Measure the average depth distance (in meters) across a series of frames.

    Valid depth pixels are > 0 (raw units). Invalid (<=0) are ignored.

    Args:
        ctx (rs.context|None): Optional existing context; created if None.
        width, height, fps (int): Stream configuration for depth.
        frames (int): Maximum number of frames to sample.
        timeout_s (float): Overall timeout for the sampling loop.
        enable_emitter (bool): Attempt to enable emitter for more reliable data.

    Returns:
        float | None: Mean depth in meters over all valid pixels from collected frames,
                      or None if no valid data was obtained.
    """
    pipe = None
    try:
        import numpy as np
        if ctx is None:
            ctx = rs.context()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        pipe = rs.pipeline(ctx)
        profile = pipe.start(cfg)

        # Configure sensor options (best-effort)
        try:
            depth_sensor = profile.get_device().first_depth_sensor()
            if enable_emitter and depth_sensor.supports(rs.option.emitter_enabled):
                depth_sensor.set_option(rs.option.emitter_enabled, 1)
            if depth_sensor.supports(rs.option.thermal_compensation):
                depth_sensor.set_option(rs.option.thermal_compensation, 0)
            depth_scale = depth_sensor.get_depth_scale()
        except Exception:
            # Fallback default RealSense depth scale (commonly 0.001) if retrieval fails
            depth_scale = 0.001

        start = time.time()
        collected = 0
        valid_sum_m = 0.0
        valid_count = 0

        while collected < frames and (time.time() - start) < timeout_s:
            try:
                fs = pipe.wait_for_frames(3000)
            except Exception:
                continue
            depth = fs.get_depth_frame()
            if not depth:
                continue
            data = np.asanyarray(depth.get_data())  # uint16
            if data.size == 0:
                continue
            valid_mask = data > 0  # ignore zero / invalid
            count = valid_mask.sum()
            if count == 0:
                # No valid points in this frame; keep trying
                collected += 1
                continue
            # Accumulate sums in meters (raw * scale)
            valid_sum_m += float(data[valid_mask].sum()) * depth_scale
            valid_count += int(count)
            collected += 1
            # Early break heuristic: if we already have plenty of samples and elapsed > 1.5s
            if collected >= 5 and (time.time() - start) > 1.5:
                break

        pipe.stop()
        if valid_count == 0:
            return None
        return valid_sum_m / valid_count
    except Exception as e:
        log.w(f"measure_average_depth failed: {e}")
        try:
            if pipe:
                pipe.stop()
        except Exception:
            pass
        return None


def is_mipi_device():
    ctx = rs.context()
    device = ctx.query_devices()[0]
    return device.supports(rs.camera_info.connection_type) and device.get_info(rs.camera_info.connection_type) == "GMSL"

# for step 2 -  not in use for now

def get_current_rect_params(auto_calib_device):
    """Return per-eye principal points (ppx, ppy) using intrinsic matrices; minimal logging."""
    calib_table = auto_calib_device.get_calibration_table()
    if isinstance(calib_table, list) or (hasattr(calib_table, '__iter__') and not isinstance(calib_table, (bytes, bytearray, str))):
        calib_table = bytes(calib_table)
    if len(calib_table) < 280:
        log.e("Calibration table is too small")
        return None
    try:
        header_size = 16
        intrinsic_left_offset = header_size
        intrinsic_right_offset = header_size + 36
        rect_params_base_offset = header_size + 36 + 36 + 36 + 36 + 4 + 4 + 88  # 256
        rect_params_1280_800_offset = rect_params_base_offset + (8 * 16)
        if (intrinsic_right_offset + 36 > len(calib_table) or
            intrinsic_left_offset + 36 > len(calib_table) or
            rect_params_1280_800_offset + 16 > len(calib_table)):
            log.e("Calibration table too small for intrinsics")
            return None
        left_intrinsics_raw = struct.unpack('<9f', calib_table[intrinsic_left_offset:intrinsic_left_offset + 36])
        right_intrinsics_raw = struct.unpack('<9f', calib_table[intrinsic_right_offset:intrinsic_right_offset + 36])
        rect_params_1280_800 = struct.unpack('<4f', calib_table[rect_params_1280_800_offset:rect_params_1280_800_offset + 16])
        width = 1280.0
        height = 800.0
        # Empirically determined indices: ppx uses element 2 (normalized), ppy uses element 3 (direct * height)
        left_ppx = left_intrinsics_raw[2] * width
        right_ppx = right_intrinsics_raw[2] * width
        left_ppy = left_intrinsics_raw[3] * height
        right_ppy = right_intrinsics_raw[3] * height
        offsets_dict = {
            'intrinsic_left_offset': intrinsic_left_offset,
            'intrinsic_right_offset': intrinsic_right_offset
        }
        return (left_ppx, left_ppy), (right_ppx, right_ppy), offsets_dict
    except Exception as e:
        log.e(f"Error reading principal points: {e}")
        return None


def modify_calibration_table(device, new_calib_params, rect_param_offset, image_width=None, image_height=None):
    """
    Modifies the calibration table of an auto-calibrated RealSense device
    Args:
        device: RealSense device object that supports auto calibration
        new_calib_params: tuple of (fx, fy, ppx, ppy) new calibration parameters
        rect_param_offset: offset in calibration table for the specific resolution
        image_width: original image width (for scaling)
        image_height: original image height (for scaling)
    Returns:
        bool: True if successful, False otherwise
    """
    global _global_original_calib_table
    
    # Get the auto calibrated device interface
    auto_calib_device = rs.auto_calibrated_device(device)
    if not auto_calib_device:
        log.e("Device does not support auto calibration")
        return False
    
    # Read current calibration table if not already stored
    if _global_original_calib_table is None:
        log.i("Reading current calibration table...")
        _global_original_calib_table = auto_calib_device.get_calibration_table()
        
        # Convert to bytes if it's returned as a list
        if isinstance(_global_original_calib_table, list):
            _global_original_calib_table = bytes(_global_original_calib_table)
        elif hasattr(_global_original_calib_table, '__iter__') and not isinstance(_global_original_calib_table, (bytes, bytearray, str)):
            _global_original_calib_table = bytes(_global_original_calib_table)
            
        log.i(f"Original calibration table size: {len(_global_original_calib_table)} bytes")
    
    # Make a copy for modification
    modified_calib_table = bytearray(_global_original_calib_table)  # Use bytearray for easier modification

    try:
        new_fx, new_fy, new_ppx, new_ppy = new_calib_params
        
        # If we're working with 256x144 that was scaled from 424x240, we need to reverse the scaling
        if image_width == 256 and image_height == 144:
            # Reverse scale factors: 424/256 ≈ 1.656, 240/144 ≈ 1.667
            scale_x = 424.0 / 256.0
            scale_y = 240.0 / 144.0
            
            # Reverse scale the parameters back to 424x240 space for storage
            storage_fx = new_fx * scale_x
            storage_fy = new_fy * scale_y
            storage_ppx = new_ppx * scale_x
            storage_ppy = new_ppy * scale_y
            
            log.i(f"  Modifying calibration - fx: {new_fx:.3f}, fy: {new_fy:.3f}, ppx: {new_ppx:.3f}, ppy: {new_ppy:.3f}")
            log.i(f"  Storing as 424x240 - fx: {storage_fx:.3f}, fy: {storage_fy:.3f}, ppx: {storage_ppx:.3f}, ppy: {storage_ppy:.3f}")
            
            # Pack the reverse-scaled values
            new_rect_params = struct.pack('<ffff', storage_fx, storage_fy, storage_ppx, storage_ppy)
        else:
            log.i(f"  Modifying calibration - fx: {new_fx:.3f}, fy: {new_fy:.3f}, ppx: {new_ppx:.3f}, ppy: {new_ppy:.3f}")
            # Pack new values normally
            new_rect_params = struct.pack('<ffff', new_fx, new_fy, new_ppx, new_ppy)
        
        # Replace the section in the bytearray
        modified_calib_table[rect_param_offset:rect_param_offset + 16] = new_rect_params
        
        # Update CRC32 for the modified table
        # CRC is calculated over all data after the header (excluding the CRC field itself)
        header_fmt = '<IIII'  # 4 uint32 values in header
        header_size = struct.calcsize(header_fmt)
        
        # Extract original header values
        header_data = struct.unpack(header_fmt, modified_calib_table[:header_size])
        table_type, table_size, param, old_crc32 = header_data
        
        actual_data = bytes(modified_calib_table[header_size:])
        new_crc32 = zlib.crc32(actual_data) & 0xffffffff
        
        # Update the CRC in the header
        new_header = struct.pack('<IIII', table_type, table_size, param, new_crc32)
        modified_calib_table[:header_size] = new_header
        
        # Convert back to bytes for setting
        modified_calib_table = bytes(modified_calib_table)
        
        log.i("  Calibration table modification completed successfully")
        
        # Set the modified calibration table (temporarily)
        # Convert bytes to list for the API
        table_as_list = list(modified_calib_table)
        auto_calib_device.set_calibration_table(table_as_list)
        
        # Write the modified calibration table to flash
        auto_calib_device.write_calibration()
        log.i("✓ Calibration table written to flash successfully")
        
        return True
        
    except Exception as e:
        log.e(f"Failed to modify calibration table: {e}")
        return False

        
def restore_calibration_table(device):
    global _global_original_calib_table

    # Get the auto calibrated device interface
    auto_calib_device = rs.auto_calibrated_device(device)
    if not auto_calib_device:
        log.e("Device does not support auto calibration")
        return False

    auto_calib_device.reset_to_factory_calibration()
    return True

def get_d400_calibration_table(device):
    """Get current calibration table from device"""
    try:
        # device is already an auto_calibrated_device
        calib_table = device.get_calibration_table()
        # Convert to bytes if it's a list
        if isinstance(calib_table, list):
            calib_table = bytes(calib_table)
        return calib_table
    except Exception as e:
        log.e(f"-E- Failed to get calibration table: {e}")
        return None
def write_calibration_table_with_crc(device, modified_data):
    """Write modified calibration table with updated CRC"""
    try:
        # Calculate CRC32 for the data after the header (skip first 16 bytes)
        actual_data = modified_data[16:]
        new_crc32 = zlib.crc32(actual_data) & 0xffffffff
        
        # Get old CRC for comparison
        old_crc32 = struct.unpack('<I', modified_data[12:16])[0]
        
        # Update CRC in the header
        final_data = bytearray(modified_data)
        final_data[12:16] = struct.pack('<I', new_crc32)
                
        # Convert to list of ints for the API
        calib_list = list(final_data)
        
        # Write to device - device is already auto_calibrated_device
        device.set_calibration_table(calib_list)
        device.write_calibration()
        
        return True, bytes(final_data)
        
    except Exception as e:
        log.e(f"-E- Error writing calibration table: {e}")
        return False, str(e)

def modify_extrinsic_calibration(device, pixel_correction, modify_ppy=True):
    """Modify either raw right intrinsic ppx or ppy by pixel_correction (in pixels).

    Args:
        device: auto_calibrated_device (rs.auto_calibrated_device)
        pixel_correction (float): delta in pixels to add
        modify_ppy (bool): if True adjust ppy else adjust ppx
    Returns:
        tuple: (success(bool), modified_table_bytes(or error str), new_ppx, new_ppy)
    """
    try:
        calib_table = get_d400_calibration_table(device)
        if not calib_table:
            return False, "Failed to get calibration table", None, None
        modified_data = bytearray(calib_table)
        header_size = 16
        right_intrinsics_offset = header_size + 36  # skip left 9 floats (left eye)
        right_intrinsics = list(struct.unpack('<9f', modified_data[right_intrinsics_offset:right_intrinsics_offset+36]))
        width = 1280.0
        height = 800.0
        original_raw_ppx = right_intrinsics[2] * width
        original_raw_ppy = right_intrinsics[3] * height
        if modify_ppy:
            corrected_raw_ppy = original_raw_ppy + pixel_correction
            right_intrinsics[3] = corrected_raw_ppy / height
            log.i(f"  Raw Right ppy original={original_raw_ppy:.6f} modified={corrected_raw_ppy:.6f}")
        else:
            corrected_raw_ppx = original_raw_ppx + pixel_correction
            right_intrinsics[2] = corrected_raw_ppx / width
            log.i(f"  Raw Right ppx original={original_raw_ppx:.6f} modified={corrected_raw_ppx:.6f}")
        modified_data[right_intrinsics_offset:right_intrinsics_offset+36] = struct.pack('<9f', *right_intrinsics)
        write_ok, modified_table_or_err = write_calibration_table_with_crc(device, bytes(modified_data))
        if not write_ok:
            return False, modified_table_or_err, None, None
        new_ppx = right_intrinsics[2] * width
        new_ppy = right_intrinsics[3] * height
        return True, modified_table_or_err, new_ppx, new_ppy
    except Exception as e:
        log.e(f"Error modifying calibration: {e}")
        return False, str(e), None, None

# ---------------------------------------------------------------------------
# Advanced OCC calibration test helper (moved from test_occ_calibrations.py)
# ---------------------------------------------------------------------------

# Pixel correction constant for DSCalibrationModifier-style corrections
PIXEL_CORRECTION = -1
EPSILON = 0.001  # Small tolerance for floating point precision

# Health factor thresholds
HEALTH_FACTOR_THRESHOLD = 0.3
HEALTH_FACTOR_THRESHOLD_AFTER_MODIFICATION = 0.75

def on_chip_calibration_json(occ_json_file, host_assistance):
    """Return OCC JSON string (default if file not provided)."""
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
