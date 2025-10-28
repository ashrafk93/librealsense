# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#test:donotrun:!dds
#test:device D555

import pyrealsense2 as rs
from rspy import test, log

def check_option_in_list(option_id, options_list):
    for option in options_list:
        if option_id == option:
            return True
    return False

def set_get_filter_option_value(embedded_filter, option, value_to_assign):
    initial_value = embedded_filter.get_option(option)
    option_step =  embedded_filter.get_option_range(option).step
    embedded_filter.set_option(option, value_to_assign)
    test.check_approx_abs(embedded_filter.get_option(option), value_to_assign, option_step)
    embedded_filter.set_option(option, initial_value)
    test.check_approx_abs(embedded_filter.get_option(option), value_to_assign, option_step)

def parse_fw_version(version_str):
    """Parse version string like '7.56.36850.1227' into tuple of integers"""
    try:
        return tuple(map(int, version_str.split('.')))
    except (ValueError, AttributeError):
        return (0, 0, 0, 0)

def is_fw_version_below(curr_fw, min_fw):
    current_fw_version = parse_fw_version(curr_fw)
    min_fw_version = parse_fw_version(min_fw)
    return current_fw_version < min_fw_version


dev, _ = test.find_first_device_or_exit()
depth_sensor = dev.first_depth_sensor()
fw_version = dev.get_info(rs.camera_info.firmware_version)

if is_fw_version_below(fw_version, '7.56.36850.1229'):
    test.unexpected_exception()

# this test will get options values for the decimation embedded filter, from the depth sensor
# - get DDS device
# - get depth sensor
# - get embedded decimation filter
# - get options values for this filter
# - check 2 options available
with test.closure("Get Decimation embedded filter options"):
    decimation_embedded_filter = depth_sensor.get_embedded_filter(rs.embedded_filter_type.decimation)
    test.check(decimation_embedded_filter)

    decimation_options = decimation_embedded_filter.get_supported_options()
    test.check_equal(len(decimation_options), 2)
    check_option_in_list(rs.option.embedded_filter_enabled, decimation_options)
    check_option_in_list(rs.option.filter_magnitude, decimation_options)

with test.closure("Decimation embedded filter set/get options"):
    set_get_filter_option_value(decimation_embedded_filter, rs.option.embedded_filter_enabled, 1.0)
    # not setting magnitude because it is R/O option
    test.check_equal(decimation_embedded_filter.get_option(rs.option.filter_magnitude), 2.0)

# same test for temporal filter
with test.closure("Get Temporal embedded filter options"):
    temporal_embedded_filter = depth_sensor.get_embedded_filter(rs.embedded_filter_type.temporal)
    test.check(temporal_embedded_filter)

    temporal_options = temporal_embedded_filter.get_supported_options()
    test.check_equal(len(temporal_options), 4)
    check_option_in_list(rs.option.embedded_filter_enabled, temporal_options)
    check_option_in_list(rs.option.filter_smooth_alpha, temporal_options)
    check_option_in_list(rs.option.filter_smooth_delta, temporal_options)
    check_option_in_list(rs.option.holes_fill, temporal_options)

# below should be uncommented after feature enabled in FW
'''
with test.closure("Temporal embedded filter set/get options"):
    set_get_filter_option_value(temporal_embedded_filter, rs.option.embedded_filter_enabled, 1.0)
    set_get_filter_option_value(temporal_embedded_filter, rs.option.filter_smooth_alpha, 0.2)
    set_get_filter_option_value(temporal_embedded_filter, rs.option.filter_smooth_delta, 30.0)
    set_get_filter_option_value(temporal_embedded_filter, rs.option.holes_fill, 6)
'''

test.print_results_and_exit()
