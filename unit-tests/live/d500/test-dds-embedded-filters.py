# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#test:donotrun:!dds
#test:device D555

import pyrealsense2 as rs
from rspy import test, log


dev, _ = test.find_first_device_or_exit()
depth_sensor = dev.first_depth_sensor()

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

# same test for temporal filter
with test.closure("Get Temporal embedded filter options"):
    temporal_embedded_filter = depth_sensor.get_embedded_filter(rs.embedded_filter_type.temporal)
    test.check(temporal_embedded_filter)

    temporal_options = temporal_embedded_filter.get_supported_options()
    test.check_equal(len(temporal_options), 4)

test.print_results_and_exit()
