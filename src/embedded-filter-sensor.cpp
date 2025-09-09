// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#include "embedded-filter-sensor.h"


namespace librealsense {
	void embedded_filter_sensor::set(rs2_embedded_filter embedded_filter_type, std::vector<uint8_t> params)
	{

	}

	std::vector<uint8_t> embedded_filter_sensor::get(rs2_embedded_filter embedded_filter_type)
	{
		return std::vector<uint8_t>();
	}

	bool embedded_filter_sensor::supports(rs2_embedded_filter embedded_filter_type) const
	{

		return false;
	}


}
