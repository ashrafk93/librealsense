// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2021 RealSense, Inc. All Rights Reserved.

#pragma once

#include "core/extension.h"

namespace librealsense {
class decimation_embedded_filter
{
public:
    virtual ~decimation_embedded_filter() = default;
};

MAP_EXTENSION( RS2_EXTENSION_DECIMATION_EMBEDDED_FILTER, librealsense::decimation_embedded_filter);

}  // namespace librealsense
