// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2021 RealSense, Inc. All Rights Reserved.

#pragma once

#include "core/extension.h"

namespace librealsense {
class temporal_embedded_filter
{
public:
    virtual ~temporal_embedded_filter() = default;
};

MAP_EXTENSION( RS2_EXTENSION_TEMPORAL_EMBEDDED_FILTER, librealsense::temporal_embedded_filter);

}  // namespace librealsense
