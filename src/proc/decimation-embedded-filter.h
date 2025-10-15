// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#pragma once

#include "core/extension.h"
#include <src/core/options-container.h>
#include <src/core/options-watcher.h>

namespace librealsense {
class decimation_embedded_filter
    : virtual public embedded_filter_interface
    , public options_container
{
public:
    virtual ~decimation_embedded_filter() = default;

protected:
    options_watcher _options_watcher;
};

MAP_EXTENSION( RS2_EXTENSION_DECIMATION_EMBEDDED_FILTER, librealsense::decimation_embedded_filter);

}  // namespace librealsense
