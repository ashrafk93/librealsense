// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2023 RealSense, Inc. All Rights Reserved.

#pragma once

#include "embedded-filter-interface.h"
#include "extension.h"

#include <functional>
#include <vector>


namespace librealsense {


class query_embedded_filters_interface
{
public:
    virtual ~query_embedded_filters_interface() = default;

    virtual embedded_filters query_embedded_filters() const = 0;};

MAP_EXTENSION( RS2_EXTENSION_QUERY_EMBEDDED_FILTERS, librealsense::query_embedded_filters_interface );


}  // namespace librealsense
