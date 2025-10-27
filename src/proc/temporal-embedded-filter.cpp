#include "temporal-embedded-filter.h"

rsutils::subscription librealsense::temporal_embedded_filter::register_options_changed_callback(options_watcher::callback&& cb)
{
    return _options_watcher.subscribe(std::move(cb));
}
