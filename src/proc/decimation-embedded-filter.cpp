#include "decimation-embedded-filter.h"

rsutils::subscription librealsense::decimation_embedded_filter::register_options_changed_callback(options_watcher::callback&& cb)
{
    return _options_watcher.subscribe(std::move(cb));
}
