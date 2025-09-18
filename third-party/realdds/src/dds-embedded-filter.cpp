// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#include <realdds/dds-embedded-filter.h>
#include <realdds/dds-device.h>
#include <realdds/dds-stream-base.h>
#include <realdds/dds-exceptions.h>

#include <rsutils/json.h>
#include <stdexcept>
#include <cstring>

using rsutils::json;


namespace realdds {


// Base class implementation
dds_embedded_filter::dds_embedded_filter()
    : _name("")
    , _options(json{})
    , _filter_type(RS2_EMBEDDED_FILTER_TYPE_DECIMATION)
    , _initialized(false)
{
}

void dds_embedded_filter::init(const std::string& name, rs2_embedded_filter_type type)
{
    verify_uninitialized();
    _name = name;
    _filter_type = type;
}

void dds_embedded_filter::init_options(const rsutils::json& options)
{
    verify_uninitialized();
    _options = options;
}

void dds_embedded_filter::init_stream(std::shared_ptr< dds_stream_base > const& stream)
{
    // NOTE: this happens AFTER we're "initialized" (i.e., the value is set before we get here)
    if (_stream.lock())
        DDS_THROW(runtime_error, "filter '" << get_name() << "' already has a stream");
    if (!stream)
        DDS_THROW(runtime_error, "null stream");
    _stream = stream;
}

void dds_embedded_filter::init_default_values(const rsutils::json& defaults)
{
    verify_uninitialized();
    // For now, just mark as initialized without processing defaults
    // This can be expanded later when specific defaults are needed
    _initialized = true;
}

void dds_embedded_filter::verify_uninitialized() const
{
    if (_initialized) {
        throw std::runtime_error("Cannot re-initialize embedded filter");
    }
}

rsutils::json dds_embedded_filter::props_to_json() const
{
    json props;
    props["name"] = _name;
    props["type"] = embedded_filter_type_to_string(_filter_type);
    if (!_options.is_null()) {
        props["options"] = _options;
    }
	auto stream = _stream.lock();
    if (stream) {
        props["stream_type"] = stream->name();
    }
    props["current_values"] = _current_values;
    return props;
}

rsutils::json dds_embedded_filter::to_json() const
{
    return props_to_json();
}

std::shared_ptr<dds_embedded_filter> dds_embedded_filter::from_json(const rsutils::json& j, const std::string& stream_name)
{
    std::string name_str;
    if (j.contains("name")) 
    {
        name_str = j["name"].get<std::string>();
    }
    else
    {
		DDS_THROW(runtime_error, "missing name");
    }
    auto type = embedded_filter_type_from_string(name_str);
	// create the appropriate filter type
    auto filter = create_embedded_filter(type);
    filter->init(j["name"].get<std::string>(), type);
    
    if (j.contains("options")) {
        filter->init_options(j["options"]);
    }

    // TODO - below lines to be checked
    json defaults;
    if (j.contains("current_values")) {
        defaults = j["current_values"];
    }
    filter->init_default_values(defaults);
    
    return filter;
}

void dds_embedded_filter::check_options(const json& options) const
{
    if (!options.exists())
        DDS_THROW(runtime_error, "invalid options");

    if (options.is_null())
        DDS_THROW(runtime_error, "options null");
}

void dds_embedded_filter::set_current_value(const std::string& key, const rsutils::json& value)
{
    _current_values[key] = value;
}

rsutils::json dds_embedded_filter::get_current_value(const std::string& key) const
{
    auto it = _current_values.find(key);
    if (it != _current_values.end()) {
        return it->second;
    }
    return json{};
}

// Decimation filter implementation
dds_decimation_filter::dds_decimation_filter()
    : dds_embedded_filter()
    , _enabled(false)
    , _decimation_factor(2)
{
}

void dds_decimation_filter::set_options(const rsutils::json& options)
{
    check_options(options);
    _options = std::move(options);
    /*auto dev = _dev.lock();
    if (!dev)
        DDS_THROW(runtime_error, "device no longer available");
    dev->set_embedded_filter(shared_from_this(), _options);*/
}

rsutils::json dds_decimation_filter::get_options()
{
    /*auto dev = _dev.lock();
    if (!dev)
        DDS_THROW(runtime_error, "device no longer available");
    auto options = dev->query_embedded_filter(shared_from_this())
    check_options(options);
    _options = std::move(options);;*/
    return _options;
}

void dds_decimation_filter::set_enabled(bool enabled)
{
    _enabled = enabled;
    set_current_value("enabled", enabled);
}

void dds_decimation_filter::set_decimation_factor(int factor)
{
    if (factor < 1) {
        throw std::invalid_argument("Decimation factor must be at least 1");
    }
    _decimation_factor = factor;
    set_current_value("decimation_factor", factor);
}

rsutils::json dds_decimation_filter::to_json() const
{
    auto props = props_to_json();
    props["enabled"] = _enabled;
    props["decimation_factor"] = _decimation_factor;
    return props;
}

// Temporal filter implementation
dds_temporal_filter::dds_temporal_filter()
    : dds_embedded_filter()
    , _enabled(false)
    , _alpha(0.4f)
    , _delta(20)           // Changed from 20.0f to 20 (int32_t)
    , _persistency(3)
{
    _name = "Temporal Filter";
    _filter_type = RS2_EMBEDDED_FILTER_TYPE_TEMPORAL;
}

void dds_temporal_filter::set_options(const rsutils::json& options)
{
    check_options(options);
    _options = std::move(options);
}

rsutils::json dds_temporal_filter::get_options()
{
    return _options;
}

void dds_temporal_filter::set_enabled(bool enabled)
{
    _enabled = enabled;
    set_current_value("enabled", enabled);
}

void dds_temporal_filter::set_alpha(float alpha)
{
    if (alpha < 0.0f || alpha > 1.0f) {
        throw std::invalid_argument("Alpha must be between 0.0 and 1.0");
    }
    _alpha = alpha;
    set_current_value("alpha", alpha);
}

void dds_temporal_filter::set_delta(int32_t delta)              // Changed parameter type from float to int32_t
{
    if (delta < 0) {        // Changed from 0.0f to 0
        throw std::invalid_argument("Delta must be non-negative");
    }
    _delta = delta;
    set_current_value("delta", delta);
}

void dds_temporal_filter::set_persistency(int32_t persistency)    // Changed parameter type from int to int32_t
{
    if (persistency < 0) {
        throw std::invalid_argument("Persistency must be non-negative");
    }
    _persistency = persistency;
    set_current_value("persistency", persistency);
}

rsutils::json dds_temporal_filter::to_json() const
{
    auto props = props_to_json();
    props["enabled"] = _enabled;
    props["alpha"] = _alpha;
    props["delta"] = _delta;
    props["persistency"] = _persistency;
    return props;
}

// Factory and utility functions
std::shared_ptr<dds_embedded_filter> create_embedded_filter(rs2_embedded_filter_type type)
{
    switch (type) {
        case RS2_EMBEDDED_FILTER_TYPE_DECIMATION:
            return std::make_shared<dds_decimation_filter>();
        case RS2_EMBEDDED_FILTER_TYPE_TEMPORAL:
            return std::make_shared<dds_temporal_filter>();
        default:
            throw std::invalid_argument("Unknown embedded filter type");
    }
}

std::string embedded_filter_type_to_string(rs2_embedded_filter_type type)
{
    switch (type) {
        case RS2_EMBEDDED_FILTER_TYPE_DECIMATION:
            return "Decimation Filter";
        case RS2_EMBEDDED_FILTER_TYPE_TEMPORAL:
            return "Temporal Filter";
        default:
            return "unknown";
    }
}

rs2_embedded_filter_type embedded_filter_type_from_string(const std::string& type_str)
{
    if (type_str == "Decimation Filter") {
        return RS2_EMBEDDED_FILTER_TYPE_DECIMATION;
    } else if (type_str == "Temporal Filter") {
        return RS2_EMBEDDED_FILTER_TYPE_TEMPORAL;
    } else {
        throw std::invalid_argument("Unknown embedded filter type: " + type_str);
    }
}


}  // namespace realdds
