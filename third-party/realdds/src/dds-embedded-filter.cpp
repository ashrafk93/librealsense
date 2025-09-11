// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#include <realdds/dds-embedded-filter.h>

#include <rsutils/json.h>
#include <stdexcept>
#include <cstring>

using rsutils::json;


namespace realdds {


// Base class implementation
dds_embedded_filter::dds_embedded_filter()
    : _name("")
    , _options(json{})
    , _stream_type(json{})
    , _filter_type(embedded_filter_type::decimation)
    , _initialized(false)
{
}

void dds_embedded_filter::init(const std::string& name, embedded_filter_type type)
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

void dds_embedded_filter::init_stream_type(const rsutils::json& stream_type)
{
    verify_uninitialized();
    _stream_type = stream_type;
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
    if (!_stream_type.is_null()) {
        props["stream_type"] = _stream_type;
    }
    props["current_values"] = _current_values;
    return props;
}

rsutils::json dds_embedded_filter::to_json() const
{
    return props_to_json();
}

std::shared_ptr<dds_embedded_filter> dds_embedded_filter::from_json(const rsutils::json& j)
{
    if (!j.contains("type")) {
        throw std::invalid_argument("Missing 'type' field in embedded filter JSON");
    }
    
    auto type_str = j["type"].get<std::string>();
    auto type = embedded_filter_type_from_string(type_str);
    
    auto filter = create_embedded_filter(type);
    
    if (j.contains("name")) {
        filter->init(j["name"].get<std::string>(), type);
    }
    
    if (j.contains("options")) {
        filter->init_options(j["options"]);
    }
    
    if (j.contains("stream_type")) {
        filter->init_stream_type(j["stream_type"]);
    }
    
    json defaults;
    if (j.contains("current_values")) {
        defaults = j["current_values"];
    }
    filter->init_default_values(defaults);
    
    return filter;
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
    _filter_type = embedded_filter_type::decimation;
}

void dds_decimation_filter::set_filter_data(const std::vector<uint8_t>& data)
{
    if (data.empty()) {
        throw std::invalid_argument("Empty filter data");
    }
    
    // Parse decimation filter data
    // Format: [enabled_flag, decimation_factor]
    _enabled = (data[0] != 0);
    
    if (data.size() >= 2) {
        _decimation_factor = static_cast<int>(data[1]);
        if (_decimation_factor < 1) {
            _decimation_factor = 2; // Default fallback
        }
    }
    
    // Update current values
    set_current_value("enabled", _enabled);
    set_current_value("decimation_factor", _decimation_factor);
}

std::vector<uint8_t> dds_decimation_filter::get_filter_data() const
{
    std::vector<uint8_t> data;
    data.push_back(_enabled ? 1 : 0);
    data.push_back(static_cast<uint8_t>(_decimation_factor));
    return data;
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
    , _delta(20.0f)
    , _persistence(3)
{
    _filter_type = embedded_filter_type::temporal;
}

void dds_temporal_filter::set_filter_data(const std::vector<uint8_t>& data)
{
    if (data.empty()) {
        throw std::invalid_argument("Empty filter data");
    }
    
    // Parse temporal filter data
    // Format: [enabled_flag, alpha, delta, persistence]
    _enabled = (data[0] != 0);
    
    if (data.size() >= 5) {
        // Assuming float values are packed as 4 bytes each
        std::memcpy(&_alpha, &data[1], sizeof(float));
    }
    
    if (data.size() >= 9) {
        std::memcpy(&_delta, &data[5], sizeof(float));
    }
    
    if (data.size() >= 10) {
        _persistence = static_cast<int>(data[9]);
    }
    
    // Update current values
    set_current_value("enabled", _enabled);
    set_current_value("alpha", _alpha);
    set_current_value("delta", _delta);
    set_current_value("persistence", _persistence);
}

std::vector<uint8_t> dds_temporal_filter::get_filter_data() const
{
    std::vector<uint8_t> data;
    data.push_back(_enabled ? 1 : 0);
    
    // Pack float values as bytes
    const uint8_t* alpha_bytes = reinterpret_cast<const uint8_t*>(&_alpha);
    data.insert(data.end(), alpha_bytes, alpha_bytes + sizeof(float));
    
    const uint8_t* delta_bytes = reinterpret_cast<const uint8_t*>(&_delta);
    data.insert(data.end(), delta_bytes, delta_bytes + sizeof(float));
    
    data.push_back(static_cast<uint8_t>(_persistence));
    
    return data;
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

void dds_temporal_filter::set_delta(float delta)
{
    if (delta < 0.0f) {
        throw std::invalid_argument("Delta must be non-negative");
    }
    _delta = delta;
    set_current_value("delta", delta);
}

void dds_temporal_filter::set_persistence(int persistence)
{
    if (persistence < 0) {
        throw std::invalid_argument("Persistence must be non-negative");
    }
    _persistence = persistence;
    set_current_value("persistence", persistence);
}

rsutils::json dds_temporal_filter::to_json() const
{
    auto props = props_to_json();
    props["enabled"] = _enabled;
    props["alpha"] = _alpha;
    props["delta"] = _delta;
    props["persistence"] = _persistence;
    return props;
}

// Factory and utility functions
std::shared_ptr<dds_embedded_filter> create_embedded_filter(embedded_filter_type type)
{
    switch (type) {
        case embedded_filter_type::decimation:
            return std::make_shared<dds_decimation_filter>();
        case embedded_filter_type::temporal:
            return std::make_shared<dds_temporal_filter>();
        default:
            throw std::invalid_argument("Unknown embedded filter type");
    }
}

std::string embedded_filter_type_to_string(embedded_filter_type type)
{
    switch (type) {
        case embedded_filter_type::decimation:
            return "decimation";
        case embedded_filter_type::temporal:
            return "temporal";
        default:
            return "unknown";
    }
}

embedded_filter_type embedded_filter_type_from_string(const std::string& type_str)
{
    if (type_str == "decimation") {
        return embedded_filter_type::decimation;
    } else if (type_str == "temporal") {
        return embedded_filter_type::temporal;
    } else {
        throw std::invalid_argument("Unknown embedded filter type: " + type_str);
    }
}


}  // namespace realdds
