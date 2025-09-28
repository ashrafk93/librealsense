// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#include <realdds/dds-embedded-filter.h>
#include <realdds/dds-device.h>
#include <realdds/dds-stream-base.h>
#include <realdds/dds-exceptions.h>

#include <rsutils/json.h>
#include <stdexcept>
#include <cstring>
#include <algorithm>  // for std::find_if

using rsutils::json;


namespace realdds {


// Base class implementation
dds_embedded_filter::dds_embedded_filter()
    : _name("")
    , _options()
    , _filter_type(RS2_EMBEDDED_FILTER_TYPE_FIRST)
    , _initialized(false)
{
}

void dds_embedded_filter::init(const std::string& name, rs2_embedded_filter_type type)
{
    verify_uninitialized();
    _name = name;
    _filter_type = type;
}

void dds_embedded_filter::init_options(const rsutils::json& options_j)
{
    if (!_options.empty())
        DDS_THROW(runtime_error, "filter '" + _name + "' options are already initialized");

    dds_options options;
    for (auto& option_j : options_j)
    {
        //LOG_DEBUG( "[" << debug_name() << "]     ... " << option_j );
        auto option = dds_option::from_json(option_j);
        options.push_back(option);
    }

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
    if (!_options.empty()) {
        props["options"] = dds_options_to_json(_options);
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

rsutils::json dds_embedded_filter::get_options_json()
{
    return dds_options_to_json(_options);
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
{
    _name = "Decimation Filter";
    _filter_type = RS2_EMBEDDED_FILTER_TYPE_DECIMATION;
}

void dds_decimation_filter::set_options(const rsutils::json& options)
{
    check_options(options);
    
    // Expect options to be an array of option objects with "name" and "value"
    if (options.is_array()) {
        for (auto& opt_json : options) {
            if (!opt_json.contains("name") || !opt_json.contains("value")) {
                DDS_THROW(runtime_error, "Option must contain 'name' and 'value' fields");
            }
            
            std::string opt_name = opt_json["name"].get<std::string>();
            auto found_option = std::find_if(_options.begin(), _options.end(),
                [&opt_name](const std::shared_ptr<dds_option>& option) {
                    return option->get_name() == opt_name;
                });
                
            if (found_option != _options.end()) {
                (*found_option)->set_value(opt_json["value"]);
            } else {
                DDS_THROW(runtime_error, "Option '" + opt_name + "' not found in filter");
            }
        }
    } else {
        DDS_THROW(runtime_error, "Options must be provided as an array");
    }
}

rsutils::json dds_decimation_filter::to_json() const
{
    auto props = props_to_json();
    // Add any decimation-specific properties here if needed
    return props;
}

// Temporal filter implementation
dds_temporal_filter::dds_temporal_filter()
    : dds_embedded_filter()
{
    _name = "Temporal Filter";
    _filter_type = RS2_EMBEDDED_FILTER_TYPE_TEMPORAL;
}

void dds_temporal_filter::set_options(const rsutils::json& options)
{
    check_options(options);
    
    // Expect options to be an array of option objects with "name" and "value"
    if (options.is_array()) {
        for (auto& opt_json : options) {
            if (!opt_json.contains("name") || !opt_json.contains("value")) {
                DDS_THROW(runtime_error, "Option must contain 'name' and 'value' fields");
            }
            
            std::string opt_name = opt_json["name"].get<std::string>();
            auto found_option = std::find_if(_options.begin(), _options.end(),
                [&opt_name](const std::shared_ptr<dds_option>& option) {
                    return option->get_name() == opt_name;
                });
                
            if (found_option != _options.end()) {
                (*found_option)->set_value(opt_json["value"]);
            } else {
                DDS_THROW(runtime_error, "Option '" + opt_name + "' not found in filter");
            }
        }
    } else {
        DDS_THROW(runtime_error, "Options must be provided as an array");
    }
}

rsutils::json dds_temporal_filter::to_json() const
{
    auto props = props_to_json();
    // Add any temporal-specific properties here if needed
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
