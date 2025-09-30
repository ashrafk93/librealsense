// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#include "rs-dds-embedded-decimation-filter.h"
#include <stdexcept>
#include <string>
#include <cstring>
#include <algorithm>
#include <rsutils/json.h>
#include <src/core/options-registry.h>
#include <realdds/dds-option.h>
#include "rs-dds-option.h"

using rsutils::json;

namespace librealsense {

    rs_dds_embedded_decimation_filter::rs_dds_embedded_decimation_filter(const std::shared_ptr< realdds::dds_embedded_filter >& dds_embedded_filter,
        set_embedded_filter_callback set_embedded_filter_cb,
        query_embedded_filter_callback query_embedded_filter_cb)
		: rs_dds_embedded_filter(dds_embedded_filter, set_embedded_filter_cb, query_embedded_filter_cb)
		, _enabled(false)
		, _magnitude(DECIMATION_MAGNITUDE)
    {
        // Initialize options by calling add_option for each DDS option
        for (auto& dds_option : _dds_ef->get_options())
        {
            add_option(dds_option);
        }
	}

    void rs_dds_embedded_decimation_filter::enable(bool enable)
    {
        // Enable/disable decimation filter via DDS
        // This is equivalent to setting the "Toggle" option to 1 (ON)
        auto toggle_opt = get_dds_option_by_name(_dds_ef->get_options(), "Toggle");
        if (toggle_opt) {
            int32_t value_to_set = enable ? 1 : 0;
            auto toggle_opt_j = toggle_opt->to_json();
            toggle_opt_j["value"] = value_to_set;
            // below line should call the set option callback defined in add_option method
            toggle_opt->set_value(toggle_opt_j["value"]);
        }
    }

    // Helper function to find an option by name in a list of DDS options
    std::shared_ptr<realdds::dds_option> rs_dds_embedded_decimation_filter::get_dds_option_by_name(
        const std::vector<std::shared_ptr<realdds::dds_option>>& options,
        const std::string& name)
    {
        auto it = std::find_if(options.begin(), options.end(),
            [&name](const std::shared_ptr<realdds::dds_option>& option) {
                return option->get_name() == name;
            });
        return (it != options.end()) ? *it : nullptr;
    }

    void rs_dds_embedded_decimation_filter::add_option(std::shared_ptr< realdds::dds_option > option)
    {
        bool const ok_if_there = true;
        rs2_option option_id;
        
        // Map DDS option names to standard RealSense option IDs
        if (option->get_name() == "Magnitude")
        {
            option_id = RS2_OPTION_FILTER_MAGNITUDE;
        }
        else
        {
            // For other options (like "Toggle"), use the registry to create a dynamic option ID
            option_id = options_registry::register_option_by_name(option->get_name(), ok_if_there);
        }

        if (!is_valid(option_id))
        {
            LOG_ERROR("Option '" << option->get_name() << "' not found");
            throw librealsense::invalid_value_exception("Option '" + option->get_name() + "' not found");
        }

        if (get_option_handler(option_id))
            throw std::runtime_error("option '" + option->get_name() + "' already exists in sensor");

        // In below implementation: 
        // - setting one option leads to 
        //   * setting the new value for one option, and
        //   * sending also the other current values for the other filter's values
        // - getting one option leads to:
        //   * returning only the relevant option's value
        //   * the getting of the filter's options communicating with the device by DDS
        //     is not necessary, since the value is already automatically updated by the set action reply 
        auto opt = std::make_shared< rs_dds_option >(
            option,
            [=](json value) // set_option cb for the filter's options
            {
                // prepare json with all options
                json all_options_json = prepare_all_options_json(value);
                // validate values
                validate_filter_options(all_options_json);
                // set updated options to the remote device
                _set_ef_cb(all_options_json);

                // Delegate to DDS filter
                _dds_ef->set_options(all_options_json);
            },
            [=]() -> json // get_option cb for the filter's options
            {
                return option->get_value();
            });
        register_option(option_id, opt);
        _options_watcher.register_option(option_id, opt);
    }

    rsutils::json rs_dds_embedded_decimation_filter::prepare_all_options_json(const rsutils::json& new_value)
    {
        rsutils::json json_to_send = _dds_ef->get_options_json();
        
        for (auto& opt_j : json_to_send)
        {
            if (!opt_j.contains("name") || !new_value.contains("name"))
            {
                throw std::runtime_error("option json does not contain name");
            }
            if (opt_j["name"] == new_value["name"])
            {
                opt_j["value"] = new_value["value"];
            }
        }
        return json_to_send;
    }

    void rs_dds_embedded_decimation_filter::validate_filter_options(rsutils::json options_j)
    {
        // Check expected number of parameters
        if (options_j.size() != 2) {
            throw std::invalid_argument("Two parameters are expected for Decimation filter (enabled + magnitude)");
        }
        
        for (auto& opt_j : options_j)
        {
            if (!opt_j.contains("name"))
            {
                throw std::runtime_error("Option json does not contain name!");
            }
            
            std::string option_name = opt_j["name"].get<std::string>();
            
            // Find the corresponding DDS option to get its range
            auto dds_option = get_dds_option_by_name(_dds_ef->get_options(), option_name);
            if (!dds_option)
            {
                throw std::runtime_error("Option '" + option_name + "' not found in DDS filter options");
            }
            
            if (option_name == "Toggle")
            {
                int32_t toggle_val = opt_j["value"].get<int32_t>();
                
                // Check range using DDS option
                if (!dds_option->get_minimum_value().is_null() && toggle_val < dds_option->get_minimum_value().get<int32_t>())
                {
                    throw std::invalid_argument("Toggle value " + std::to_string(toggle_val) + 
                                              " is below minimum " + std::to_string(dds_option->get_minimum_value().get<int32_t>()));
                }
                if (!dds_option->get_maximum_value().is_null() && toggle_val > dds_option->get_maximum_value().get<int32_t>())
                {
                    throw std::invalid_argument("Toggle value " + std::to_string(toggle_val) + 
                                              " is above maximum " + std::to_string(dds_option->get_maximum_value().get<int32_t>()));
                }
                
                // Additional validation: Toggle should be 0 or 1
                if (toggle_val != 0 && toggle_val != 1)
                {
                    throw std::runtime_error("Toggle shall be 0 for OFF or 1 for ON");
                }
            }
            else if (option_name == "Magnitude")
            {
                int32_t mag_val = opt_j["value"].get<int32_t>();
                
                // Check range using DDS option
                if (!dds_option->get_minimum_value().is_null() && mag_val < dds_option->get_minimum_value().get<int32_t>())
                {
                    throw std::invalid_argument("Magnitude value " + std::to_string(mag_val) + 
                                              " is below minimum " + std::to_string(dds_option->get_minimum_value().get<int32_t>()));
                }
                if (!dds_option->get_maximum_value().is_null() && mag_val > dds_option->get_maximum_value().get<int32_t>())
                {
                    throw std::invalid_argument("Magnitude value " + std::to_string(mag_val) + 
                                              " is above maximum " + std::to_string(dds_option->get_maximum_value().get<int32_t>()));
                }
                
                // Additional validation: Decimation magnitude must be exactly 2 for depth sensor
                if (mag_val != DECIMATION_MAGNITUDE) {
                    throw std::invalid_argument("Decimation filter magnitude must be " + std::to_string(DECIMATION_MAGNITUDE) + ". Received: " + std::to_string(mag_val));
                }
            }
            else
            {
                // we should not get here
                throw std::runtime_error("The expected parameters for Decimation filter are toggle and magnitude");
            }
        }
        // Validation passed - parameters are valid
    }
}  // namespace librealsense
