// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#include "rs-dds-embedded-temporal-filter.h"
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

	const float rs_dds_embedded_temporal_filter::ALPHA_DEFAULT = 0.4f;
	const int32_t rs_dds_embedded_temporal_filter::DELTA_DEFAULT = 20;
	const int32_t rs_dds_embedded_temporal_filter::PERSISTENCY_DEFAULT = 3;

    rs_dds_embedded_temporal_filter::rs_dds_embedded_temporal_filter(const std::shared_ptr< realdds::dds_embedded_filter >& dds_embedded_filter,
        set_embedded_filter_callback set_embedded_filter_cb,
        query_embedded_filter_callback query_embedded_filter_cb)
		: rs_dds_embedded_filter(dds_embedded_filter, set_embedded_filter_cb, query_embedded_filter_cb)
		, _enabled(false)
		, _alpha(ALPHA_DEFAULT)
		, _delta(DELTA_DEFAULT)
		, _persistency(PERSISTENCY_DEFAULT)
    {
        // Initialize options by calling add_option for each DDS option
        for (auto& dds_option : _dds_ef->get_options())
        {
            add_option(dds_option);
        }
    }

    void rs_dds_embedded_temporal_filter::enable(bool enable)
    {
        // Enable/disable temporal filter via DDS
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
    std::shared_ptr<realdds::dds_option> rs_dds_embedded_temporal_filter::get_dds_option_by_name(
        const std::vector<std::shared_ptr<realdds::dds_option>>& options, 
        const std::string& name)
    {
        auto it = std::find_if(options.begin(), options.end(),
            [&name](const std::shared_ptr<realdds::dds_option>& option) {
                return option->get_name() == name;
            });
        return (it != options.end()) ? *it : nullptr;
    }

    void rs_dds_embedded_temporal_filter::add_option(std::shared_ptr< realdds::dds_option > option)
    {
        bool const ok_if_there = true;
        rs2_option option_id;
        
        // Map DDS option names to standard RealSense option IDs
        if (option->get_name() == "Alpha")
        {
            option_id = RS2_OPTION_FILTER_SMOOTH_ALPHA;
        }
        else if (option->get_name() == "Delta")
        {
            option_id = RS2_OPTION_FILTER_SMOOTH_DELTA;
        }
        else if (option->get_name() == "Persistency")
        {
            option_id = RS2_OPTION_HOLES_FILL;
        }
        else
        {
            // For other options (like "Toggle", "Persistency"), use the registry to create a dynamic option ID
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

    rsutils::json rs_dds_embedded_temporal_filter::prepare_all_options_json(const rsutils::json& new_value)
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

    void rs_dds_embedded_temporal_filter::validate_filter_options(rsutils::json options_j)
    {
        // Check expected number of parameters
        if (options_j.size() != 4) {
            throw std::invalid_argument("Four parameters are expected for Temporal filter (enabled + alpha + delta + persistency)");
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
            else if (option_name == "Alpha")
            {
                float alpha_val = opt_j["value"].get<float>();
                
                // Check range using DDS option
                if (!dds_option->get_minimum_value().is_null() && alpha_val < dds_option->get_minimum_value().get<float>())
                {
                    throw std::invalid_argument("Alpha value " + std::to_string(alpha_val) + 
                                              " is below minimum " + std::to_string(dds_option->get_minimum_value().get<float>()));
                }
                if (!dds_option->get_maximum_value().is_null() && alpha_val > dds_option->get_maximum_value().get<float>())
                {
                    throw std::invalid_argument("Alpha value " + std::to_string(alpha_val) + 
                                              " is above maximum " + std::to_string(dds_option->get_maximum_value().get<float>()));
                }
            }
            else if (option_name == "Delta")
            {
                int32_t delta_val = opt_j["value"].get<int32_t>();
                
                // Check range using DDS option
                if (!dds_option->get_minimum_value().is_null() && delta_val < dds_option->get_minimum_value().get<int32_t>())
                {
                    throw std::invalid_argument("Delta value " + std::to_string(delta_val) + 
                                              " is below minimum " + std::to_string(dds_option->get_minimum_value().get<int32_t>()));
                }
                if (!dds_option->get_maximum_value().is_null() && delta_val > dds_option->get_maximum_value().get<int32_t>())
                {
                    throw std::invalid_argument("Delta value " + std::to_string(delta_val) + 
                                              " is above maximum " + std::to_string(dds_option->get_maximum_value().get<int32_t>()));
                }
            }
            else if (option_name == "Persistency")
            {
                int32_t persistency_val = opt_j["value"].get<int32_t>();
                
                // Check range using DDS option
                if (!dds_option->get_minimum_value().is_null() && persistency_val < dds_option->get_minimum_value().get<int32_t>())
                {
                    throw std::invalid_argument("Persistency value " + std::to_string(persistency_val) + 
                                              " is below minimum " + std::to_string(dds_option->get_minimum_value().get<int32_t>()));
                }
                if (!dds_option->get_maximum_value().is_null() && persistency_val > dds_option->get_maximum_value().get<int32_t>())
                {
                    throw std::invalid_argument("Persistency value " + std::to_string(persistency_val) + 
                                              " is above maximum " + std::to_string(dds_option->get_maximum_value().get<int32_t>()));
                }
            }
            else
            {
                // we should not get here
                throw std::runtime_error("The expected parameters for Temporal filter are toggle, alpha, delta and persistency");
            }
        }
        // Validation passed - parameters are valid
    }
}  // namespace librealsense
