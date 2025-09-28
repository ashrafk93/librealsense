// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#include "rs-dds-embedded-temporal-filter.h"
#include <stdexcept>
#include <string>
#include <cstring>
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

    void rs_dds_embedded_temporal_filter::enable_filter(rs2_embedded_filter_type embedded_filter_type, bool enable)
    {
        // Enable/disable temporal filter via DDS
        // This is equivalent to setting the "Toggle" option to 1 (ON)
        auto toggle_opt = get_dds_option_by_name(_dds_temporal_filter->get_options(), "Toggle");
        int32_t value_to_set = enable ? 1 : 0;
        auto toggle_opt_j = toggle_opt->to_json();
        toggle_opt_j["value"] = value_to_set;
        // below line should call the set option callback defined in add_option method
        toggle_opt->set_value(toggle_opt_j);
    }

    void rs_dds_embedded_temporal_filter::add_option(std::shared_ptr< realdds::dds_option > option)
    {
        bool const ok_if_there = true;
        auto option_id = options_registry::register_option_by_name(option->get_name(), ok_if_there);

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
                _dds_temporal_filter->set_options(all_options_json);
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
        rsutils::json json_to_send = _dds_temporal_filter->get_options_json();

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
		// TODO check ranges of each parameter
        if (options_j.size() != 4) {
            throw std::invalid_argument("Four parameters are expected for Temporal filter (enabled + alpha + delta + persistency)");
        }
        for (auto& opt_j : options_j)
        {
            if (!opt_j.contains("name"))
            {
                throw std::runtime_error("Option json does not contain name!");
            }
            if (opt_j["name"] == "Toggle")
            {
                int32_t toggle_val = opt_j["value"].get<int32_t>();
                if (toggle_val != 0 && toggle_val != 1)
                {
                    throw std::runtime_error("Toggle shall be 0 for OFF or 1 for ON");
                }
            }
            else if (opt_j["name"] == "Alpha")
            {
                float alpha_val = opt_j["value"].get<float>();
            }
            else if (opt_j["name"] == "Delta")
            {
                int32_t delta_val = opt_j["value"].get<int32_t>();
            }
            else if (opt_j["name"] == "Persistency")
            {
                int32_t persistency_val = opt_j["value"].get<int32_t>();
            }
            else
            {
                // we should not get here
                throw std::runtime_error("The expected paramaters for Temporal filter are toggle, alpha, delta and persistency");
            }
        }
        // Validation passed - parameters are valid
    }
}  // namespace librealsense
