// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include <string>
#include "subdevice-model.h"
#include "embedded-filter-model.h"
#include "viewer.h"


namespace rs2
{
    embedded_filter_model::embedded_filter_model(
        subdevice_model* owner,
        const rs2_embedded_filter_type& type,
        std::shared_ptr<rs2::embedded_filter> filter,
        std::string& error_message, bool enable)
        : _owner(owner), _embedded_filter(filter), _enabled(enable)
    {
		_name = rs2_embedded_filter_type_to_string(type);

        std::stringstream ss;
        ss << "##" << ((owner) ? owner->dev.get_info(RS2_CAMERA_INFO_NAME) : _name)
            << "/" << ((owner) ? (*owner->s).get_info(RS2_CAMERA_INFO_NAME) : "_")
            << "/" << (long long)this;

        if (_owner)
            _full_name = get_embedded_filters_device_sensor_name(_owner) + "." + _name;
        else
            _full_name = _name;

        _enabled = restore_embedded_filter(_full_name.c_str(),
            filter, _enabled);

        populate_options(ss.str().c_str(), owner, owner ? &owner->_options_invalidated : nullptr, error_message);
    }

    void embedded_filter_model::save_to_config_file()
    {
        save_embedded_filter_to_config_file(_full_name.c_str(), _embedded_filter, _enabled);
    }

    void embedded_filter_model::draw_options( viewer_model & viewer,
                                               bool update_read_only_options,
                                               bool is_streaming,
                                               std::string & error_message )
    {
        for (auto& id_and_model : _options_id_to_model)
        {
            if( id_and_model.first == RS2_OPTION_EMBEDDED_FILTER_ENABLED )
                continue;

            id_and_model.second.draw_option( update_read_only_options, is_streaming, error_message, *viewer.not_model );
        }
    }

    void embedded_filter_model::embedded_filter_enable_disable(bool actual)
    {
		_embedded_filter->set_option(RS2_OPTION_EMBEDDED_FILTER_ENABLED, actual ? 1.0f : 0.0f);
		_enabled = _embedded_filter->get_option(RS2_OPTION_EMBEDDED_FILTER_ENABLED);
    }

    void embedded_filter_model::populate_options(const std::string& opt_base_label,
        subdevice_model* model,
        bool* options_invalidated,
        std::string& error_message)
    {
        for (option_value option : _embedded_filter->get_supported_option_values())
        {
            _options_id_to_model[option->id] = create_option_model( option,
                                                                opt_base_label,
                                                                model,
                                                                _embedded_filter,
                                                                model ? &model->_options_invalidated : nullptr,
                                                                error_message );
        }
    }

    bool restore_embedded_filter(const char* name, std::shared_ptr<rs2::embedded_filter> ef, bool enable)
    {
        for (auto opt : ef->get_supported_option_values())
        {
			// below continue is because the enabled status will be restored separately
			// right after this loop
            if (opt->id == RS2_OPTION_EMBEDDED_FILTER_ENABLED)
				continue;
            std::string key = name;
            key += ".";
            key += ef->get_option_name(opt->id);
            if (config_file::instance().contains(key.c_str()))
            {
                float val = config_file::instance().get(key.c_str());
                try
                {
                    auto range = ef->get_option_range(opt->id);
                    if (val >= range.min && val <= range.max)
                        ef->set_option(opt->id, val);
                }
                catch (...)
                {
                }
            }
        }

        std::string key = name;
        key += ".enabled";
        if (config_file::instance().contains(key.c_str()))
        {
            return config_file::instance().get(key.c_str());
        }
        return enable;
    }

    void save_embedded_filter_to_config_file(const char* name,
        std::shared_ptr<rs2::embedded_filter> ef, bool enable)
    {
        for (auto opt : ef->get_supported_options())
        {
			// below continue is because the enabled status will already be saved separately
			// right after this loop
            if (opt == RS2_OPTION_EMBEDDED_FILTER_ENABLED)
				continue;
            auto val = ef->get_option(opt);
            std::string key = name;
            key += ".";
            key += ef->get_option_name(opt);
            config_file::instance().set(key.c_str(), val);
        }

        std::string key = name;
        key += ".enabled";
        config_file::instance().set(key.c_str(), enable);
    }
}
