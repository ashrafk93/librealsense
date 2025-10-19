// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#pragma once

#include <librealsense2/rs.hpp>
#include <string>


namespace rs2
{
    class subdevice_model;
    class option_model;
    class viewer_model;

    class embedded_filter_model
    {
    public:
        embedded_filter_model( subdevice_model* owner,
            const rs2_embedded_filter_type& type,
            std::shared_ptr<rs2::embedded_filter> filter,
            std::string& error_message,
            bool enabled = true );
        virtual ~embedded_filter_model() = default;

        const std::string& get_name() const { return _name; }

        void save_to_config_file();

        void populate_options( const std::string& opt_base_label,
            subdevice_model* model,
            bool* options_invalidated,
            std::string& error_message );

        void draw_options( viewer_model & viewer,
                           bool update_read_only_options,
                           bool is_streaming,
                           std::string & error_message );

        std::shared_ptr<rs2::embedded_filter> get_filter() { return _embedded_filter; }

        void enable( bool e = true )
        {
            embedded_filter_enable_disable( _enabled = e );
        }
        bool is_enabled() const { return _enabled; }

        bool visible = true;

        void embedded_filter_enable_disable(bool actual);

    protected:
        bool _enabled = true;
        std::shared_ptr<rs2::embedded_filter> _embedded_filter;
        std::map< rs2_option, option_model > _options_id_to_model;
        std::string _name;
        std::string _full_name;
        subdevice_model* _owner;
    };

    bool restore_embedded_filter(const char* name,
        std::shared_ptr<rs2::embedded_filter> ef, bool enable);

    void save_embedded_filter_to_config_file(const char* name,
        std::shared_ptr<rs2::embedded_filter> pb, bool enable = true);
}
