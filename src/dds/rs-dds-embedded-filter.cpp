// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#include <third-party/realdds/include/realdds/dds-embedded-filter.h>
#include "rs-dds-embedded-filter.h" 

using rsutils::json;

namespace librealsense {


/*static option_range range_from_realdds(std::shared_ptr< realdds::dds_option > const& dds_opt)
{
    if( dds_opt->get_minimum_value().is_number() && dds_opt->get_maximum_value().is_number()
        && dds_opt->get_stepping().is_number() )
    {
        return { dds_opt->get_minimum_value(),
                 dds_opt->get_maximum_value(),
                 dds_opt->get_stepping(),
                 dds_opt->get_default_value() };
    }
    if( std::dynamic_pointer_cast< realdds::dds_boolean_option >( dds_opt ) )
    {
        return { 0, 1, 1, bool( dds_opt->get_default_value() ) ? 1.f : 0.f };
    }
    if( auto e = std::dynamic_pointer_cast< realdds::dds_enum_option >( dds_opt ) )
    {
        return { 0, float( e->get_choices().size() - 1 ), 1, (float)e->get_value_index( e->get_default_value() ) };
    }
    return { 0, 0, 0, 0 };
}


static rs2_option_type rs_type_from_dds_option( std::shared_ptr< realdds::dds_option > const & dds_opt )
{
    if( std::dynamic_pointer_cast< realdds::dds_float_option >( dds_opt ) )
        return RS2_OPTION_TYPE_FLOAT;
    if( std::dynamic_pointer_cast< realdds::dds_string_option >( dds_opt ) )
        return RS2_OPTION_TYPE_STRING;
    if( std::dynamic_pointer_cast< realdds::dds_boolean_option >( dds_opt ) )
        return RS2_OPTION_TYPE_BOOLEAN;
    if( std::dynamic_pointer_cast< realdds::dds_integer_option >( dds_opt ) )
        return RS2_OPTION_TYPE_INTEGER;
    if( std::dynamic_pointer_cast< realdds::dds_rect_option >( dds_opt ) )
        return RS2_OPTION_TYPE_RECT;
    throw not_implemented_exception( "unknown DDS option type" );
}*/


    rs_dds_embedded_filter::rs_dds_embedded_filter(const std::shared_ptr< realdds::dds_embedded_filter >& dds_embedded_filter,
    set_embedded_filter_callback set_ef_cb,
    query_embedded_filter_callback query_ef_cb)
    : embedded_filter_sensor_interface()
    , _dds_ef(dds_embedded_filter)
	, _filter_type(dds_embedded_filter->get_filter_type())
    , _set_ef_cb( set_ef_cb )
    , _query_ef_cb( query_ef_cb )
{
}

    std::vector<rs2_option> rs_dds_embedded_filter::get_filter_supported_options(rs2_embedded_filter_type embedded_filter_type) const
    {
        return get_supported_options();
    }

    option& rs_dds_embedded_filter::get_filter_option(rs2_option option_id, rs2_embedded_filter_type embedded_filter_type) const
    {
        return const_cast<option&>(get_option(option_id)); // will throw if option not found
    }

}  // namespace librealsense
