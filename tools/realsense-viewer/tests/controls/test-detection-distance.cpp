// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#include "viewer-test-helpers.h"


// Walk the object-detection sensor's Detection Distance combo through every source and check that the
// overlay source follows and the camera's control holds what the source needs (on only for Camera)
VIEWER_TEST( "controls", "detection_distance_source" )
{
    auto & model = test.find_first_device_or_exit();

    struct step
    {
        const char * label;
        rs2::detection_distance_source source;
        float control_value;
    };
    step const steps[] = { { "Off", rs2::detection_distance_source::none, 0.f },
                           { "Application", rs2::detection_distance_source::application, 0.f },
                           { "Camera", rs2::detection_distance_source::camera, 1.f },
                           { "Application", rs2::detection_distance_source::application, 0.f } };

    for( auto && sub : model.subdevices )
    {
        if( ! test.has_option( sub, RS2_OPTION_DETECTION_DISTANCE ) || ! sub->detected_objects )
            continue;   // only object-detection sensors with the control (D585 FW 7.58.40802+)

        // Start from the camera's own state
        float const initial = sub->s->get_option( RS2_OPTION_DETECTION_DISTANCE );
        IM_CHECK( sub->detected_objects->distance_source
                  == ( initial > 0.f ? rs2::detection_distance_source::camera : rs2::detection_distance_source::none ) );

        test.expand_sensor_panel( model, sub );
        for( auto const & s : steps )
        {
            test.select_control_combo_item( model, sub, RS2_OPTION_DETECTION_DISTANCE, s.label );
            IM_CHECK( sub->detected_objects->distance_source == s.source );
            IM_CHECK( test.wait_until( 10, 0.5f, [&] {
                return sub->s->get_option( RS2_OPTION_DETECTION_DISTANCE ) == s.control_value;
            } ) );
        }

        // Leave the camera as we found it
        test.select_control_combo_item( model, sub, RS2_OPTION_DETECTION_DISTANCE, initial > 0.f ? "Camera" : "Off" );
        test.collapse_sensor_panel( model, sub );
    }
}
