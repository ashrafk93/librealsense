// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.

#include <librealsense2/rs.hpp>

#include <iostream>
#include <string>
#include <thread>


rs2::device get_dds_device()
{
    // Create RealSense context
    rs2::context ctx;

    // Find devices
    auto devices = ctx.query_devices();
    if (devices.size() == 0)
    {
        std::cerr << "No RealSense devices found!" << std::endl;
		throw std::runtime_error("No RealSense devices found!");
    }

    // Find first DDS device
    rs2::device dev;
    for (auto&& d : devices)
    {
        if (strcmp(d.get_info(RS2_CAMERA_INFO_CONNECTION_TYPE), "DDS") == 0)
        {
            dev = d;
            break;
        }
    }
    return dev;
}

rs2::stream_profile get_depth_profile(rs2::depth_sensor depth_sensor, int nominal_width, int nominal_height)
{
    auto depth_profiles = depth_sensor.get_stream_profiles();
    rs2::stream_profile depth_profile;
    for (auto& p : depth_profiles)
    {
        if (p.format() == RS2_FORMAT_Z16 && p.fps() == 30)
        {
            auto vsp = p.as<rs2::video_stream_profile>();
            if (vsp.height() == nominal_height && vsp.width() == nominal_width)
            {
                depth_profile = p;
                break;
            }
        }
    }
    return depth_profile;
}


// scenario:
// get dds device, depth sensor
// set decimation filter OFF
// stream and check resolution is the nominal resolution
// set decimation filter ON
//stream and check resolution is the decimated resolution
int main( int argc, char * argv[] )
try
{
    std::cout << "RealSense DPP Decimation Filter Example" << std::endl;
    std::cout << "=========================================" << std::endl;

    // getting device
    auto dev = get_dds_device();
    if (!dev)
    {
        std::cerr << "No RealSense DDS devices found!" << std::endl;
        return EXIT_FAILURE;
    }
	std::cout << "Using device: " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
    
    // getting depth sensor
	auto depth_sensor = dev.first<rs2::depth_sensor>();
	if (!depth_sensor)
	{
		std::cerr << "Device has no depth sensor!" << std::endl;
		return EXIT_FAILURE;
	}

	// setting VGA resolution profile
    auto nominal_width = 1280;
	auto nominal_height = 720;
    auto depth_profile = get_depth_profile(depth_sensor, nominal_width, nominal_height);

    if (!depth_profile)
    {
        std::cerr << "No suitable depth profile found!" << std::endl;
        return EXIT_FAILURE;
	}

	// setting decimation filter OFF
	std::cout << "Setting decimation filter OFF" << std::endl;
    rs2::embedded_filter_sensor embed_filter_sensor = depth_sensor.as<rs2::embedded_filter_sensor>();
    if (!embed_filter_sensor.supports(RS2_EMBEDDED_FILTER_TYPE_DECIMATION))
    {
        throw std::runtime_error("Depth sensor does not support embedded decimation filter!");
    }
    std::vector<uint8_t> request;
    uint8_t on_off = 0; // OFF
    request.push_back(on_off);
    embed_filter_sensor.set_filter(RS2_EMBEDDED_FILTER_TYPE_DECIMATION, request);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    auto ans = embed_filter_sensor.get_filter(RS2_EMBEDDED_FILTER_TYPE_DECIMATION);
    if (ans.empty() || ans[0] != 0)
    {
        throw std::runtime_error("Decimation filter deactivation did not work!");
    }

	// streaming sensor and checking resolution is the nominal resolution
	depth_sensor.open(depth_profile);
	depth_sensor.start([&](rs2::frame f)
        {
            auto vsp = f.as<rs2::video_frame>();
            if (vsp)
            {
                auto frame_width = vsp.get_width();
                auto frame_height = vsp.get_height();
				std::cout << "Frame received: " << frame_width << "x" << frame_height << std::endl;
                if (frame_width != nominal_width || frame_height != nominal_height)
                {
                    std::cerr << "Error: Frame resolution does not match nominal resolution!" << std::endl;
                    exit(EXIT_FAILURE);
                }
            }
		});
	std::this_thread::sleep_for(std::chrono::seconds(3));
    depth_sensor.stop();
	depth_sensor.close();

	// setting decimation filter ON
	std::cout << "Setting decimation filter ON" << std::endl;
    request.clear();
    on_off = 1; // ON
    request.push_back(on_off);
    embed_filter_sensor.set_filter(RS2_EMBEDDED_FILTER_TYPE_DECIMATION, request);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ans = embed_filter_sensor.get_filter(RS2_EMBEDDED_FILTER_TYPE_DECIMATION);
    if (ans.empty() || ans[0] == 0)
    {
        throw std::runtime_error("Decimation filter activation did not work!");
    }

    // streaming sensor and checking resolution is the decimated resolution
	auto decimation_factor = 2;
	auto expected_decimation_width = nominal_width / decimation_factor;
	auto expected_decimation_height = nominal_height / decimation_factor;

    depth_sensor.open(depth_profile);
    depth_sensor.start([&](rs2::frame f)
        {
            auto vsp = f.as<rs2::video_frame>();
            if (vsp)
            {
                auto frame_width = vsp.get_width();
                auto frame_height = vsp.get_height();
                std::cout << "Frame received: " << frame_width << "x" << frame_height << std::endl;
                if (frame_width != expected_decimation_width || frame_height != expected_decimation_height)
                {
                    std::cerr << "Error: Frame resolution does not match decimated resolution!" << std::endl;
                    exit(EXIT_FAILURE);
                }
            }
        });
    std::this_thread::sleep_for(std::chrono::seconds(3));
    depth_sensor.stop();
    depth_sensor.close();

	std::cout << "Decimation filter test passed successfully." << std::endl;    

    return EXIT_SUCCESS;
}
catch( const rs2::error & e )
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    "
              << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch( const std::exception & e )
{
    std::cerr << "Error: " << e.what() << std::endl;
    return EXIT_FAILURE;
}
