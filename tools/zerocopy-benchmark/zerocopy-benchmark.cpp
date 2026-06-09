// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.
//
// rs-zerocopy-benchmark
// ---------------------
// Compares the CUDA zero-copy frame path against the existing CUDA path and the CPU baseline.
// Build the SDK three ways and run this against each (see build-matrix.sh):
//     off       BUILD_WITH_CUDA=OFF                              (CPU/SIMD baseline)
//     cuda      BUILD_WITH_CUDA=ON  BUILD_WITH_CUDA_ZEROCOPY=OFF (current CUDA path)
//     zerocopy  BUILD_WITH_CUDA=ON  BUILD_WITH_CUDA_ZEROCOPY=ON  (zero-copy; engages on integrated GPU)
//
// Two modes:
//   (default)   depth processing -- runs rs2::align + rs2::pointcloud; reports per-call time,
//               effective FPS, process CPU%, peak RSS.
//   --rgb-nn    RGB -> GPU        -- models feeding a color frame to a GPU consumer (NN). Per
//               frame, compares an explicit cudaMemcpy upload vs the public get_gpu_data() API;
//               reports H2D + kernel time and CPU%.
//
// Timing: depth mode uses host wall-clock around the (SDK-owned) processing calls; --rgb-nn uses
// cudaEvents for the H2D/kernel it owns. First 30 frames excluded as warmup in both modes.
// Dependency-light: only librealsense2 (+ CUDA for --rgb-nn).
//
// Usage:
//   rs-zerocopy-benchmark [--seconds N] [--width W --height H] [--no-align] [--no-pointcloud]
//   rs-zerocopy-benchmark --rgb-nn [--seconds N] [--width W --height H]

#include <librealsense2/rs.hpp>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <algorithm>
#include <vector>
#include <cmath>

#ifndef _WIN32
#include <sys/resource.h>  // getrusage — process CPU time (the key zero-copy metric on Jetson)
#endif

#include "rgb-nn.h"  // CUDA stand-in kernel + upload helper for --rgb-nn (no-op unless CUDA build)

namespace {

struct stat_accum
{
    double sum = 0, min = 1e30, max = 0;
    int n = 0;
    void add( double ms )
    {
        sum += ms;
        min = std::min( min, ms );
        max = std::max( max, ms );
        ++n;
    }
    double avg() const { return n ? sum / n : 0.0; }
    void print( const char * label ) const
    {
        if( n )
            printf( "  %-22s avg=%7.3f  min=%7.3f  max=%7.3f ms  (n=%d)\n",
                    label, avg(), min, max, n );
        else
            printf( "  %-22s (not run)\n", label );
    }
};

double now_ms()
{
    using namespace std::chrono;
    return duration< double, std::milli >( steady_clock::now().time_since_epoch() ).count();
}

// Total process CPU time (user+sys) in ms. Differenced over the timed window, divided by
// wall time, this gives CPU% relative to one core — the metric where zero-copy's eliminated
// capture/host<->device copies actually show up on Jetson (lower CPU at the same FPS).
double cpu_ms()
{
#ifndef _WIN32
    struct rusage ru;
    getrusage( RUSAGE_SELF, &ru );
    auto tv = []( const timeval & t ) { return t.tv_sec * 1000.0 + t.tv_usec / 1000.0; };
    return tv( ru.ru_utime ) + tv( ru.ru_stime );
#else
    return 0.0;
#endif
}

double peak_rss_mb()
{
#ifndef _WIN32
    struct rusage ru;
    getrusage( RUSAGE_SELF, &ru );
    return ru.ru_maxrss / 1024.0;  // ru_maxrss is KB on Linux
#else
    return 0.0;
#endif
}

// --rgb-nn: model the customer use case (RGB frame consumed on the GPU by a NN) and compare
// the two ways to get the frame onto the GPU:
//   upload  - cudaMemcpy(host -> device) then the preprocess kernel        (today's path)
//   direct  - get_gpu_data() then the same kernel on the frame in place    (zero-copy path)
// Runs each as its own phase so each gets a clean CPU% (getrusage over its window). The delta
// is the per-frame host->device upload that zero-copy removes -- and it scales with cameras.
#ifdef RGBNN_HAVE_CUDA
int run_rgb_nn( int seconds, int width, int height )
{
    rs2::pipeline pipe;
    rs2::config cfg;
    if( width && height )
        cfg.enable_stream( RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, 30 );
    else
        cfg.enable_stream( RS2_STREAM_COLOR, RS2_FORMAT_RGB8 );
    auto prof = pipe.start( cfg );

    auto vsp = prof.get_stream( RS2_STREAM_COLOR ).as< rs2::video_stream_profile >();
    const int w = vsp.width(), h = vsp.height();
    const std::size_t bytes = (std::size_t)w * h * 3;
    printf( "  mode    : RGB -> GPU (feed a color frame to a GPU kernel)\n" );
    printf( "  device  : %s\n", prof.get_device().get_info( RS2_CAMERA_INFO_NAME ) );
    printf( "  config  : color %dx%d RGB8 (%zu bytes/frame), %ds/method, warmup 30 frames excluded\n",
            w, h, bytes, seconds );
    printf( "  timing  : cudaEvent (GPU-side) for H2D + kernel; total = H2D + kernel\n" );

    void *  d_in  = rgbnn::dev_alloc( bytes );        // upload target
    float * d_out = rgbnn::dev_alloc_float( (std::size_t)w * h );  // preprocess output
    if( ! d_in || ! d_out ) { printf( "device alloc failed\n" ); return 1; }

    struct phase_result { double cpu_pct = 0, total_ms = 0, h2d_ms = 0, kernel_ms = 0, fps = 0; int n = 0; bool gpu_used = false; };

    // 0 = H2D-copy (always manual cudaMemcpy), 1 = get_gpu_data (strict + manual fallback),
    // 2 = get_gpu_data_or_upload (SDK gives a device ptr, uploading internally if needed).
    enum { M_H2D_COPY = 0, M_GET_GPU_DATA = 1, M_OR_UPLOAD = 2 };

    auto run_phase = [&]( int method ) -> phase_result {
        phase_result r;
        stat_accum total, h2d, kern;
        const int warmup = 30;
        int idx = 0;
        double cpu0 = 0, wall0 = 0;
        bool started = false;
        const double t_end = now_ms() + seconds * 1000.0;
        while( now_ms() < t_end )
        {
            rs2::frameset fs;
            if( ! pipe.try_wait_for_frames( &fs, 1000 ) ) continue;
            auto color = fs.get_color_frame();
            if( ! color ) continue;
            ++idx;
            const bool timed = idx > warmup;
            if( timed && ! started ) { cpu0 = cpu_ms(); wall0 = now_ms(); started = true; }

            double this_h2d = 0;
            const unsigned char * rgb_dev = nullptr;
            if( method == M_GET_GPU_DATA )
            {
                rgb_dev = static_cast< const unsigned char * >( color.get_gpu_data() );
                if( rgb_dev ) r.gpu_used = true;
            }
            else if( method == M_OR_UPLOAD )
            {
                // The SDK returns a device ptr, uploading internally if there's no zero-copy.
                // Its internal upload can't be event-timed from here, so host-time the call when
                // it copies (slightly jittery, but it's the real API-call cost). Zero-copy -> 0.
                bool copied = false;
                double c0 = now_ms();
                rgb_dev = static_cast< const unsigned char * >( color.get_gpu_data_or_upload( &copied ) );
                double c1 = now_ms();
                if( copied ) this_h2d = c1 - c0;
                else if( rgb_dev ) r.gpu_used = true;
            }
            if( ! rgb_dev )  // M_H2D_COPY, or a method returned null -> manual upload
            {
                this_h2d = rgbnn::upload_ms( d_in, color.get_data(), bytes );  // cudaEvent-timed
                rgb_dev = static_cast< const unsigned char * >( d_in );
            }
            // cudaEvent-timed kernel (GPU-side, immune to host scheduling jitter); also syncs.
            double kms = rgbnn::preprocess( rgb_dev, d_out, w, h );

            if( timed )
            {
                if( this_h2d > 0 ) h2d.add( this_h2d );
                kern.add( kms );
                total.add( this_h2d + kms );
            }
        }
        if( started )
        {
            double wall = now_ms() - wall0;
            r.cpu_pct = wall > 0 ? ( cpu_ms() - cpu0 ) / wall * 100.0 : 0;
            r.fps = wall > 0 ? total.n / ( wall / 1000.0 ) : 0;
        }
        r.total_ms = total.avg();
        r.h2d_ms = h2d.avg();
        r.kernel_ms = kern.avg();
        r.n = total.n;
        return r;
    };

    printf( "\n-- method 1: H2D-copy (manual cudaMemcpy host->device + kernel) --\n" );
    phase_result up = run_phase( M_H2D_COPY );
    printf( "-- method 2: get_gpu_data (zero-copy; null -> manual upload) + kernel --\n" );
    phase_result dir = run_phase( M_GET_GPU_DATA );
    pipe.stop();
    rgbnn::dev_free( d_in );
    rgbnn::dev_free( d_out );

    // Both rows measured on THIS build. "H2D-copy" is the baseline (how you'd feed the GPU today,
    // without the API); "get_gpu_data" is the zero-copy API. The difference is the saving.
    // Numbers are AVERAGES per frame over the run; total = H2D + kernel (the GPU op reading the
    // frame -- the uploaded buffer for H2D-copy, the mapped frame in place for get_gpu_data).
    printf( "\n=== feed RGB to GPU: H2D-copy vs get_gpu_data (avg per frame, %dx%d) ===\n", w, h );
    printf( "  H2D-copy: %d frames | get_gpu_data: %d frames | ~%ds each | 30-frame warmup excluded\n",
            up.n, dir.n, seconds );
    printf( "  %-14s | %-12s | %-12s | %-12s | %-8s | %-8s\n", "method", "total(ms)", "H2D(ms)", "kernel(ms)", "FPS", "CPU%" );
    printf( "  ---------------+--------------+--------------+--------------+----------+--------\n" );
    printf( "  %-14s | %-12.3f | %-12.3f | %-12.3f | %-8.1f | %-6.1f\n", "H2D-copy",
            up.total_ms,  up.h2d_ms,  up.kernel_ms,  up.fps,  up.cpu_pct );
    printf( "  %-14s | %-12.3f | %-12.3f | %-12.3f | %-8.1f | %-6.1f%s\n", "get_gpu_data",
            dir.total_ms, dir.h2d_ms, dir.kernel_ms, dir.fps, dir.cpu_pct,
            dir.gpu_used ? "" : "  (no GPU ptr -> fell back to H2D-copy!)" );
    if( dir.gpu_used )
    {
        printf( "\n  H2D saved/frame : %.3f ms   (the host->device upload get_gpu_data removes)\n", up.h2d_ms );
        printf( "  CPU saved       : %.1f%% -> %.1f%% of 1 core\n", up.cpu_pct, dir.cpu_pct );
        printf( "  bandwidth saved : %.1f MB/s/camera at %.0f fps\n", bytes * up.fps / 1e6, up.fps );
        printf( "  (multiply by N cameras for the aggregate host CPU + bus saving)\n" );
    }
    return 0;
}
#endif  // RGBNN_HAVE_CUDA

}  // namespace

int main( int argc, char ** argv )
{
    int seconds = 20;
    int width = 0, height = 0;  // 0 -> let the SDK pick a default profile
    bool do_align = true;
    bool do_pointcloud = true;
    bool rgb_nn = false;

    for( int i = 1; i < argc; ++i )
    {
        std::string a = argv[i];
        if( a == "--seconds" && i + 1 < argc )            seconds = std::atoi( argv[++i] );
        else if( a == "--no-align" )                      do_align = false;
        else if( a == "--no-pointcloud" )                 do_pointcloud = false;
        else if( a == "--width" && i + 1 < argc )         width = std::atoi( argv[++i] );
        else if( a == "--height" && i + 1 < argc )        height = std::atoi( argv[++i] );
        else if( a == "--rgb-nn" )                        rgb_nn = true;
        else { printf( "unknown/!! arg: %s\n", a.c_str() ); }
    }

    printf( "============================================================\n" );
    printf( "  rs-zerocopy-benchmark   (librealsense %s)\n", RS2_API_VERSION_STR );
    printf( "============================================================\n" );

    if( rgb_nn )
    {
#ifdef RGBNN_HAVE_CUDA
        if( ! rgbnn::cuda_available() )
        {
            printf( "--rgb-nn: no CUDA device available at runtime.\n" );
            return 1;
        }
        try { return run_rgb_nn( seconds, width, height ); }
        catch( const rs2::error & e )
        {
            fprintf( stderr, "RealSense error in %s(%s): %s\n",
                     e.get_failed_function().c_str(), e.get_failed_args().c_str(), e.what() );
            return 1;
        }
        catch( const std::exception & e ) { fprintf( stderr, "error: %s\n", e.what() ); return 1; }
#else
        printf( "--rgb-nn requires a CUDA build (BUILD_WITH_CUDA=ON). Rebuild with CUDA.\n" );
        return 1;
#endif
    }

    try
    {
        rs2::pipeline pipe;
        rs2::config cfg;
        if( width && height )
        {
            cfg.enable_stream( RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30 );
            cfg.enable_stream( RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, 30 );
        }
        else
        {
            cfg.enable_stream( RS2_STREAM_DEPTH );
            cfg.enable_stream( RS2_STREAM_COLOR );
        }

        rs2::pipeline_profile prof = pipe.start( cfg );
        auto dev = prof.get_device();

        char res_str[32];
        if( width && height ) snprintf( res_str, sizeof( res_str ), "%dx%d", width, height );
        else                  snprintf( res_str, sizeof( res_str ), "device default" );

        printf( "  mode    : depth processing (align->color, pointcloud)\n" );
        printf( "  device  : %s (fw %s)\n",
                dev.get_info( RS2_CAMERA_INFO_NAME ),
                dev.supports( RS2_CAMERA_INFO_FIRMWARE_VERSION )
                    ? dev.get_info( RS2_CAMERA_INFO_FIRMWARE_VERSION ) : "?" );
        printf( "  config  : depth+color %s, %ds, warmup 30 frames excluded\n", res_str, seconds );
        printf( "  timing  : wall-clock around each SDK call (align.process / pointcloud.calculate)\n" );

        rs2::pointcloud pc;
        rs2::align align_to_color( RS2_STREAM_COLOR );

        stat_accum st_align, st_pc, st_interval;

        const double t_end = now_ms() + seconds * 1000.0;
        double last_frame_ms = 0;
        int frame_idx = 0;
        const int warmup = 30;  // exclude warmup frames (alloc, AWB, one-time CUDA init/spikes)

        // CPU% is sampled over the timed window only (started when warmup completes).
        double cpu_t0 = 0, wall_t0 = 0, cpu_pct = 0;
        bool cpu_started = false;

        while( now_ms() < t_end )
        {
            rs2::frameset fs;
            if( ! pipe.try_wait_for_frames( &fs, 1000 ) )
                continue;

            ++frame_idx;
            const bool timed = ( frame_idx > warmup );

            if( timed && ! cpu_started )
            {
                cpu_t0 = cpu_ms();
                wall_t0 = now_ms();
                cpu_started = true;
            }

            double t_now = now_ms();
            if( timed && last_frame_ms > 0 )
                st_interval.add( t_now - last_frame_ms );
            last_frame_ms = t_now;

            if( do_align )
            {
                double t0 = now_ms();
                rs2::frameset aligned = align_to_color.process( fs );
                double t1 = now_ms();
                if( timed )
                    st_align.add( t1 - t0 );
                fs = aligned;  // feed aligned depth into the point cloud
            }

            if( do_pointcloud )
            {
                auto depth = fs.get_depth_frame();
                auto color = fs.get_color_frame();
                if( depth )
                {
                    if( color )
                        pc.map_to( color );
                    double t0 = now_ms();
                    rs2::points pts = pc.calculate( depth );
                    double t1 = now_ms();
                    if( timed )
                        st_pc.add( t1 - t0 );
                    (void)pts;
                }
            }
        }

        if( cpu_started )
        {
            double wall = now_ms() - wall_t0;
            if( wall > 0 )
                cpu_pct = ( cpu_ms() - cpu_t0 ) / wall * 100.0;
        }

        pipe.stop();

        auto row = []( const char * name, const stat_accum & s ) {
            if( s.n )
                printf( "  %-22s | %8.3f | %8.3f | %8.3f | %6d\n", name, s.avg(), s.min, s.max, s.n );
            else
                printf( "  %-22s | %s\n", name, "(not run)" );
        };
        printf( "\n  per-call timing (ms), warmup excluded\n" );
        printf( "  %-22s | %8s | %8s | %8s | %6s\n", "block", "avg", "min", "max", "n" );
        printf( "  -----------------------+----------+----------+----------+-------\n" );
        row( "align.process()", st_align );
        row( "pointcloud.calculate()", st_pc );
        row( "frame interval", st_interval );
        printf( "\n  effective FPS : %.2f\n", st_interval.n ? 1000.0 / st_interval.avg() : 0.0 );
        printf( "  process CPU   : %.1f%% of 1 core\n", cpu_pct );
        printf( "  peak RSS      : %.1f MB\n", peak_rss_mb() );
        return 0;
    }
    catch( const rs2::error & e )
    {
        fprintf( stderr, "RealSense error in %s(%s): %s\n",
                 e.get_failed_function().c_str(), e.get_failed_args().c_str(), e.what() );
        return 1;
    }
    catch( const std::exception & e )
    {
        fprintf( stderr, "error: %s\n", e.what() );
        return 1;
    }
}
