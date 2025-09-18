// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.
#pragma once

#include <rsutils/json.h>
#include <rsutils/type/ip-address.h>
#include <include/librealsense2/h/rs_types.h>

#include <memory>
#include <map>
#include <vector>
#include <string>


namespace realdds {

class dds_device;
class dds_stream_base;

// Class dds_embedded_filter - Handles DDS communication, JSON serialization, stream association
// Abstract base class for all embedded filters.
// Embedded filter types are Decimation and Temporal filter
class dds_embedded_filter : public std::enable_shared_from_this< dds_embedded_filter >
{
protected:
    std::string _name;
    rsutils::json _options;
    rs2_embedded_filter_type _filter_type;
    std::map< std::string, rsutils::json > _current_values;
    bool _initialized;
    std::weak_ptr< dds_device > _dev;

private:
    friend class dds_stream_base;
    std::weak_ptr< dds_stream_base > _stream;
    void init_stream( std::shared_ptr< dds_stream_base > const & );

public:
    dds_embedded_filter();

    // Initialization functions - must be called before first set_value()
    virtual void init( const std::string & name, rs2_embedded_filter_type type );
    virtual void init_options( rsutils::json const & options );
    virtual void init_default_values( rsutils::json const & defaults );

    // Core functionality
    virtual void set_options(rsutils::json const& options) = 0;
    virtual rsutils::json get_options() = 0;
    virtual bool supports_filter() const = 0;
    virtual bool is_enabled() const = 0;

    // Getters
    std::string const & get_name() const { return _name; }
    rs2_embedded_filter_type get_filter_type() const { return _filter_type; }
    bool is_initialized() const { return _initialized; }
    std::shared_ptr< dds_stream_base > get_stream() const { return _stream.lock(); }

    // JSON serialization
    virtual rsutils::json to_json() const;
    static std::shared_ptr< dds_embedded_filter > from_json( rsutils::json const & j, const std::string& stream_name);

protected:
    void verify_uninitialized() const;  // throws if already has a value (use by init_ functions)
    virtual rsutils::json props_to_json() const;

    // Helper methods for derived classes
    void set_current_value( std::string const & key, rsutils::json const & value );
    rsutils::json get_current_value( std::string const & key ) const;
    void check_options( rsutils::json const & options ) const;
};

// Decimation filter implementation
class dds_decimation_filter : public dds_embedded_filter
{
private:
    bool _enabled;
    int _decimation_factor;

public:
    dds_decimation_filter();
    virtual ~dds_decimation_filter() = default;

    // Override base class methods
    void set_options(rsutils::json const& options) override;
    rsutils::json get_options() override;
    bool supports_filter() const override { return true; }

    // Decimation-specific methods
    void set_enabled( bool enabled );
    bool is_enabled() const { return _enabled; }

    void set_decimation_factor( int factor );
    int get_decimation_factor() const { return _decimation_factor; }

    rsutils::json to_json() const override;
};

// Temporal filter implementation
class dds_temporal_filter : public dds_embedded_filter
{
private:
    bool _enabled;
    float _alpha;
    int32_t _delta;
    int32_t _persistency;

public:
    dds_temporal_filter();
    virtual ~dds_temporal_filter() = default;

    // Override base class methods
    void set_options(rsutils::json const& options) override;
    rsutils::json get_options() override;
    bool supports_filter() const override { return true; }

    // Temporal-specific methods
    void set_enabled( bool enabled );
    bool is_enabled() const { return _enabled; }

    void set_alpha( float alpha );
    float get_alpha() const { return _alpha; }

    void set_delta( int32_t delta );
    int32_t get_delta() const { return _delta; }

    void set_persistency( int32_t persistency );
    int32_t get_persistency() const { return _persistency; }

    rsutils::json to_json() const override;
};

typedef std::vector< std::shared_ptr< dds_embedded_filter > > dds_embedded_filters;

// Factory function to create appropriate filter based on type
std::shared_ptr< dds_embedded_filter > create_embedded_filter( rs2_embedded_filter_type type );

// Utility functions
std::string embedded_filter_type_to_string( rs2_embedded_filter_type type );
rs2_embedded_filter_type embedded_filter_type_from_string( std::string const & type_str );

}  // namespace realdds
