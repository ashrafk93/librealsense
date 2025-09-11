// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2025 RealSense, Inc. All Rights Reserved.
#pragma once

#include <rsutils/json.h>
#include <rsutils/type/ip-address.h>

#include <memory>
#include <map>
#include <vector>
#include <string>


namespace realdds {


// Forward declarations
enum class embedded_filter_type {
    decimation,
    temporal
};

// Abstract base class for all embedded filters.
// Embedded filter types are Decimation and Temporal filter
class dds_embedded_filter
{
protected:
    std::string _name;
    rsutils::json _options;
    rsutils::json _stream_type;
    embedded_filter_type _filter_type;
    std::map<std::string, rsutils::json> _current_values;
    bool _initialized;

public:
    dds_embedded_filter();
    virtual ~dds_embedded_filter() = default;

    // Initialization functions - must be called before first set_value()
    virtual void init(const std::string& name, embedded_filter_type type);
    virtual void init_options(const rsutils::json& options);
    virtual void init_stream_type(const rsutils::json& stream_type);
    virtual void init_default_values(const rsutils::json& defaults);

    // Core functionality
    virtual void set_filter_data(const std::vector<uint8_t>& data) = 0;
    virtual std::vector<uint8_t> get_filter_data() const = 0;
    virtual bool supports_filter() const = 0;

    // Getters
    const std::string& get_name() const { return _name; }
    rsutils::json const& get_options() const { return _options; }
    rsutils::json const& get_stream_type() const { return _stream_type; }
    embedded_filter_type get_filter_type() const { return _filter_type; }
    bool is_initialized() const { return _initialized; }

    // JSON serialization
    virtual rsutils::json to_json() const;
    static std::shared_ptr<dds_embedded_filter> from_json(const rsutils::json& j);

protected:
    void verify_uninitialized() const;  // throws if already has a value (use by init_ functions)
    virtual rsutils::json props_to_json() const;
    
    // Helper methods for derived classes
    void set_current_value(const std::string& key, const rsutils::json& value);
    rsutils::json get_current_value(const std::string& key) const;
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
    void set_filter_data(const std::vector<uint8_t>& data) override;
    std::vector<uint8_t> get_filter_data() const override;
    bool supports_filter() const override { return true; }

    // Decimation-specific methods
    void set_enabled(bool enabled);
    bool is_enabled() const { return _enabled; }
    
    void set_decimation_factor(int factor);
    int get_decimation_factor() const { return _decimation_factor; }

    rsutils::json to_json() const override;
};

// Temporal filter implementation  
class dds_temporal_filter : public dds_embedded_filter
{
private:
    bool _enabled;
    float _alpha;
    float _delta;
    int _persistence;

public:
    dds_temporal_filter();
    virtual ~dds_temporal_filter() = default;

    // Override base class methods
    void set_filter_data(const std::vector<uint8_t>& data) override;
    std::vector<uint8_t> get_filter_data() const override;
    bool supports_filter() const override { return true; }

    // Temporal-specific methods
    void set_enabled(bool enabled);
    bool is_enabled() const { return _enabled; }
    
    void set_alpha(float alpha);
    float get_alpha() const { return _alpha; }
    
    void set_delta(float delta);
    float get_delta() const { return _delta; }
    
    void set_persistence(int persistence);
    int get_persistence() const { return _persistence; }

    rsutils::json to_json() const override;
};

typedef std::vector<std::shared_ptr<dds_embedded_filter>> dds_embedded_filters;

// Factory function to create appropriate filter based on type
std::shared_ptr<dds_embedded_filter> create_embedded_filter(embedded_filter_type type);

// Utility functions
std::string embedded_filter_type_to_string(embedded_filter_type type);
embedded_filter_type embedded_filter_type_from_string(const std::string& type_str);

}  // namespace realdds
