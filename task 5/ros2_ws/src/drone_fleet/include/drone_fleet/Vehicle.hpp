#pragma once
#include <string>
#include <vector>
#include "DroneExceptions.hpp"

class Vehicle {
public:
    explicit Vehicle(const std::string& name, float initial_battery = 100.0f);
    virtual ~Vehicle() = default;

    
    virtual std::string get_info() const = 0;

    
    void drain_battery(float amount);
    void charge_battery(float amount, int duration_seconds);
    bool is_critical() const;     // true when battery < 20 %

    
    std::string get_flight_log() const;

    
    void set_status(const std::string& new_status);

    
    const std::string& get_name()    const { return name_; }
    float              get_battery() const { return battery_level_; }
    const std::string& get_status()  const { return status_; }

protected:
    void log_event(const std::string& event);   // appends timestamped entry
    std::string timestamp_now() const;

    std::string name_;

private:
    float                    battery_level_;
    std::string              status_;
    std::vector<std::string> flight_log_;

    static const std::vector<std::string> VALID_STATES;
    static constexpr float CRITICAL_THRESHOLD = 20.0f;
};
