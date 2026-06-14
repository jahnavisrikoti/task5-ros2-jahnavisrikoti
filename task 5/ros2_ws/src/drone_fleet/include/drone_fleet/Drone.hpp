#pragma once
#include "Vehicle.hpp"

class Drone : public Vehicle {
public:
    Drone(const std::string& name,
          float initial_battery  = 100.0f,
          float max_altitude     = 120.0f,
          float speed            = 5.0f);
    virtual ~Drone() = default;

    virtual void take_off(float target_altitude);
    virtual void land();
    virtual void emergency_stop();   

    virtual std::string get_info() const override;

    
    float get_altitude()     const { return altitude_; }
    float get_max_altitude() const { return max_altitude_; }
    float get_speed()        const { return speed_; }

protected:
    float altitude_;
    float max_altitude_;

private:
    float speed_;
};
