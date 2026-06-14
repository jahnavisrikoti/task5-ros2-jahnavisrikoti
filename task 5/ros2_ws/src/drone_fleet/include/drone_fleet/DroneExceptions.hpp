#pragma once
#include <stdexcept>
#include <string>

class DroneException : public std::runtime_error {
public:
    explicit DroneException(const std::string& msg) : std::runtime_error(msg) {}
};

class BatteryDepletedError : public DroneException {
public:
    explicit BatteryDepletedError(const std::string& name)
        : DroneException("[" + name + "] Battery is fully depleted — cannot drain further.") {}
};

class InvalidStateError : public DroneException {
public:
    InvalidStateError(const std::string& name, const std::string& currentState,
                      const std::string& requiredState)
        : DroneException("[" + name + "] Invalid state '" + currentState +
                         "' — required: '" + requiredState + "'.") {}
};

class AltitudeError : public DroneException {
public:
    AltitudeError(const std::string& name, float requested, float max)
        : DroneException("[" + name + "] Altitude " + std::to_string(requested) +
                         "m exceeds max " + std::to_string(max) + "m.") {}
};
