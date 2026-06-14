#include "Drone.hpp"
#include <sstream>
#include <iomanip>

// ─── Constructor ──────────────────────────────────────────────────────────────
Drone::Drone(const std::string& name,
             float initial_battery,
             float max_altitude,
             float speed)
    : Vehicle(name, initial_battery),
      altitude_(0.0f),
      max_altitude_(max_altitude),
      speed_(speed)
{}

// ─── Take off ─────────────────────────────────────────────────────────────────
void Drone::take_off(float target_altitude) {
    if (target_altitude > max_altitude_) {
        throw AltitudeError(name_, target_altitude, max_altitude_);
    }
    if (get_status() == "flying") {
        // Already airborne — adjust altitude
        altitude_ = target_altitude;
        log_event("Altitude adjusted to " + std::to_string(altitude_) + "m");
        return;
    }
    set_status("flying");
    altitude_ = target_altitude;
    drain_battery(2.0f);   // takeoff cost
    log_event("Took off to " + std::to_string(altitude_) + "m");
}

// ─── Land ─────────────────────────────────────────────────────────────────────
void Drone::land() {
    if (get_status() != "flying") return;
    altitude_ = 0.0f;
    set_status("idle");
    log_event("Landed successfully.");
}

// ─── Emergency stop ───────────────────────────────────────────────────────────
void Drone::emergency_stop() {
    log_event("⚠ EMERGENCY STOP triggered — draining 30% battery as penalty.");
    set_status("emergency");
    try {
        drain_battery(30.0f);
    } catch (const BatteryDepletedError&) {
        // Battery already zero — absorb gracefully
        log_event("Battery already depleted during emergency stop.");
    }
    altitude_ = 0.0f;
    log_event("Emergency stop complete. Drone grounded.");
}

// ─── get_info ─────────────────────────────────────────────────────────────────
std::string Drone::get_info() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1);
    oss << "[Drone] Name: "         << get_name()
        << " | Battery: "           << get_battery()   << "%"
        << " | Status: "            << get_status()
        << " | Altitude: "          << altitude_        << "m"
        << " | MaxAlt: "            << max_altitude_    << "m"
        << " | Speed: "             << speed_           << "m/s"
        << " | Critical: "          << (is_critical() ? "YES" : "no");
    return oss.str();
}
