#include "Vehicle.hpp"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <stdexcept>

// ─── Static member ────────────────────────────────────────────────────────────
const std::vector<std::string> Vehicle::VALID_STATES = {
    "idle", "flying", "charging", "emergency"
};

// ─── Constructor ──────────────────────────────────────────────────────────────
Vehicle::Vehicle(const std::string& name, float initial_battery)
    : name_(name),
      battery_level_(std::max(0.0f, std::min(100.0f, initial_battery))),
      status_("idle")
{
    log_event("Vehicle initialised with battery " +
              std::to_string(static_cast<int>(battery_level_)) + "%");
}

// ─── Status setter (validated + logged) ──────────────────────────────────────
void Vehicle::set_status(const std::string& new_status) {
    if (std::find(VALID_STATES.begin(), VALID_STATES.end(), new_status) ==
        VALID_STATES.end()) {
        throw InvalidStateError(name_, new_status, "idle|flying|charging|emergency");
    }
    log_event("Status changed: " + status_ + " → " + new_status);
    status_ = new_status;
}

// ─── Battery drain ────────────────────────────────────────────────────────────
void Vehicle::drain_battery(float amount) {
    if (battery_level_ <= 0.0f) {
        throw BatteryDepletedError(name_);
    }
    battery_level_ -= amount;
    if (battery_level_ < 0.0f) battery_level_ = 0.0f;
    log_event("Battery drained by " + std::to_string(amount) +
              "% → " + std::to_string(battery_level_) + "%");
}

// ─── Battery charge ───────────────────────────────────────────────────────────
void Vehicle::charge_battery(float amount, int duration_seconds) {
    if (status_ != "charging") {
        throw InvalidStateError(name_, status_, "charging");
    }
    battery_level_ += amount;
    if (battery_level_ > 100.0f) battery_level_ = 100.0f;
    log_event("Charged for " + std::to_string(duration_seconds) +
              "s → battery: " + std::to_string(battery_level_) + "%");
}

// ─── Critical check ───────────────────────────────────────────────────────────
bool Vehicle::is_critical() const {
    return battery_level_ < CRITICAL_THRESHOLD;
}

// ─── Flight log ───────────────────────────────────────────────────────────────
std::string Vehicle::get_flight_log() const {
    std::ostringstream oss;
    oss << "=== Flight Log for " << name_ << " (" << flight_log_.size() << " entries) ===\n";
    for (const auto& entry : flight_log_) {
        oss << "  " << entry << "\n";
    }
    return oss.str();
}

// ─── Protected helpers ────────────────────────────────────────────────────────
void Vehicle::log_event(const std::string& event) {
    flight_log_.push_back("[" + timestamp_now() + "] " + event);
}

std::string Vehicle::timestamp_now() const {
    using namespace std::chrono;
    auto now        = system_clock::now();
    auto time_t_now = system_clock::to_time_t(now);
    auto ms         = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

    std::ostringstream oss;
    oss << std::put_time(std::localtime(&time_t_now), "%H:%M:%S")
        << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}
