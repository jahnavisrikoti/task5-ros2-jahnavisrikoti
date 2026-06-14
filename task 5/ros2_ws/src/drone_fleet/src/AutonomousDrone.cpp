#include "AutonomousDrone.hpp"
#include <sstream>
#include <iomanip>
#include <cmath>
#include <stdexcept>

// ─── Constructor ──────────────────────────────────────────────────────────────
AutonomousDrone::AutonomousDrone(const std::string& name,
                                 const std::string& mission_name,
                                 const std::vector<WP>& waypoints,
                                 const WP& home_position,
                                 float initial_battery,
                                 float max_altitude,
                                 float speed)
    : MissionDrone(name, mission_name, waypoints, initial_battery, max_altitude, speed),
      ai_mode_("manual"),
      home_position_(home_position)
{}

// ─── Set AI mode ─────────────────────────────────────────────────────────────
void AutonomousDrone::set_ai_mode(const std::string& mode) {
    if (mode != "manual" && mode != "auto" && mode != "return_home") {
        throw std::invalid_argument("[" + name_ + "] Unknown AI mode: " + mode);
    }
    ai_mode_ = mode;
    log_event("AI mode set to: " + mode);

    if (mode == "return_home") {
        // Insert home_position as the very next waypoint
        waypoints_.insert(waypoints_.begin() + current_waypoint_index_, home_position_);
        log_event("Home position inserted as next waypoint.");
    }
}

// ─── Detect obstacle ─────────────────────────────────────────────────────────
void AutonomousDrone::detect_obstacle(const WP& position, const std::string& severity) {
    std::ostringstream entry;
    entry << "[" << timestamp_now() << "] OBSTACLE (" << severity << ") at ("
          << std::get<0>(position) << ","
          << std::get<1>(position) << ","
          << std::get<2>(position) << ")";
    obstacle_log_.push_back(entry.str());
    log_event("Obstacle detected — severity: " + severity);

    if (severity == "high") {
        log_event("High severity! Calling emergency_stop().");
        emergency_stop();
    }
}

// ─── Auto-replan (avoid within 5 units) ──────────────────────────────────────
std::vector<WP> AutonomousDrone::auto_replan(const std::vector<WP>& obstacles) {
    std::vector<WP> safe_waypoints;
    for (const auto& wp : waypoints_) {
        bool blocked = false;
        for (const auto& obs : obstacles) {
            if (euclidean_distance(wp, obs) < 5.0f) {
                blocked = true;
                log_event("Waypoint (" + std::to_string(std::get<0>(wp)) + "," +
                          std::to_string(std::get<1>(wp)) + "," +
                          std::to_string(std::get<2>(wp)) + ") blocked by obstacle.");
                break;
            }
        }
        if (!blocked) safe_waypoints.push_back(wp);
    }
    log_event("Replanned: kept " + std::to_string(safe_waypoints.size()) +
              "/" + std::to_string(waypoints_.size()) + " waypoints.");
    return safe_waypoints;
}

// ─── get_info ─────────────────────────────────────────────────────────────────
std::string AutonomousDrone::get_info() const {
    std::ostringstream oss;
    oss << MissionDrone::get_info() << "\n";
    oss << "  [Autonomous] Mode: " << ai_mode_
        << " | Home: (" << std::get<0>(home_position_) << ","
                        << std::get<1>(home_position_) << ","
                        << std::get<2>(home_position_) << ")"
        << " | Obstacles logged: " << obstacle_log_.size();
    return oss.str();
}

// ─── Helpers ─────────────────────────────────────────────────────────────────
float AutonomousDrone::euclidean_distance(const WP& a, const WP& b) const {
    float dx = std::get<0>(a) - std::get<0>(b);
    float dy = std::get<1>(a) - std::get<1>(b);
    float dz = std::get<2>(a) - std::get<2>(b);
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}
