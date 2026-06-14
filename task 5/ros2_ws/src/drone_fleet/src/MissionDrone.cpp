#include "MissionDrone.hpp"
#include <sstream>
#include <iomanip>
#include <stdexcept>


MissionDrone::MissionDrone(const std::string& name,
                           const std::string& mission_name,
                           const std::vector<WP>& waypoints,
                           float initial_battery,
                           float max_altitude,
                           float speed)
    : Drone(name, initial_battery, max_altitude, speed),
      mission_name_(mission_name),
      waypoints_(waypoints),
      current_waypoint_index_(0)
{}


WP MissionDrone::next_waypoint() {
    if (mission_complete()) {
        throw std::out_of_range("[" + name_ + "] Mission already complete — no next waypoint.");
    }
    WP current = waypoints_[current_waypoint_index_];
    drain_battery(1.5f);

    
    visited_waypoints_.emplace_back(current, timestamp_now());

    log_event("Visiting waypoint " + std::to_string(current_waypoint_index_ + 1) +
              "/" + std::to_string(waypoints_.size()) +
              " (" + std::to_string(std::get<0>(current)) + ", " +
                     std::to_string(std::get<1>(current)) + ", " +
                     std::to_string(std::get<2>(current)) + ")");
    ++current_waypoint_index_;
    return current;
}


void MissionDrone::skip_waypoint(const std::string& reason) {
    if (mission_complete()) return;
    WP skipped = waypoints_[current_waypoint_index_];
    log_event("Skipping waypoint " + std::to_string(current_waypoint_index_ + 1) +
              " — reason: " + reason);
    visited_waypoints_.emplace_back(skipped, "SKIPPED@" + timestamp_now());
    ++current_waypoint_index_;
}


bool MissionDrone::mission_complete() const {
    return current_waypoint_index_ >= static_cast<int>(waypoints_.size());
}


void MissionDrone::reset_mission() {
    current_waypoint_index_ = 0;
    visited_waypoints_.clear();
    log_event("Mission '" + mission_name_ + "' restarted.");
}


std::string MissionDrone::mission_summary() const {
    std::ostringstream oss;
    oss << "\n╔══════════════════════════════════════════╗\n";
    oss << "║      MISSION SUMMARY: " << mission_name_ << "\n";
    oss << "╠══════════════════════════════════════════╣\n";
    oss << "║ Drone     : " << name_                               << "\n";
    oss << "║ Battery   : " << std::fixed << std::setprecision(1)
                            << get_battery()                        << "%\n";
    oss << "║ Waypoints : " << current_waypoint_index_
                            << "/" << waypoints_.size()             << "\n";
    oss << "║ Complete  : " << (mission_complete() ? "YES" : "NO") << "\n";
    oss << "╠══════════════════════════════════════════╣\n";
    for (size_t i = 0; i < visited_waypoints_.size(); ++i) {
        const auto& [wp, ts] = visited_waypoints_[i];
        oss << "║ WP" << (i + 1) << " (" << std::get<0>(wp) << ","
                                          << std::get<1>(wp) << ","
                                          << std::get<2>(wp) << ") @ " << ts << "\n";
    }
    oss << "╚══════════════════════════════════════════╝\n";
    return oss.str();
}


std::string MissionDrone::get_info() const {
    std::ostringstream oss;
    oss << Drone::get_info() << "\n";
    oss << "  [Mission] Name: " << mission_name_
        << " | WP: " << current_waypoint_index_
        << "/" << waypoints_.size()
        << " | Done: " << (mission_complete() ? "YES" : "NO");
    return oss.str();
}
