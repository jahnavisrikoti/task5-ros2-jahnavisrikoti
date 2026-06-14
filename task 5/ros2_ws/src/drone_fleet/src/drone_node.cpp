#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>

// Include the OOP drone classes
#include "drone_fleet/MissionDrone.hpp"
#include "drone_fleet/DroneExceptions.hpp"

using namespace std::chrono_literals;

// ─── Helper: current ISO timestamp ───────────────────────────────────────────
static std::string iso_now() {
    using namespace std::chrono;
    auto now        = system_clock::now();
    auto t          = system_clock::to_time_t(now);
    auto ms         = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&t), "%Y-%m-%dT%H:%M:%S")
        << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

// ─── Drone Node ───────────────────────────────────────────────────────────────
class DroneNode : public rclcpp::Node {
public:
    DroneNode() : rclcpp::Node("drone_node") {
        // ── Declare ROS 2 parameters ─────────────────────────────────────────
        declare_parameter<std::string>("drone_name",      "Alpha");
        declare_parameter<double>     ("initial_battery", 100.0);
        declare_parameter<std::string>("mission_name",    "DefaultMission");

        std::string name        = get_parameter("drone_name").as_string();
        double      init_bat    = get_parameter("initial_battery").as_double();
        std::string mission_nm  = get_parameter("mission_name").as_string();

        // ── Fixed 5 waypoints ────────────────────────────────────────────────
        std::vector<WP> waypoints = {
            {10.0f,  5.0f,  15.0f},
            {20.0f, 10.0f,  20.0f},
            {30.0f, 15.0f,  25.0f},
            {40.0f, 10.0f,  20.0f},
            {50.0f,  5.0f,  15.0f}
        };

        drone_ = std::make_unique<MissionDrone>(
            name, mission_nm, waypoints,
            static_cast<float>(init_bat), 120.0f, 5.0f);

        drone_name_ = name;
        publish_count_ = 0;

        // ── Publishers ────────────────────────────────────────────────────────
        pub_status_   = create_publisher<std_msgs::msg::String>(
            "/drone/" + name + "/status", 10);
        pub_alert_    = create_publisher<std_msgs::msg::String>(
            "/drone/" + name + "/alert", 10);
        pub_complete_ = create_publisher<std_msgs::msg::String>(
            "/drone/" + name + "/mission_complete", 10);
        pub_telemetry_= create_publisher<std_msgs::msg::String>(
            "/drone/" + name + "/telemetry", 10);

        // ── Take off to begin ─────────────────────────────────────────────────
        try {
            drone_->take_off(15.0f);
        } catch (const DroneException& e) {
            RCLCPP_WARN(get_logger(), "Take-off warning: %s", e.what());
        }

        // ── 1-second status timer ─────────────────────────────────────────────
        timer_status_ = create_wall_timer(
            1s, std::bind(&DroneNode::publish_status, this));

        // ── 2-second telemetry timer ──────────────────────────────────────────
        timer_telemetry_ = create_wall_timer(
            2s, std::bind(&DroneNode::publish_telemetry, this));

        RCLCPP_INFO(get_logger(), "DroneNode '%s' started — battery: %.1f%%",
                    name.c_str(), init_bat);
    }

private:
    // ── Status publish (every 1 s) ────────────────────────────────────────────
    void publish_status() {
        ++publish_count_;

        // Drain battery every publish
        try {
            drone_->drain_battery(0.5f);
        } catch (const BatteryDepletedError&) {
            RCLCPP_ERROR(get_logger(), "[%s] Battery depleted!", drone_name_.c_str());
            return;
        }

        // Advance waypoint every 3 publishes
        if (publish_count_ % 3 == 0 && !drone_->mission_complete()) {
            try {
                drone_->next_waypoint();
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "Waypoint error: %s", e.what());
            }
        }

        // Check critical
        if (drone_->is_critical()) {
            auto alert = std_msgs::msg::String();
            alert.data = "[ALERT][" + iso_now() + "] " + drone_name_ +
                         " battery critical: " +
                         std::to_string(drone_->get_battery()) + "%";
            pub_alert_->publish(alert);
            RCLCPP_WARN(get_logger(), "%s", alert.data.c_str());
            try { drone_->land(); } catch (...) {}
        }

        // Check mission complete
        if (drone_->mission_complete()) {
            auto msg = std_msgs::msg::String();
            msg.data = "[MISSION_COMPLETE][" + iso_now() + "] " + drone_name_ +
                       " finished mission '" + drone_->get_mission_name() + "'";
            pub_complete_->publish(msg);
            RCLCPP_INFO(get_logger(), "%s", msg.data.c_str());
            drone_->reset_mission();
        }

        // Build and publish status string
        // Format: "name:Alpha|battery:87.3|altitude:15.2|status:flying|waypoint:2/5|speed:5.0"
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1);
        oss << "name:"     << drone_->get_name()
            << "|battery:" << drone_->get_battery()
            << "|altitude:"<< drone_->get_altitude()
            << "|status:"  << drone_->get_status()
            << "|waypoint:"<< drone_->get_waypoint_index()
            << "/"         << drone_->get_waypoint_count()
            << "|speed:"   << drone_->get_speed();

        auto msg = std_msgs::msg::String();
        msg.data = oss.str();
        pub_status_->publish(msg);
    }

    // ── Telemetry publish (every 2 s) — hand-rolled JSON ─────────────────────
    void publish_telemetry() {
        std::ostringstream j;
        j << std::fixed << std::setprecision(2);
        j << "{"
          << "\"timestamp\":\"" << iso_now()                    << "\","
          << "\"name\":\""      << drone_->get_name()           << "\","
          << "\"battery\":"     << drone_->get_battery()        << ","
          << "\"altitude\":"    << drone_->get_altitude()       << ","
          << "\"status\":\""    << drone_->get_status()         << "\","
          << "\"waypoint\":"    << drone_->get_waypoint_index() << ","
          << "\"total_wp\":"    << drone_->get_waypoint_count() << ","
          << "\"speed\":"       << drone_->get_speed()          << ","
          << "\"critical\":"    << (drone_->is_critical() ? "true" : "false") << ","
          << "\"mission\":\""   << drone_->get_mission_name()   << "\""
          << "}";

        auto msg = std_msgs::msg::String();
        msg.data = j.str();
        pub_telemetry_->publish(msg);
    }

    // ── Members ───────────────────────────────────────────────────────────────
    std::unique_ptr<MissionDrone>                          drone_;
    std::string                                            drone_name_;
    int                                                    publish_count_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    pub_status_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    pub_alert_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    pub_complete_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    pub_telemetry_;

    rclcpp::TimerBase::SharedPtr                           timer_status_;
    rclcpp::TimerBase::SharedPtr                           timer_telemetry_;
};

// ─── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneNode>());
    rclcpp::shutdown();
    return 0;
}
