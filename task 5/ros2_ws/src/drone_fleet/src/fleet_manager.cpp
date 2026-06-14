#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <map>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <functional>

using namespace std::chrono_literals;
using String  = std_msgs::msg::String;
using Trigger = std_srvs::srv::Trigger;

// ─── Per-drone state parsed from telemetry JSON ───────────────────────────────
struct DroneState {
    std::string name       = "—";
    double      battery    = 0.0;
    double      altitude   = 0.0;
    std::string status     = "unknown";
    int         waypoint   = 0;
    int         total_wp   = 0;
    double      speed      = 0.0;
    bool        critical   = false;
    std::string timestamp  = "—";
    bool        received   = false;
};

// ─── Minimal hand-rolled JSON parser ─────────────────────────────────────────
// Extracts string/number fields by key from a flat JSON object.
static std::string json_str(const std::string& json, const std::string& key) {
    // Handles: "key":"value"
    std::string pattern = "\"" + key + "\":\"";
    auto pos = json.find(pattern);
    if (pos == std::string::npos) return "";
    pos += pattern.size();
    auto end = json.find('"', pos);
    if (end == std::string::npos) return "";
    return json.substr(pos, end - pos);
}

static double json_num(const std::string& json, const std::string& key) {
    // Handles: "key":123.45
    std::string pattern = "\"" + key + "\":";
    auto pos = json.find(pattern);
    if (pos == std::string::npos) return 0.0;
    pos += pattern.size();
    if (pos < json.size() && json[pos] == '"') return 0.0; // it's a string
    try { return std::stod(json.substr(pos)); }
    catch (...) { return 0.0; }
}

static bool json_bool(const std::string& json, const std::string& key) {
    std::string pattern = "\"" + key + "\":";
    auto pos = json.find(pattern);
    if (pos == std::string::npos) return false;
    pos += pattern.size();
    return json.substr(pos, 4) == "true";
}

static std::string iso_now() {
    using namespace std::chrono;
    auto now = system_clock::now();
    auto t   = system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&t), "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

// ─── Fleet Manager Node ───────────────────────────────────────────────────────
class FleetManager : public rclcpp::Node {
public:
    FleetManager() : rclcpp::Node("fleet_manager") {
        const std::vector<std::string> drones = {"Alpha", "Beta", "Gamma"};

        for (const auto& d : drones) {
            states_[d] = DroneState{};
            states_[d].name = d;

            // Status subscriber (/drone/<name>/status)
            subs_status_.push_back(
                create_subscription<String>(
                    "/drone/" + d + "/status", 10,
                    [this, d](const String::SharedPtr msg) {
                        parse_status(d, msg->data);
                    }));

            // Alert subscriber
            subs_alert_.push_back(
                create_subscription<String>(
                    "/drone/" + d + "/alert", 10,
                    [this, d](const String::SharedPtr msg) {
                        RCLCPP_WARN(get_logger(),
                            "[%s] ⚠ ALERT @ %s: %s",
                            d.c_str(), iso_now().c_str(), msg->data.c_str());
                    }));

            // Mission complete subscriber
            subs_complete_.push_back(
                create_subscription<String>(
                    "/drone/" + d + "/mission_complete", 10,
                    [this, d](const String::SharedPtr msg) {
                        RCLCPP_INFO(get_logger(),
                            "[%s] ✓ MISSION COMPLETE: %s",
                            d.c_str(), msg->data.c_str());
                    }));

            // Telemetry subscriber
            subs_telemetry_.push_back(
                create_subscription<String>(
                    "/drone/" + d + "/telemetry", 10,
                    [this, d](const String::SharedPtr msg) {
                        parse_telemetry(d, msg->data);
                    }));
        }

        // ── Fleet report timer (every 5 s) ────────────────────────────────────
        timer_report_ = create_wall_timer(
            5s, std::bind(&FleetManager::print_fleet_report, this));

        // ── Service /fleet/status_report ──────────────────────────────────────
        srv_report_ = create_service<Trigger>(
            "/fleet/status_report",
            [this](const Trigger::Request::SharedPtr /*req*/,
                         Trigger::Response::SharedPtr res) {
                print_fleet_report();
                res->success = true;
                res->message = "Fleet report printed to console.";
            });

        RCLCPP_INFO(get_logger(), "FleetManager online — monitoring Alpha, Beta, Gamma.");
    }

private:
    // ── Parse pipe-delimited status string ───────────────────────────────────
    void parse_status(const std::string& drone_name, const std::string& data) {
        // Format: "name:Alpha|battery:87.3|altitude:15.2|status:flying|waypoint:2/5|speed:3.2"
        auto& s = states_[drone_name];
        s.received = true;

        auto extract = [&](const std::string& key) -> std::string {
            std::string prefix = key + ":";
            auto pos = data.find(prefix);
            if (pos == std::string::npos) return "";
            pos += prefix.size();
            auto end = data.find('|', pos);
            return end == std::string::npos ? data.substr(pos) : data.substr(pos, end - pos);
        };

        s.status   = extract("status");
        s.altitude = std::stod(extract("altitude").empty() ? "0" : extract("altitude"));
        s.battery  = std::stod(extract("battery").empty()  ? "0" : extract("battery"));
        s.speed    = std::stod(extract("speed").empty()    ? "0" : extract("speed"));

        // waypoint: "2/5"
        std::string wp = extract("waypoint");
        auto slash = wp.find('/');
        if (slash != std::string::npos) {
            try { s.waypoint  = std::stoi(wp.substr(0, slash)); } catch (...) {}
            try { s.total_wp  = std::stoi(wp.substr(slash + 1)); } catch (...) {}
        }
    }

    // ── Parse telemetry JSON ──────────────────────────────────────────────────
    void parse_telemetry(const std::string& drone_name, const std::string& json) {
        auto& s    = states_[drone_name];
        s.received = true;
        s.battery  = json_num(json, "battery");
        s.altitude = json_num(json, "altitude");
        s.status   = json_str(json, "status");
        s.waypoint = static_cast<int>(json_num(json, "waypoint"));
        s.total_wp = static_cast<int>(json_num(json, "total_wp"));
        s.speed    = json_num(json, "speed");
        s.critical = json_bool(json, "critical");
        s.timestamp= json_str(json, "timestamp");
    }

    // ── Print formatted fleet report ──────────────────────────────────────────
    void print_fleet_report() {
        std::ostringstream oss;
        oss << "\n╔══════════════════════════════════════════════════════════════╗\n";
        oss << "║           FLEET STATUS REPORT — " << iso_now() << "         ║\n";
        oss << "╠═══════╦═══════════╦═══════════╦═════════════╦═════════════╣\n";
        oss << "║ Drone ║ Battery % ║ Altitude  ║ Waypoint    ║ Status      ║\n";
        oss << "╠═══════╬═══════════╬═══════════╬═════════════╬═════════════╣\n";

        for (const auto& [name, s] : states_) {
            if (!s.received) {
                oss << "║ " << std::left << std::setw(6) << name
                    << "║  (no data yet)                                       ║\n";
                continue;
            }
            std::string wp_str = std::to_string(s.waypoint) + "/" + std::to_string(s.total_wp);
            std::string bat_str = std::to_string(static_cast<int>(s.battery)) + "%"
                                  + (s.critical ? " ⚠" : "  ");
            oss << "║ " << std::left  << std::setw(6)  << name    << "║ "
                        << std::right << std::setw(8)  << bat_str << "  ║ "
                        << std::right << std::setw(7)  << std::fixed << std::setprecision(1)
                                                        << s.altitude << "m ║ "
                        << std::left  << std::setw(12) << wp_str  << "║ "
                        << std::left  << std::setw(12) << s.status << "║\n";
        }
        oss << "╚═══════╩═══════════╩═══════════╩═════════════╩═════════════╝\n";
        RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
    }

    // ── Members ───────────────────────────────────────────────────────────────
    std::map<std::string, DroneState>                              states_;
    std::vector<rclcpp::Subscription<String>::SharedPtr>           subs_status_;
    std::vector<rclcpp::Subscription<String>::SharedPtr>           subs_alert_;
    std::vector<rclcpp::Subscription<String>::SharedPtr>           subs_complete_;
    std::vector<rclcpp::Subscription<String>::SharedPtr>           subs_telemetry_;
    rclcpp::TimerBase::SharedPtr                                   timer_report_;
    rclcpp::Service<Trigger>::SharedPtr                            srv_report_;
};

// ─── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FleetManager>());
    rclcpp::shutdown();
    return 0;
}
