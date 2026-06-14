#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <deque>
#include <map>
#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <cmath>

using namespace std::chrono_literals;
using String = std_msgs::msg::String;

// ─── Helpers ──────────────────────────────────────────────────────────────────
static double json_num(const std::string& json, const std::string& key) {
    std::string pattern = "\"" + key + "\":";
    auto pos = json.find(pattern);
    if (pos == std::string::npos) return 0.0;
    pos += pattern.size();
    if (pos < json.size() && json[pos] == '"') return 0.0;
    try { return std::stod(json.substr(pos)); }
    catch (...) { return 0.0; }
}

static std::string iso_now() {
    using namespace std::chrono;
    auto now = system_clock::now();
    auto t   = system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&t), "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

// ─── Per-drone tracking ───────────────────────────────────────────────────────
struct DroneHealth {
    // Circular buffer of (timestamp_sec, battery) pairs — last 10 samples
    std::deque<std::pair<double, double>> samples;   // max size 10
    double last_battery    = 100.0;
    bool   received        = false;

    static constexpr size_t BUFFER_SIZE = 10;

    void add_sample(double battery) {
        using namespace std::chrono;
        double t = duration<double>(system_clock::now().time_since_epoch()).count();
        samples.push_back({t, battery});
        if (samples.size() > BUFFER_SIZE) samples.pop_front();
        last_battery = battery;
        received     = true;
    }

    // Battery drain rate: (battery_first - battery_last) / elapsed_seconds
    // Positive = draining
    double drain_rate_per_second() const {
        if (samples.size() < 2) return 0.0;
        const auto& oldest = samples.front();
        const auto& newest = samples.back();
        double elapsed = newest.first - oldest.first;
        if (elapsed <= 0.0) return 0.0;
        return (oldest.second - newest.second) / elapsed;
    }

    // Time until battery reaches critical (20%)
    double seconds_to_critical() const {
        double rate = drain_rate_per_second();
        if (rate <= 0.0) return 9999.0;
        double margin = last_battery - 20.0;
        return margin < 0.0 ? 0.0 : margin / rate;
    }

    // Time until battery reaches 0%
    double seconds_to_depletion() const {
        double rate = drain_rate_per_second();
        if (rate <= 0.0) return 9999.0;
        return last_battery / rate;
    }
};

// ─── Health Monitor Node ──────────────────────────────────────────────────────
class HealthMonitor : public rclcpp::Node {
public:
    HealthMonitor() : rclcpp::Node("health_monitor") {
        const std::vector<std::string> drones = {"Alpha", "Beta", "Gamma"};

        // ── Health warning publisher ───────────────────────────────────────────
        pub_warning_ = create_publisher<String>("/fleet/health_warning", 10);
        pub_summary_ = create_publisher<String>("/fleet/health_summary", 10);

        // ── Subscribe to telemetry for each drone ─────────────────────────────
        for (const auto& d : drones) {
            health_[d] = DroneHealth{};
            subs_.push_back(
                create_subscription<String>(
                    "/drone/" + d + "/telemetry", 10,
                    [this, d](const String::SharedPtr msg) {
                        on_telemetry(d, msg->data);
                    }));
        }

        // ── Diagnostics table every 10 s ──────────────────────────────────────
        timer_diag_ = create_wall_timer(
            10s, std::bind(&HealthMonitor::print_diagnostics, this));

        RCLCPP_INFO(get_logger(), "HealthMonitor online — tracking battery drain rates.");
    }

private:
    void on_telemetry(const std::string& name, const std::string& json) {
        double bat = json_num(json, "battery");
        auto& h    = health_[name];
        h.add_sample(bat);

        // Check drain rate threshold
        double rate = h.drain_rate_per_second();
        if (rate > DRAIN_THRESHOLD_) {
            std::ostringstream warn;
            warn << std::fixed << std::setprecision(3);
            warn << "[" << iso_now() << "] ⚠ HIGH DRAIN: " << name
                 << " draining at " << rate << "%/s (threshold: "
                 << DRAIN_THRESHOLD_ << "%/s)";

            auto msg  = String();
            msg.data  = warn.str();
            pub_warning_->publish(msg);
            RCLCPP_WARN(get_logger(), "%s", warn.str().c_str());
        }
    }

    void print_diagnostics() {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3);

        oss << "\n╔═════════════════════════════════════════════════════════════════╗\n";
        oss << "║           HEALTH DIAGNOSTICS — " << iso_now() << "            ║\n";
        oss << "╠═══════╦══════════╦══════════════╦════════════════╦════════════╣\n";
        oss << "║ Drone ║ Battery% ║ Drain %/s    ║ → Critical (s) ║ → Empty(s) ║\n";
        oss << "╠═══════╬══════════╬══════════════╬════════════════╬════════════╣\n";

        // Build JSON health summary simultaneously
        std::ostringstream json;
        json << std::fixed << std::setprecision(3);
        json << "{\"timestamp\":\"" << iso_now() << "\",\"drones\":[";
        bool first = true;

        for (auto& [name, h] : health_) {
            std::string bat_str  = h.received ? std::to_string(static_cast<int>(h.last_battery)) + "%" : "—";
            std::string rate_str = h.received ? std::to_string(h.drain_rate_per_second()) : "—";
            std::string crit_str = h.received ? std::to_string(h.seconds_to_critical()) : "—";
            std::string empt_str = h.received ? std::to_string(h.seconds_to_depletion()) : "—";

            oss << "║ " << std::left  << std::setw(6)  << name    << "║ "
                        << std::right << std::setw(8)  << bat_str << "║ "
                        << std::right << std::setw(13) << rate_str<< "║ "
                        << std::right << std::setw(15) << crit_str<< "║ "
                        << std::right << std::setw(11) << empt_str<< "║\n";

            if (!first) json << ",";
            first = false;
            if (h.received) {
                json << "{"
                     << "\"name\":\""        << name                           << "\","
                     << "\"battery\":"        << h.last_battery                 << ","
                     << "\"drain_rate\":"     << h.drain_rate_per_second()       << ","
                     << "\"secs_to_critical\":"<< h.seconds_to_critical()        << ","
                     << "\"secs_to_empty\":"  << h.seconds_to_depletion()        << ","
                     << "\"samples\":"        << h.samples.size()
                     << "}";
            } else {
                json << "{\"name\":\"" << name << "\",\"status\":\"no_data\"}";
            }
        }
        oss << "╚═══════╩══════════╩══════════════╩════════════════╩════════════╝\n";
        RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());

        json << "]}";
        auto summary = String();
        summary.data = json.str();
        pub_summary_->publish(summary);
    }

    // ── Members ───────────────────────────────────────────────────────────────
    std::map<std::string, DroneHealth>                    health_;
    std::vector<rclcpp::Subscription<String>::SharedPtr>  subs_;
    rclcpp::Publisher<String>::SharedPtr                  pub_warning_;
    rclcpp::Publisher<String>::SharedPtr                  pub_summary_;
    rclcpp::TimerBase::SharedPtr                          timer_diag_;

    static constexpr double DRAIN_THRESHOLD_ = 1.5; // %/s
};

// ─── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HealthMonitor>());
    rclcpp::shutdown();
    return 0;
}
