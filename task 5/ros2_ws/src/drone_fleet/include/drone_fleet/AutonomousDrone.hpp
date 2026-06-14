#pragma once
#include "MissionDrone.hpp"
#include <deque>

class AutonomousDrone : public MissionDrone {
public:
    AutonomousDrone(const std::string& name,
                    const std::string& mission_name,
                    const std::vector<WP>& waypoints,
                    const WP& home_position,
                    float initial_battery = 100.0f,
                    float max_altitude    = 120.0f,
                    float speed           = 5.0f);
    virtual ~AutonomousDrone() = default;

    void set_ai_mode(const std::string& mode);          
    void detect_obstacle(const WP& position,
                         const std::string& severity);  
    std::vector<WP> auto_replan(const std::vector<WP>& obstacles);

    const std::string& get_ai_mode()      const { return ai_mode_; }
    const WP&          get_home_position()const { return home_position_; }

    
    virtual std::string get_info() const override;

private:
    std::string              ai_mode_;
    WP                       home_position_;
    std::vector<std::string> obstacle_log_;   // timestamped

    float euclidean_distance(const WP& a, const WP& b) const;
};
