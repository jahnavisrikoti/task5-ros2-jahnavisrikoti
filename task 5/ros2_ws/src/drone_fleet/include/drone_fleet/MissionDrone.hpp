#pragma once
#include "Drone.hpp"
#include <tuple>
#include <vector>

using WP  = std::tuple<float, float, float>;
using WPV = std::pair<WP, std::string>;     // waypoint + timestamp

class MissionDrone : public Drone {
public:
    MissionDrone(const std::string& name,
                 const std::string& mission_name,
                 const std::vector<WP>& waypoints,
                 float initial_battery = 100.0f,
                 float max_altitude    = 120.0f,
                 float speed           = 5.0f);
    virtual ~MissionDrone() = default;

     WP          next_waypoint(); 
    void        skip_waypoint(const std::string& reason);
    bool        mission_complete() const;
    std::string mission_summary()  const;
    void        reset_mission();

   
    const std::string& get_mission_name()    const { return mission_name_; }
    int                get_waypoint_index()  const { return current_waypoint_index_; }
    int                get_waypoint_count()  const { return static_cast<int>(waypoints_.size()); }

    
    virtual std::string get_info() const override;

protected:
    std::string       mission_name_;
    std::vector<WP>   waypoints_;
    int               current_waypoint_index_;

private:
    std::vector<WPV>  visited_waypoints_;     
};
