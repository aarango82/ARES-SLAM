#ifndef RADAR_PREPROCESSOR_H
#define RADAR_PREPROCESSOR_H

#include "rclcpp/rclcpp.hpp"
// placeholder for delphi_esr_msgs/msg/EsrTrack
// need to set up dataset
#include "nav_msgs/msg/odometry.hpp" 
#include <vector>
#include <cmath>
#include <functional>


std::vector<RadarTrack> parse_simulated_tracks(const nav_msgs::msg::Odometry::SharedPtr msg);

class RadarPreprocessorNode : public rclcpp::Node
{
public:
    RadarPreprocessorNode();

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr radar_sub_;


    std::vector<RadarTrack> filter_by_velocity(const std::vector<RadarTrack>& input_tracks, double max_lateral_vel);

    std::vector<RadarTrack> filter_by_rcs(const std::vector<RadarTrack>& input_tracks, double min_rcs_dB);

    std::vector<RadarTrack> filter_by_roi(const std::vector<RadarTrack>& input_tracks, double min_dist_m, double max_dist_m);

    void radar_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif // RADAR_PREPROCESSOR_H