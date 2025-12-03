#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp" 
#include "radar_tracking.h" // Includes the mock RadarTrack struct
#include <vector>
#include <cmath>
#include <functional>
#include <algorithm>


class RadarPreprocessorNode : public rclcpp::Node
{
public:
    // Constructor
    RadarPreprocessorNode() : rclcpp::Node("radar_preprocessor_node")
    {
        RCLCPP_INFO(this->get_logger(), "Radar Preprocessor Node Initialized.");
        // The subscription uses nav_msgs::msg::Odometry as a placeholder
        radar_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/delphi_esr/tracks", // Placeholder topic name
            10,
            std::bind(&RadarPreprocessorNode::radar_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr radar_sub_;

    

    std::vector<RadarTrack> filter_by_velocity(const std::vector<RadarTrack>& input_tracks, double max_lateral_vel)
    {
        std::vector<RadarTrack> filtered_tracks;
        for (const auto& track : input_tracks)
        {
            if (std::abs(track.lateral_vel_mps) <= max_lateral_vel)
            {
                filtered_tracks.push_back(track);
            }
        }
        return filtered_tracks;
    }

    std::vector<RadarTrack> filter_by_rcs(const std::vector<RadarTrack>& input_tracks, double min_rcs_dB)
    {
        std::vector<RadarTrack> filtered_tracks;
        for (const auto& track : input_tracks)
        {
            if (track.rcs_dB >= min_rcs_dB)
            {
                filtered_tracks.push_back(track);
            }
        }
        return filtered_tracks;
    }

    std::vector<RadarTrack> filter_by_roi(const std::vector<RadarTrack>& input_tracks, double min_dist_m, double max_dist_m)
    {
        std::vector<RadarTrack> filtered_tracks;
        for (const auto& track : input_tracks)
        {
            if (track.range_m >= min_dist_m && track.range_m <= max_dist_m)
            {
                filtered_tracks.push_back(track);
            }
        }
        return filtered_tracks;
    }

    void radar_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received radar message. Starting preprocessing.");

        std::vector<RadarTrack> tracks = extract_tracks_from_msg(msg);
        RCLCPP_INFO(this->get_logger(), "Initial tracks count: %zu", tracks.size());

        // placeholder -- hardcoded limits
        const double MAX_LATERAL_VEL = 1.0;  
        const double MIN_RCS_DB = 5.0;       
        const double MIN_RANGE_M = 1.0;      
        const double MAX_RANGE_M = 50.0;     

        
        tracks = filter_by_velocity(tracks, MAX_LATERAL_VEL);
        RCLCPP_INFO(this->get_logger(), "After Vel Filter (max %.1f m/s): %zu tracks left.", 
                    MAX_LATERAL_VEL, tracks.size());

        tracks = filter_by_rcs(tracks, MIN_RCS_DB);
        RCLCPP_INFO(this->get_logger(), "After RCS Filter (min %.1f dB): %zu tracks left.", 
                    MIN_RCS_DB, tracks.size());
        

        tracks = filter_by_roi(tracks, MIN_RANGE_M, MAX_RANGE_M);
        RCLCPP_INFO(this->get_logger(), "After ROI Filter (%.1f-%.1f m): %zu tracks left.", 
                    MIN_RANGE_M, MAX_RANGE_M, tracks.size());

        for (const auto& track : tracks) {
            RCLCPP_INFO(this->get_logger(), "Final Feature Track ID %d: Range=%.2f m, LatVel=%.2f m/s, RCS=%.1f dB",
                        track.track_id, track.range_m, track.lateral_vel_mps, track.rcs_dB);
        }

        RCLCPP_INFO(this->get_logger(), "Preprocessing complete for this frame.");
    }
};

// Main function
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadarPreprocessorNode>());
    rclcpp::shutdown();
    return 0;
}