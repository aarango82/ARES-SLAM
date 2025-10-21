#ifndef LIDSOR_FILTER_H
#define LIDSOR_FILTER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class LidsorFilter : public rclcpp::Node
{
public:
  LidsorFilter();

private:
    void filter_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr unfiltered_pc);

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

    std::string input_topic_;
    std::string output_topic_;

    int k_;
    double s_;
    double r_;
    double i_;
    double d_;
};

#endif