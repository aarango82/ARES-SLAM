#include <memory>

#include "rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lidsor_filter.h"
#include "utils.h"

using std::placeholders::_1;

LidsorFilter::LidsorFilter() : Node("LidsorFilter") {
    this->declare_parameter("input_topic", "/unfiltered_pc");
    this->declare_parameter("output_topic", "/filtered_pc");

    this->declare_parameter("k", 10); //Minimum number of nearest neighbors
    this->declare_parameter("s", 1.0); //Multiplication factor for standard deviation
    this->declare_parameter("r", 1.0); //Multiplication factor for range
    this->declare_parameter("i", 25.0); //Intensity threshold
    this->declare_parameter("d", 30.0); //Distance threshold

    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();

    k_ = this->get_parameter("k").as_int();
    s_ = this->get_parameter("s").as_double();
    r_ = this->get_parameter("r").as_double();
    i_ = this->get_parameter("i").as_double();
    d_ = this->get_parameter("d").as_double();

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, 10, std::bind(&LidsorFilter::filter_pointcloud, this, _1));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
}

void LidsorFilter::filter_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    //Create PCL format for easier processing
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *input_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtrating_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    
    //If beyond distance of plausible noise points, immediately add to filtered cloud. Else store for further processing
    for (const auto &point : input_cloud->points)
    {
        double distance = euclidean_distance(pcl::PointXYZI(0, 0, 0), point);
        if (distance < d_) {
            filtrating_cloud->points.push_back(point);
        } else {
            filtered_cloud->points.push_back(point);
        }
    }

    //Compute mean distance between points to compute global threshold
    std::vector<double> distances;
    for (const auto& point : filtrating_cloud->points) {
        std::vector<pcl::PointXYZI> neighbors = k_nearest_neighbors(filtrating_cloud, point, k_);

        double mean_dist_neighbors = 0;
        for (const auto& neighbor : neighbors) {
            mean_dist_neighbors += euclidean_distance(point, neighbor);
        }

        mean_dist_neighbors /= k_;
        distances.push_back(mean_dist_neighbors);
    }

    double mean = compute_mean(distances);
    double stddev = compute_stddev(distances);

    double global_threshold = mean + stddev * s_;

    //Remove noise points
    for (int i = 0; i < filtrating_cloud->points.size(); i++) {
        pcl::PointXYZI point = filtrating_cloud->points.at(i);

        double distance = euclidean_distance(pcl::PointXYZI(0, 0, 0), point);
        double dynamic_threshold = global_threshold * r_ * distance;
        if (distances[i] > dynamic_threshold && point.intensity < i_) {
            continue; //Remove point
        } else {
            filtered_cloud->points.push_back(point);
        }
    }

    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    //Convert back to ROS2
    sensor_msgs::msg::PointCloud2 output_cloud_msg;
    pcl::toROSMsg(*filtered_cloud, output_cloud_msg);
    output_cloud_msg.header = msg->header;

    publisher_->publish(output_cloud_msg);
}