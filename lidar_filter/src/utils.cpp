#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lidar_filter/utils.h"

double euclidean_distance(pcl::PointXYZI p1, pcl::PointXYZI p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<pcl::PointXYZI> k_nearest_neighbors(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZI point, int k) {
    std::vector<std::pair<double, pcl::PointXYZI>> distances;
    distances.reserve(cloud->points.size());

    for (const auto& p : cloud->points) {
        double dist = euclidean_distance(point, p);
        distances.emplace_back(dist, p);
    }

    //Sort by distance
    std::sort(distances.begin(), distances.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    std::vector<pcl::PointXYZI> neighbors;
    neighbors.reserve(std::min(k, (int)distances.size()));

    for (int i = 0; i < k && i < (int)distances.size(); ++i) {
        neighbors.push_back(distances[i].second);
    }

    return neighbors;
}

double compute_mean(std::vector<double> vec) {
    double sum = std::accumulate(vec.begin(), vec.end(), 0.0);
    return sum / vec.size();
}

double compute_stddev(std::vector<double> vec) {
    if (vec.size() < 2) return 0.0;
    double mean = compute_mean(vec);
    double accum = 0.0;

    for (double v : vec) {
        double diff = v - mean;
        accum += std::pow(diff, 2);
    }

    return std::sqrt(accum / (vec.size() - 1));
}