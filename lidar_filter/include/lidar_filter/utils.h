#ifndef UTILS_H
#define UTILS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

double euclidean_distance(pcl::PointXYZI p1, pcl::PointXYZI p2);

std::vector<pcl::PointXYZI> k_nearest_neighbors(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointXYZI point, int k);

double compute_stddev(std::vector<double> vec);
double compute_mean(std::vector<double> vec);

#endif