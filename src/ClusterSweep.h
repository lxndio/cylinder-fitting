#pragma once

#include "pmp/MatVec.h"
#include "pmp/Types.h"

#include <Eigen/Dense>
#include <optional>
#include <vector>

namespace ClusterSweep {
/// Cluster Sweep Algorithm
std::vector<std::vector<pmp::Point>>
cluster(std::vector<pmp::Point> &data, pmp::vec3 direction, unsigned precision);

bool point_height_pair_cmp(std::pair<pmp::Point, double> a,
                           std::pair<pmp::Point, double> b);

std::vector<pmp::Point> get_points_in_height_range(
    std::vector<std::pair<pmp::Point, double>> &point_heights, double from,
    double to);

std::vector<unsigned> cluster_dbscan(std::vector<pmp::Point> points, float eps,
                                     unsigned min_pts, int &max_cluster_id);

std::vector<pmp::Point> get_points_from_cluster(std::vector<pmp::Point> points,
                                                std::vector<unsigned> clusters,
                                                unsigned cluster);

std::optional<unsigned>
point_in_cluster(std::vector<std::vector<pmp::Point>> clustered_points,
                 pmp::Point point);

void insert_unique(std::vector<pmp::Point> &to, std::vector<pmp::Point> from);
} // namespace ClusterSweep
