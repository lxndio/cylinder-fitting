#include "ClusterSweep.h"
#include "Clustering.h"
#include "ygor-clustering/YgorClustering.hpp"
#include "ygor-clustering/YgorClusteringDBSCAN.hpp"
#include "ygor-clustering/YgorClusteringDatum.hpp"

#include <optional>
#include <utility>
#include <vector>

using namespace pmp;

namespace ClusterSweep {
std::vector<std::vector<Point>> cluster(std::vector<Point> &data,
                                        vec3 direction, unsigned precision,
                                        unsigned min_pts, float eps) {
    std::vector<std::vector<Point>> clustered_points;
    std::vector<std::vector<Point>> last_added_points;
    std::vector<std::pair<Point, double>> point_heights;

    // Calculate height on direction vector for each point
    double norm_direction = norm(direction);

    for (Point point : data) {
        double s = dot(direction, point) / norm_direction;
        point_heights.push_back(std::make_pair(point, s));
    }

    // Sort points by height
    std::sort(point_heights.begin(), point_heights.end(),
              point_height_pair_cmp);

    // Calculate step height for sweep
    double top = point_heights.back().second;
    double bottom = point_heights.front().second;
    double step_height = fabs(top - bottom) / (double)precision;

    // Sweep with 0.75 * step_height so that some points are always included
    // in two steps for easier side registration
    for (double height = top; height >= bottom; height -= 0.75 * step_height) {
        std::vector<Point> points = get_points_in_height_range(
            point_heights, height, height - step_height);
    
        Clustering clustering;
        std::vector<std::optional<unsigned>> clusters =
            clustering.set_points(points)
                ->cluster_dbscan(min_pts, eps)
                ->get_clusters();

        int max_cluster_id = clustering.get_max_cluster_id();

        for (int cluster = 0; cluster <= max_cluster_id; cluster++) {
            // Collect points from cluster
            std::vector<Point> cluster_points =
                get_points_from_cluster(points, clusters, cluster);
            int c = -1;

            // std::cout << "points in cluster " << cluster << ": "
            //           << cluster_points.size() << std::endl;

            if (cluster_points.empty())
                continue;

            // Check if points overlap with an already existing cluster
            for (Point point : cluster_points) {
                auto cluster = point_in_cluster(last_added_points, point);
                if (cluster.has_value()) {
                    c = cluster.value();
                    break;
                }
            }

            // If points do not overlap with an already existing cluster make a
            // new cluster
            if (c == -1) {
                c = clustered_points.size();
                clustered_points.push_back({});
                last_added_points.push_back({});
            }

            // if (c >= clustered_points.size()) clustered_points.push_back({});
            // if (c >= last_added_points.size())
            // last_added_points.push_back({});
            // TODO make more efficient using HashSet etc.
            insert_unique(clustered_points[c], cluster_points);
            last_added_points[c] = cluster_points;
        }
    }

    return clustered_points;
}

bool point_height_pair_cmp(std::pair<Point, double> a,
                           std::pair<Point, double> b) {
    return a.second < b.second;
}

std::vector<Point>
get_points_in_height_range(std::vector<std::pair<Point, double>> &point_heights,
                           double a, double b) {
    std::vector<Point> res;
    double from = std::min(a, b);
    double to = std::max(a, b);

    // TODO do this more cleverly as point_heights is already sorted
    for (std::pair<Point, double> p : point_heights) {
        if (p.second >= from && p.second < to) {
            res.push_back(p.first);
        }
    }

    return res;
}

std::vector<Point>
get_points_from_cluster(std::vector<Point> points,
                        std::vector<std::optional<unsigned>> clusters,
                        unsigned cluster) {
    std::vector<Point> res;

    for (int i = 0; i < points.size(); i++) {
        if (clusters[i].has_value() && clusters[i].value() == cluster) {
            res.push_back(points[i]);
        }
    }

    return res;
}

std::optional<unsigned>
point_in_cluster(std::vector<std::vector<Point>> clustered_points,
                 Point point) {
    for (int cluster = 0; cluster < clustered_points.size(); cluster++) {
        if (std::find(clustered_points[cluster].begin(),
                      clustered_points[cluster].end(),
                      point) != clustered_points[cluster].end()) {
            return cluster;
        }
    }

    return {};
}

void insert_unique(std::vector<Point> &to, std::vector<Point> from) {
    for (Point point : from) {
        if (std::find(to.begin(), to.end(), point) == to.end()) {
            to.push_back(point);
        }
    }
}
} // namespace ClusterSweep
