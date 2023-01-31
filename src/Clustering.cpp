#include "Clustering.h"
#include "pmp/MatVec.h"
#include "pmp/Types.h"
#include <cstdio>
#include <map>
#include <optional>
#include <set>
#include <unordered_set>
#include <vector>

Clustering *Clustering::set_points(std::vector<Point> points) {
    this->points = {};

    for (Point p : points) {
        this->points.push_back(ClusterPoint(p));
    }

    return this;
}

std::vector<std::optional<unsigned>> Clustering::get_clusters() {
    return this->clusters;
}

unsigned Clustering::get_max_cluster_id() { return this->max_cluster_id; }

std::vector<ClusterPoint> Clustering::range_query(ClusterPoint q, float eps) {
    std::vector<ClusterPoint> neighbors;

    for (ClusterPoint p : this->points) {
        if (distance(q.to_point(), p.to_point()) <= eps) {
            neighbors.push_back(p);
        }
    }

    return neighbors;
}

Clustering *Clustering::cluster_dbscan(unsigned min_pts, float eps) {
    int cluster_cnt = -1;
    std::map<ClusterPoint, DBSCANLabel, ClusterPoint::Compare> labels;

    for (ClusterPoint p : this->points) {
        labels[p] = DBSCANLabel();
    }

    for (ClusterPoint p : this->points) {
        if (!labels[p].is_undefined())
            continue;

        std::vector<ClusterPoint> neighbors = this->range_query(p, eps);

        if (neighbors.size() < min_pts) {
            labels[p].set_noise();
            continue;
        }

        cluster_cnt++;
        labels[p].set_cluster(cluster_cnt);

        neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), p),
                        neighbors.end());

        // Keep points in neighbors also in a set for faster checking
        // if a point already exists in neighbors
        std::unordered_set<ClusterPoint, ClusterPoint::HashFunction> s(
            neighbors.begin(), neighbors.end());
        s.erase(p);

        // Couldn't just iterate over s because when inserting new elements
        // the iterator doesn't work correctly anymore
        for (unsigned i = 0; i < neighbors.size(); i++) {
            ClusterPoint q = neighbors[i];

            if (labels[q].is_noise())
                labels[q].set_cluster(cluster_cnt);
            if (!labels[q].is_undefined())
                continue;
            labels[q].set_cluster(cluster_cnt);

            std::vector<ClusterPoint> inner_neighbors =
                this->range_query(q, eps);

            if (inner_neighbors.size() >= min_pts) {
                for (ClusterPoint in_p : inner_neighbors) {
                    if (s.find(in_p) == s.end()) {
                        neighbors.push_back(in_p);
                    }
                }

                s.insert(inner_neighbors.begin(), inner_neighbors.end());
            }
        }
    }

    this->clusters = std::vector<std::optional<unsigned>>(this->points.size());

    for (std::pair<ClusterPoint, DBSCANLabel> label : labels) {
        unsigned i =
            std::find(this->points.begin(), this->points.end(), label.first) -
            this->points.begin();

        if (!label.second.is_undefined() && !label.second.is_noise()) {
            this->clusters[i] = label.second.get_cluster();
        } else {
            this->clusters[i] = {};
        }
    }

    this->max_cluster_id = cluster_cnt;

    return this;
}
