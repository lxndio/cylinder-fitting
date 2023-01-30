#pragma once

#include "pmp/MatVec.h"
#include "pmp/Types.h"

#include <functional>
#include <optional>
#include <vector>

using namespace pmp;

class ClusterPoint {

public:
    float x;
    float y;
    float z;

    ClusterPoint(Point p) {
        this->x = p[0];
        this->y = p[1];
        this->z = p[2];
    }

    Point to_point() {
        return Point(this->x, this->y, this->z);
    }

    bool operator==(const ClusterPoint& p) const {
        return this->x == p.x && this->y == p.y && this->z == p.z;
    }

    struct Compare {
        bool operator()(const ClusterPoint& lhs, const ClusterPoint& rhs) const {
            if (lhs.x != rhs.x) return lhs.x < rhs.x;
            if (lhs.y != rhs.y) return lhs.y < rhs.y;
            return lhs.z < rhs.z;
        }
    };

    struct HashFunction {
        // https://ianyepan.github.io/posts/cpp-custom-hash/
        size_t operator()(const ClusterPoint& p) const {
            return std::hash<float>{}(p.x) ^ std::hash<float>{}(p.y) ^ std::hash<float>{}(p.z);
        }
    };
};

class Clustering {
    std::vector<ClusterPoint> points;
    std::vector<std::optional<unsigned>> clusters;
    unsigned max_cluster_id;

    std::vector<ClusterPoint> range_query(ClusterPoint q, float eps);

public:
    Clustering() {}

    Clustering* set_points(std::vector<Point> points);

    std::vector<std::optional<unsigned>> get_clusters();

    unsigned get_max_cluster_id();

    Clustering* cluster_dbscan(unsigned min_pts, float eps);
};

class DBSCANLabel {
    bool undefined = true;
    bool noise = false;
    unsigned cluster = 0;

public:
    DBSCANLabel() {}

    void set_noise() {
        this->undefined = false;
        this->cluster = 0;
        this->noise = true;
    }

    void set_cluster(unsigned cluster) {
        this->undefined = false;
        this->noise = false;
        this->cluster = cluster;
    }

    bool is_undefined() {
        return this->undefined;
    }

    bool is_noise() {
        return this->noise;
    }

    unsigned get_cluster() {
        return this->cluster;
    }
};
