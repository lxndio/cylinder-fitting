#pragma once

#include "pmp/MatVec.h"
#include "pmp/Types.h"

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

    bool operator<(const ClusterPoint& p) const {
        return this->x < p.x;
    }
};

class Clustering {
    std::vector<ClusterPoint> points;
    std::vector<std::optional<unsigned>> clusters;

    std::vector<ClusterPoint> range_query(ClusterPoint q, unsigned eps);

public:
    Clustering() {}

    Clustering* set_points(std::vector<Point> points);

    std::vector<std::optional<unsigned>> get_clusters();

    Clustering* cluster_dbscan(unsigned min_pts, unsigned eps);
};

class DBSCANLabel {
    bool undefined = true;
    bool noise = false;
    unsigned cluster = 0;

public:
    DBSCANLabel() {}

    void set_noise() {
        this->undefined = false;
        this->noise = true;
    }

    void set_cluster(unsigned cluster) {
        this->undefined = false;
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
