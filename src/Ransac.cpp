#include "Ransac.h"
#include "pmp/MatVec.h"
#include "pmp/algorithms/DistancePointTriangle.h"
#include "Clusters.h"
#include "Viewer.h"

#include <algorithm>
#include <unordered_set>
#include <vector>
#include <random>
#include <iterator>

using namespace pmp;

Ransac::Ransac(std::vector<vec3> points, double eps, int minPts, double grid_size) {
    this->points = points;
    this->eps = eps;
    this->minPts = minPts;
    this->grid_size = grid_size;
    this->connected_components = std::vector<std::vector<vec3>>();
}

std::vector<vec3> Ransac::run(int iterations) {
    std::vector<vec3> best_cs;

    for (int i = 0; i < iterations; i++) {
        // Randomly choose two points
        vec3 p1 = *choose_rand(points.begin(), points.end());
        vec3 p2 = *choose_rand(points.begin(), points.end());

        // Calculate consensus set
        std::vector<vec3> cs;

        for (int i = 0; i < points.size(); i++) {
            Point p = points[i];

            if (dist_point_line(p, p1, p2) < eps) {
                cs.push_back(p);
            }
        }

        // Check if it is a consensus set and if it is the best one yet
        if (cs.size() >= minPts && cs.size() > best_cs.size()) {
            best_cs = cs;
        }
    }

    return best_cs;
}

std::vector<vec3> Ransac::run_on_cc(unsigned cc, int iterations) {
    std::vector<vec3> connected_component = this->connected_components[cc];
    std::vector<vec3> best_cs;

    for (int i = 0; i < iterations; i++) {
        // Randomly choose two points
        vec3 p1;
        vec3 p2;
        unsigned not_found = 0;
        
        do {
            p1 = *choose_rand(connected_component.begin(), connected_component.end());
            p2 = *choose_rand(connected_component.begin(), connected_component.end());

            if (distance(p1, p2) < this->eps * 2.0) not_found++;
        } while (distance(p1, p2) < this->eps * 2.0 && not_found < 5);

        // Calculate consensus set
        std::vector<vec3> cs;

        for (int i = 0; i < connected_component.size(); i++) {
            Point p = connected_component[i];

            if (dist_point_line(p, p1, p2) < eps) {
                cs.push_back(p);
            }
        }

        // Check if it is a consensus set and if it is the best one yet
        if (cs.size() >= minPts && cs.size() > best_cs.size() && this->are_connected(cs)) {
            best_cs = cs;
        }
    }

    return best_cs;
}

void Ransac::find_connected_components() {
    std::vector<vec3> unassigned_points = this->points;

    while (!unassigned_points.empty()) {
        std::vector<vec3> connected_component;
        std::vector<vec3> unvisited;
        unvisited.push_back(*choose_rand(unassigned_points.begin(), unassigned_points.end()));

        while (!unvisited.empty()) {
            vec3 p = *unvisited.begin();
            unvisited.erase(unvisited.begin());

            std::vector<vec3> neighbors = this->get_new_neighbors(connected_component, p, this->grid_size);

            connected_component.insert(connected_component.end(), neighbors.begin(), neighbors.end());
            unvisited.insert(unvisited.end(), neighbors.begin(), neighbors.end());
            
            unassigned_points.erase(remove_if(unassigned_points.begin(), unassigned_points.end(),
                [&](auto x) {
                    return std::find(neighbors.begin(), neighbors.end(), x) != neighbors.end();
                }), unassigned_points.end());
        }

        this->connected_components.push_back(connected_component);
    }
}

int Ransac::calculate_iterations(double p, int s, double eps) {
    return log(1.0 - p) / log(1.0 - pow(1.0 - eps, s));
}

double Ransac::dist_point_line(vec3 p, vec3 l1, vec3 l2) {
    vec3 d = l2 - l1;
    
    return norm(cross(p - l1, d)) / norm(d);
}

std::vector<vec3> Ransac::get_new_neighbors(std::vector<vec3> &connected_component, vec3 p, float eps) {
    std::vector<vec3> neighbors;

    for (vec3 q : this->points) {
        if (distance(p, q) <= eps
                && std::find(connected_component.begin(), connected_component.end(), q) == connected_component.end()) {
            neighbors.push_back(q);
        }
    }

    return neighbors;
}

std::vector<vec3> Ransac::get_connected_component(vec3 p) {
    for (std::vector<vec3> cc : this->connected_components) {
        if (std::find(cc.begin(), cc.end(), p) != cc.end()) {
            return cc;
        }
    }

    return std::vector<vec3>();
}

bool Ransac::are_connected(std::vector<vec3> points) {
    std::vector<vec3> unassigned_points = points;
    std::vector<vec3> connected_component;
    std::vector<vec3> unvisited;
    unvisited.push_back(*choose_rand(points.begin(), points.end()));

    while (!unvisited.empty()) {
        vec3 p = *unvisited.begin();
        unvisited.erase(unvisited.begin());

        std::vector<vec3> neighbors = this->get_new_neighbors(connected_component, p, this->grid_size);

        connected_component.insert(connected_component.end(), neighbors.begin(), neighbors.end());
        unvisited.insert(unvisited.end(), neighbors.begin(), neighbors.end());
        
        unassigned_points.erase(remove_if(unassigned_points.begin(), unassigned_points.end(),
            [&](auto x) {
                return std::find(neighbors.begin(), neighbors.end(), x) != neighbors.end();
            }), unassigned_points.end());
    }

    return unassigned_points.empty();
}

// Source: https://stackoverflow.com/questions/6942273/how-to-get-a-random-element-from-a-c-container
template<typename Iter, typename RandomGenerator>
Iter choose_rand(Iter start, Iter end, RandomGenerator &g) {
    std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
    std::advance(start, dis(g));

    return start;
}

template<typename Iter>
Iter choose_rand(Iter start, Iter end) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    
    return choose_rand(start, end, gen);
}
