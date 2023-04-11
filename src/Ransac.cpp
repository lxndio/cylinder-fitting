#include "Ransac.h"
#include "Eigen/src/Core/Matrix.h"
#include "pca.h"
#include "pmp/MatVec.h"
#include "pmp/algorithms/DistancePointTriangle.h"
#include "Clusters.h"
#include "Viewer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <unordered_set>
#include <vector>
#include <random>
#include <iterator>

using namespace pmp;

Ransac::Ransac(double eps, int minPts, double grid_size) {
    this->eps = eps;
    this->minPts = minPts;
    this->grid_size = grid_size;
    this->connected_components = std::vector<std::vector<vec3>>();
}

std::vector<vec3> Ransac::run(std::vector<vec3> points, int iterations) {
    if (points.empty()) return std::vector<vec3>();

    std::vector<vec3> best_cs;

    // Get connected components and determine the one with the most points
    std::vector<std::vector<vec3>> connected_components = this->find_connected_components(points);
    unsigned largest_cc = 0;
    unsigned largest_cc_size = 0;

    for (int i = 0; i < connected_components.size(); i++) {
        if (connected_components[i].size() > largest_cc_size) {
            largest_cc = i;
            largest_cc_size = connected_components[i].size();
        }
    }

    std::vector<vec3> &cc = connected_components[largest_cc];

    for (int i = 0; i < iterations; i++) {
        // Randomly choose two points
        vec3 p1;
        vec3 p2;
        unsigned not_found = 0;
        
        do {
            // Choose points from largest connected component
            p1 = *choose_rand(cc.begin(), cc.end());
            p2 = *choose_rand(cc.begin(), cc.end());
        } while (p1 != p2 && distance(p1, p2) < this->eps * 2.0 && ++not_found < 10);

        // if (not_found >= 10) break; 

        // Calculate consensus set
        std::vector<vec3> cs;

        for (vec3 p : cc) {
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

std::vector<std::vector<vec3>> Ransac::run_on_cc(unsigned cc, int iterations) {
    std::vector<std::vector<vec3>> clustered_points;
    std::vector<vec3> remaining_points = this->connected_components[cc];

    while (remaining_points.size() > 0.07 * this->connected_components[cc].size()) {
        std::vector<vec3> cs = this->run(remaining_points, iterations);

        if (cs.empty()) break;

        clustered_points.push_back(cs);
        
        for (vec3 point : cs) {
            remaining_points.erase(std::remove(remaining_points.begin(), remaining_points.end(), point), remaining_points.end());
        }
    }

    return clustered_points;
}

std::vector<std::vector<vec3>> Ransac::forward_search(std::vector<std::vector<vec3>> clusters, int iterations) {
    unsigned nclusters = clusters.size();
    std::vector<vec3> old_centers(nclusters);
    std::vector<std::tuple<vec3, vec3>> axes;

    for (int it = 0; it < iterations; it++) {
        std::cout << "iterations: " << it << std::endl;
        old_centers = std::vector<vec3>(nclusters);
        
        for (int a = 0; a < axes.size(); a++) {
            old_centers[a] = std::get<0>(axes[a]);
        }

        // Compute axis for each cluster (defined by (center, direction))
        axes = std::vector<std::tuple<vec3, vec3>>();

        for (std::vector<vec3> cluster : clusters) {
            // Convert points to Eigen::Vector3d
            std::vector<Eigen::VectorXd> points_eigen;

            for (int i = 0; i < cluster.size(); i++) {
                points_eigen.push_back(Eigen::Vector3d{cluster[i][0], cluster[i][1], cluster[i][2]});
            }

            // Find axis using PCA
            PCA pca;
            
            if (pca.train(points_eigen, 1)) {
                Eigen::Vector3d center_eigen = pca.mean();
                Eigen::Vector3d eigenvec_eigen = pca.eigenvectors().col(0);
                
                vec3 center(center_eigen[0], center_eigen[1], center_eigen[2]);
                vec3 eigenvec(eigenvec_eigen[0], eigenvec_eigen[1], eigenvec_eigen[2]);

                axes.push_back(std::make_tuple(center, eigenvec));
            }
        }

        // Break if not at least one axis was found for each cluster,
        // shouldn't happen but can if one cluster becomes empty
        if (axes.size() < nclusters) break;

        // Check if centers don't change anymore
        bool changed = false;

        for (int a = 0; a < nclusters; a++) {
            if (distance(std::get<0>(axes[a]), old_centers[a]) > 0.001) {
                changed = true;
                break;
            }
        }

        if (!changed) break;

        // Get all points without cluster information
        std::vector<vec3> points;

        for (std::vector<vec3> cluster : clusters) {
            points.insert(points.end(), cluster.begin(), cluster.end());
        }

        // Assign each point to closest axis' clusters
        clusters = std::vector<std::vector<vec3>>(nclusters);

        for (vec3 point : points) {
            double lowest_dev = std::numeric_limits<double>::infinity();
            unsigned closest_axis = 0;

            for (int a = 0; a < nclusters; a++) {
                double dev = dist_point_line_direction(point, std::get<0>(axes[a]), std::get<1>(axes[a]));

                if (dev < lowest_dev) {
                    lowest_dev = dev;
                    closest_axis = a;
                }
            }
            
            clusters[closest_axis].push_back(point);
        }
    }

    return clusters;
}

std::vector<std::vector<vec3>> Ransac::find_connected_components(std::vector<vec3> &include) {
    std::vector<std::vector<vec3>> connected_components;
    std::vector<vec3> unassigned_points = include;

    while (!unassigned_points.empty()) {
        std::vector<vec3> connected_component;
        std::vector<vec3> unvisited;
        unvisited.push_back(*choose_rand(unassigned_points.begin(), unassigned_points.end()));

        while (!unvisited.empty()) {
            vec3 p = *unvisited.begin();
            unvisited.erase(unvisited.begin());

            std::vector<vec3> neighbors = get_neighbors(include, connected_component, p, this->grid_size);

            connected_component.insert(connected_component.end(), neighbors.begin(), neighbors.end());
            unvisited.insert(unvisited.end(), neighbors.begin(), neighbors.end());
            
            unassigned_points.erase(remove_if(unassigned_points.begin(), unassigned_points.end(),
                [&](auto x) {
                    return std::find(neighbors.begin(), neighbors.end(), x) != neighbors.end();
                }), unassigned_points.end());
        }

        connected_components.push_back(connected_component);
    }

    return connected_components;
}

std::vector<vec3> Ransac::find_connected_points(vec3 p, std::vector<vec3> &include) {
    std::vector<std::vector<vec3>> connected_components = this->find_connected_components(include);

    return this->get_vec_with_point(p, connected_components);
}

int Ransac::calculate_iterations(double p, int s, double eps) {
    return log(1.0 - p) / log(1.0 - pow(1.0 - eps, s));
}

double Ransac::dist_point_line(vec3 p, vec3 l1, vec3 l2) {
    vec3 d = l2 - l1;
    
    return norm(cross(p - l1, d)) / norm(d);
}

double Ransac::dist_point_line_direction(vec3 p, vec3 l1, vec3 d) {
    return norm(cross(p - l1, d)) / norm(d);
}

std::vector<vec3> Ransac::get_neighbors(std::vector<vec3> &include, std::vector<vec3> &exclude, vec3 p, float eps) {
    std::vector<vec3> neighbors;

    for (vec3 q : include) {
        if (distance(p, q) <= eps && std::find(exclude.begin(), exclude.end(), q) == exclude.end()) {
            neighbors.push_back(q);
        }
    }

    return neighbors;
}

std::vector<vec3> Ransac::get_vec_with_point(vec3 p, std::vector<std::vector<vec3>> connected_components) {
    for (std::vector<vec3> cc : connected_components) {
        if (std::find(cc.begin(), cc.end(), p) != cc.end()) {
            return cc;
        }
    }

    return std::vector<vec3>();
}

bool Ransac::are_connected(std::vector<vec3> &points, std::vector<vec3> &include) {
    std::vector<vec3> unassigned_points = points;
    std::vector<vec3> connected_component;
    std::vector<vec3> unvisited;
    unvisited.push_back(*choose_rand(points.begin(), points.end()));

    while (!unvisited.empty()) {
        vec3 p = *unvisited.begin();
        unvisited.erase(unvisited.begin());

        std::vector<vec3> neighbors = get_neighbors(include, connected_component, p, this->grid_size);

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
