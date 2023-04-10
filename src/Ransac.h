#pragma once

#include "pmp/MatVec.h"

#include <optional>
#include <random>
#include <vector>

using namespace pmp;

class Ransac
{
public:
    /// constructor
    Ransac(std::vector<vec3> points, double eps, int minPts, double grid_size);

    /// run the RANSAC algorithm
    std::vector<vec3> run(std::vector<vec3> points, int iterations);

    /// run the RANSAC algorithm on a specific connected component
    std::vector<std::vector<vec3>> run_on_cc(unsigned cc, int iterations);

    /// run forward search on RANSAC points to optimize solution
    /// with maximum number of `iterations`
    std::vector<std::vector<vec3>> forward_search(std::vector<std::vector<vec3>> points, int iterations);

    /// generate graph from points using grid size and find
    /// connected components in that graph
    std::vector<std::vector<vec3>> find_connected_components(std::vector<vec3> &include);

    /// get points that are in the same connected component as `p`
    std::vector<vec3> find_connected_points(vec3 p, std::vector<vec3> &include);

    // calculate a sensible number of iterations for given parameters
    static int calculate_iterations(double p, int s, double eps);

    /// calculate distance of point from line
    static double dist_point_line(vec3 p, vec3 l1, vec3 l2);

    static double dist_point_line_direction(vec3 p, vec3 l1, vec3 d);

    /// get all points neighboring a point `p` in a specific radius `eps`
    /// that are contained in `include` but not in `exclude`
    static std::vector<vec3> get_neighbors(std::vector<vec3> &include, std::vector<vec3> &exclude, vec3 p, float eps);

    /// get the vector from a list of vectors containing a specific point
    std::vector<vec3> get_vec_with_point(vec3 p, std::vector<std::vector<vec3>> connected_components);

    /// check if all specified points are connected in include
    bool are_connected(std::vector<vec3> &points, std::vector<vec3> &include);

    /// connected components
    std::vector<std::vector<vec3>> connected_components;

private:
    /// point set to work on
    std::vector<vec3> points;

    /// maximum distance of points to model
    double eps;

    /// minimum number of points in eps range
    int minPts;

    /// grid size (distance of points with uniform spacing)
    double grid_size;
};

template<typename Iter, typename RandomGenerator>
Iter choose_rand(Iter start, Iter end, RandomGenerator &g);

template<typename Iter>
Iter choose_rand(Iter start, Iter end);
