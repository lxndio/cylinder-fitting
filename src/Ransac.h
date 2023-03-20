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
    Ransac(std::vector<vec3> points, double eps, int minPts, double);

    /// run the RANSAC algorithm
    std::vector<vec3> run(int iterations);

    /// run the RANSAC algorithm on a specific connected component
    std::vector<vec3> run_on_cc(unsigned cc, int iterations);

    /// generate graph from points using grid size and find
    /// connected components in that graph
    void find_connected_components();

    // calculate a sensible number of iterations for given parameters
    static int calculate_iterations(double p, int s, double eps);

    /// calculate distance of point from line
    static double dist_point_line(vec3 p, vec3 l1, vec3 l2);

    /// get all points neighboring a point in a specific radius
    /// that have not already been found
    std::vector<vec3> get_new_neighbors(std::vector<vec3> &connected_component, vec3 p, float eps);

    /// get the connected component containing a specific point
    std::vector<vec3> get_connected_component(vec3 p);

    /// check if all specified points are connected
    bool are_connected(std::vector<vec3> points);

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
