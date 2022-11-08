#pragma once

#include "pmp/MatVec.h"

#include <random>
#include <vector>

using namespace pmp;

class Ransac
{
public:
    /// constructor
    Ransac(unsigned int cluster, double eps, int minPts);

    // run the RANSAC algorithm
    std::vector<unsigned int> run(int iterations);

    // calculate a sensible number of iterations for given parameters
    static int calculate_iterations(double p, int s, double eps);

    /// calculate distance of point from line
    static double dist_point_line(vec3 p, vec3 l1, vec3 l2);

private:
    /// the points to work with
    std::vector<vec3> all_points;

    /// the cluster to work with
    unsigned int cluster;

    /// maximum distance of points to model
    double eps;

    /// minimum number of points in eps range
    int minPts;
};

template<typename Iter, typename RandomGenerator>
Iter choose_rand(Iter start, Iter end, RandomGenerator &g);

template<typename Iter>
Iter choose_rand(Iter start, Iter end);
