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
    Ransac(double eps, int minPts);

    // run the RANSAC algorithm
    std::vector<vec3> run(std::vector<vec3> points, int iterations);

    // calculate a sensible number of iterations for given parameters
    static int calculate_iterations(double p, int s, double eps);

    /// calculate distance of point from line
    static double dist_point_line(vec3 p, vec3 l1, vec3 l2);

private:
    /// maximum distance of points to model
    double eps;

    /// minimum number of points in eps range
    int minPts;
};

template<typename Iter, typename RandomGenerator>
Iter choose_rand(Iter start, Iter end, RandomGenerator &g);

template<typename Iter>
Iter choose_rand(Iter start, Iter end);
