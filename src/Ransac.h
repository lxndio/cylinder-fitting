#pragma once

#include "pmp/MatVec.h"

#include <random>
#include <vector>

using namespace pmp;

class Ransac
{
public:
    /// constructor
    Ransac(std::vector<vec3> points, double eps, int minPts);

    std::vector<unsigned int> run(int iterations);

    static int calculate_iterations(double p, int s, double eps);

private:
    /// the points to work with
    std::vector<vec3> points;

    /// maximum distance of points to model
    double eps;

    /// minimum number of points in eps range
    int minPts;
};

template<typename Iter, typename RandomGenerator>
Iter choose_rand(Iter start, Iter end, RandomGenerator &g);

template<typename Iter>
Iter choose_rand(Iter start, Iter end);
