#include "Ransac.h"
#include "pmp/MatVec.h"
#include "pmp/algorithms/DistancePointTriangle.h"

#include <vector>
#include <random>
#include <iterator>

using namespace pmp;

Ransac::Ransac(std::vector<vec3> points, double eps, int minPts)
{
    this->points = points;
    this->eps = eps;
    this->minPts = minPts;
}

std::vector<unsigned int> Ransac::run(int iterations)
{
    std::vector<unsigned int> best_cs;

    for (int i; i < iterations; i++)
    {
        // Randomly choose two points
        vec3 p1 = *choose_rand(points.begin(), points.end());
        vec3 p2 = *choose_rand(points.begin(), points.end());

        // Calculate consensus set
        // cs does not store the points but their indices
        std::vector<unsigned int> cs;

        for (int i = 0; i < points.size(); i++)
        {
            Point p = points[i];
            Point np;

            if (p != p1 && p != p2 && dist_point_line_segment(p, p1, p2, np) < eps)
            {
                cs.push_back(i);
            }
        }

        // Check if it is a consensus set and if it is the best one yet
        if (cs.size() >= minPts && cs.size() > best_cs.size())
        {
            best_cs = cs;
        }
    }

    return best_cs;
}

int Ransac::calculate_iterations(double p, int s, double eps)
{
    return log(1.0 - p) / log(1.0 - pow(1.0 - eps, s));
}

// Source: https://stackoverflow.com/questions/6942273/how-to-get-a-random-element-from-a-c-container
template<typename Iter, typename RandomGenerator>
Iter choose_rand(Iter start, Iter end, RandomGenerator &g)
{
    std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
    std::advance(start, dis(g));

    return start;
}

template<typename Iter>
Iter choose_rand(Iter start, Iter end)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    
    return choose_rand(start, end, gen);
}
