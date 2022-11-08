#include "Ransac.h"
#include "pmp/MatVec.h"
#include "pmp/algorithms/DistancePointTriangle.h"
#include "Clusters.h"
#include "Viewer.h"

#include <vector>
#include <random>
#include <iterator>

using namespace pmp;

Ransac::Ransac(unsigned int cluster, double eps, int minPts)
{
    this->all_points = Viewer::pointset_.points_;
    this->cluster = cluster;
    this->eps = eps;
    this->minPts = minPts;
}

std::vector<unsigned int> Ransac::run(int iterations)
{
    std::vector<unsigned int> best_cs;
    std::vector<Point> points = Clusters::get_points_from_cluster(cluster);

    for (int i = 0; i < iterations; i++)
    {
        // Randomly choose two points
        vec3 p1 = *choose_rand(points.begin(), points.end());
        vec3 p2 = *choose_rand(points.begin(), points.end());

        // Calculate consensus set
        // cs does not store the points but their indices
        std::vector<unsigned int> cs;

        // Iterate over all points and check if point is in cluster
        // later so we can get the global ID of the point and not
        // just the ID in the cluster
        for (int i = 0; i < all_points.size(); i++)
        {
            // If point is in cluster
            if (Viewer::clusters_[i] == cluster)
            {
                Point p = all_points[i];
                // Point np;

                // if (p != p1 && p != p2 && dist_point_line_segment(p, p1, p2, np) < eps)
                // {
                //     cs.push_back(i);
                // }
                if (p != p1 && p != p2 && dist_point_line(p, p1, p2) < eps)
                {
                    cs.push_back(i);
                }
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

double Ransac::dist_point_line(vec3 p, vec3 l1, vec3 l2)
{
    vec3 d = l2 - l1;
    
    return norm(cross(p - l1, d)) / norm(d);
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
