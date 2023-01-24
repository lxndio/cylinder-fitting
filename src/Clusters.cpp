#include "Clusters.h"
#include "pmp/MatVec.h"
#include "pmp/Types.h"
#include "Viewer.h"

#include <vector>

using namespace pmp;

std::vector<Point> Clusters::get_points_from_cluster(unsigned int cluster)
{
    std::vector<Point> res;

    for (int i = 0; i < Viewer::pointset_.points_.size(); i++)
    {
        if (Viewer::clusters_[i] == cluster && Viewer::pointset_.data_[i])
        {
            res.push_back(Viewer::pointset_.points_[i]);
        }
    }

    return res;
}
