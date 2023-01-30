#pragma once

#include "pmp/MatVec.h"
#include "pmp/Types.h"
#include <optional>

using namespace pmp;

namespace Clusters {

std::vector<Point> get_points_from_cluster(std::vector<std::optional<unsigned>> clusters, unsigned int cluster);

}
