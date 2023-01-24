#include "ClusterSweep.h"
#include "ygor-clustering/YgorClustering.hpp"
#include "ygor-clustering/YgorClusteringDBSCAN.hpp"
#include "ygor-clustering/YgorClusteringDatum.hpp"

#include <utility>
#include <vector>

using namespace pmp;

namespace ClusterSweep {
std::vector<std::vector<Point>> cluster(std::vector<Point> &data,
                                        vec3 direction, unsigned precision) {
  std::vector<std::vector<Point>> clustered_points;
  std::vector<std::vector<Point>> last_added_points;
  std::vector<std::pair<Point, double>> point_heights;

  // Calculate height on direction vector for each point
  double norm_direction = norm(direction);

  for (Point point : data) {
    double s = dot(direction, point) / norm_direction;
    point_heights.push_back(std::make_pair(point, s));
  }

  // Sort points by height
  std::sort(point_heights.begin(), point_heights.end(), point_height_pair_cmp);

  // Calculate step height for sweep
  double top = point_heights.back().second;
  double bottom = point_heights.front().second;
  double step_height = (top - bottom) / (double)precision;

  // Sweep
  for (double height = top; height >= bottom; height -= 0.75 * step_height) {
    std::vector<Point> points =
        get_points_in_height_range(point_heights, height, height - step_height);

    int max_cluster_id;
    std::vector<unsigned> clusters =
        cluster_dbscan(points, 5.0, 6, max_cluster_id);
    std::cout << "clusters found: " << max_cluster_id << std::endl;

    for (int cluster = 0; cluster <= max_cluster_id; cluster++) {
      // Collect points from cluster
      std::vector<Point> cluster_points =
          get_points_from_cluster(points, clusters, cluster);
      int c = -1;

      std::cout << "points in cluster " << cluster << ": "
                << cluster_points.size() << std::endl;

      // Check if points overlap with an already existing cluster
      for (Point point : cluster_points) {
        auto cluster = point_in_cluster(last_added_points, point);
        if (cluster.has_value()) {
          c = cluster.value();
          break;
        }
      }

      // If points do not overlap with an already existing cluster make a new
      // cluster
      if (c == -1) {
        c = clustered_points.size();
        clustered_points.push_back({});
        last_added_points.push_back({});
      }

      // if (c >= clustered_points.size()) clustered_points.push_back({});
      // if (c >= last_added_points.size()) last_added_points.push_back({});
      // TODO make more efficient using HashSet etc.
      insert_unique(clustered_points[c], cluster_points);
      last_added_points[c] = cluster_points;
    }
  }

  return clustered_points;
}

bool point_height_pair_cmp(std::pair<Point, double> a,
                           std::pair<Point, double> b) {
  return a.second < b.second;
}

std::vector<Point>
get_points_in_height_range(std::vector<std::pair<Point, double>> &point_heights,
                           double from, double to) {
  std::vector<Point> res;

  // TODO do this more cleverly as point_heights is already sorted
  for (std::pair<Point, double> p : point_heights) {
    if (p.second <= from && p.second > to) {
      res.push_back(p.first);
    }
  }

  return res;
}

std::vector<unsigned> cluster_dbscan(std::vector<Point> points, float eps,
                                     unsigned min_pts, int &max_cluster_id) {
  typedef ClusteringDatum<3, double, 1, int> CDat_t;

  constexpr size_t MaxElementsInANode = 6;
  typedef boost::geometry::index::rstar<MaxElementsInANode> RTreeParameter_t;
  typedef boost::geometry::index::rtree<CDat_t, RTreeParameter_t> RTree_t;
  typedef boost::geometry::model::box<CDat_t> Box_t;

  RTree_t rtree;

  for (int i = 0; i < points.size(); i++) {
    Point p = points[i];

    rtree.insert(CDat_t({p[0], p[1], p[2]}, {i}));
  }

  DBSCAN<RTree_t, CDat_t>(rtree, eps, min_pts);

  constexpr auto RTreeSpatialQueryGetAll = [](const CDat_t &) -> bool {
    return true;
  };
  RTree_t::const_query_iterator it;
  it = rtree.qbegin(boost::geometry::index::satisfies(RTreeSpatialQueryGetAll));

  std::vector<unsigned int> clusters(points.size());

  max_cluster_id = 0;

  for (; it != rtree.qend(); ++it) {
    int cluster_id = it->CID.Raw;
    int point_id = it->Attributes[0];

    clusters[point_id] = cluster_id;

    if (cluster_id > max_cluster_id) {
      max_cluster_id = cluster_id;
    }
  }

  return clusters;
}

std::vector<Point> get_points_from_cluster(std::vector<Point> points,
                                           std::vector<unsigned> clusters,
                                           unsigned cluster) {
  std::vector<Point> res;

  for (int i = 0; i < points.size(); i++) {
    if (clusters[i] == cluster) {
      res.push_back(points[i]);
    }
  }

  return res;
}

std::optional<unsigned>
point_in_cluster(std::vector<std::vector<Point>> clustered_points,
                 Point point) {
  for (int cluster = 0; cluster < clustered_points.size(); cluster++) {
    if (std::find(clustered_points[cluster].begin(),
                  clustered_points[cluster].end(),
                  point) != clustered_points[cluster].end()) {
      return cluster;
    }
  }

  return {};
}

void insert_unique(std::vector<Point> &to, std::vector<Point> from) {
  for (Point point : from) {
    if (std::find(to.begin(), to.end(), point) == to.end()) {
      to.push_back(point);
    }
  }
}
} // namespace ClusterSweep
