//=============================================================================

#include "Viewer.h"
#include "Clusters.h"
#include "CylinderFitting.h"
#include "ClusterSweep.h"
#include "Ransac.h"
#include "fmt/format.h"
#include "pca.h"
#include "pmp/MatVec.h"
#include "pmp/Types.h"
#include "pmp/visualization/MeshViewer.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <fstream>
#include <imgui.h>
#include <iterator>
// #include <opencv2/core.hpp>
// #include <opencv2/core/hal/interface.h>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>
// #include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

//=============================================================================

// define input directories if something went wrong in cmake
#ifndef POINTSET_DIRECTORY
#if defined(_WIN32) || defined(__XCODE__)
#define POINTSET_DIRECTORY ("../../data/pointsets/")
#else
#define POINTSET_DIRECTORY ("../data/pointsets/")
#endif
#endif

#ifndef MESH_DIRECTORY
#if defined(_WIN32) || defined(__XCODE__)
#define MESH_DIRECTORY ("../../data/meshes/")
#else
#define MESH_DIRECTORY ("../data/meshes/")
#endif
#endif

// const std::string point_dir = POINTSET_DIRECTORY;
const std::string point_dir = "../data/pointsets/";

std::vector<std::string> pointsets;

PointSet Viewer::pointset_;

std::vector<unsigned int> Viewer::clusters_;

const Color colors[] = {
    Color(125.0, 0.0, 0.0),   // red
    Color(0.0, 125.0, 0.0),   // green
    Color(0.0, 0.0, 125.0),   // blue
    Color(80.0, 80.0, 20.0),  // yellow
    Color(0.0, 125.0, 125.0), // cyan
    Color(125.0, 0.0, 125.0)  // purple
};

const char colors_short[] = {'r', 'g', 'b', 'y', 'c', 'p'};

// Directions of cylinders
std::vector<vec3> directions;

// Pairwise angles between cylinders
std::vector<std::vector<double>> angles(6, std::vector<double>(5));

//=============================================================================

Viewer::Viewer(const char *title, int width, int height)
    : MeshViewer(title, width, height) {
  // setup draw modes for viewer
  clear_draw_modes();
  add_draw_mode("Smooth Shading");
  add_draw_mode("Hidden Line");
  add_draw_mode("Texture");
  set_draw_mode("Smooth Shading");

  for (auto &entry : fs::recursive_directory_iterator(point_dir)) {
    if (entry.is_regular_file()) {
      pointsets.push_back(entry.path());
    }
  }
}

//-----------------------------------------------------------------------------

bool Viewer::load_data(const char *_filename) {
  bool ok;
  mesh_.clear();

  ok = pointset_.read_data(_filename);
  if (!ok) {
    std::cerr << "cannot read file " << _filename << std::endl;
    return false;
  }

  // update scene center and bounds
  BoundingBox bb = pointset_.bounds();
  set_scene((vec3)bb.center(), 0.5 * bb.size());

  if (mesh_.n_vertices() != pointset_.n_vertices())
    draw_pointset_ = true;

  return true;
}

//-----------------------------------------------------------------------------

void Viewer::draw(const std::string &draw_mode) {
  MeshViewer::draw(draw_mode);

  if (draw_pointset_)
    pointset_.draw(projection_matrix_, modelview_matrix_, "Points");
}

//-----------------------------------------------------------------------------

void Viewer::keyboard(int key, int scancode, int action, int mods) {
  if (action != GLFW_PRESS && action != GLFW_REPEAT)
    return;

  switch (key) {
  case GLFW_KEY_BACKSPACE: // reload model
  {
    clusters_.clear();
    directions.clear();
    angles = std::vector(6, std::vector<double>(5));

    load_data(filename_.c_str());
    break;
  }
  default: {
    MeshViewer::keyboard(key, scancode, action, mods);
    break;
  }
  }
}

//-----------------------------------------------------------------------------

void Viewer::process_imgui() {
  if (ImGui::CollapsingHeader("Load pointset or mesh",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    static std::string current_pointset = "- load pointset -";
    static std::string current_mesh = "- load mesh -";

    ImGui::Text("Point Cloud");
    ImGui::Indent(10);

    ImGui::PushItemWidth(150);
    if (ImGui::BeginCombo("##PC to load", current_pointset.c_str())) {
      ImGui::Selectable("- load pointset -", false);
      for (auto item : pointsets) {
        bool is_selected = (current_pointset == item);
        if (ImGui::Selectable(item.c_str(), is_selected)) {
          current_mesh = "- load mesh -";
          current_pointset = item;
          std::string fn = current_pointset;
          filename_ = fn;
          load_data(fn.c_str());
        }
      }
      ImGui::EndCombo();
    }
    ImGui::PopItemWidth();

    // output point statistics
    ImGui::BulletText("%d points", (int)pointset_.points_.size());
    ImGui::Unindent(10);

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    static float eps = 5.0;

    ImGui::Text("DBSCAN Epsilon");
    ImGui::SliderFloat("##DBSCAN Epsilon", &eps, 1.0, 10.0);

    ImGui::Spacing();

    static int min_pts = 6;

    ImGui::Text("DBSCAN MinPts");
    ImGui::SliderInt("##DBSCAN MinPts", &min_pts, 1, 20);

    ImGui::Spacing();

    if (ImGui::Button("DBSCAN Clustering")) {
      clusters_ = pointset_.cluster_dbscan(eps, min_pts, max_cluster_id_);
      std::cout << "Max cluster ID: " << max_cluster_id_ << '\n';
      std::cout << "Points: " << pointset_.points_.size()
                << ", Clusters: " << clusters_.size() << '\n';

      for (int i = 0; i < pointset_.vertices_size(); i++) {
        if (clusters_[i] < 6)
          pointset_.colors_[i] = colors[clusters_[i]];
        else
          pointset_.colors_[i] = Color(0.0, 0.0, 0.0);
      }

      pointset_.update_opengl();
    }

    ImGui::Spacing();

    static float ransac_eps = 2.5;
    static int ransac_min_pts = 25;

    ImGui::Text("RANSAC Epsilon");
    ImGui::SliderFloat("##RANSAC Epsilon", &ransac_eps, 0.1, 5.0);

    ImGui::Text("RANSAC MinPts");
    ImGui::SliderInt("##RANSAC MinPts", &ransac_min_pts, 10, 100);

    ImGui::Spacing();

    if (ImGui::Button("RANSAC")) {
      // Apply RANSAC to clusters
      for (int cluster = 0; cluster <= max_cluster_id_; cluster++) {
        std::vector<vec3> points = Clusters::get_points_from_cluster(cluster);

        auto ransac = Ransac(cluster, ransac_eps, ransac_min_pts);
        std::vector<unsigned int> cs = ransac.run(5);
        std::cout << "Cluster: " << points.size() << ", CS: " << cs.size()
                  << '\n';

        for (int i = 0; i < pointset_.points_.size(); i++) {
          // If point is outlier (it is not in the consensus set)
          if (clusters_[i] == cluster &&
              std::find(cs.begin(), cs.end(), i) == cs.end()) {
            // Remove from any handled cluster and set color
            clusters_[i] = max_cluster_id_ + 10;
            pointset_.colors_[i] = Color(25.0, 25.0, 25.0);
          }
        }
      }

      pointset_.update_opengl();
    }

    if (ImGui::Button("Fit cylinders") && max_cluster_id_ < 50) {
      fit_cylinders();
      calculate_angles();
    }

    if (ImGui::Button("Fit w/ PCA") && max_cluster_id_ < 50) {
      fit_cylinders_pca_2();
      calculate_angles();
    }

    if (ImGui::Button("Cluster Sweep") && max_cluster_id_ < 50) {
      for (int cluster = 0; cluster <= max_cluster_id_; cluster++) {
        // Collect points from cluster
        std::vector<Point> points = Clusters::get_points_from_cluster(cluster);
        std::vector<std::vector<Point>> clustered_points = ClusterSweep::cluster(points, directions[cluster], 20);

        if (clustered_points.size() >= 2) {
          std::cout << "Cluster Sweep, cluster 1: " << clustered_points[0].size() << ", cluster 2: " << clustered_points[1].size() << std::endl;
        
          for (Point point : clustered_points[0]) {
            unsigned i = std::find(pointset_.points_.begin(), pointset_.points_.end(), point) - pointset_.points_.begin();
            pointset_.colors_[i] = Color(125.0, 0.0, 125.0);
          }

          for (Point point : clustered_points[1]) {
            unsigned i = std::find(pointset_.points_.begin(), pointset_.points_.end(), point) - pointset_.points_.begin();
            pointset_.colors_[i] = Color(0.0, 125.0, 0.0);
          }
        } else {
          std::cout << "Cluster sweep < 2" << std::endl;
        }
      }

      pointset_.update_opengl();
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::Text("Angles:");
    for (int i = 0; i < 6; i++) {
      char other_colors[5];
      int pos = 0;

      for (int c = 0; c < 6; c++) {
        if (colors_short[c] != colors_short[i]) {
          other_colors[pos++] = colors_short[c];
        }
      }

      ImGui::Text("%s", fmt::format("{0}-{1}: {6:.3}° {0}-{2}: {7:.3}° {0}-{3}: "
                              "{8:.3}° {0}-{4}: {9:.3}° {0}-{5}: {10:.3}°",
                              colors_short[i], other_colors[0], other_colors[1],
                              other_colors[2], other_colors[3], other_colors[4],
                              angles[i][0], angles[i][1], angles[i][2],
                              angles[i][3], angles[i][4])
                      .c_str());
    }
  }
}

//-----------------------------------------------------------------------------

void Viewer::fit_cylinders() {
  for (int cluster = 0; cluster <= max_cluster_id_; cluster++) {
    // Collect points from cluster
    std::vector<Point> points = Clusters::get_points_from_cluster(cluster);

    // Fit cylinder
    auto cf = CylinderFitting(points);
    cf.preprocess();

    double rsqr;
    vec3 c;
    vec3 w;

    double err = cf.fit(rsqr, c, w);

    vec3 nw = normalize(w);

    // Calculate point avg
    vec3 avg(0.0);

    for (int i = 0; i < points.size(); i++) {
      avg += points[i];
    }

    avg /= points.size();

    // Store direction to calculate angles later
    directions.push_back(nw);

    // Draw cylinder
    draw_cylinder(avg, nw, rsqr, 100.0, Color(50.0, 50.0, 50.0));
  }
}

//-----------------------------------------------------------------------------

// void Viewer::fit_cylinders_pca()
// {
//     for (int cluster = 0; cluster <= max_cluster_id_; cluster++)
//     {
//         // Collect points from cluster
//         std::vector<Point> points =
//         Clusters::get_points_from_cluster(cluster); int size =
//         static_cast<int>(points.size());

//         // Convert data types
//         cv::Mat data_pts = cv::Mat(size, 3, CV_64F);

//         for (int i = 0; i < data_pts.rows; i++)
//         {
//             data_pts.at<double>(i, 0) = points[i][0];
//             data_pts.at<double>(i, 1) = points[i][1];
//             data_pts.at<double>(i, 2) = points[i][2];
//         }

//         // Fit cylinder using PCA
//         cv::PCA pca(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);

//         Point center = Point(
//             pca.mean.at<double>(0, 0),
//             pca.mean.at<double>(0, 1),
//             pca.mean.at<double>(0, 2)
//         );

//         vec3 eigenvec = Point(
//             pca.eigenvectors.at<double>(0, 0),
//             pca.eigenvectors.at<double>(0, 1),
//             pca.eigenvectors.at<double>(0, 2)
//         );

//         // Store direction to calculate angles later
//         directions.push_back(eigenvec);

//         // Draw cylinder
//         draw_cylinder(center, eigenvec, 2.0, 100.0, Color(50.0, 50.0, 50.0));
//     }
// }

//-----------------------------------------------------------------------------

void Viewer::fit_cylinders_pca_2() {
  for (int cluster = 0; cluster <= max_cluster_id_; cluster++) {
    // Collect points from cluster
    std::vector<Point> points = Clusters::get_points_from_cluster(cluster);
    int size = static_cast<int>(points.size());

    // Convert Point to Eigen::Vector3d
    std::vector<Eigen::VectorXd> points_eigen;

    for (int i = 0; i < size; i++) {
      points_eigen.push_back(
          Eigen::Vector3d{points[i][0], points[i][1], points[i][2]});
    }

    // Fit cylinder using PCA
    PCA pca;
    pca.train(points_eigen, 1);

    Eigen::Vector3d center_eigen = pca.mean();
    Eigen::Vector3d eigenvec_eigen = pca.eigenvectors().col(0);

    Point center(center_eigen[0], center_eigen[1], center_eigen[2]);
    Point eigenvec(eigenvec_eigen[0], eigenvec_eigen[1], eigenvec_eigen[2]);

    std::cout << "Cluster: " << cluster << ", variance: " << pca.eigenvalues().row(0) << std::endl;

    // Store direction to calculate angles later
    directions.push_back(eigenvec);

    // Draw cylinder
    draw_cylinder(center, eigenvec, 2.0, 100.0, Color(50.0, 50.0, 50.0));
  }
}

//-----------------------------------------------------------------------------

void Viewer::calculate_angles() {
  // For now it only goes up to six as no more cylinders are colored
  for (int i = 0; i < 6; i++) {
    int angle_j = 0;

    for (int j = 0; j < 6; j++) {
      if (i != j) {
        angles[i][angle_j++] =
            acos(dot(directions[i], directions[j]) /
                 (norm(directions[i]) * norm(directions[j]))) *
            (360.0 / (2.0 * M_PI));
      }
    }
  }
}

//-----------------------------------------------------------------------------

void Viewer::draw_cylinder(vec3 center, vec3 direction, double radius,
                           double length, Color color) {
  for (double z = -length / 2.0; z < length / 2.0; z += 2.0) {
    vec3 z_center(center + z * direction);

    // for (double a = 0.0; a < 360.0; a += 10.0)
    // {
    //     Point p(
    //         z_center[0] + radius * cos(a),
    //         z_center[1] + radius * sin(a),
    //         center[2] + z
    //     );
    //     Normal n = normalize(p);

    //     pointset_.points_.push_back(p);
    //     pointset_.normals_.push_back(n);
    //     pointset_.colors_.push_back(color);
    // }

    pointset_.points_.push_back(z_center);
    Normal n = normalize(z_center);
    pointset_.normals_.push_back(n);
    pointset_.colors_.push_back(color);
  }

  pointset_.recalculate();
}

//=============================================================================
