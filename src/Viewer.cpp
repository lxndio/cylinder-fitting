//=============================================================================

#include "Viewer.h"
#include "ClusterSweep.h"
#include "Clustering.h"
#include "Clusters.h"
#include "CylinderFitting.h"
#include "Ransac.h"
#include "fmt/format.h"
#include "imgui_internal.h"
#include "pca.h"
#include "pmp/MatVec.h"
#include "pmp/Types.h"
#include "pmp/visualization/MeshViewer.h"

#include <algorithm>
#include <cassert>
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

const std::string point_dir = "../data/pointsets/";

std::vector<std::string> pointsets;
PointSet Viewer::pointset_;

const Color colors[] = {
    Color(255.0, 0.0, 0.0),   // red
    Color(255.0, 69.0, 0.0),  // orange
    Color(148.0, 0.0, 211.0), // violet
    Color(0.0, 255.0, 0.0),   // green
    Color(0.0, 139.0, 139.0), // cyan
    Color(0.0, 0.0, 255.0),   // blue
    Color(139.0, 69.0, 19.0), // brown
};

const std::string color_names[] = {
    "red", "orange", "violet", "green", "cyan", "blue", "brown",
};

const char colors_short[] = {'r', 'o', 'v', 'g', 'c', 'b', 'n'};

const unsigned colors_qty = 7;

// Directions of cylinders
std::vector<vec3> directions;

// Pairwise angles between cylinders
std::vector<std::vector<double>> angles(7, std::vector<double>(7));

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

        for (int i = 0; i < 7; i++) {
            angles[i] = std::vector<double>(7);
        }

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
    if (ImGui::CollapsingHeader("Cylinder Fitting",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        static std::string current_pointset = "- load point set -";
        static std::string current_mesh = "- load mesh -";

        ImGui::Text("Point Set");
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
            this->pointset_.only_data_points();

            dbscan_clustering(min_pts, eps);
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // if (ImGui::Button("Fit cylinders") && max_cluster_id_ < 50) {
        //   fit_cylinders();
        //   calculate_angles();
        // }

        if (ImGui::Button("Fit w/ PCA") && max_cluster_id_ < 50) {
            this->pointset_.only_data_points();

            fit_cylinders_pca();
            calculate_angles();
        }

        // ImGui::Spacing();

        // if (ImGui::Button("<-")) {}

        // ImGui::SameLine();
        // ImGui::Text("0");
        // ImGui::SameLine();

        // if (ImGui::Button("->")) {}

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        static float cs_eps = 5.0;

        ImGui::Text("DBSCAN Epsilon");
        ImGui::SliderFloat("##CS DBSCAN Epsilon", &cs_eps, 1.0, 10.0);

        ImGui::Spacing();

        static int cs_min_pts = 6;

        ImGui::Text("CS DBSCAN MinPts");
        ImGui::SliderInt("##CS DBSCAN MinPts", &cs_min_pts, 1, 20);

        ImGui::Spacing();

        static int cs_precision = 5;

        ImGui::Text("CS Precision");
        ImGui::SliderInt("##CS Precision", &cs_precision, 1, 40);

        ImGui::Spacing();

        if (ImGui::Button("Cluster Sweep") && max_cluster_id_ < 50) {
            this->pointset_.only_data_points();

            unsigned int old_max_cluster_id = max_cluster_id_;

            for (int cluster = 0; cluster <= old_max_cluster_id; cluster++) {
                std::cout << "Running cluster sweep for cluster " << cluster
                          << std::endl;
                std::vector<Point> points =
                    Clusters::get_points_from_cluster(this->clusters_, cluster);
                std::vector<std::vector<Point>> clustered_points =
                    ClusterSweep::cluster(points, directions[cluster], cs_precision,
                                          cs_min_pts, cs_eps);

                // Apply new clusters only if cluster sweep detected more than
                // one cluster
                if (clustered_points.size() >= 2) {
                    // First detected cluster will take the place of the
                    // original cluster of the points on which cluster sweep was
                    // run
                    for (int i = 0; i < this->clusters_.size(); i++) {
                        // TODO can setting points to 0 cause issues if there
                        // are points that aren't clustered again by cluster
                        // sweep and therefore remain in cluster 0 after this?
                        if (this->clusters_[i] == cluster)
                            this->clusters_[i] = 0;
                    }

                    for (int c = 0; c < clustered_points.size(); c++) {
                        int new_cluster_id;

                        if (c == 0) {
                            new_cluster_id = cluster;
                        } else {
                            new_cluster_id = ++this->max_cluster_id_;
                        }

                        for (Point point : clustered_points[c]) {
                            unsigned i =
                                std::find(pointset_.points_.begin(),
                                          pointset_.points_.end(), point) -
                                pointset_.points_.begin();

                            this->clusters_[i] = new_cluster_id;
                            this->pointset_.colors_[i] =
                                colors[new_cluster_id % colors_qty];
                        }
                    }
                }
            }

            pointset_.update_opengl();
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::Text("Angles:");

        if (ImGui::BeginTable("angletable", 8)) {
            ImGui::TableSetupColumn("");

            for (std::string color : color_names) {
                ImGui::TableSetupColumn(color.c_str());
            }

            ImGui::TableHeadersRow();

            for (int c0 = 0; c0 < colors_qty; c0++) {
                ImGui::TableNextColumn();
                ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, ImColor(220, 220, 220));
                ImGui::Text("%s", color_names[c0].c_str());
                ImGui::TableNextColumn();

                for (int c1 = 0; c1 < colors_qty; c1++) {
                    if (c0 == c1) {
                        ImGui::TableNextColumn();
                        continue;
                    }

                    ImGui::Text("%s", fmt::format("{0:.3}", angles[c0][c1]).c_str());
                    ImGui::TableNextColumn();
                }

                ImGui::TableNextRow();
            }

            ImGui::EndTable();
        }

        // for (int i = 0; i < 6; i++) {
        //     char other_colors[5];
        //     int pos = 0;

        //     for (int c = 0; c < 6; c++) {
        //         if (colors_short[c] != colors_short[i]) {
        //             other_colors[pos++] = colors_short[c];
        //         }
        //     }

        //     ImGui::Text(
        //         "%s",
        //         fmt::format("{0}-{1}: {6:.3}° {0}-{2}: {7:.3}° {0}-{3}: "
        //                     "{8:.3}° {0}-{4}: {9:.3}° {0}-{5}: {10:.3}°",
        //                     colors_short[i], other_colors[0], other_colors[1],
        //                     other_colors[2], other_colors[3], other_colors[4],
        //                     angles[i][0], angles[i][1], angles[i][2],
        //                     angles[i][3], angles[i][4])
        //             .c_str());
        // }
    }
}

//-----------------------------------------------------------------------------

void Viewer::dbscan_clustering(unsigned min_pts, float eps) {
    Clustering clustering = Clustering();

    clusters_ = clustering.set_points(this->pointset_.points_)
                    ->cluster_dbscan(min_pts, eps)
                    ->get_clusters();

    this->max_cluster_id_ = clustering.get_max_cluster_id();

    for (int i = 0; i < pointset_.points_.size(); i++) {
        if (this->clusters_[i].has_value()) {
            this->pointset_.colors_[i] = colors[this->clusters_[i].value() % colors_qty];
        } else {
            this->pointset_.colors_[i] = Color(0.0, 0.0, 0.0);
        }
    }

    pointset_.update_opengl();
}

//-----------------------------------------------------------------------------

void Viewer::fit_cylinders() {
    for (int cluster = 0; cluster <= max_cluster_id_; cluster++) {
        // Collect points from cluster
        std::vector<Point> points =
            Clusters::get_points_from_cluster(this->clusters_, cluster);

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
        draw_line(avg, nw, rsqr, 100.0, Color(50.0, 50.0, 50.0));
    }
}

//-----------------------------------------------------------------------------

void Viewer::fit_cylinders_pca() {
    directions.clear();

    for (int cluster = 0; cluster <= max_cluster_id_; cluster++) {
        // Collect points from cluster
        std::vector<Point> points =
            Clusters::get_points_from_cluster(this->clusters_, cluster);
        int size = static_cast<int>(points.size());

        if (points.size() == 0)
            continue;

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

        std::cout << "Cluster: " << cluster
                  << ", variance: " << pca.eigenvalues().row(0) << std::endl;

        // Store direction to calculate angles later
        directions.push_back(eigenvec);

        // Draw cylinder
        draw_line(center, eigenvec, 2.0, 100.0, Color(50.0, 50.0, 50.0));
    }
}

//-----------------------------------------------------------------------------

void Viewer::calculate_angles() {
    // For now it only goes up to six as no more cylinders are colored
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            if (i != j && i < directions.size() && j < directions.size()) {
                angles[i][j] =
                    acos(dot(directions[i], directions[j]) /
                         (norm(directions[i]) * norm(directions[j]))) *
                    (360.0 / (2.0 * M_PI));
            } else {
                angles[i][j] = 0.0;
            }
        }
    }
}

//-----------------------------------------------------------------------------

void Viewer::draw_line(vec3 center, vec3 direction, double radius,
                       double length, Color color) {
    for (double z = -length / 2.0; z < length / 2.0; z += 2.0) {
        vec3 z_center(center + z * direction);

        pointset_.points_.push_back(z_center);
        Normal n = normalize(z_center);
        pointset_.normals_.push_back(n);
        pointset_.colors_.push_back(color);
        pointset_.data_.push_back(false);
    }

    pointset_.recalculate();
}

//=============================================================================
