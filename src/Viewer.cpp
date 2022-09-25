//=============================================================================

#include "Viewer.h"
#include "pmp/MatVec.h"
#include "pmp/Types.h"
#include "pmp/visualization/MeshViewer.h"
#include "CylinderFitting.h"

#include <imgui.h>
#include <fstream>
#include <string>
#include <filesystem>

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

const Color colors[] = {
    Color(125.0, 0.0, 0.0),
    Color(0.0, 125.0, 0.0),
    Color(0.0, 0.0, 125.0),
    Color(125.0, 125.0, 0.0),
    Color(0.0, 125.0, 125.0),
    Color(125.0, 0.0, 125.0)
};

//=============================================================================

Viewer::Viewer(const char* title, int width, int height)
    : MeshViewer(title, width, height)
{
    // setup draw modes for viewer
    clear_draw_modes();
    add_draw_mode("Smooth Shading");
    add_draw_mode("Hidden Line");
    add_draw_mode("Texture");
    set_draw_mode("Smooth Shading");

    for (auto & entry : fs::recursive_directory_iterator(point_dir))
    {
        if (entry.is_regular_file())
        {
            pointsets.push_back(entry.path());
        }
    }
}

//-----------------------------------------------------------------------------

bool Viewer::load_data(const char* _filename)
{
    bool ok;
    mesh_.clear();
    
    ok = pointset_.read_data(_filename);
    if (!ok)
    {
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

void Viewer::draw(const std::string& draw_mode)
{
    MeshViewer::draw(draw_mode);

    if (draw_pointset_)
        pointset_.draw(projection_matrix_, modelview_matrix_, "Points");
}

//-----------------------------------------------------------------------------

void Viewer::keyboard(int key, int scancode, int action,
                                        int mods)
{
    if (action != GLFW_PRESS && action != GLFW_REPEAT)
        return;

    switch (key)
    {
        case GLFW_KEY_BACKSPACE: // reload model
        {
            load_data(filename_.c_str());
            break;
        }
        default:
        {
            MeshViewer::keyboard(key, scancode, action, mods);
            break;
        }
    }
}

//-----------------------------------------------------------------------------

void Viewer::process_imgui()
{
    if (ImGui::CollapsingHeader("Load pointset or mesh",
                                ImGuiTreeNodeFlags_DefaultOpen))
    {
        static std::string current_pointset = "- load pointset -";
        static std::string current_mesh = "- load mesh -";

        ImGui::Text("Point Cloud");
        ImGui::Indent(10);

        ImGui::PushItemWidth(150);
        if (ImGui::BeginCombo("##PC to load", current_pointset.c_str()))
        {
            ImGui::Selectable("- load pointset -", false);
            for (auto item : pointsets)
            {
                bool is_selected = (current_pointset == item);
                if (ImGui::Selectable(item.c_str(), is_selected))
                {
                    current_mesh = "- load mesh -";
                    current_pointset = item;
                    std::string fn = current_pointset;
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

        static float eps = 0.0;

        ImGui::Text("DBSCAN Epsilon");
        ImGui::SliderFloat("##DBSCAN Epsilon", &eps, 1.0, 10.0);

        ImGui::Spacing();

        if (ImGui::Button("Cluster"))
        {
            clusters_ = pointset_.cluster(eps, max_cluster_id_);
            std::cout << max_cluster_id_ << '\n';

            for (int i = 0; i < pointset_.vertices_size(); i++)
            {
                if (clusters_[i] < 6) pointset_.colors_[i] = colors[clusters_[i]];
                else pointset_.colors_[i] = Color(0.0, 0.0, 0.0);
            }

            pointset_.update_opengl();
        }
        
        if (ImGui::Button("Fit cylinders"))
        {
            for (int cluster = 0; cluster < 6; cluster++)
            {
                // Collect points from cluster
                std::vector<Point> points;

                for (int i = 0; i < pointset_.points_.size(); i++)
                {
                    if (clusters_[i] == cluster)
                    {
                        points.push_back(pointset_.points_[i]);
                    }
                }

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

                for (int i = 0; i < points.size(); i++)
                {
                    avg += points[i];
                }

                avg /= points.size();

                // Draw cylinder
                draw_cylinder(avg, nw, rsqr, 100.0);
            }
        }
    }
}

//-----------------------------------------------------------------------------

void Viewer::draw_cylinder(vec3 center, vec3 direction, double radius, double length)
{
    for (double z = -length / 2.0; z < length / 2.0; z += 2.0)
    {
        vec3 z_center(center + z * direction);

        for (double a = 0.0; a < 360.0; a += 10.0)
        {
            Point p(
                z_center[0] + radius * cos(a),
                z_center[1] + radius * sin(a),
                center[2] + z
            );
            Normal n = normalize(p);
            Color c(255.0, 0.0, 0.0);

            pointset_.points_.push_back(p);
            pointset_.normals_.push_back(n);
            pointset_.colors_.push_back(c);
        }
    }

    pointset_.recalculate();
}

//=============================================================================
