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
        ImGui::SliderFloat("##DBSCAN Epsilon", &eps, 0.0, 10.0);

        ImGui::Spacing();

        if (ImGui::Button("Cluster"))
        {
            auto clusters = pointset_.cluster(eps);

            for (int i = 0; i < pointset_.vertices_size(); i++)
            {
                switch (clusters[i])
                {
                    case 0:
                        pointset_.colors_[i] = Color(125.0, 0.0, 0.0);
                        break;
                    case 1:
                        pointset_.colors_[i] = Color(0.0, 125.0, 0.0);
                        break;
                    case 2:
                        pointset_.colors_[i] = Color(0.0, 0.0, 125.0);
                        break;
                    case 3:
                        pointset_.colors_[i] = Color(125.0, 125.0, 0.0);
                        break;
                    case 4:
                        pointset_.colors_[i] = Color(0.0, 125.0, 125.0);
                        break;
                }
            }

            pointset_.update_opengl();
        }

        if (ImGui::Button("Cylinder"))
        {
            double center_x = 0.0;
            double center_y = 0.0;
            double bottom_z = -200.0;
            double radius = 50.0;

            for (double z = 0.0; z < 100.0; z += 5.0)
            {
                for (double a = 0.0; a < 360.0; a += 10.0)
                {
                    Point p(
                        center_x + radius * cos(a),
                        center_y + radius * sin(a),
                        bottom_z + z
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
        
        if (ImGui::Button("Fit cylinder"))
        {
            auto cf = CylinderFitting(pointset_.points_);
            cf.preprocess();

            double rsqr;
            vec3 c;
            vec3 w;
            
            double err = cf.fit(rsqr, c, w);

            vec3 nw = normalize(w);

            std::cout << "err: " << err << '\n';
            std::cout << "c: " << c << '\n';
            std::cout << "w: " << w << '\n';
            std::cout << "nw: " << nw << '\n';
            std::cout << "rsqr: " << rsqr << '\n';

            // Calculate point avg
            vec3 avg(0.0);

            for (int i = 0; i < pointset_.points_.size(); i++)
            {
                avg += pointset_.points_[i];
            }

            avg /= pointset_.points_.size();

            // Draw cylinder
            for (double z = -50.0; z < 50.0; z += 2.0)
            {
                vec3 center(avg + z * nw);

                for (double a = 0.0; a < 360.0; a += 10.0)
                {
                    Point p(
                        center[0] + rsqr * cos(a),
                        center[1] + rsqr * sin(a),
                        avg[2] + z
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
    }
}

//=============================================================================
