//=============================================================================

#include "Viewer.h"

#include <imgui.h>
#include <fstream>
#include "csv.hpp"

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
const std::string point_dir = "data/pointsets/"; // TODO

std::vector<std::string> pointsets = {
    "bunny.pts",       "mario_hq.pts",   "mario_lq.pts", "mario_laser.pts",
    "open_sphere.pts", "sphere.pts",     "plane.pts",    "myscan_hq.txt",
    "myscan_lq.txt",   "myscan_face.txt"};

//=============================================================================

Viewer::Viewer(const char* title, int width,
                                                   int height)
    : MeshViewer(title, width, height)
{
    // setup draw modes for viewer
    clear_draw_modes();
    add_draw_mode("Smooth Shading");
    add_draw_mode("Hidden Line");
    add_draw_mode("Texture");
    set_draw_mode("Smooth Shading");

    // check which pointsets exist
    // {
    //     std::ifstream ifs;
    //     auto filenames = pointsets;
    //     pointsets.clear();
    //     for (auto filename : filenames)
    //     {
    //         ifs.open(std::string(POINTSET_DIRECTORY) + filename);
    //         if (ifs)
    //             pointsets.push_back(filename);
    //         ifs.close();
    //     }
    // }
}

//-----------------------------------------------------------------------------

bool Viewer::load_data(const char* _filename)
{
    std::string filename(_filename);

    rapidcsv::Document doc(filename);
    
    std::vector<double> xs = doc.GetColumn<double>("x_coord");
    std::vector<double> ys = doc.GetColumn<double>("y_coord");
    std::vector<double> zs = doc.GetColumn<double>("z_coord");

    for (double x : xs) {
        std::cout << x << ", ";
    }

    // // load as pointset
    // ok = pointset_.read_data(_filename);
    // if (!ok)
    // {
    //     std::cerr << "cannot read file " << _filename << std::endl;
    //     return false;
    // }

    // // update scene center and bounds
    // BoundingBox bb = pointset_.bounds();
    // set_scene((vec3)bb.center(), 0.5 * bb.size());

    // if (mesh_.n_vertices() != pointset_.n_vertices())
    //     draw_pointset_ = true;

    // filename_ = filename;

    // // reinit reconstructor
    // if (reconstructor_ != nullptr)
    // {
    //     delete reconstructor_;
    // }
    // reconstructor_ = new SurfaceReconstruction(pointset_, mesh_);

    // if (draw_mode_names_[draw_mode_] == "Texture")
    //     update_curvature_ = true;

    return true;
}

//-----------------------------------------------------------------------------

void Viewer::draw(const std::string& draw_mode)
{
    // if (draw_pointset_)
    //     pointset_.draw(projection_matrix_, modelview_matrix_, "Points");
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
                    std::string fn = point_dir + current_pointset;
                    load_data(fn.c_str());
                }
            }
            ImGui::EndCombo();
        }
        ImGui::PopItemWidth();

        // output point statistics
        // ImGui::BulletText("%d points", (int)pointset_.points_.size());
        ImGui::BulletText("%d points", 0); // TODO
        ImGui::Unindent(10);

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::Text("Mesh");
        ImGui::Indent(10);
    }
}

//=============================================================================
