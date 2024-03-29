//=============================================================================
#pragma once
//=============================================================================

#include <optional>
#include <pmp/visualization/MeshViewer.h>
#include <utilities/PointSet.h>
#include <vector>

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

using namespace pmp;

//== CLASS DEFINITION =========================================================

/// 3D Viewer of all mesh and point cloud processing functions.
class Viewer : public pmp::MeshViewer {
  public:
    /// constructor
    Viewer(const char *title, int width, int height);

    /// load points from file \p filename
    bool load_data(const char *_filename);

    /// draw the scene in different draw modes
    virtual void draw(const std::string &draw_mode) override;

    /// run DBSCAN clustering
    void dbscan_clustering(unsigned min_pts, float eps);

    /// fit cylinders
    void fit_cylinders();

    /// fit cylinders using PCA
    void fit_cylinders_pca();

    /// prepare and run cluster sweep
    void cluster_sweep();

    /// export data to file
    void save(unsigned type);

    /// get points from a specific cluster
    std::vector<Point> get_points_from_cluster(int cluster);

    /// calculate pairwise angles between cylinders
    void calculate_angles();

    /// draw line
    void draw_line(vec3 center, vec3 direction, double radius, double length,
                   Color color);

    static PointSet pointset_;
    static PointSet pointset_before_cs_;

  protected:
    /// this function handles keyboard events
    void keyboard(int key, int code, int action, int mod) override;

    /// draw the scene in different draw modes
    virtual void process_imgui() override;

  private:
    // the loaded point set
    // PointSet pointset_;

    // draw the pointset?
    bool draw_pointset_;

    // UI state
    bool interactive_dbscan = false;
    bool changed_dbscan = false;
    float eps = 5.0;
    int min_pts = 6;

    bool interactive_cs = false;
    bool changed_cs = false;
    float cs_eps = 5.0;
    int cs_min_pts = 6;
    int cs_precision = 5;

    unsigned cs_cluster_current = 0;

    bool exported = false;
};

//=============================================================================
