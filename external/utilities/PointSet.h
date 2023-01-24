//=============================================================================
//
//   Exercise code for the lecture "Geometry Processing"
//   by Prof. Dr. Mario Botsch, TU Dortmund
//
//   Copyright (C) Computer Graphics Group, TU Dortmund.
//
//=============================================================================

#ifndef POINT_SET_H
#define POINT_SET_H

// our includes
#include <pmp/Types.h>
#include <pmp/visualization/SurfaceMeshGL.h>

// system includes
#include <vector>


//==============================================================================
/// \addtogroup utilities
/// @{
//==============================================================================


/// A class representing a point set with normals and colors.
class PointSet : public pmp::SurfaceMeshGL
{
public:

    /// constructor
    PointSet();

    /// encapsulates read functions
    bool read_data(const char* _filename);

    /// resets points and normals to original
    void reset();

    /// copies point cloud data to Surfacemesh for openGL rendering
    void update_opengl();

    /// compute a color per vertex of `mesh` from
    void compute_vertex_colors(SurfaceMesh &mesh);

    /// recalculate
    void recalculate();

    /// cluster the point set using DBSCAN clustering
    std::vector<unsigned int> cluster_dbscan(float eps, unsigned int min_pts, unsigned int &max_cluster_id);

    /// cluster the point set using mean shift clustering
    std::vector<unsigned int> cluster_mean_shift(unsigned int &max_cluster_id);

    /// Write a point set with normals to a .xyz file.
    bool write_xyz(const char* filename) const;

    /// Write a point set with normals and colors to a .cnoff file.
    bool write_cnoff(const char* filename) const;

    /// Write a point set with normals and colors to a binary .pts file.
    bool write_pts(const char* filename) const;

private:

    /// Read a point set with normals from a .xyz file.
    bool read_xyz(const char* filename);

    /// Read a point set with normals and colors from a .cnoff file.
    bool read_cnoff(const char* filename);

    /// Read a point set with normals and colors from a .cxyz file.
    bool read_cxyz(const char* filename);

    /// Read a point set with normals and colors from a .txt file.
    bool read_txt(const char* filename);

    /// Read a point set with normals and colors from a binary .pts file.
    bool read_pts(const char* filename);

    /// Read a point set without normals and colors from a .csv file.
    bool read_csv(const char* filename);

public:

    std::vector<pmp::Point>  points_;
    std::vector<pmp::Normal> normals_;
    std::vector<pmp::Color>  colors_;

    /// Determines if corresponding point is from data or only in the PointSet
    /// to be drawn for visualization
    std::vector<bool> data_;

    std::vector<pmp::Point>  orig_points_;
    std::vector<pmp::Normal> orig_normals_;

    bool has_colors_;
};


//==============================================================================
/// @}
//==============================================================================
#endif // POINT_SET_H
//==============================================================================
