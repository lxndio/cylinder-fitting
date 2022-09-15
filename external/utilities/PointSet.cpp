//=============================================================================
//
//   Exercise code for the lecture "Geometry Processing"
//   by Prof. Dr. Mario Botsch, TU Dortmund
//
//   Copyright (C) Computer Graphics Group, TU Dortmund.
//
//=============================================================================

// our includes
#include "PointSet.h"
#include "csv-parser/csv.h"
#include "kDTree.h"
#include "pmp/Types.h"
// #include <pmp/algorithms/SurfaceNormals.h>
// #include <pmp/Timer.h>
#include "ygor-clustering/YgorClustering.hpp"

// system includes
#include <fstream>
#include <string>
#include <clocale>

using namespace pmp;

//=============================================================================


// helper function from PMP
template <typename T>
void tfread(FILE* in, const T& t)
{
    size_t n_items = fread((char*)&t, 1, sizeof(t), in);
    assert(n_items > 0);
}

// helper function from PMP
template <typename T>
void tfwrite(FILE* out, const T& t)
{
    size_t n_items = fwrite((char*)&t, 1, sizeof(t), out);
    assert(n_items > 0);
}


//-----------------------------------------------------------------------------


PointSet::PointSet()
    :SurfaceMeshGL()
{
    has_colors_ = false;
}


//-----------------------------------------------------------------------------


bool PointSet::read_data(const char *_filename)
{
    std::setlocale(LC_NUMERIC, "C");

    // extract file extension
    std::string filename(_filename);
    std::string::size_type dot(filename.rfind("."));
    std::string ext = filename.substr(dot+1, filename.length()-dot-1);
    std::transform(ext.begin(), ext.end(), ext.begin(), tolower);

    bool ok = false;
    if (ext == "xyz")
    {
        ok = read_xyz(_filename);
        has_colors_ = false;
    }
    else if (ext == "cnoff")
    {
        ok = read_cnoff(_filename);
        has_colors_ = true;
    }
    else if (ext == "txt")
    {
        ok = read_txt(_filename);
        has_colors_ = true;
    }
    else if (ext == "pts")
    {
        ok = read_pts(_filename);
    }
    else if (ext == "csv")
    {
        ok = read_csv(_filename);
        has_colors_ = true;
    }
    // else
    // {
    //     has_colors_ = false;
    //     try
    //     {
    //         read(_filename);

    //         // make sure normals are in v:normal
    //         SurfaceNormals::compute_vertex_normals(*this);

    //         // set pointset to mesh vertices and normals
    //         auto vnormal = get_vertex_property<Normal>("v:normal");
    //         auto vpoint = get_vertex_property<Point>("v:point");

    //         points_.resize(n_vertices());
    //         normals_.resize(n_vertices());
    //         for (auto v : vertices())
    //         {
    //             points_[v.idx()] = vpoint[v];
    //             normals_[v.idx()] = vnormal[v];
    //         }

    //         ok = true;
    //     }
    //     catch (const IOException& e)
    //     {
    //         ok = false;
    //     }
    // }

    if (!ok)
    {
        std::cerr << "Cannot read " << filename << std::endl;
        return false;
    }

    clear();
    for(size_t i = 0; i < points_.size(); i++)
    {
        add_vertex(points_[i]);
    }

    auto vnormal = vertex_property<Normal>("v:normal");
    for(auto v : vertices())
    {
        vnormal[v] = normals_[v.idx()];
    }

    set_specular(0.15);
    if(!has_colors_)
    {
        set_front_color(Color(1,0,0));
        auto vcolor = get_vertex_property<Normal>("v:color");
        if (vcolor) remove_vertex_property(vcolor);
    }
    else
    {
        auto vcolor = vertex_property<Color>("v:color");
        for(auto v : vertices())
        {
            vcolor[v] = colors_[v.idx()];
        }
    }

    set_point_size(3);

    orig_normals_ = normals_;
    orig_points_ = points_;

    update_opengl_buffers();

    // for debugging and file conversion
    // write_pts("output.pts");

    return true;
}


//-----------------------------------------------------------------------------


void PointSet::reset()
{
    points_ = orig_points_;
    normals_ = orig_normals_;

    update_opengl();
}


//-----------------------------------------------------------------------------


void PointSet::update_opengl()
{
    auto vpoint = get_vertex_property<Point>("v:point");
    auto vnormal = get_vertex_property<Normal>("v:normal");
    auto vcolor = get_vertex_property<Color>("v:color");
    for(auto v : vertices())
    {
        vpoint[v] = points_[v.idx()];
        vnormal[v] = normals_[v.idx()];
        vcolor[v] = colors_[v.idx()];
    }

    update_opengl_buffers();
}


//-----------------------------------------------------------------------------


void PointSet::compute_vertex_colors(SurfaceMesh &mesh)
{
    if(colors_.empty() || !has_colors_)
    {
        std::cerr << "pointset has no colors!" << std::endl;
        return;
    }

    std::cout << "computing vertex colors..." << std::endl;

    kDTree tree(points_);
    // tree.build(10,99);

    // auto vcolor = mesh.vertex_property<Color>("v:color");
    // for(auto v : mesh.vertices())
    // {
    //     Point p = mesh.position(v);
    //     int i_nearest = tree.nearest(p).nearest;

    //     vcolor[v] = colors_[i_nearest];
    // }

    // TODO fix

    std::cout << "done" << std::endl;
}


//-----------------------------------------------------------------------------

void PointSet::cluster()
{

}

//-----------------------------------------------------------------------------

bool
PointSet::
read_xyz(const char* filename)
{
    FILE* in = fopen(filename, "r");
    if (!in) return false;

    char line[200];
    float x, y, z;
    float nx, ny, nz;
    int n;

    points_.clear();
    normals_.clear();
    colors_.clear();

    while (in && !feof(in) && fgets(line, 200, in))
    {
        n = sscanf(line, "%f %f %f %f %f %f", &x, &y, &z, &nx, &ny, &nz);
        if (n >= 6)
        {
            points_.push_back(pmp::Point(x,y,z));
            normals_.push_back(pmp::Normal(nx,ny,nz));
            colors_.push_back(pmp::Color(0.0,0.0,0.0));
        }
    }

    fclose(in);
    return true;
}


//-----------------------------------------------------------------------------


bool
PointSet::
read_cnoff(const char* filename)
{
    FILE* in = fopen(filename, "r");
    if (!in) return false;

    char line[200];
    float x, y, z;
    float nx, ny, nz;
    float cx, cy, cz;
    int n;

    points_.clear();
    normals_.clear();
    colors_.clear();

    while (in && !feof(in) && fgets(line, 200, in))
    {
        n = sscanf(line, "%f %f %f %f %f %f %f %f %f", &x, &y, &z, &nx, &ny, &nz, &cx, &cy, &cz);
        if (n >= 9)
        {
            points_.push_back(pmp::Point(x,y,z));
            normals_.push_back(pmp::Normal(nx,ny,nz));
            colors_.push_back(pmp::Color(cx/255.0,cy/255.0,cz/255.0));
        }
    }

    fclose(in);
    return true;
}


//-----------------------------------------------------------------------------


bool
PointSet::
read_cxyz(const char* filename)
{
    std::ifstream ifs(filename);
    if (!ifs) return false;

    float x, y, z;
    float nx, ny, nz;
    float cx, cy, cz;

    points_.clear();
    normals_.clear();
    colors_.clear();

    std::string dummy;
    std::getline(ifs, dummy);
    std::getline(ifs, dummy);
    while (ifs && !ifs.eof())
    {
        ifs >> x >> y >> z;
        ifs >> nx >> ny >> nz;
        ifs >> cx >> cy >> cz;
        points_.push_back(pmp::Point(x,y,z));
        normals_.push_back(pmp::Normal(nx,ny,nz));
        colors_.push_back(pmp::Color(cx,cy,cz));
    }

    ifs.close();

    return true;
}


//-----------------------------------------------------------------------------


bool
PointSet::
read_txt(const char* filename)
{
    FILE* in = fopen(filename, "r");
    if (!in) return false;

    char line[200];
    float x, y, z;
    float nx, ny, nz;
    float cx, cy, cz;
    int n;

    points_.clear();
    normals_.clear();
    colors_.clear();

    while (in && !feof(in) && fgets(line, 200, in))
    {
        n = sscanf(line, "%f %f %f %f %f %f %f %f %f", &x, &y, &z, &cx, &cy, &cz, &nx, &ny, &nz);
        if (n >= 9)
        {
            points_.push_back(pmp::Point(x,y,z));
            normals_.push_back(pmp::Normal(nx,ny,nz));
            colors_.push_back(pmp::Color(cx/255.0,cy/255.0,cz/255.0));
        }
    }

    fclose(in);
    return true;
}


//-----------------------------------------------------------------------------


bool
PointSet::
write_xyz(const char* filename) const
{
    std::ofstream ofs(filename);

    for (unsigned int i(0); i < points_.size(); i++)
    {
        ofs << points_[i];
        ofs << " ";
        ofs << normals_[i];
        ofs << std::endl;
    }

    ofs.close();

    return true;
}


//-----------------------------------------------------------------------------


bool
PointSet::
write_cnoff(const char* filename) const
{
    std::ofstream ofs(filename);

    ofs << "CNOFF" << std::endl << points_.size() << " 0 0" << std::endl;

    for (unsigned int i(0); i < points_.size(); i++)
    {
        ofs << points_[i];
        ofs << " ";
        ofs << normals_[i];
        ofs << " ";
        ofs << floor(colors_[i][0] * 255);
        ofs << " ";
        ofs << floor(colors_[i][1] * 255);
        ofs << " ";
        ofs << floor(colors_[i][2] * 255);
        ofs << std::endl;
    }

    ofs.close();

    return true;
}


//-----------------------------------------------------------------------------


bool
PointSet::
write_pts(const char* filename) const
{
    FILE* out = fopen(filename, "wb");
    if (!out) return false;

    unsigned int n = points_.size();
    tfwrite(out, n);
    tfwrite(out, has_colors_);

    fwrite((char*)points_.data(), sizeof(pmp::Point), n, out);
    fwrite((char*)normals_.data(), sizeof(pmp::Normal), n, out);
    if (has_colors_)
        fwrite((char*)colors_.data(), sizeof(pmp::Color), n, out);

    fclose(out);
    return true;
}


//-----------------------------------------------------------------------------


bool
PointSet::
read_pts(const char* filename)
{
    FILE* in = fopen(filename, "rb");
    if (!in) return false;

    unsigned int n;
    tfread(in, n);
    tfread(in, has_colors_);

    std::cout << n << " points " << (has_colors_ ? "with" : "without") << " colors\n";

    points_.resize(n);
    fread((char*)points_.data(), sizeof(pmp::Point), n, in);

    normals_.resize(n);
    fread((char*)normals_.data(), sizeof(pmp::Normal), n, in);

    colors_.resize(n, pmp::Color(0,0,0));
    if (has_colors_)
        fread((char*)colors_.data(), sizeof(pmp::Color), n, in);

    fclose(in);
    return true;
}


//-----------------------------------------------------------------------------


bool
PointSet::
read_csv(const char* _filename)
{
    std::string filename(_filename);
    csv::CSVReader reader(filename);

    points_.clear();
    normals_.clear();
    colors_.clear();

    for (csv::CSVRow& row : reader)
    {
        const Scalar x = row["x_coord"].get<Scalar>();
        const Scalar y = row["y_coord"].get<Scalar>();
        const Scalar z = row["z_coord"].get<Scalar>();
        
        const pmp::Point p(x, y, z);
        const pmp::Normal n = normalize(p);
        const pmp::Color c(0.0, 0.0, 0.0);

        points_.push_back(p);
        normals_.push_back(n);
        colors_.push_back(c);
    }

    return true;
}


//=============================================================================
