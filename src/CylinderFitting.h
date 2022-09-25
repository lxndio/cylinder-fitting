#pragma once

#include "PointSet.h"

using namespace pmp;

class CylinderFitting
{
public:
    /// constructor
    CylinderFitting(std::vector<vec3> points);

    /// preprocess the points for fitting
    void preprocess();

    /// fit a cylinder to the points
    double fit(double &rsqr, vec3 &c, vec3 &w);

private:
    /// calculate error
    double error(Matrix<Scalar, 6, 1> &mu, Matrix<Scalar, 3, 3> &f0, Matrix<Scalar, 3, 6> &f1, Matrix<Scalar, 6, 6> &f2, vec3 &w, vec3 &pc, double &rsqr);

    /// the points
    std::vector<vec3> points_;

    /// the points translated by the average
    std::vector<vec3> points_avg_;

    vec3 avg_;

    Matrix<Scalar, 6, 1> mu_;

    Matrix<Scalar, 3, 3> f0_;

    Matrix<Scalar, 3, 6> f1_;

    Matrix<Scalar, 6, 6> f2_;
};

/// calculate the trace of a square matrix
template<typename Scalar, int M>
Scalar trace(Matrix<Scalar, M, M> mat);
