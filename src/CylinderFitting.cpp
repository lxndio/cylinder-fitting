#include "CylinderFitting.h"
#include "pmp/MatVec.h"
#include <algorithm>
#include <cmath>
#include <limits>

CylinderFitting::CylinderFitting(std::vector<vec3> points)
{
    points_ = points;
}

void CylinderFitting::preprocess()
{
    const unsigned int n = points_.size();

    // Calculate mean of samples
    avg_ = vec3(0.0);

    for (int i = 0; i < n; i++)
    {
        avg_ += points_[i];
    }

    avg_ /= (double)n;

    // Subract mean from every sample for numerical robustness (p. 31)
    for (int i = 0; i < n; i++)
    {
        points_avg_.push_back(points_[i] - avg_);
    }

    Matrix<Scalar, 6, 1> zero(0.0);
    std::vector<Matrix<Scalar, 6, 1>> products;
    mu_ = zero;

    for (int i = 0; i < n; i++)
    {
        products.push_back(Matrix<Scalar, 6, 1>(0.0));

        products[i][0] = points_avg_[i][0] * points_avg_[i][0];
        products[i][1] = points_avg_[i][0] * points_avg_[i][1];
        products[i][2] = points_avg_[i][0] * points_avg_[i][2];
        products[i][3] = points_avg_[i][1] * points_avg_[i][1];
        products[i][4] = points_avg_[i][1] * points_avg_[i][2];
        products[i][5] = points_avg_[i][2] * points_avg_[i][2];

        mu_[0] += products[i][0];
        mu_[1] += 2 * products[i][1];
        mu_[2] += 2 * products[i][2];
        mu_[3] += products[i][3];
        mu_[4] += 2 * products[i][4];
        mu_[5] += products[i][5];
    }

    mu_ /= (double)n;

    f0_ = Matrix<Scalar, 3, 3>(0.0);
    f1_ = Matrix<Scalar, 3, 6>(0.0);
    f2_ = Matrix<Scalar, 6, 6>(0.0);

    for (int i = 0; i < n; i++)
    {
        Matrix<Scalar, 6, 1> delta;

        delta[0] = products[i][0] - mu_[0];
        delta[1] = 2 * products[i][1] - mu_[1];
        delta[2] = 2 * products[i][2] - mu_[2];
        delta[3] = products[i][3] - mu_[3];
        delta[4] = 2 * products[i][4] - mu_[4];
        delta[5] = products[i][5] - mu_[5];

        f0_(0, 0) += products[i][0];
        f0_(0, 1) += products[i][1];
        f0_(0, 2) += products[i][2];
        f0_(1, 1) += products[i][3];
        f0_(1, 2) += products[i][4];
        f0_(2, 2) += products[i][5];

        // Apply outer product u x v = u * v^T
        f1_ += points_avg_[i] * transpose(delta);
        f2_ += delta * transpose(delta);
    }

    f0_ /= (double)n;
    f0_(1, 0) = f0_(0, 1);
    f0_(2, 0) = f0_(0, 2);
    f0_(2, 1) = f0_(1, 2);
    f1_ /= (double)n;
    f2_ /= (double)n;
}

double CylinderFitting::error(Matrix<Scalar, 6, 1> &mu, Matrix<Scalar, 3, 3> &f0, Matrix<Scalar, 3, 6> &f1, Matrix<Scalar, 6, 6> &f2, vec3 &w, vec3 &pc, double &rsqr)
{
    Matrix<Scalar, 3, 3> p = Matrix<Scalar, 3, 3>::identity() - (w * transpose(w));
    
    Matrix<Scalar, 3, 3> s;
    // s(0, 0) = 0.0;
    // s(0, 1) = -w[2];
    // s(0, 2) = w[1];
    // s(1, 0) = w[2];
    // s(1, 1) = 0.0;
    // s(1, 2) = -w[0];
    // s(2, 0) = -w[1];
    // s(2, 1) = w[0];
    // s(2, 2) = 0.0;
    s(0, 0) = 0.0;
    s(1, 0) = -w[2];
    s(2, 0) = w[1];
    s(0, 1) = w[2];
    s(1, 1) = 0.0;
    s(2, 1) = -w[0];
    s(0, 2) = -w[1];
    s(1, 2) = w[0];
    s(2, 2) = 0.0;

    Matrix<Scalar, 3, 3> a = p * f0 * p;
    Matrix<Scalar, 3, 3> hat_a = -(s * a * s);
    Matrix<Scalar, 3, 3> hat_aa = hat_a * a;
    double tr = trace(hat_aa);
    Matrix<Scalar, 3, 3> q = hat_a / tr;
    
    Matrix<Scalar, 6, 1> pp;
    pp(0, 0) = p(0, 0);
    pp(1, 0) = p(0, 1);
    pp(2, 0) = p(0, 2);
    pp(3, 0) = p(1, 1);
    pp(4, 0) = p(1, 2);
    pp(5, 0) = p(2, 2);

    vec3 alpha = f1 * pp;
    vec3 beta = q * alpha;
    
    double error = (dot(pp, f2 * pp) - 4 * dot(alpha, beta) + 4 * dot(beta, f0 * beta)) / (double)points_avg_.size();
    pc = beta;
    rsqr = dot(pp, mu) + dot(beta, beta);

    return error;
}

double CylinderFitting::fit(double &rsqr, vec3 &c, vec3 &w)
{
    double min_error = std::numeric_limits<double>::infinity();
    w = vec3(0.0);
    c = vec3(0.0);
    rsqr = 0.0;

    const int jmax = 10;
    const int imax = 10;

    for (int j = 0; j <= jmax; j++)
    {
        double phi = (M_PI / 2) * j / (double)jmax;
        phi = std::max(0.0, std::min(phi, M_PI / 2));
        double csphi = cos(phi);
        double snphi = sin(phi);

        for (int i = 0; i < imax; i++)
        {
            double theta = (2 * M_PI) * i / (double)imax;
            theta = std::max(0.0, std::min(theta, 2 * M_PI));
            double cstheta = cos(theta);
            double sntheta = sin(theta);
            vec3 current_w(cstheta * snphi, sntheta * snphi, csphi);
            vec3 current_c(0.0);
            double current_rsqr(0.0);
            double err = error(mu_, f0_, f1_, f2_, current_w, current_c, current_rsqr);
        
            if (err < min_error)
            {
                min_error = err;
                w = current_w;
                c = current_c;
                rsqr = current_rsqr;
            }
        }
    }

    c += avg_;

    return min_error;
}

template <typename Scalar, int M>
Scalar trace(Matrix<Scalar, M, M> mat)
{
    Scalar res(0.0);
    
    for (int i = 0; i < M; i++)
    {
        res += mat(i, i);
    }

    return res;
}
