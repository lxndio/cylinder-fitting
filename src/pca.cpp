#include "pca.h"
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/SVD/JacobiSVD.h"
#include <algorithm>
#include <pmp/Timer.h>
#include <iostream>
#include <vector>

using namespace pmp;

bool PCA::train(std::vector<Eigen::VectorXd>& data, const int ncomponents)
{
    // check input parameters
    if (data.empty())
    {
        std::cerr << "PCA::train: empty data\n";
        return false;
    }
    if (ncomponents < 1)
    {
        std::cerr << "PCA::train: bad #components\n";
        return false;
    }

    // n: number of training vectors
    // m: dimension of training vectors
    const unsigned int n = data.size();
    const unsigned int m = data[0].size();

    for (int i = 0; i < n; ++i)
    {
        if (data[i].size() != m)
        {
            std::cerr << "PCA::train: data dimension does not match\n";
            return false;
        }
    }

    pca_mean_ = Eigen::VectorXd::Zero(m);

    for (int i = 0; i < n; i++)
    {
        pca_mean_ += data[i];
    }

    pca_mean_ /= n;

    Eigen::MatrixXd x(m, n);
    
    for (int i = 0; i < n; i++)
    {
        x.col(i) = (data[i] - pca_mean_) / sqrt(n - 1);
    }

    Eigen::MatrixXd c = x * x.transpose();

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(c);

    auto pca_matrix = eigensolver.eigenvectors().rowwise().reverse();
    auto eigenvalues = eigensolver.eigenvalues().reverse();

    pca_matrix_ = pca_matrix.leftCols(ncomponents);
    eigenvalues_ = eigenvalues.topRows(ncomponents);

    return true;
}
