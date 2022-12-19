#pragma once

#include "Eigen/Dense"
#include <vector>
class RobPCA
{
public:
    /// Default constructor.
    RobPCA() {}

    /**
     * Perform robust PCA (Hubert et al., 2005) on input data.
     *
     * @param data Input data points.
     * @param ncomponents Number of principal components to be computed.
     * @param alpha Robustness parameter (defaults to 0.75).
     *
     * @returns true or false depending on whether the computation was successful.
    */
    bool robpca(std::vector<Eigen::VectorXd>& data, const int ncomponents, const double alpha = 0.75);

    /// Returns the data mean vector.
    Eigen::VectorXd mean() const { return mean_; }

    /// Returns the computed eigenvectors.
    Eigen::MatrixXd eigenvectors() const { return eigenvectors_; }

    /// Returns the computed eigenvalues.
    Eigen::VectorXd eigenvalues() const { return eigenvalues_; }

private:
    /// data mean vector
    Eigen::VectorXd mean_;

    /// computed eigenvectors
    Eigen::MatrixXd eigenvectors_;

    /// computed eigenvalues
    Eigen::VectorXd eigenvalues_;
};
