#pragma once

#include <Eigen/Dense>
#include <vector>

/// Principal Component Analysis
class PCA
{
public:
    /// default constructor
    PCA() {}

    /// perform PCA on training data, specify how many components to use
    bool train(std::vector<Eigen::VectorXd>& data, const int ncomponents);

    /// return mean vector
    Eigen::VectorXd mean() const { return pca_mean_; }

    /// return eigenvectors
    Eigen::MatrixXd eigenvectors() const { return pca_matrix_; }

    /// dimension of the vector space
    unsigned int dimension() const { return pca_matrix_.rows(); }

    /// number of principal components
    unsigned int components() const { return pca_matrix_.cols(); }

private:
    // PCA model matrix
    Eigen::MatrixXd pca_matrix_;

    // PCA mean vector
    Eigen::VectorXd pca_mean_;

    // PCA eigenvalues
    Eigen::VectorXd eigenvalues_;
};
