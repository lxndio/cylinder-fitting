#include "pca.h"
#include <pmp/Timer.h>
#include <iostream>

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

    // start timer
    std::cout << "Compute PCA:\n" << std::flush;
    Timer timer;
    timer.start();

    /** \todo Compute a PCA model.
     * 
     *  1. Compute a mean mesh from our `n` input meshes passed in variable `data` and store it in `pca_mean_` (data and mean are of type Eigen::VectorXd).
     *  2. Construct the data matrix `X` as defined in the lecture slides.
     *  3. Compute the PCA matrix `U` using an eigen-decomposition of `X.transpose()*X` (most efficient version in the lecture slides)
     *  4. Print out the amount of data variance that the model covers with 1,2,3,...,n principal components.
     *  5. Store the first `ncomponents` columns of `U` in `pca_matrix_` and the first `ncomponents` eigenvalues in `eigenvalues_`
     *
     *  Hints:
     *   - Use `Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>` as eigen-decomposition solver.
     *     'https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html' gives an example how to
     *     use it in section 'Computing eigenvalues and eigenvectors'.
     *   - Eigenvalues and eigenvectors will be in reverse order. You can use `vec.reverse()`
     *     to reverse a vector `vec` as well as `mat.rowwise().reverse()` to reverse the rows of a matrix `mat`
     *   - There are functions like `topRows(n)` and `leftCols(n)` to get the first n rows/columns of a vector or matrix.
     *   - The faces and skulls datasets can be represented by 10 and 6 components, respectively.
     *     (If you use more components, you might get some warning that eigenvectors are not orthogonal.)
     *
     */

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

    std::cout << "===================================================" << std::endl;
    std::cout << "data[0]: " << data[0].rows() << " x " << data[0].cols() << std::endl;
    std::cout << "x: " << x.rows() << " x " << x.cols() << std::endl;
    std::cout << "c: " << c.rows() << " x " << c.cols() << std::endl;
    std::cout << "===================================================" << std::endl;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(c);

    pca_matrix_ = eigensolver.eigenvectors().rightCols(ncomponents);
    eigenvalues_ = eigensolver.eigenvalues().rightCols(ncomponents);

    // output final dimensions for debugging
    std::cout << "  PCA matrix is " << pca_matrix_.rows() << " x "
              << pca_matrix_.cols() << std::endl;
    std::cout << "  PCA mean is " << pca_mean_.rows() << " x "
              << pca_mean_.cols() << std::endl;
    std::cout << "  Eigenvalues are " << eigenvalues_.transpose() << std::endl;

    // how long did it take?
    timer.stop();
    std::cout << "done (" << timer << ")\n\n";

    return true;
}
