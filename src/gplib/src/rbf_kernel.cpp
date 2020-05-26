#include "gplib/rbf_kernel.hpp"
#include <math.h>

namespace librav{

    // Factory method
    Kernel::Ptr RBFKernel::Create(const Eigen::VectorXd& lengths){
        Kernel::Ptr ptr(new RBFKernel(lengths));
        for (size_t ii = 0; ii < lengths.size(); ii++){
            ptr->SetParams(ii,lengths(ii));
        }
        // std::cout << "The parameter is " << std::endl << ptr->Params() << std::endl;
        return ptr;
    }

    // Constructor
    RBFKernel::RBFKernel(const Eigen::VectorXd& lengths): Kernel(lengths) {};

    // Pure virtual methods to be implemented in a derived class.
    // RBF: k(x, y) = params_(0) * exp(-0.5 (x-y)^T inv(L) (x-y))
    double RBFKernel::Evaluate(const Eigen::VectorXd& x, const Eigen::VectorXd& y) const {
        const Eigen::VectorXd diff = x - y;
        Eigen::VectorXd length_scale(x.size());
        for(size_t ii = 0; ii < x.size(); ii++){
            length_scale(ii) = params_(1);
        }
        // return std::exp(-0.5 * diff.cwiseQuotient(length_scale).squaredNorm()) * params_(0);
        return std::exp(-0.5 * diff.cwiseQuotient(length_scale).squaredNorm());
    }
    
    double RBFKernel::Partial(const Eigen::VectorXd& x, const Eigen::VectorXd& y, size_t ii) const{
        const Eigen::VectorXd diff = x - y;

        // Evaluate the kernel
        Eigen::VectorXd length_scale(x.size());
        for(size_t ii = 0; ii < x.size(); ii++){
            length_scale(ii) = params_(1);
        }
        const double kernel = std::exp(-0.5 * diff.cwiseQuotient(length_scale).squaredNorm());
        return kernel * diff(ii) * diff(ii) / (params_(ii) * params_(ii) * params_(ii));
    }

    void RBFKernel::Gradient(const Eigen::VectorXd& x, const Eigen::VectorXd& y, Eigen::VectorXd& gradient) const{
        const Eigen::VectorXd diff = x - y;
        // Evaluate the kernel
        Eigen::VectorXd length_scale(x.size());
        for(size_t ii = 0; ii < x.size(); ii++){
            length_scale(ii) = params_(1);
        }
        
        const double kernel = std::exp(-0.5 * diff.cwiseQuotient(length_scale).squaredNorm());
        gradient = kernel * diff.cwiseProduct(diff).cwiseQuotient(params_.cwiseProduct(params_).cwiseProduct(params_));
    }

    long double RBFKernel::EvaluateGradient(const Eigen::VectorXd& x, const Eigen::VectorXd& y, size_t ii, size_t jj) const{
        const Eigen::VectorXd diff_scale = x - y;
        const Eigen::VectorXd diff = x/params_(1) - y/params_(1);
        // Evaluate the kernel
        Eigen::VectorXd length_scale(x.size());
        for(size_t ii = 0; ii < x.size(); ii++){
            length_scale(ii) = params_(1);
        }
        // Eigen::VectorXd K_grad(2);
        const double kernel = std::exp(-0.5 * diff_scale.cwiseQuotient(length_scale).squaredNorm());
        // K_grad = kernel * diff.cwiseProduct(diff).cwiseQuotient(params_.cwiseProduct(params_).cwiseProduct(params_));
        long double K_grad = kernel * (diff(0) * diff(0) + diff(1) * diff(1));
        // std::cout << "ii " << ii << ", jj " << jj << ", kernel is " << kernel << " ,squareform is " << (diff(0) * diff(0) + diff(1) * diff(1)) << std::endl;  
        return K_grad;
    }

}






