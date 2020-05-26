#ifndef GP_RBF_KERNEL_HPP
#define GP_RBF_KERNEL_HPP

#include "gplib/kernel.hpp"

namespace librav{
    class RBFKernel: public Kernel{
        public:
            static Kernel::Ptr Create(const Eigen::VectorXd& lengths);
            ~RBFKernel(){};
            // Mehtods 
            double Evaluate(const Eigen::VectorXd& x, const Eigen::VectorXd& y) const;
            long double EvaluateGradient(const Eigen::VectorXd& x, const Eigen::VectorXd& y, size_t ii, size_t jj) const;
            double Partial(const Eigen::VectorXd& x, const Eigen::VectorXd& y, size_t ii) const;
            void Gradient(const Eigen::VectorXd& x, const Eigen::VectorXd& y, Eigen::VectorXd& gradient) const;
            // void ParamsTrainning();
            
        private:
            explicit RBFKernel(const Eigen::VectorXd& lengths);
    };
}


#endif /* GP_RBF_KERNEL_HPP */