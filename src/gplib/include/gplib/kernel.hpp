#ifndef GP_KERNELS_HPP
#define GP_KERNELS_HPP

#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Core>
#include <math.h>


namespace librav{
    class Kernel{
        public:
            typedef std::shared_ptr<Kernel> Ptr;
            typedef std::shared_ptr<const Kernel> ConstPtr;
            virtual ~Kernel() {};

            // Pure virtual methods to be implemented in a derived class
            virtual double Evaluate(const Eigen::VectorXd& x, const Eigen::VectorXd& y) const = 0;
            virtual long double EvaluateGradient(const Eigen::VectorXd& x, const Eigen::VectorXd& y, size_t ii, size_t jj) const = 0;
            virtual double Partial(const Eigen::VectorXd& x, const Eigen::VectorXd& y, size_t ii) const = 0;
            virtual void Gradient(const Eigen::VectorXd& x, const Eigen::VectorXd& y, Eigen::VectorXd& gradient) const = 0;

            // Access and reset params.
            Eigen::VectorXd Params() {return params_;};
            const Eigen::VectorXd& ImmutableParams() const {return params_;};
            void Reset(const Eigen::VectorXd& params){
                params_ = params;
            }
            void Adjust(double diff, size_t ii){
                params_(ii) += diff;
            }
            void SetParams(const size_t ii, const double param_ii){
                params_(ii) = std::exp(param_ii);
                // params_(ii) = param_ii;
            }
            
        
        protected:
            explicit Kernel(const Eigen::VectorXd& params): params_(params) {};
            Eigen::VectorXd params_;
    };
}

#endif /* GP_KERNELS_HPP */