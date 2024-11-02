//
// Created by airlab on 11/1/24.
//

#ifndef CBOCONFIG_H
#define CBOCONFIG_H
#define USE_NLOPT
#include <limbo/opt/nlopt_no_grad.hpp>
#include "../pch.h"



namespace limbo
{
    struct Params {
        struct bayes_opt_cboptimizer : public defaults::bayes_opt_cboptimizer {
        };

        struct init_randomsampling {
            BO_PARAM(int, samples, 15);
        };


        struct kernel : public defaults::kernel {
            BO_PARAM(double, noise, 0.0);
        };

        struct kernel_exp : public defaults::kernel_exp {
        };

        struct kernel_squared_exp_ard : public defaults::kernel_squared_exp_ard {
        };

        struct bayes_opt_bobase : public defaults::bayes_opt_bobase {
            BO_PARAM(bool, stats_enabled, false);
        };

        struct stop_maxiterations {
            BO_PARAM(int, iterations, 1);
        };

        struct acqui_eci : public defaults::acqui_eci {
        };

        struct mean_constant {
            BO_PARAM(double, constant, 0.90);
        };

        struct mean_data{
            //    BO_PARAM(double, ard, 0.90);
        };

#ifdef USE_NLOPT
        struct opt_nloptnograd : public defaults::opt_nloptnograd {
        };
#elif defined(USE_LIBCMAES)
        struct opt_cmaes : public defaults::opt_cmaes {
        };
#else
        struct opt_gridsearch : public defaults::opt_gridsearch {
        };
#endif
    };
}

#endif //CBOCONFIG_H
