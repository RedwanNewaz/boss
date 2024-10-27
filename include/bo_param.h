//
// Created by airlab on 11/22/23.
//

#ifndef WINDOWBOPLANNER_BO_PARAM_H
#define WINDOWBOPLANNER_BO_PARAM_H
#define USE_NLOPT
// #include <limbo/limbo.hpp>
#include "boss_pch.h"


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




namespace airlab
{
    class Config{
    public:
        double max_speed = 0.345;
        double min_speed = -0.345;
        double max_yawrate = 60.0 * M_PI / 180.0;
        double max_accel = 0.4;
        double robot_radius = 0.345 / 2;
        double max_dyawrate = 60.0 * M_PI / 180.0;

        double v_reso = 0.01;
        double yawrate_reso = 0.2 * M_PI / 180.0;

        double dt = 0.03;
        double predict_time = 3.0;
        double to_goal_cost_gain = 1.0;
        double speed_cost_gain = 0.10;
        double to_obstacle_cost_gain = 1.0;

        double goal_radius = 1.0;
        double obstacle_length = 0.25;


        void update_param(std::function<double(const std::string&)>&f)
        {
            max_speed = f("max_speed");
            min_speed = f("min_speed");
            max_yawrate = f("max_yawrate");
            max_accel = f("max_accel");
            robot_radius = f("robot_radius");
            max_dyawrate = f("max_dyawrate");

            v_reso = f("v_reso");
            yawrate_reso = f("yawrate_reso");

            dt = f("dt");
            predict_time = f("predict_time");
            to_goal_cost_gain = f("to_goal_cost_gain");
            speed_cost_gain = f("speed_cost_gain");
            to_obstacle_cost_gain = f("to_obstacle_cost_gain");

            goal_radius = f("goal_radius");
            obstacle_length = f("obstacle_length");
        }
    };
}


#endif //WINDOWBOPLANNER_BO_PARAM_H
