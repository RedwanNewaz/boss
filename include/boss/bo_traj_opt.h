
#pragma once
#include "bo_param.h"
#include "collision_checker.h"

using Traj = std::vector<std::array<double, 5>>;
using Obstacle = std::vector<std::array<double, 2>>;
using State = std::array<double, 5>;
using Window = std::array<double, 4>;
using Point = std::array<double, 2>;
using Control = std::array<double, 2>;


namespace airlab
{
   
    class bo_planner{
    public:

        BO_PARAM(size_t, dim_in, 2);
        BO_PARAM(size_t, dim_out, 1);
        BO_PARAM(size_t, nb_constraints, 1);

        bo_planner(const State &x, Control &u, const Config &config, const Point &goal, const Obstacle &ob):
        config_(config), goal_(goal), x_(x), ob_(ob)
        {

        }

        Traj compute_control(Control &u)
        {
            // # Dynamic Window control
            m_dw = calc_dynamic_window(x_, config_);
            Traj traj = calc_final_input(x_, u, m_dw, config_, goal_, ob_);
            return traj;
        }

        void set_dynamic_obstacles(const Obstacle& dynObs)
        {
            dyn_ob_.clear();
            std::copy(dynObs.begin(), dynObs.end(), std::back_inserter(dyn_ob_));
        }





        Traj compute_control(const State &x, Control &u, const Config &config, const Point &goal, const Obstacle &ob)
        {
            // # Dynamic Window control
            m_dw = calc_dynamic_window(x, config);
            Traj traj = calc_final_input(x, u, m_dw, config, goal, ob);
            return traj;
        }

        Eigen::VectorXd operator()(const Eigen::VectorXd& u) const
        {
            Eigen::VectorXd res(2);
            // we _maximize in [0:1]
            Eigen::Vector2d uu = scaledU(u);
            
            Traj traj = calc_trajectory(x_, uu(0), uu(1), config_, goal_);

            res(0) = exp(-calc_to_goal_cost(traj, goal_, config_));

            // testing the constraints
            // 0: infeasible 1: feasible
            res(1) =  config_.to_obstacle_cost_gain * calc_obstacle_cost(traj, ob_, config_);

            return res;
        }

        Eigen::Vector2d scaledU(const Eigen::VectorXd& u)const
        {
            Eigen::Vector2d uu;
            uu(0) = config_.min_speed + (config_.max_speed - config_.min_speed) * u(0); // max_speed = 0.345;
            uu(1) = -config_.max_yawrate + 2 * config_.max_yawrate * u(1);
            return uu;
        }



    private:
        Window m_dw;
        Config config_; 
        Point goal_; 
        State x_;
        Obstacle ob_, dyn_ob_;  

    protected:
        Window calc_dynamic_window(const State &x, const Config &config) const {
        return {{
                        std::max((x[3] - config.max_accel * config.dt), config.min_speed),
                        std::min((x[3] + config.max_accel * config.dt), config.max_speed),
                        std::max((x[4] - config.max_dyawrate * config.dt), -config.max_yawrate),
                        std::min((x[4] + config.max_dyawrate * config.dt), config.max_yawrate)
                }};
        }

        State motion(State x, Control u, double dt) const  
        {

            x[2] += u[1] * dt;
            x[2] = fmod(x[2] + M_PI, 2 * M_PI) - M_PI;

            x[0] += u[0] * std::cos(x[2]) * dt;
            x[1] += u[0] * std::sin(x[2]) * dt;
            x[3] = u[0];
            x[4] = u[1];
            return x;
        }

        Traj calc_trajectory(State x, double v, double y, Config config, const Point& goal) const {
            Traj traj;
            traj.push_back(x);
            double time = 0.0;
            double distance = std::numeric_limits<double>::max();
            while (time <= config.predict_time && distance >  config.goal_radius){
                x = motion(x, std::array<double, 2>{{v, y}}, config.dt);
                double dx = goal[0] - x[0];
                double dy = goal[1] - x[1];
                distance = sqrt(dx * dx + dy * dy);
                traj.push_back(x);
                time += config.dt;            
            }
            return traj;
        }

        double calc_to_goal_cost(const Traj& traj, const Point& goal, const Config& config) const {

            // double minCost = std::numeric_limits<double>::max();
            // for(const auto & state : traj) {
            //     auto dx = goal[0] - state[0];
            //     auto dy = goal[1] - state[1];
            //     double cost = sqrt(dx * dx + dy * dy);
            //     minCost = std::min(minCost, cost);
            // }
            
            int N = traj.size() - 1;
            auto state = traj[N];
            auto dx = goal[0] - state[0];
            auto dy = goal[1] - state[1];
            double minCost = sqrt(dx * dx + dy * dy);

            return minCost;

        }

        double calc_obstacle_cost(const Traj& traj, const Obstacle& ob, const Config& config) const {

            CollisionChecker cc (ob, traj, config.robot_radius, config.obstacle_length);
           
            double min_dist = cc.get_min_dist(); 

            if(!dyn_ob_.empty())
            {
                CollisionChecker dcc (dyn_ob_, traj, config.robot_radius, 2.0 * config.obstacle_length);
                min_dist = std::min(min_dist, (double)dcc.get_min_dist());
            }
        
          
            return min_dist;
        }

        Traj calc_final_input(const State &x, Control &u, const Window &dw,
                                                  const Config &config, const Point &goal,
                                                  const std::vector<std::array<double, 2>> &ob) const {
           
        using namespace limbo; 
        using Stop_t = boost::fusion::vector<stop::MaxIterations<Params>>;
        using Stat_t = boost::fusion::vector<stat::Samples<Params>,
                stat::BestObservations<Params>,
                stat::AggregatedObservations<Params>>;
        // using Mean_t = mean::Constant<Params>;
        // using Kernel_t = kernel::Exp<Params>;
        using Mean_t = mean::Data<Params>;
        using Kernel_t = kernel::SquaredExpARD<Params>;
        using GP_t = model::GP<Params, Kernel_t, Mean_t>;
        using Constrained_GP_t = model::GP<Params, Kernel_t, Mean_t>;

        using Acqui_t = experimental::acqui::ECI<Params, GP_t, Constrained_GP_t>;
        using Init_t = init::RandomSampling<Params>;

       
       
        tools::par::init();
        experimental::bayes_opt::CBOptimizer<Params,
        modelfun<GP_t>,
        acquifun<Acqui_t>,
        statsfun<Stat_t>,
        initfun<Init_t>,
        stopcrit<Stop_t>,
        experimental::constraint_modelfun<Constrained_GP_t>>
        opt;

        Traj best_traj;
        double best_cost = 0;

        int loop_count = 0;

        while(loop_count++ < 100)
        {
            opt.optimize(*this);
            auto uu = opt.best_sample();
            auto v = scaledU(uu);
            u[0] = v(0);
            u[1] = v(1);
            //if(safe)///
            
            Traj traj = calc_trajectory(x_, v(0), v(1), config, goal);  
            auto min_dist = calc_obstacle_cost(traj, ob_, config_);
            if(min_dist > config_.robot_radius)
                return traj;
            else if(min_dist > best_cost)
            {
                best_cost = min_dist; 
                best_traj = traj; 
            }
        }

        return best_traj;
     
        }     

    };  
}  



