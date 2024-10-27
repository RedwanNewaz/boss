#pragma once 

#include "bo_traj_opt.h"
#include "param_manager2.h"


class BossInterface{
public:
    BossInterface(const std::string& configFile)
    {
        goal_[0] = goal_[1] = 2;
        control_[0] = control_[1] = 0.0;
        auto parameters_ = std::make_shared<param_manager2>(configFile);

        std::function<double(const std::string&)> f = [&](const std::string& param)
        {return parameters_->get_param<double>(param);};
        config_.update_param(f);
        //get static obstacles
        parameters_->get_obstacles(obstacles_);
    }


    void setGoal(double x, double y)
    {
        goal_[0] = x;
        goal_[1] = y;
    }
    void clearDynObs()
    {
        dyn_obstacles_.clear();
    }

    void setDynObs(double x, double y)
    {
        dyn_obstacles_.emplace_back(Point{x, y});
    }

    bool terminate(double x, double y)
    {
        double dx = (goal_[0] - x); 
        double dy = (goal_[1] - y);
        return sqrt(dx * dx + dy * dy) <= config_.goal_radius;
    }

    std::vector<double> getControl() const
    {
        return std::vector<double>{control_[0], control_[1]};
    }

    std::vector<std::vector<double>> getTraj(double x, double y, double theta)
    {
        // compute local trajectory using dynamic window
        State state({{x, y, theta, control_[0], control_[1]}});
        airlab::bo_planner dwa_(state, control_, config_, goal_, obstacles_);
        if (!dyn_obstacles_.empty())
            dwa_.set_dynamic_obstacles(dyn_obstacles_);
        auto ltraj = dwa_.compute_control(control_);

        std::vector<std::vector<double>> result;
        for(auto& s: ltraj)
        {
            std::vector<double> elem;
            for (int i = 0; i < s.size(); ++i) {
                elem.push_back(s[i]);
            }
            result.emplace_back(elem);
        }
        return result;
    }

    double getSampleTime() const
    {
        return config_.dt;
    }
private:
    airlab::Config config_;
    Point goal_;
    Control control_;
    Obstacle obstacles_, dyn_obstacles_;
};