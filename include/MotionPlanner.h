//
// Created by airlab on 10/28/24.
//

#ifndef OMPL_BOW_MOTIONPLANNER_H
#define OMPL_BOW_MOTIONPLANNER_H
#include "pch.h"
#include "collision_checker.h"
#include "model.h"
#include "boss/Boss.h"

typedef std::shared_ptr<CollisionChecker> CCPtr;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;


class MotionPlanner {
public:
    MotionPlanner(const CCPtr &cc, const ParamPtr &param, double lower, double upper, bool verbose=true);
    void plan(std::vector<double> start_pos, std::vector<double> goal_pos, double timeout=5.0);

protected:
    void save_results(const std::stringstream& ss) const;
    bool isStateValid(const oc::SpaceInformation *si, const ob::State *state) const;
    ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
    {
        return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
    }
private:
    CCPtr _cc;
    ParamPtr _param;
    bool _verbose;
    std::shared_ptr<ob::SE2StateSpace> _space;

    void m_findSolution(oc::SimpleSetup &ss) const;
};


#endif //OMPL_BOW_MOTIONPLANNER_H
