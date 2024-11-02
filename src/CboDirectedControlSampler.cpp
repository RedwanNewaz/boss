//
// Created by airlab on 11/1/24.
//

#include "CboDirectedControlSampler.h"

#include "Koules/KoulesStateSpace.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ob = ompl::base;
namespace oc = ompl::control;
unsigned int CboDirectedControlSampler::sampleTo(oc::Control *control, const ob::State *source, ob::State *dest) {

    // sample goal location
    auto *goal_s = dynamic_cast<ompl::base::GoalSampleableRegion *>(goal_.get());

    if (goalState_ == nullptr)
        goalState_ = si_->allocState();
    goal_s->sampleGoal(goalState_);

    if (currentState_ == nullptr)
        currentState_ = si_->allocState();

    si_->copyState(currentState_, source);

    // configure CBO
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

    limbo::tools::par::init();
    experimental::bayes_opt::CBOptimizer<Params,
    modelfun<GP_t>,
    acquifun<Acqui_t>,
    statsfun<Stat_t>,
    initfun<Init_t>,
    stopcrit<Stop_t>,
    experimental::constraint_modelfun<Constrained_GP_t>>opt;
    opt.optimize(*this);

    auto u = opt.best_sample();
    auto act = getControl(u);
    auto traj = getTraj(currentState_, act);
    double bestCost = 1e7;
    unsigned int index = 0;
    for (size_t i = 0;  i < traj.size(); ++i) {
        auto runningCost = si_->distance(goalState_, traj[i]);
        if (!si_->isValid(traj[i])) {
            int j =  i > 0 ? i - 1 : 0;
            si_->copyState(dest, traj[j]);
            return j;
        }

        if (goal_->isSatisfied(traj[i]))
            return i + 1;
        if (runningCost < bestCost) {
            index = i;
            bestCost = runningCost;
            si_->copyState(dest, traj[i]);
        }
    }
    return index;
}

Eigen::VectorXd CboDirectedControlSampler::operator()(const Eigen::VectorXd& u) const
{
    Eigen::VectorXd res(2);
    // we _maximize in [0:1]
    ompl::base::State * xstate = si_->allocState();
    si_->copyState(xstate, currentState_);
    auto control = getControl(u);
    auto pstates = getTraj(xstate, control);


    auto [valid, cost] = evalValidTraj(pstates);
    res(0) = exp(-cost);
    res(1) = valid ? 1.0: 0.0;

    return res;
}

ompl::control::Control *CboDirectedControlSampler::getControl(const Eigen::VectorXd& u) const
{
    ompl::control::Control *control = si_->allocControl();
    control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[0] = u(0);
    control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[1] = u(1);
    return control;
}

std::pair<bool, double> CboDirectedControlSampler::evalValidTraj(const std::vector<ompl::base::State *>& traj) const
{
    if (traj.size() < 1)
        return std::make_pair(false,10e7);

    double cost = si_->distance(goalState_, traj.back());
    return std::make_pair(true,cost);
}


std::vector<ompl::base::State *> CboDirectedControlSampler::getTraj(ompl::base::State * xstate, ompl::control::Control * control) const
{
    // double pred_time = _param->get_param<double>("predict_time");
    // double dt = _param->get_param<double>("dt");
    // unsigned int cd = static_cast<int>(pred_time / dt);
    unsigned int cd = 20;
    std::vector<ompl::base::State *> pstates;
    cd = si_->propagateWhileValid(xstate, control, cd, pstates, true);
    return pstates;
}