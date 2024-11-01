//
// Created by airlab on 10/31/24.
//

#include "boss/Boss.h"


ompl::control::Boss::Boss(const SpaceInformationPtr &si, ParamPtr param):
        base::Planner(si, "BOSS"), _param(param) {
    siC_ = si.get();
}

ompl::control::Boss::~Boss(void) {

}

// entry point
ompl::base::PlannerStatus ompl::control::Boss::solve(const ompl::base::PlannerTerminationCondition &ptc) {
    // make sure the planner is configured correctly; ompl::base::Planner::checkValidity
    // ensures that there is at least one input state and a ompl::base::Goal object specified
    checkValidity();
    // get a handle to the Goal from the ompl::base::ProblemDefinition member, pdef_
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);
    if (goal_s == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    if (!goal_s->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }


    // get input states with PlannerInputStates helper, pis_
    _currentState = si_->allocState();
    _goalState = si_->allocState();
    while (const base::State *st = pis_.nextStart())
    {
        // st will contain a start state.  Typically this state will
        si_->copyState(_currentState, st);
        // be cloned here and inserted into the Planner's data structure.
    }

    // if needed, sample states from the goal region (and wait until a state is sampled)
    const base::State *st = pis_.nextGoal(ptc);
    si_->copyState(_goalState, st);

    // periodically check if ptc() returns true.
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


    // if it does, terminate planning.
    auto path(std::make_shared<PathControl>(si_));
    bool approximate = false;
    double approxdif;
    limbo::tools::par::init();
    experimental::bayes_opt::CBOptimizer<Params,
    modelfun<GP_t>,
    acquifun<Acqui_t>,
    statsfun<Stat_t>,
    initfun<Init_t>,
    stopcrit<Stop_t>,
    experimental::constraint_modelfun<Constrained_GP_t>>opt;
    while (ptc() == false)
    {
        // Start planning here.
        opt.optimize(*this);
        auto u = opt.best_sample();
        base::State * xstate = predict(u);
        bool valid = validityChecker_->isValid(xstate);
        if(valid)
        {
            _currentState = xstate;
            base::State * state = si_->allocState();
            si_->copyState(state, _currentState);
            path->append(state);
        }
        approxdif = si_->distance(_goalState, xstate);
        approximate = goal->isSatisfied(_currentState, &approxdif);


        if(approximate)
            break;

        // call routines from SpaceInformation (si_) as needed. i.e.,
        // si_->allocStateSampler() for sampling,
        // si_->checkMotion(state1, state2) for state validity, etc...

        // use the Goal pointer to evaluate whether a sampled state satisfies the goal requirements

        // use log macros for informative messaging, i.e., logInfo("Planner found a solution!");
    }

    // When a solution path is computed, save it here
    pdef_->addSolutionPath(path, approximate, approxdif, getName());
//    std::cout << u.transpose() << "  distance " << approxdif << std::endl;
    double x = _currentState->as<base::SE2StateSpace::StateType>()->getX();
    double y = _currentState->as<base::SE2StateSpace::StateType>()->getY();
    OMPL_INFORM("[%s] state (%lf, %lf) remain = %lf", getName().c_str(), x, y, approxdif);

//    return base::PlannerStatus::EXACT_SOLUTION;
    return approximate ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::INFEASIBLE;
}

void ompl::control::Boss::clear(void) {
    Planner::clear();
}

void ompl::control::Boss::setup(void) {
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
//    sc.configurePlannerRange(maxDistance_);
    opt_ = pdef_->getOptimizationObjective();
    validityChecker_ = si_->getStateValidityChecker();

}


std::vector<ompl::base::State *> ompl::control::Boss::getTraj(base::State * xstate, Control * control) const
{
    double pred_time = _param->get_param<double>("predict_time");
    double dt = _param->get_param<double>("dt");
    unsigned int cd = static_cast<int>(pred_time / dt);
    std::vector<base::State *> pstates;
    cd = siC_->propagateWhileValid(xstate, control, cd, pstates, true);
    return pstates;
}

