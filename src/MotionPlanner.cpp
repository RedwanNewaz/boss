//
// Created by airlab on 10/28/24.
//

#include "MotionPlanner.h"

MotionPlanner::MotionPlanner(const CCPtr &cc, const ParamPtr &param, double lower, double upper, bool verbose) :
_cc(cc), _param(param), _verbose(verbose)
{
    // Construct the robot state space in which we're planning. We're
    _space = std::make_shared<ob::SE2StateSpace>();
    // Construct a space information instance for this state space
    ob::RealVectorBounds bounds(2);
    bounds.setLow(lower);
    bounds.setHigh(upper);
    _space->setBounds(bounds);
}

void MotionPlanner::plan(std::vector<double> start_pos, std::vector<double> goal_pos, double timeout) {
    ob::ScopedState<> start(_space);

    // Set our robot's starting state
    start->as<ob::SE2StateSpace::StateType>()->setX(start_pos[0]);
    start->as<ob::SE2StateSpace::StateType>()->setY(start_pos[1]);
    start->as<ob::SE2StateSpace::StateType>()->setYaw(M_PI_2);

    // Set our robot's goal state
    ob::ScopedState<> goal(_space);
    goal->as<ob::SE2StateSpace::StateType>()->setX(goal_pos[0]);
    goal->as<ob::SE2StateSpace::StateType>()->setY(goal_pos[1]);
    goal->as<ob::SE2StateSpace::StateType>()->setYaw(M_PI_2);

    // create a control space
    auto cspace = std::make_shared<UnicycleControlSpace>(_space);

    // set the bounds for the control space
    // since u, w have two different domains, control will be generated in normalized domain [0, 1]
    // we then map it to u and w domains
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(0.0);
    cbounds.setHigh(1.0);

    cspace->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    // set state validity checking for this space
    oc::SpaceInformation *si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker(
            [&](const ob::State *state) { return this->isStateValid(si, state); });

    // Setting the propagation routine for this space:
    // KinematicCarModel does NOT use ODESolver
    ss.setStatePropagator(std::make_shared<KinematicCarModel>(ss.getSpaceInformation(), _param));
    ss.setStartAndGoalStates(start, goal, _param->get_param<double>("goal_radius"));
    ss.setup();

    m_findSolution(ss);

}

void MotionPlanner::m_findSolution(oc::SimpleSetup &ss) const {
    ob::PlannerStatus solved = ss.solve(2);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;

        if(_verbose)
            ss.getSolutionPath().asGeometric().printAsMatrix(std::cout);
        const auto &pathStates = ss.getSolutionPath().getStates();
        std::stringstream ss;
        for (size_t i = 0; i < pathStates.size(); ++i)
        {
            // Get the state
            const ob::State* state = pathStates[i];
            // Convert the state to a vector of doubles
            std::vector<double> stateVector;
            _space->copyToReals(stateVector, state);
            std::copy(stateVector.begin(), stateVector.end(), std::ostream_iterator<double>(ss, ", "));
            ss << "\n";
        }
        save_results(ss);
    }
}

void MotionPlanner::save_results(const std::stringstream &ss) const {
    std::ofstream outFile("output.csv");
    if (outFile.is_open()) {
        // Write the stringstream contents to the file
        outFile << ss.str();
        // Close the file
        outFile.close();
        std::cout << "Stringstream contents have been written to output.csv" << std::endl;
    } else {
        std::cerr << "Unable to open file for writing" << std::endl;
    }
}

bool MotionPlanner::isStateValid(const oc::SpaceInformation *si, const ob::State *state) const {
    float x = static_cast<float>(state->as<ob::SE2StateSpace::StateType>()->getX());
    float y = static_cast<float>(state->as<ob::SE2StateSpace::StateType>()->getY());
    float yaw = static_cast<float>(state->as<ob::SE2StateSpace::StateType>()->getYaw());

    return !_cc->collide(x, y, yaw);
}
