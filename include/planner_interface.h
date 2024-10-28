#pragma once 

#include <iostream>
#include <array>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>


 #include <ompl/control/SpaceInformation.h>
 #include <ompl/control/spaces/RealVectorControlSpace.h>
 #include <ompl/control/ODESolver.h>
 #include <ompl/config.h>
 #include <valarray>
 #include <limits>

#include <cmath>
#include <memory>
#include <fstream>
#include <sstream>

#include "collision_checker.h"
#include "model.h"

typedef std::shared_ptr<CollisionChecker> CCPtr;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;


 

class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si, CCPtr cc) :
        ob::StateValidityChecker(si), _cc(cc) {}
 
    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State* state) const
    {
         // Fetch the state's x and y coordinates
                  // Fetch the state's x and y coordinates
        float x = static_cast<float>(state->as<ob::SE2StateSpace::StateType>()->getX());
        float y = static_cast<float>(state->as<ob::SE2StateSpace::StateType>()->getY());
        float yaw = static_cast<float>(state->as<ob::SE2StateSpace::StateType>()->getYaw());

        return !_cc->collide(x, y, yaw);
    }
 
    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ob::State* state) const
    {
          // Fetch the state's x and y coordinates
        float x = static_cast<float>(state->as<ob::SE2StateSpace::StateType>()->getX());
        float y = static_cast<float>(state->as<ob::SE2StateSpace::StateType>()->getY());
        float yaw = static_cast<float>(state->as<ob::SE2StateSpace::StateType>()->getYaw());

        return _cc->get_min_dist(x, y, yaw);
    }
private:
    CCPtr _cc;
};

class PlannerInterface{

public:
    PlannerInterface(CCPtr cc, double lower, double upper, ParamPtr param):
    _cc(cc), _param(param)
    {
        // Construct the robot state space in which we're planning. We're
        // _space = std::make_shared<ob::SE2StateSpace>(2);
        _space = std::make_shared<ob::SE2StateSpace>();
            // Construct a space information instance for this state space
        ob::RealVectorBounds bounds(2);
        bounds.setLow(lower);
        bounds.setHigh(upper);
        _space->setBounds(bounds);
    }


    void plan(std::vector<double> start_pos, std::vector<double> goal_pos, double timeout=5.0)
    {
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


        // create control space 
         // create a control space
        auto cspace = std::make_shared<UnicycleControlSpace>(_space);
    
        // set the bounds for the control space
        ob::RealVectorBounds cbounds(2);
        cbounds.setLow(_param->get_param<double>("min_speed"));
        cbounds.setHigh(_param->get_param<double>("max_speed"));
    
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
    
        
        ss.setStartAndGoalStates(start, goal, 0.05);
    
        ss.setup();
    
        ob::PlannerStatus solved = ss.solve(10.0);
    
        if (solved)
        {
            std::cout << "Found solution:" << std::endl;
    
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

    


private:
    CCPtr _cc;
    ParamPtr _param;
    std::shared_ptr<ob::SE2StateSpace> _space;

protected:
    ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
    {
        return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
    }

    bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
    {
        float x = static_cast<float>(state->as<ob::SE2StateSpace::StateType>()->getX());
        float y = static_cast<float>(state->as<ob::SE2StateSpace::StateType>()->getY());
        float yaw = static_cast<float>(state->as<ob::SE2StateSpace::StateType>()->getYaw());

        return !_cc->collide(x, y, yaw);
    }

    void save_results(const std::stringstream& ss) const 
    {
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

};
