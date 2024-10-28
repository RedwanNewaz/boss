#pragma once 
#include "pch.h"
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

