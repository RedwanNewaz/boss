#pragma once 
#include <memory>
#include <iostream>
#include <array>
#include <valarray>
#include <limits>

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

#include "param_manager2.h"

typedef std::shared_ptr<param_manager2>  ParamPtr;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;


  class UnicycleControlSpace : public oc::RealVectorControlSpace
 {
 public:
  
     UnicycleControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 2)
     {
     }
 };

  
 // Kinematic car model object definition.  This class does NOT use ODESolver to propagate the system.
 class KinematicCarModel : public oc::StatePropagator
 {
     public:
         KinematicCarModel(const oc::SpaceInformationPtr &si, ParamPtr param) : oc::StatePropagator(si), param_(param)
         {
             space_     = si->getStateSpace();
             carLength_ = 0.2;
             timeStep_  = param->get_param<double>("dt");
         }
  
         void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const override
         {
             EulerIntegration(state, control, duration, result);
         }
  
     protected:
         // Explicit Euler Method for numerical integration.
        void EulerIntegration(const ob::State *start, const oc::Control *control, const double duration, ob::State *result) const
        {
            double t = timeStep_;
            std::valarray<double> dstate;
            space_->copyState(result, start);
            while (t < duration + std::numeric_limits<double>::epsilon())
            {
                ode(result, control, dstate);
                update(result, timeStep_ * dstate);
                t += timeStep_;
            }
            if (t + std::numeric_limits<double>::epsilon() > duration)
            {
                ode(result, control, dstate);
                update(result, (t - duration) * dstate);
            }
        }

        double scaleYawRate(double w) const
        {
            double max_yawrate = param_->get_param<double>("max_yawrate");
            return -max_yawrate + 2.0 * max_yawrate * w;
        }

        void ode(const ob::State *state, const oc::Control *control, std::valarray<double> &dstate) const
        {
            const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
            double theta = state->as<ob::SE2StateSpace::StateType>()->getYaw();

            dstate.resize(3);
            dstate[2] = scaleYawRate(u[1]);
            theta += dstate[2] * timeStep_;
            theta = fmod(theta + M_PI, 2 * M_PI) - M_PI;
            dstate[0] = u[0] * cos(theta);
            dstate[1] = u[0] * sin(theta);
            
        }
  
         void update(ob::State *state, const std::valarray<double> &dstate) const
         {
             ob::SE2StateSpace::StateType &s = *state->as<ob::SE2StateSpace::StateType>();
             s.setX(s.getX() + dstate[0]);
             s.setY(s.getY() + dstate[1]);
             double yaw = s.getYaw() + dstate[2];
             yaw = fmod(yaw + M_PI, 2 * M_PI) - M_PI;
             s.setYaw(yaw);
             space_->enforceBounds(state);
         }
  
         ob::StateSpacePtr        space_;
         double                   carLength_;
         double                   timeStep_;
         ParamPtr                 param_;  
 };
  