//
// Created by airlab on 10/31/24.
//

#ifndef OMPL_BOW_BOSS_H
#define OMPL_BOW_BOSS_H
#include <ompl/control/planners/PlannerIncludes.h>
#include <ompl/base/Planner.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/control/SpaceInformation.h>
#include <algorithm>
#include "boss/bo_param.h"
#include "param_manager2.h"
#include "pch.h"
#define DEBUG(x) std::cout << x << std::endl



namespace ompl{
    namespace control{
    class Boss : public base::Planner{
        public:
            BO_PARAM(size_t, dim_in, 2);
            BO_PARAM(size_t, dim_out, 1);
            BO_PARAM(size_t, nb_constraints, 1);

            explicit Boss(const SpaceInformationPtr &si, ParamPtr param);
            ~Boss(void) override;
            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);
            virtual void clear(void);
            virtual void setup(void);

            Control *getControl(const Eigen::VectorXd& u) const
            {
                Control *control = siC_->allocControl();
                control->as<RealVectorControlSpace::ControlType>()->values[0] = u(0);
                control->as<RealVectorControlSpace::ControlType>()->values[1] = u(1);
                return control;
            }

            base::State * predict(const Eigen::VectorXd& u) const
            {
                base::State * xstate = siC_->allocState();

                siC_->copyState(xstate, _currentState);
                auto control = getControl(u);
                int step =  1;
                siC_->propagate(xstate, control, step, xstate);
                return xstate;
            }

            std::vector<base::State *> getTraj(base::State * xstate, Control * control)const;



            std::pair<bool, double> evalValidTraj(const std::vector<base::State *>& traj) const
            {
                if (traj.size() < 1)
                    return std::make_pair(false,10e7);

                double cost = si_->distance(_goalState, traj.back());
                return std::make_pair(true,cost);
            }

            Eigen::VectorXd operator()(const Eigen::VectorXd& u) const
            {
                Eigen::VectorXd res(2);
                // we _maximize in [0:1]
                base::State * xstate = si_->allocState();
                si_->copyState(xstate, _currentState);
                auto control = getControl(u);
                auto pstates = getTraj(xstate, control);


                auto [valid, cost] = evalValidTraj(pstates);
                res(0) = exp(-cost);
                res(1) = valid ? 1.0: 0.0;

                return res;
            }




        protected:


        private:
            ParamPtr _param;
            /** \brief Objective we're optimizing */
            base::OptimizationObjectivePtr opt_;
            base::StateValidityCheckerPtr validityChecker_;
            base::State *_currentState, *_goalState;
            /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
            const control::SpaceInformation *siC_;


        };
    }
}




#endif //OMPL_BOW_BOSS_H
