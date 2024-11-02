//
// Created by airlab on 11/1/24.
//

#include "CboDirectedControlSampler.h"

#include "Koules/KoulesStateSpace.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

unsigned int CboDirectedControlSampler::sampleTo(oc::Control *control, const ob::State *source, ob::State *dest)
{
    OMPL_INFORM("[CboDirectedControlSampler] sampling ");
    const double* dstPos = dest->as<KoulesStateSpace::StateType>()->values;
    double stepSize = si_->getPropagationStepSize();
    unsigned int steps = propagateMax_ ? si_->getMaxControlDuration() :
        cs_.sampleStepCount(si_->getMinControlDuration(), si_->getMaxControlDuration());

    cs_.steer(control, source, dstPos[0], dstPos[1]);
    // perform the first step of propagation
    statePropagator_->propagate(source, control, stepSize, dest);
    // if we reached the goal, we're done
    if (goal_->isSatisfied(dest))
        return 1;
    // if we found a valid state after one step, we can go on
    if (si_->isValid(dest))
    {
        ob::State *temp1 = dest, *temp2 = si_->allocState(), *toDelete = temp2;
        unsigned int r = steps;
        for (unsigned int i = 1 ; i < steps ; ++i)
        {
            statePropagator_->propagate(temp1, control, stepSize, temp2);
            if (goal_->isSatisfied(dest))
            {
                si_->copyState(dest, temp2);
                si_->freeState(toDelete);
                return i + 1;
            }
            if (si_->isValid(temp2))
                std::swap(temp1, temp2);
            else
            {
                // the last valid state is temp1;
                r = i;
                break;
            }
        }
        // if we finished the for-loop without finding an invalid state, the last valid state is temp1
        // make sure dest contains that information
        if (dest != temp1)
            si_->copyState(dest, temp1);
        si_->freeState(toDelete);
        return r;
    }
    // if the first propagation step produced an invalid step, return 0 steps
    // the last valid state is the starting one (assumed to be valid)
    else
    {
        if (dest != source)
            si_->copyState(dest, source);
        return 0;
    }
}