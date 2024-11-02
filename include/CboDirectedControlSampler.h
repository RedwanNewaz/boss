//
// Created by airlab on 11/1/24.
//

#ifndef OMPL_BOW_CBODIRECTEDCONTROLSAMPLER_H
#define OMPL_BOW_CBODIRECTEDCONTROLSAMPLER_H
#include "Koules/KoulesControlSpace.h"
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/Goal.h>

class CboDirectedControlSampler : public ompl::control::DirectedControlSampler
{
public:
    // The KoulesDirectedControlSampler attempts to steer the system towards
    // a desired state. It takes as additional arguments a goal pointer and a
    // a flag called propagateMax. The goal pointer is needed to stop the
    // motion when the goal is reached before the desired state. The
    // propagateMax flag indicates that the motion is always extended
    // upto the maximum control duration (rather than a randomly sampled limit
    // between the min and max control duration)
    CboDirectedControlSampler(const ompl::control::SpaceInformation *si,
                                 const ompl::base::GoalPtr &goal, bool propagateMax)
            : DirectedControlSampler(si), cs_(si->getControlSpace().get()),
              goal_(goal), statePropagator_(si->getStatePropagator()),
              propagateMax_(propagateMax)
    {
    }
    // This sampleTo implementation contains a modified version of the method
    // ompl::control::SpaceInformation::propagateWhileValid, with the key difference
    // that sampleTo also terminates when the goal is reached.
    virtual unsigned int sampleTo(ompl::control::Control *control, const ompl::base::State *source, ompl::base::State *dest);

    virtual unsigned int sampleTo(ompl::control::Control *control, const ompl::control::Control * /* previous */,
                                  const ompl::base::State *source, ompl::base::State *dest)
    {
        return sampleTo(control, source, dest);
    }
protected:
    KoulesControlSampler                     cs_;
    ompl::RNG                                rng_;
    const ompl::base::GoalPtr                goal_;
    const ompl::control::StatePropagatorPtr  statePropagator_;
    bool                                     propagateMax_;

};


#endif //OMPL_BOW_CBODIRECTEDCONTROLSAMPLER_H
