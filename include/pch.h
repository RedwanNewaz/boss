//
// Created by airlab on 10/28/24.
//

#ifndef OMPL_BOW_PCH_H
#define OMPL_BOW_PCH_H

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
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/config.h>
#include <valarray>
#include <limits>

#include <cmath>
#include <memory>
#include <fstream>
#include <sstream>


#define USE_NLOPT
#include <limbo/acqui/gp_ucb.hpp>
#include <limbo/kernel/exp.hpp>
#include <limbo/mean/data.hpp>
#include <limbo/model/gp.hpp>
#include <limbo/tools/macros.hpp>
#include <limbo/stat.hpp>
#include <limbo/bayes_opt/boptimizer.hpp>

#include <limbo/experimental/acqui/eci.hpp>
#include <limbo/experimental/bayes_opt/cboptimizer.hpp>

#endif //OMPL_BOW_PCH_H
