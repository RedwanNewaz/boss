#pragma once
#define USE_NLOPT

#include "fcl/config.h"
#include "fcl/geometry/bvh/BVH_model.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree_array.h"
#include "fcl/broadphase/default_broadphase_callbacks.h"
#include "fcl/geometry/geometric_shape_to_BVH_model.h"
#include "fcl/broadphase/broadphase_SSaP.h"

#include <limbo/acqui/gp_ucb.hpp>
#include <limbo/kernel/exp.hpp>
#include <limbo/mean/data.hpp>
#include <limbo/model/gp.hpp>
#include <limbo/tools/macros.hpp>
#include <limbo/stat.hpp>
#include <limbo/bayes_opt/boptimizer.hpp>

#include <limbo/experimental/acqui/eci.hpp>
#include <limbo/experimental/bayes_opt/cboptimizer.hpp>
