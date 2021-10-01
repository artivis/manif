#include "manif/SE3.h"
#include "ceres_test_utils.h"

#include <ceres/ceres.h>

using namespace manif;

MANIF_TEST_JACOBIANS_CERES(SE3d);

MANIF_RUN_ALL_TEST;
