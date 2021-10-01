#include "manif/Rn.h"
#include "ceres_test_utils.h"

#include <ceres/ceres.h>

using namespace manif;

MANIF_TEST_JACOBIANS_CERES(R1d);
MANIF_TEST_JACOBIANS_CERES(R3d);
MANIF_TEST_JACOBIANS_CERES(R5d);
MANIF_TEST_JACOBIANS_CERES(R7d);
MANIF_TEST_JACOBIANS_CERES(R9d);

MANIF_RUN_ALL_TEST;
