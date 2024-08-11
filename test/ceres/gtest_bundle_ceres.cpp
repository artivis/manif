#include "manif/Bundle.h"
#include "manif/Rn.h"
#include "manif/SO2.h"
#include "manif/SO3.h"

#include "ceres_test_utils.h"

#include <ceres/ceres.h>

using namespace manif;

using Group = Bundle<double, SO3, R3, SO2, R2>;

MANIF_TEST_JACOBIANS_CERES(Group);

MANIF_RUN_ALL_TEST;
