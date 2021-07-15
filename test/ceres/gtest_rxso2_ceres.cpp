#include "manif/RxSO2.h"
#include "ceres_test_utils.h"

#include <ceres/ceres.h>

using namespace manif;

MANIF_TEST_JACOBIANS_CERES(RxSO2d);

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
