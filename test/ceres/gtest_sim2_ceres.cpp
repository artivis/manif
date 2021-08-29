#include "manif/Sim2.h"
#include "ceres_test_utils.h"

#include <ceres/ceres.h>

using namespace manif;

MANIF_TEST_JACOBIANS_CERES(Sim2d);

int main(int argc, char** argv)
{
  std::srand((unsigned int) time(0));
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
