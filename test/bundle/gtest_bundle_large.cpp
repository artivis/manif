#include <gtest/gtest.h>

#include "manif/manif.h"

#include "../common_tester.h"

using namespace manif;

using GroupC = Bundle<double, R2, SO2, SE2, SO3, SE3>;
using GroupD = Bundle<double, SE2, R7, SO3, R2, R2>;

MANIF_TEST(GroupC);
MANIF_TEST(GroupD);

MANIF_TEST_MAP(GroupC);
MANIF_TEST_MAP(GroupD);

MANIF_TEST_JACOBIANS(GroupC);
MANIF_TEST_JACOBIANS(GroupD);

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
