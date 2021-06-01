#include <gtest/gtest.h>

#include "manif/manif.h"

#include "../common_tester.h"

using namespace manif;

using GroupB1 = Bundle<double, R3>;
using GroupB2 = Bundle<double, SO2>;
using GroupB3 = Bundle<double, SE2>;
using GroupB4 = Bundle<double, SO3>;
using GroupB5 = Bundle<double, SE3>;

MANIF_TEST(GroupB1);
MANIF_TEST(GroupB2);
MANIF_TEST(GroupB3);
MANIF_TEST(GroupB4);
MANIF_TEST(GroupB5);

MANIF_TEST_MAP(GroupB1);
MANIF_TEST_MAP(GroupB2);
MANIF_TEST_MAP(GroupB3);
MANIF_TEST_MAP(GroupB4);
MANIF_TEST_MAP(GroupB5);

MANIF_TEST_JACOBIANS(GroupB1);
MANIF_TEST_JACOBIANS(GroupB2);
MANIF_TEST_JACOBIANS(GroupB3);
MANIF_TEST_JACOBIANS(GroupB4);
MANIF_TEST_JACOBIANS(GroupB5);

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
