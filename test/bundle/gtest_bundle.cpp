#include <gtest/gtest.h>

#include "manif/Bundle.h"
#include "manif/Rn.h"
#include "manif/SO2.h"
#include "manif/SO3.h"
#include "manif/SE2.h"
#include "manif/SE3.h"

#include "../common_tester.h"

#include <Eigen/StdVector>

#include <array>

using namespace manif;

using GroupA = Bundle<double, R2, SO3, R1>;


TEST(Bundle, StaticSizes)
{
  static_assert(GroupA::BundleSize == 3, "Size error");
  static_assert(GroupA::Dim == 6, "Dimension error");
  static_assert(GroupA::DoF == 6, "Dimension error");
  static_assert(GroupA::RepSize == 7, "Dimension error");

  static_assert(GroupA::DoF == R2d::DoF + SO3d::DoF + R1d::DoF, "Dimension error");
  static_assert(GroupA::RepSize == R2d::RepSize + SO3d::RepSize + R1d::RepSize, "Dimension error");
  static_assert(GroupA::Dim == R2d::Dim + SO3d::Dim + R1d::Dim, "Dimension error");

  static_assert(
    GroupA::Vector::RowsAtCompileTime ==
    R2d::Vector::RowsAtCompileTime + SO3d::Vector::RowsAtCompileTime +
    R1d::Vector::RowsAtCompileTime, "Dimension error");

  static_assert(
    GroupA::Transformation::RowsAtCompileTime ==
    R2d::Transformation::RowsAtCompileTime + SO3d::Transformation::RowsAtCompileTime +
    R1d::Transformation::RowsAtCompileTime, "Dimension error");

  static_assert(
    GroupA::Transformation::ColsAtCompileTime ==
    R2d::Transformation::ColsAtCompileTime + SO3d::Transformation::ColsAtCompileTime +
    R1d::Transformation::ColsAtCompileTime, "Dimension error");

  static_assert(
    GroupA::Tangent::Dim == R2d::Tangent::Dim + SO3d::Tangent::Dim + R1d::Tangent::Dim,
    "Dimension error");
  static_assert(
    GroupA::Tangent::DoF == R2d::Tangent::DoF + SO3d::Tangent::DoF + R1d::Tangent::DoF,
    "Dimension error");
  static_assert(
    GroupA::Tangent::RepSize ==
    R2d::Tangent::RepSize + SO3d::Tangent::RepSize + R1d::Tangent::RepSize, "Dimension error");

  static_assert(
    GroupA::Tangent::LieAlg::RowsAtCompileTime ==
    R2d::Tangent::LieAlg::RowsAtCompileTime + SO3d::Tangent::LieAlg::RowsAtCompileTime +
    R1d::Tangent::LieAlg::RowsAtCompileTime, "Dimension error");

  static_assert(
    GroupA::Tangent::LieAlg::ColsAtCompileTime ==
    R2d::Tangent::LieAlg::ColsAtCompileTime + SO3d::Tangent::LieAlg::ColsAtCompileTime +
    R1d::Tangent::LieAlg::ColsAtCompileTime, "Dimension error");

  static_assert(
    GroupA::Jacobian::RowsAtCompileTime ==
    R2d::Jacobian::RowsAtCompileTime + SO3d::Jacobian::RowsAtCompileTime +
    R1d::Jacobian::RowsAtCompileTime, "Dimension error");

  static_assert(
    GroupA::Jacobian::ColsAtCompileTime ==
    R2d::Jacobian::ColsAtCompileTime + SO3d::Jacobian::ColsAtCompileTime +
    R1d::Jacobian::ColsAtCompileTime, "Dimension error");

  static_assert(
    GroupA::Tangent::Jacobian::RowsAtCompileTime ==
    R2d::Tangent::Jacobian::RowsAtCompileTime + SO3d::Tangent::Jacobian::RowsAtCompileTime +
    R1d::Tangent::Jacobian::RowsAtCompileTime, "Dimension error");

  static_assert(
    GroupA::Tangent::Jacobian::ColsAtCompileTime ==
    R2d::Tangent::Jacobian::ColsAtCompileTime + SO3d::Tangent::Jacobian::ColsAtCompileTime +
    R1d::Tangent::Jacobian::ColsAtCompileTime, "Dimension error");
}


TEST(Bundle, Interface)
{
  GroupA G = GroupA::Random();

  auto Glog = G.log();
  EXPECT_EIGEN_NEAR(Glog.block<0>().coeffs(), G.block<0>().log().coeffs());
  EXPECT_EIGEN_NEAR(Glog.block<1>().coeffs(), G.block<1>().log().coeffs());
  EXPECT_EIGEN_NEAR(Glog.block<2>().coeffs(), G.block<2>().log().coeffs());

  auto Ginv = G.inverse();
  EXPECT_EIGEN_NEAR(G.block<0>().inverse().coeffs(), Ginv.block<0>().coeffs());
  EXPECT_EIGEN_NEAR(G.block<1>().inverse().coeffs(), Ginv.block<1>().coeffs());
  EXPECT_EIGEN_NEAR(G.block<2>().inverse().coeffs(), Ginv.block<2>().coeffs());

  auto G_Ginv = G.compose(Ginv);
  EXPECT_EIGEN_NEAR(G_Ginv.block<0>().inverse().coeffs(), R2d::Identity().coeffs());
  EXPECT_EIGEN_NEAR(G_Ginv.block<1>().inverse().coeffs(), SO3d::Identity().coeffs());
  EXPECT_EIGEN_NEAR(G_Ginv.block<2>().inverse().coeffs(), R1d::Identity().coeffs());

  typename GroupA::Vector vec;

  auto adj = G.adj();
  Eigen::Matrix2d adj0 = adj.block<2, 2>(0, 0);
  EXPECT_EIGEN_NEAR(adj0, G.block<0>().adj());
  Eigen::Matrix3d adj1 = adj.block<3, 3>(2, 2);
  EXPECT_EIGEN_NEAR(adj1, G.block<1>().adj());
  Eigen::Matrix<double, 1, 1> adj2 = adj.block<1, 1>(5, 5);
  EXPECT_EIGEN_NEAR(adj2, G.block<2>().adj());
}


TEST(Bundle, Map)
{
  std::array<double, GroupA::RepSize> data;

  Eigen::Map<GroupA> map(data.data());
  map = GroupA::Random();

  GroupA::DataType datatype = Eigen::Map<GroupA::DataType>(data.data());
  GroupA copy(datatype);

  auto diff = (map.inverse() * copy).log().coeffs();

  EXPECT_EIGEN_NEAR(diff, GroupA::Tangent::DataType::Zero());
}


TEST(BundleTangent, Interface)
{
  typename GroupA::Tangent tangent = GroupA::Tangent::Random();
  auto exp = tangent.exp();

  EXPECT_EIGEN_NEAR(exp.block<0>().coeffs(), tangent.block<0>().exp().coeffs());
  EXPECT_EIGEN_NEAR(exp.block<1>().coeffs(), tangent.block<1>().exp().coeffs());
  EXPECT_EIGEN_NEAR(exp.block<2>().coeffs(), tangent.block<2>().exp().coeffs());
}


TEST(BundleTangent, Jacobians)
{
  auto tangent = GroupA::Random().log();

  {
    auto jac = tangent.rjac();
    Eigen::Matrix2d jac0 = jac.block<2, 2>(0, 0);
    Eigen::Matrix3d jac1 = jac.block<3, 3>(2, 2);
    Eigen::Matrix<double, 1, 1> jac2 = jac.block<1, 1>(5, 5);

    EXPECT_EIGEN_NEAR(jac0, tangent.block<0>().rjac());
    EXPECT_EIGEN_NEAR(jac1, tangent.block<1>().rjac());
    EXPECT_EIGEN_NEAR(jac2, tangent.block<2>().rjac());
  }

  {
    auto jac = tangent.ljac();
    Eigen::Matrix2d jac0 = jac.block<2, 2>(0, 0);
    Eigen::Matrix3d jac1 = jac.block<3, 3>(2, 2);
    Eigen::Matrix<double, 1, 1> jac2 = jac.block<1, 1>(5, 5);

    EXPECT_EIGEN_NEAR(jac0, tangent.block<0>().ljac());
    EXPECT_EIGEN_NEAR(jac1, tangent.block<1>().ljac());
    EXPECT_EIGEN_NEAR(jac2, tangent.block<2>().ljac());
  }

  {
    auto jac = tangent.rjacinv();
    Eigen::Matrix2d jac0 = jac.block<2, 2>(0, 0);
    Eigen::Matrix3d jac1 = jac.block<3, 3>(2, 2);
    Eigen::Matrix<double, 1, 1> jac2 = jac.block<1, 1>(5, 5);

    EXPECT_EIGEN_NEAR(jac0, tangent.block<0>().rjacinv());
    EXPECT_EIGEN_NEAR(jac1, tangent.block<1>().rjacinv());
    EXPECT_EIGEN_NEAR(jac2, tangent.block<2>().rjacinv());
  }

  {
    auto jac = tangent.ljacinv();
    Eigen::Matrix2d jac0 = jac.block<2, 2>(0, 0);
    Eigen::Matrix3d jac1 = jac.block<3, 3>(2, 2);
    Eigen::Matrix<double, 1, 1> jac2 = jac.block<1, 1>(5, 5);

    EXPECT_EIGEN_NEAR(jac0, tangent.block<0>().ljacinv());
    EXPECT_EIGEN_NEAR(jac1, tangent.block<1>().ljacinv());
    EXPECT_EIGEN_NEAR(jac2, tangent.block<2>().ljacinv());
  }

  {
    auto jac = tangent.smallAdj();
    Eigen::Matrix2d jac0 = jac.block<2, 2>(0, 0);
    Eigen::Matrix3d jac1 = jac.block<3, 3>(2, 2);
    Eigen::Matrix<double, 1, 1> jac2 = jac.block<1, 1>(5, 5);

    EXPECT_EIGEN_NEAR(jac0, tangent.block<0>().smallAdj());
    EXPECT_EIGEN_NEAR(jac1, tangent.block<1>().smallAdj());
    EXPECT_EIGEN_NEAR(jac2, tangent.block<2>().smallAdj());
  }
}

MANIF_TEST(GroupA);
MANIF_TEST_MAP(GroupA);
MANIF_TEST_JACOBIANS(GroupA);

using GroupB1 = Bundle<double, R3>;
using GroupB2 = Bundle<double, SO2>;
using GroupB3 = Bundle<double, SE2>;
using GroupB4 = Bundle<double, SO3>;
using GroupB5 = Bundle<double, SE3>;

using GroupC = Bundle<double, R2, SO2, SE2, SO3, SE3>;
using GroupD = Bundle<double, SE2, R7, SO3, R2, R2>;

MANIF_TEST(GroupB1);
MANIF_TEST(GroupB2);
MANIF_TEST(GroupB3);
MANIF_TEST(GroupB4);
MANIF_TEST(GroupB5);
MANIF_TEST(GroupC);
MANIF_TEST(GroupD);

MANIF_TEST_MAP(GroupB1);
MANIF_TEST_MAP(GroupB2);
MANIF_TEST_MAP(GroupB3);
MANIF_TEST_MAP(GroupB4);
MANIF_TEST_MAP(GroupB5);
MANIF_TEST_MAP(GroupC);
MANIF_TEST_MAP(GroupD);

MANIF_TEST_JACOBIANS(GroupB1);
MANIF_TEST_JACOBIANS(GroupB2);
MANIF_TEST_JACOBIANS(GroupB3);
MANIF_TEST_JACOBIANS(GroupB4);
MANIF_TEST_JACOBIANS(GroupB5);
MANIF_TEST_JACOBIANS(GroupC);
MANIF_TEST_JACOBIANS(GroupD);


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
