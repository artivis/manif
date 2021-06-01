#include <gtest/gtest.h>

#include "manif/manif.h"

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
  EXPECT_EIGEN_NEAR(Glog.element<0>().coeffs(), G.element<0>().log().coeffs());
  EXPECT_EIGEN_NEAR(Glog.element<1>().coeffs(), G.element<1>().log().coeffs());
  EXPECT_EIGEN_NEAR(Glog.element<2>().coeffs(), G.element<2>().log().coeffs());

  auto Ginv = G.inverse();
  EXPECT_EIGEN_NEAR(G.element<0>().inverse().coeffs(), Ginv.element<0>().coeffs());
  EXPECT_EIGEN_NEAR(G.element<1>().inverse().coeffs(), Ginv.element<1>().coeffs());
  EXPECT_EIGEN_NEAR(G.element<2>().inverse().coeffs(), Ginv.element<2>().coeffs());

  auto G_Ginv = G.compose(Ginv);
  EXPECT_EIGEN_NEAR(G_Ginv.element<0>().inverse().coeffs(), R2d::Identity().coeffs());
  EXPECT_EIGEN_NEAR(G_Ginv.element<1>().inverse().coeffs(), SO3d::Identity().coeffs());
  EXPECT_EIGEN_NEAR(G_Ginv.element<2>().inverse().coeffs(), R1d::Identity().coeffs());

  typename GroupA::Vector vec;

  auto adj = G.adj();
  Eigen::Matrix2d adj0 = adj.block<2, 2>(0, 0);
  EXPECT_EIGEN_NEAR(adj0, G.element<0>().adj());
  Eigen::Matrix3d adj1 = adj.block<3, 3>(2, 2);
  EXPECT_EIGEN_NEAR(adj1, G.element<1>().adj());
  Eigen::Matrix<double, 1, 1> adj2 = adj.block<1, 1>(5, 5);
  EXPECT_EIGEN_NEAR(adj2, G.element<2>().adj());
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

  EXPECT_EIGEN_NEAR(exp.element<0>().coeffs(), tangent.element<0>().exp().coeffs());
  EXPECT_EIGEN_NEAR(exp.element<1>().coeffs(), tangent.element<1>().exp().coeffs());
  EXPECT_EIGEN_NEAR(exp.element<2>().coeffs(), tangent.element<2>().exp().coeffs());
}


TEST(BundleTangent, Jacobians)
{
  auto tangent = GroupA::Random().log();

  {
    auto jac = tangent.rjac();
    Eigen::Matrix2d jac0 = jac.block<2, 2>(0, 0);
    Eigen::Matrix3d jac1 = jac.block<3, 3>(2, 2);
    Eigen::Matrix<double, 1, 1> jac2 = jac.block<1, 1>(5, 5);

    EXPECT_EIGEN_NEAR(jac0, tangent.element<0>().rjac());
    EXPECT_EIGEN_NEAR(jac1, tangent.element<1>().rjac());
    EXPECT_EIGEN_NEAR(jac2, tangent.element<2>().rjac());
  }

  {
    auto jac = tangent.ljac();
    Eigen::Matrix2d jac0 = jac.block<2, 2>(0, 0);
    Eigen::Matrix3d jac1 = jac.block<3, 3>(2, 2);
    Eigen::Matrix<double, 1, 1> jac2 = jac.block<1, 1>(5, 5);

    EXPECT_EIGEN_NEAR(jac0, tangent.element<0>().ljac());
    EXPECT_EIGEN_NEAR(jac1, tangent.element<1>().ljac());
    EXPECT_EIGEN_NEAR(jac2, tangent.element<2>().ljac());
  }

  {
    auto jac = tangent.rjacinv();
    Eigen::Matrix2d jac0 = jac.block<2, 2>(0, 0);
    Eigen::Matrix3d jac1 = jac.block<3, 3>(2, 2);
    Eigen::Matrix<double, 1, 1> jac2 = jac.block<1, 1>(5, 5);

    EXPECT_EIGEN_NEAR(jac0, tangent.element<0>().rjacinv());
    EXPECT_EIGEN_NEAR(jac1, tangent.element<1>().rjacinv());
    EXPECT_EIGEN_NEAR(jac2, tangent.element<2>().rjacinv());
  }

  {
    auto jac = tangent.ljacinv();
    Eigen::Matrix2d jac0 = jac.block<2, 2>(0, 0);
    Eigen::Matrix3d jac1 = jac.block<3, 3>(2, 2);
    Eigen::Matrix<double, 1, 1> jac2 = jac.block<1, 1>(5, 5);

    EXPECT_EIGEN_NEAR(jac0, tangent.element<0>().ljacinv());
    EXPECT_EIGEN_NEAR(jac1, tangent.element<1>().ljacinv());
    EXPECT_EIGEN_NEAR(jac2, tangent.element<2>().ljacinv());
  }

  {
    auto jac = tangent.smallAdj();
    Eigen::Matrix2d jac0 = jac.block<2, 2>(0, 0);
    Eigen::Matrix3d jac1 = jac.block<3, 3>(2, 2);
    Eigen::Matrix<double, 1, 1> jac2 = jac.block<1, 1>(5, 5);

    EXPECT_EIGEN_NEAR(jac0, tangent.element<0>().smallAdj());
    EXPECT_EIGEN_NEAR(jac1, tangent.element<1>().smallAdj());
    EXPECT_EIGEN_NEAR(jac2, tangent.element<2>().smallAdj());
  }
}

MANIF_TEST(GroupA);
MANIF_TEST_MAP(GroupA);
MANIF_TEST_JACOBIANS(GroupA);

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
