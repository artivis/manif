#ifndef _MANIF_MANIF_TEST_AUTODIFF_COMMON_TESTER_AUTODIFF_H_
#define _MANIF_MANIF_TEST_AUTODIFF_COMMON_TESTER_AUTODIFF_H_

// autodiff include
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>

#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

// autodiff reverse does not support jacobians at the moment
// furthermore, it can't be included in the same compile unit as real/dual
// #include <autodiff/reverse/var.hpp>
// #include <autodiff/reverse/var/eigen.hpp>

#include "manif/autodiff/autodiff.h"

#include "../gtest_manif_utils.h"

#define MANIF_TEST_AUTODIFF(manifold, ad)                                                               \
  using TEST_##manifold##_JACOBIANS_AUTODIFF_##ad##_TESTER = manif::CommonTesterAutodiff<manifold, ad>; \
  INSTANTIATE_TEST_CASE_P(                                                                              \
    TEST_##manifold##_JACOBIANS_AUTODIFF_##ad##_TESTS,                                                  \
    TEST_##manifold##_JACOBIANS_AUTODIFF_##ad##_TESTER,                                                 \
    ::testing::Values(                                                                                  \
      std::make_tuple(                                                                                  \
        manifold::Identity(),                                                                           \
        manifold::Identity(),                                                                           \
        manifold::Tangent::Zero()                                                                       \
      ),                                                                                                \
      std::make_tuple(                                                                                  \
        (manifold::Tangent::Random()*1e-8).exp(),                                                       \
        (manifold::Tangent::Random()*1e-8).exp(),                                                       \
        manifold::Tangent::Random()*1e-8                                                                \
      ),                                                                                                \
      std::make_tuple(                                                                                  \
        manifold::Random(),                                                                             \
        manifold::Random(),                                                                             \
        manifold::Tangent::Random()                                                                     \
      )                                                                                                 \
    ));                                                                                                 \
  TEST_P(TEST_##manifold##_JACOBIANS_AUTODIFF_##ad##_TESTER, TEST_##manifold##_AUTODIFF_##ad##_RMINUS_JACOBIAN)     \
  { evalRminusJac(); } \
  TEST_P(TEST_##manifold##_JACOBIANS_AUTODIFF_##ad##_TESTER, TEST_##manifold##_AUTODIFF_##ad##_LMINUS_JACOBIAN)     \
  { evalLminusJac(); } \
  TEST_P(TEST_##manifold##_JACOBIANS_AUTODIFF_##ad##_TESTER, TEST_##manifold##_AUTODIFF_##ad##_LOG_JACOBIAN)     \
  { evalLogJac(); } \
  TEST_P(TEST_##manifold##_JACOBIANS_AUTODIFF_##ad##_TESTER, TEST_##manifold##_AUTODIFF_##ad##_EXP_JACOBIAN)     \
  { evalExpJac(); } \
  TEST_P(TEST_##manifold##_JACOBIANS_AUTODIFF_##ad##_TESTER, TEST_##manifold##_AUTODIFF_##ad##_INVERSE_JACOBIAN)     \
  { evalInverseJac(); } \
  TEST_P(TEST_##manifold##_JACOBIANS_AUTODIFF_##ad##_TESTER, TEST_##manifold##_AUTODIFF_##ad##_RPLUS_JACOBIAN)     \
  { evalRplusJac(); } \
  TEST_P(TEST_##manifold##_JACOBIANS_AUTODIFF_##ad##_TESTER, TEST_##manifold##_AUTODIFF_##ad##_LPLUS_JACOBIAN)     \
  { evalLplusJac(); } \
  TEST_P(TEST_##manifold##_JACOBIANS_AUTODIFF_##ad##_TESTER, TEST_##manifold##_AUTODIFF_##ad##_COMPOSE_JACOBIAN)     \
  { evalComposeJac(); } \
  TEST_P(TEST_##manifold##_JACOBIANS_AUTODIFF_##ad##_TESTER, TEST_##manifold##_AUTODIFF_##ad##_BETWEEN_JACOBIAN)     \
  { evalBetweenJac(); } \
  TEST_P(TEST_##manifold##_JACOBIANS_AUTODIFF_##ad##_TESTER, TEST_##manifold##_AUTODIFF_##ad##_ACT_JACOBIAN)     \
  { evalActJac(); }

namespace manif{
  template <typename T> using ddual = autodiff::HigherOrderDual<1, T>;
  // template <typename T> using rreal = autodiff::Real<1, T>;
  // template <typename T> using vvar = autodiff::Variable<T>;
}

#define __MANIF_MAKE_TEST_AUTODIFF(manifold, type, ad)       \
  using manifold##type = manifold<type>;    \
  using ad##type##manifold = ad<type>;    \
  MANIF_TEST_AUTODIFF(manifold##type, ad##type##manifold)

#define __MANIF_MAKE_TEST_AUTODIFF_ALL_AD(manifold, type)   \
  __MANIF_MAKE_TEST_AUTODIFF(manifold, type, ddual)         \

  // @todo real seem to have a few issues
  //__MANIF_MAKE_TEST_AUTODIFF(manifold, type, rreal)
  // @note reverse does not support jacobians atm.
  //__MANIF_MAKE_TEST_AUTODIFF(manifold, type, vvar)

#define __MANIF_MAKE_TEST_AUTODIFF_ALL_TYPES(manifold)  \
  __MANIF_MAKE_TEST_AUTODIFF_ALL_AD(manifold, double)

  // @todo float test is too flaky (in [1e-1, 1e-6])
  //__MANIF_MAKE_TEST_AUTODIFF_ALL_AD(manifold, float)

#define MANIF_TEST_AUTODIFF_ALL                 \
  using namespace autodiff;                     \
  using namespace manif;                        \
  __MANIF_MAKE_TEST_AUTODIFF_ALL_TYPES(R2)      \
  __MANIF_MAKE_TEST_AUTODIFF_ALL_TYPES(R5)      \
  __MANIF_MAKE_TEST_AUTODIFF_ALL_TYPES(SO2)     \
  __MANIF_MAKE_TEST_AUTODIFF_ALL_TYPES(SO3)     \
  __MANIF_MAKE_TEST_AUTODIFF_ALL_TYPES(SE2)     \
  __MANIF_MAKE_TEST_AUTODIFF_ALL_TYPES(SE3)     \
  __MANIF_MAKE_TEST_AUTODIFF_ALL_TYPES(SE_2_3)

namespace manif {

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, Derived::DoF, Derived::RepSize>
autodiffLocalReParameterizationJacobian(const manif::LieGroupBase<Derived>& _state) {

  using Scalar = typename Derived::Scalar;
  using ad = autodiff::HigherOrderDual<1, Scalar>;
  using LieGroup = typename Derived::template LieGroupTemplate<ad>;
  using Tangent = typename Derived::Tangent::template TangentTemplate<ad>;

  using Jac = Eigen::Matrix<Scalar, Derived::DoF, Derived::RepSize>;

  LieGroup state = _state.template cast<ad>();
  LieGroup state_other = state;
  Tangent delta;

  auto f = [](const auto& x, const auto& y){
    return x - y;
  };

  Jac J_t_x = autodiff::jacobian(
    f, autodiff::wrt(state), autodiff::at(state, state_other), delta
  );

  MANIF_ASSERT(Tangent::Zero().isApprox(delta));
  MANIF_ASSERT(Derived::RepSize == J_t_x.cols());
  MANIF_ASSERT(Derived::DoF == J_t_x.rows());

  return J_t_x;
}

template <typename _LieGroup, typename Ad>
class CommonTesterAutodiff : public testing::TestWithParam<std::tuple<
  _LieGroup, _LieGroup, typename _LieGroup::Tangent
>> {
  using LieGroup  = _LieGroup;
  using Scalar    = typename LieGroup::Scalar;
  using Tangent   = typename LieGroup::Tangent;
  using Jacobian  = typename LieGroup::Jacobian;

  template <typename _Scalar>
  using LieGroupTemplate = typename LieGroup::template LieGroupTemplate<_Scalar>;

  template <typename _Scalar>
  using TangentTemplate = typename Tangent::template TangentTemplate<_Scalar>;

  using LieGroupAd = LieGroupTemplate<Ad>;
  using TangentAd = TangentTemplate<Ad>;

  using MatrixXs = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

  const LieGroup& getState() const {
    return std::get<0>(this->GetParam());
  }

  const LieGroup& getStateOther() const {
    return std::get<1>(this->GetParam());
  }

  const Tangent& getDelta() const {
    return std::get<2>(this->GetParam());
  }

public:

  CommonTesterAutodiff()  = default;
  ~CommonTesterAutodiff() = default;

  void evalRminusJac() {
    // functor to be evaluated
    auto f = [](const auto& s, const auto& so){
      return s.rminus(so);
    };

    LieGroupAd x = getState().template cast<Ad>();
    LieGroupAd xo = getStateOther().template cast<Ad>();
    TangentAd t;

    MatrixXs adJ_t_x = jacobian(f, wrt(x), at(x, xo), t);
    MatrixXs adJ_t_xo = jacobian(f, wrt(xo), at(x, xo), t);

    Jacobian J_t_x, J_t_xo;
    EXPECT_MANIF_NEAR(
      getState().rminus(getStateOther(), J_t_x, J_t_xo),
      t.template cast<Scalar>(),
      tol_
    );
    EXPECT_EIGEN_NEAR(
      J_t_x, adJ_t_x * autodiffLocalParameterizationJacobian<Ad>(getState()), tol_
    );
    EXPECT_EIGEN_NEAR(
      J_t_xo, adJ_t_xo * autodiffLocalParameterizationJacobian<Ad>(getStateOther()),
      tol_
    );
  }

  void evalLminusJac() {
    auto f = [](const auto& s, const auto& so){
      return s.lminus(so);
    };

    LieGroupAd x = getState().template cast<Ad>();
    LieGroupAd xo = getStateOther().template cast<Ad>();
    TangentAd t;

    MatrixXs adJ_t_x = jacobian(f, wrt(x), at(x, xo), t);
    MatrixXs adJ_t_xo = jacobian(f, wrt(xo), at(x, xo), t);

    Jacobian J_t_x, J_t_xo;
    EXPECT_MANIF_NEAR(
      getState().lminus(getStateOther(), J_t_x, J_t_xo),
      t.template cast<Scalar>(),
      tol_
    );
    EXPECT_EIGEN_NEAR(
      J_t_x, adJ_t_x * autodiffLocalParameterizationJacobian<Ad>(getState()), tol_
    );
    EXPECT_EIGEN_NEAR(
      J_t_xo, adJ_t_xo * autodiffLocalParameterizationJacobian<Ad>(getStateOther()),
      tol_
    );
  }

  void evalRplusJac() {
    auto f = [](const auto& s, const auto& t){
      return s.rplus(t);
    };

    LieGroupAd x = getState().template cast<Ad>();
    TangentAd t = getDelta().template cast<Ad>();
    LieGroupAd xo;

    MatrixXs adJ_xo_x = jacobian(f, wrt(x), at(x, t), xo);
    MatrixXs adJ_xo_t = jacobian(f, wrt(t), at(x, t), xo);

    Jacobian J_xo_x, J_xo_t;
    EXPECT_MANIF_NEAR(
      getState().rplus(getDelta(), J_xo_x, J_xo_t), xo.template cast<Scalar>(), tol_
    );
    EXPECT_EIGEN_NEAR(
      J_xo_x,
      autodiffLocalReParameterizationJacobian(xo.template cast<Scalar>()) *
      adJ_xo_x *
      autodiffLocalParameterizationJacobian<Ad>(getState()),
      tol_
    );
    EXPECT_EIGEN_NEAR(
      J_xo_t,
      autodiffLocalReParameterizationJacobian(xo.template cast<Scalar>()) *
      adJ_xo_t,
      tol_
    );

    // @todo The following compiles. But should it?
    // MatrixXs adJ_xo_x_t = jacobian(f, wrt(x, t), at(x, t), xo);
    // EXPECT_EQ(LieGroup::RepSize, adJ_xo_x_t);
    // EXPECT_EIGEN_NEAR(
    //   J_xo_x,
    //   autodiffLocalReParameterizationJacobian(xo.template cast<Scalar>()) *
    //   adJ_xo_x_t.block(LieGroup::RepSize, LieGroup::RepSize, 0, 0) *
    //   autodiffLocalParameterizationJacobian<Ad>(getState())
    // );
  }

  void evalLplusJac() {
    auto f = [](const auto& s, const auto& t){
      return s.lplus(t);
    };

    LieGroupAd x = getState().template cast<Ad>();
    TangentAd t = getDelta().template cast<Ad>();
    LieGroupAd xo;

    MatrixXs adJ_xo_x = jacobian(f, wrt(x), at(x, t), xo);
    MatrixXs adJ_xo_t = jacobian(f, wrt(t), at(x, t), xo);

    Jacobian J_xo_x, J_xo_t;
    EXPECT_MANIF_NEAR(
      getState().lplus(getDelta(), J_xo_x, J_xo_t), xo.template cast<Scalar>(), tol_
    );
    EXPECT_EIGEN_NEAR(
      J_xo_x,
      autodiffLocalReParameterizationJacobian(xo.template cast<Scalar>()) *
      adJ_xo_x *
      autodiffLocalParameterizationJacobian<Ad>(getState()),
      tol_
    );
    EXPECT_EIGEN_NEAR(
      J_xo_t,
      autodiffLocalReParameterizationJacobian(xo.template cast<Scalar>()) *
      adJ_xo_t,
      tol_
    );
  }

  void evalLogJac() {
    auto f = [](const auto& v){
      return v.log();
    };

    LieGroupAd x = getState().template cast<Ad>();
    TangentAd t;

    MatrixXs adJ_t_x = jacobian(f, wrt(x), at(x), t);

    Jacobian J_t_x;
    EXPECT_MANIF_NEAR(getState().log(J_t_x), t.template cast<Scalar>(), tol_);
    EXPECT_EIGEN_NEAR(
      J_t_x, adJ_t_x * autodiffLocalParameterizationJacobian<Ad>(getState()), tol_
    );
  }

  void evalExpJac() {
    auto f = [](const auto& v){
      return v.exp();
    };

    TangentAd t = getDelta().template cast<Ad>();
    LieGroupAd x;

    MatrixXs adJ_x_t = jacobian(f, wrt(t), at(t), x);

    Jacobian J_x_t;
    EXPECT_MANIF_NEAR(getDelta().exp(J_x_t), x.template cast<Scalar>(), tol_);
    EXPECT_EIGEN_NEAR(
      J_x_t,
      autodiffLocalReParameterizationJacobian(x.template cast<Scalar>()) * adJ_x_t,
      tol_
    );
  }

  void evalInverseJac() {
    auto f = [](const auto& v){
      return v.inverse();
    };

    LieGroupAd x = getState().template cast<Ad>(), y;

    MatrixXs adJ_y_x = jacobian(f, wrt(x), at(x), y);

    Jacobian J_y_x;
    EXPECT_MANIF_NEAR(getState().inverse(J_y_x), y.template cast<Scalar>(), tol_);

    EXPECT_EIGEN_NEAR(
      J_y_x,
      autodiffLocalReParameterizationJacobian(getState().inverse()) *
      adJ_y_x *
      autodiffLocalParameterizationJacobian<Ad>(getState()),
      tol_
    );
  }

  void evalComposeJac() {
    auto f = [](const auto& x, const auto& y){
      return x * y;
    };

    LieGroupAd x = getState().template cast<Ad>();
    LieGroupAd y = getStateOther().template cast<Ad>();
    LieGroupAd z;

    MatrixXs adJ_z_x = jacobian(f, wrt(x), at(x, y), z);
    MatrixXs adJ_z_y = jacobian(f, wrt(y), at(x, y), z);

    Jacobian J_z_x, J_z_y;
    EXPECT_MANIF_NEAR(
      getState().compose(getStateOther(), J_z_x, J_z_y),
      z.template cast<Scalar>(),
      tol_
    );
    EXPECT_EIGEN_NEAR(
      J_z_x,
      autodiffLocalReParameterizationJacobian(z.template cast<Scalar>()) *
      adJ_z_x *
      autodiffLocalParameterizationJacobian<Ad>(getState()),
      tol_
    );
    EXPECT_EIGEN_NEAR(
      J_z_y,
      autodiffLocalReParameterizationJacobian(z.template cast<Scalar>()) *
      adJ_z_y *
      autodiffLocalParameterizationJacobian<Ad>(getStateOther()),
      tol_
    );
  }

  void evalBetweenJac() {
    auto f = [](const auto& x, const auto& y){
      return x.between(y);
    };

    LieGroupAd x = getState().template cast<Ad>();
    LieGroupAd y = getStateOther().template cast<Ad>();
    LieGroupAd z;

    MatrixXs adJ_z_x = jacobian(f, wrt(x), at(x, y), z);
    MatrixXs adJ_z_y = jacobian(f, wrt(y), at(x, y), z);

    Jacobian J_z_x, J_z_y;
    EXPECT_MANIF_NEAR(
      getState().between(getStateOther(), J_z_x, J_z_y),
      z.template cast<Scalar>(),
      tol_
    );
    EXPECT_EIGEN_NEAR(
      J_z_x,
      autodiffLocalReParameterizationJacobian(z.template cast<Scalar>()) *
      adJ_z_x *
      autodiffLocalParameterizationJacobian<Ad>(getState()),
      tol_
    );
    EXPECT_EIGEN_NEAR(
      J_z_y,
      autodiffLocalReParameterizationJacobian(z.template cast<Scalar>()) *
      adJ_z_y *
      autodiffLocalParameterizationJacobian<Ad>(getStateOther()),
      tol_
    );
  }

  void evalActJac() {
    using Vector = typename LieGroupAd::Vector;

    auto f = [](const auto& x, const auto& p){
      return x.act(p);
    };

    LieGroupAd x = getState().template cast<Ad>();
    Vector v = Vector::Random(), vo;

    MatrixXs adJ_vo_x = jacobian(f, wrt(x), at(x, v), vo);
    MatrixXs adJ_vo_v = jacobian(f, wrt(v), at(x, v), vo);

    Eigen::Matrix<Scalar, LieGroup::Dim, LieGroup::DoF> J_vo_x;
    Eigen::Matrix<Scalar, LieGroup::Dim, LieGroup::Dim> J_vo_v;
    EXPECT_EIGEN_NEAR(
      getState().act(v.template cast<Scalar>(), J_vo_x, J_vo_v),
      vo.template cast<Scalar>(),
      tol_
    );
    EXPECT_EIGEN_NEAR(
      J_vo_x, adJ_vo_x * autodiffLocalParameterizationJacobian<Ad>(getState()), tol_
    );
    EXPECT_EIGEN_NEAR(J_vo_v, adJ_vo_v, tol_);
  }

protected:
  // relax eps for float type
  Scalar tol_ = (std::is_same<Scalar, float>::value)? 1e-5 : 1e-10;
};

} // namespace manif

#endif // _MANIF_MANIF_TEST_AUTODIFF_COMMON_TESTER_AUTODIFF_H_