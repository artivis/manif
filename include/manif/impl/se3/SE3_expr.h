#ifndef _MANIF_MANIF_SE3_EXPR_H_
#define _MANIF_MANIF_SE3_EXPR_H_

namespace manif {
namespace internal {

//! @brief Random specialization for SE3Base objects.
template <typename Derived>
struct RandomEvaluatorImpl<SE3Base<Derived>>
{
  template <typename T>
  static void run(T& m)
  {
    // @note:
    // Quaternion::UnitRandom is not available in Eigen 3.3-beta1
    // which is the default version in Ubuntu 16.04
    // So we copy its implementation here.

    using std::sqrt;
    using std::sin;
    using std::cos;

    using Scalar      = typename SE3Base<Derived>::Scalar;
    using Translation = typename SE3Base<Derived>::Translation;
    using Quaternion  = typename SE3Base<Derived>::QuaternionDataType;
    using LieGroup    = typename SE3Base<Derived>::LieGroup;

    const Scalar u1 = Eigen::internal::random<Scalar>(0, 1),
                 u2 = Eigen::internal::random<Scalar>(0, 2*EIGEN_PI),
                 u3 = Eigen::internal::random<Scalar>(0, 2*EIGEN_PI);
    const Scalar a = sqrt(1. - u1),
                 b = sqrt(u1);

    m = LieGroup(Translation::Random(),
                 Quaternion(a * sin(u2), a * cos(u2), b * sin(u3), b * cos(u3)));

    //m = Derived(Translation::Random(), Quaternion::UnitRandom());
  }
};

//! @brief Assignment assert specialization for SE3Base objects
template <typename Derived>
struct AssignmentEvaluatorImpl<SE3Base<Derived>>
{
  template <typename T>
  static void run_impl(const T& data)
  {
    using std::abs;
    using Scalar = typename SE3Base<Derived>::Scalar;
    MANIF_CHECK(abs(data.template tail<4>().norm()-Scalar(1)) <
                Constants<Scalar>::eps_s,
                "SE3 assigned data not normalized !",
                invalid_argument);
  }
};

/**
 * @brief Get the inverse.
 * @note See Eqs. (170,176).
 */
template <typename _Derived>
struct ExprEvaluator<InverseExpr<SE3Base<_Derived>>>
{
  using Ret = ReturnType<InverseExpr<SE3Base<_Derived>>>;

  template <typename T>
  static Ret run(const T& m, typename T::OptJacobianRef& J)
  {
    using Scalar = typename T::Scalar;

    if (J)
    {
      (*J) = -m.adj();
    }

    const SO3<Scalar> so3inv = m.asSO3().inverse();

    return Ret(-so3inv.act(m.translation()), so3inv);
  }
};

/**
 * @brief Composition of this and another SE3 element.
 * @note See Eq. (171) and Eqs. (177,178).
 */
template <typename _Derived, typename _DerivedOther>
struct ExprEvaluator<ComposeExpr<SE3Base<_Derived>, SE3Base<_DerivedOther>>>
{
  using Ret = ReturnType<ComposeExpr<SE3Base<_Derived>, SE3Base<_DerivedOther>>>;

  template <typename Lhs, typename Rhs>
  static Ret run(const Lhs& lhs, const Rhs& rhs,
                 typename Lhs::OptJacobianRef& J_mc_ma,
                 typename Rhs::OptJacobianRef& J_mc_mb)
  {
    static_assert(
      std::is_base_of<SE3Base<typename Rhs::LieGroup>, Rhs>::value,
      "Argument does not inherit from SE3Base !");

      // const auto& m_se3 = static_cast<const SE3Base<_DerivedOther>&>(m);

      if (J_mc_ma)
      {
        (*J_mc_ma) = rhs.inverse().adj();
      }

      if (J_mc_mb)
      {
        J_mc_mb->setIdentity();
      }

      return Ret(lhs.rotation()*rhs.translation() + lhs.translation(),
                 lhs.asSO3().compose(rhs.asSO3()).quat());
  }
};

/**
 * @brief Rigid motion action on a 3D point.
 * @note See Eq. (181) & Eqs. (182,183).
 */
template <typename _Derived, typename _DerivedOther>
struct ExprEvaluator<ActExpr<SE3Base<_Derived>, _DerivedOther>>
{
  using Scalar = typename _Derived::Scalar;
  using OptJacobianLhsRef = tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 6>>>;
  using OptJacobianRhsRef = tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, 3, 3>>>;

  using Ret = ReturnType<ActExpr<SE3Base<_Derived>, _DerivedOther>>;

  template <typename Lhs, typename Rhs>
  static Ret run(const Lhs& lhs, const Rhs& rhs,
                 OptJacobianLhsRef& J_ret_lhs,
                 OptJacobianRhsRef& J_ret_rhs)
  {
    using Rotation = typename _Derived::Rotation;

    assert_vector_dim(rhs, 3);
    const Rotation R(lhs.rotation());

    if (J_ret_lhs)
    {
      J_ret_lhs->template topLeftCorner<3,3>()  =  R;
      J_ret_lhs->template topRightCorner<3,3>() = -R * skew(rhs);
    }

    if (J_ret_rhs)
    {
      (*J_ret_rhs) = R;
    }

    return lhs.translation() + R * rhs;
  }
};

/**
 * @brief Get the adjoint matrix of SE3 at this.
 * @note See Eq. (175).
 */
template <typename _Derived>
struct ExprEvaluator<AdjExpr<SE3Base<_Derived>>>
{
  using Ret = ReturnType<AdjExpr<SE3Base<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    using Jacobian = typename T::Jacobian;

    /// @note Chirikjian (close to Eq.10.94)
    /// says
    ///       Ad(g) = |  R  0 |
    ///               | T.R R |
    ///
    /// considering vee(log(g)) = (w;v)
    /// with T = [t]_x
    ///
    /// but this is
    ///       Ad(g) = | R T.R |
    ///               | 0  R  |
    ///
    /// considering vee(log(g)) = (v;w)

    Jacobian Adj = Jacobian::Zero();
    Adj.template topLeftCorner<3,3>() = m.rotation();
    Adj.template bottomRightCorner<3,3>() =
        Adj.template topLeftCorner<3,3>();
    Adj.template topRightCorner<3,3>() =
      skew(m.translation()) * Adj.template topLeftCorner<3,3>();

    return Adj;
  }
};

/**
 * @brief Get the SE3 corresponding Lie algebra element in vector form.
 * @note This is the log() map in vector form.
 * @note See Eq. (173) & Eq. (79,179,180) and following notes.
 * @see SE3Tangent.
 */
template <typename _Derived>
struct ExprEvaluator<LogExpr<SE3Base<_Derived>>>
{
  using Ret = ReturnType<LogExpr<SE3Base<_Derived>>>;

  template <typename T>
  static Ret run(const T& m, typename T::OptJacobianRef& J)
  {
    using Scalar = typename T::Scalar;
    using Tangent = typename T::Tangent;

    const SO3Tangent<Scalar> so3tan = m.asSO3().log();

    Tangent tan((typename Tangent::DataType() <<
                 so3tan.ljac().inverse()*m.translation(),
                 so3tan.coeffs()).finished());

    if (J)
    {
      (*J) = tan.rjacinv();
    }

    return tan;
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_SE3_EXPR_H_
