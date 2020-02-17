#ifndef _MANIF_MANIF_SO3TANGENT_EXPR_H_
#define _MANIF_MANIF_SO3TANGENT_EXPR_H_

namespace manif {
namespace internal {

//! @brief Generator specialization for SO3TangentBase objects.
template <typename Derived>
struct GeneratorEvaluator<SO3TangentBase<Derived>>
{
  static typename SO3TangentBase<Derived>::LieAlg
  run(const unsigned int i)
  {
    using LieAlg = typename SO3TangentBase<Derived>::LieAlg;
    using Scalar = typename SO3TangentBase<Derived>::Scalar;

    switch (i)
    {
      case 0:
      {
        static const LieAlg E0(
              (LieAlg() << Scalar(0), Scalar(0), Scalar( 0),
                           Scalar(0), Scalar(0), Scalar(-1),
                           Scalar(0), Scalar(1), Scalar( 0) ).finished());
        return E0;
      }
      case 1:
      {
        static const LieAlg E1(
              (LieAlg() << Scalar( 0), Scalar(0), Scalar(1),
                           Scalar( 0), Scalar(0), Scalar(0),
                           Scalar(-1), Scalar(0), Scalar(0) ).finished());
        return E1;
      }
      case 2:
      {
        static const LieAlg E2(
              (LieAlg() << Scalar(0), Scalar(-1), Scalar(0),
                           Scalar(1), Scalar( 0), Scalar(0),
                           Scalar(0), Scalar( 0), Scalar(0) ).finished());
        return E2;
      }
      default:
        MANIF_THROW("Index i must be in [0,2]!", invalid_argument);
        break;
    }

    return LieAlg{};
  }
};

//! @brief Random specialization for SO3TangentBase objects.
template <typename Derived>
struct RandomEvaluatorImpl<SO3TangentBase<Derived>>
{
  static void run(SO3TangentBase<Derived>& m)
  {
    // in [-1,1]  / in [-PI,PI]
    m.coeffs().setRandom() *= MANIF_PI;
  }
};

/**
 * @brief Hat operator of SO3.
 * @note An element of the Lie algebra so3 (skew-symmetric matrix).
 * @note See example 3 of the paper.
 */
template <typename _Derived>
struct ExprEvaluator<HatExpr<SO3TangentBase<_Derived>>>
{
  using Ret = ReturnType<HatExpr<SO3TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    return skew(m.coeffs());
  }
};

/**
 * @brief Get the SO3 element.
 * @note This is the exp() map with the argument in vector form.
 * @note See Eq. (132) and Eq. (143).
 */
template <typename _Derived>
struct ExprEvaluator<ExpExpr<SO3TangentBase<_Derived>>>
{
  using Ret = ReturnType<ExpExpr<SO3TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m, typename T::OptJacobianRef& J_m_t)
  {
    using Scalar = typename T::Scalar;
    using DataType = typename T::DataType;
    using Jacobian = typename T::Jacobian;
    using LieAlg = typename T::LieAlg;
    using std::sqrt;
    using std::cos;
    using std::sin;

    const DataType& theta_vec = m.coeffs();
    const Scalar theta_sq = theta_vec.squaredNorm();
    const Scalar theta    = sqrt(theta_sq);

    if (theta_sq > Constants<Scalar>::eps_s)
    {
      if (J_m_t)
      {
        Jacobian M1, M2;

        const LieAlg W = m.hat();

        M1.noalias() = (Scalar(1.0) - cos(theta)) / theta_sq * W;
        M2.noalias() = (theta - sin(theta)) / (theta_sq * theta) * (W * W);;

        *J_m_t = Jacobian::Identity() - M1 + M2;
      }

      return Ret( Eigen::AngleAxis<Scalar>(theta, theta_vec.normalized()) );
    }
    else
    {
      if (J_m_t)
      {
        *J_m_t = Jacobian::Identity() - Scalar(0.5) * m.hat();
      }

      return Ret(m.x()/Scalar(2), m.y()/Scalar(2), m.z()/Scalar(2), Scalar(1));
    }

  }
};

/**
 * @brief Get the right Jacobian of SO3.
 * @note See Eq. (143).
 */
template <typename _Derived>
struct ExprEvaluator<RjacExpr<SO3TangentBase<_Derived>>>
{
  using Ret = ReturnType<RjacExpr<SO3TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    return m.ljac().transpose();
  }
};

/**
 * @brief Get the inverse of the right Jacobian of SO3.
 * @note See Eq. (144).
 */
template <typename _Derived>
struct ExprEvaluator<RjacinvExpr<SO3TangentBase<_Derived>>>
{
  using Ret = ReturnType<RjacinvExpr<SO3TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    return m.ljacinv().transpose();
  }
};

/**
 * @brief Get the left Jacobian of SO3.
 * @note See Eq. (145).
 */
template <typename _Derived>
struct ExprEvaluator<LjacExpr<SO3TangentBase<_Derived>>>
{
  using Ret = ReturnType<LjacExpr<SO3TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    using Scalar = typename T::Scalar;
    using LieAlg = typename T::LieAlg;
    using Jacobian = typename T::Jacobian;

    using std::sqrt;
    using std::cos;
    using std::sin;

    const Scalar theta_sq = m.coeffs().squaredNorm();

    const LieAlg W = m.hat();

    // Small angle approximation
    if (theta_sq <= Constants<Scalar>::eps_s)
      return Jacobian::Identity() - Scalar(0.5) * W;

    const Scalar theta = sqrt(theta_sq); // rotation angle
    Jacobian M1, M2;
    M1.noalias() = (Scalar(1) - cos(theta)) / theta_sq * W;
    M2.noalias() = (theta - sin(theta)) / (theta_sq * theta) * (W * W);

    return Jacobian::Identity() + M1 + M2;
  }
};

/**
 * @brief Get the inverse of the left Jacobian of SO3.
 * @note See Eq. (146).
 */
template <typename _Derived>
struct ExprEvaluator<LjacinvExpr<SO3TangentBase<_Derived>>>
{
  using Ret = ReturnType<LjacinvExpr<SO3TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    using Scalar = typename T::Scalar;
    using LieAlg = typename T::LieAlg;
    using Jacobian = typename T::Jacobian;

    using std::sqrt;
    using std::cos;
    using std::sin;

    const Scalar theta_sq = m.coeffs().squaredNorm();

    const LieAlg W = m.hat();

    if (theta_sq <= Constants<Scalar>::eps_s)
      return Jacobian::Identity() + Scalar(0.5) * W;

    const Scalar theta = sqrt(theta_sq); // rotation angle
    Jacobian M;
    M.noalias() = (Scalar(1) / theta_sq -
                   (Scalar(1) + cos(theta)) /
                   (Scalar(2) * theta * sin(theta))) * (W * W);

    return Jacobian::Identity() - Scalar(0.5) * W + M;
  }
};

/**
 * @brief smallAdj
 */
template <typename _Derived>
struct ExprEvaluator<SmallAdjExpr<SO3TangentBase<_Derived>>>
{
  using Ret = ReturnType<SmallAdjExpr<SO3TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    return m.hat();
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_SO3TANGENT_EXPR_H_
