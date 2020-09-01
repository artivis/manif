#ifndef _MANIF_MANIF_SE3TANGENT_EXPR_H_
#define _MANIF_MANIF_SE3TANGENT_EXPR_H_

namespace manif {
namespace internal {

//! @brief Generator specialization for SE3TangentBase objects.
template <typename Derived>
struct GeneratorEvaluator<SE3TangentBase<Derived>>
{
  static typename SE3TangentBase<Derived>::LieAlg
  run(const unsigned int i)
  {
    using LieAlg = typename SE3TangentBase<Derived>::LieAlg;
    using Scalar = typename SE3TangentBase<Derived>::Scalar;

    switch (i)
    {
      case 0:
      {
        static const LieAlg E0(
                (LieAlg() << Scalar(0), Scalar(0), Scalar(0), Scalar(1),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0) ).finished());
        return E0;
      }
      case 1:
      {
        static const LieAlg E1(
                (LieAlg() << Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(1),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0) ).finished());
        return E1;
      }
      case 2:
      {
        static const LieAlg E2(
                (LieAlg() << Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(1),
                             Scalar(0), Scalar(0), Scalar(0), Scalar(0) ).finished());
        return E2;
      }
      case 3:
      {
        static const LieAlg E3(
                (LieAlg() << Scalar(0), Scalar(0), Scalar( 0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(-1), Scalar(0),
                             Scalar(0), Scalar(1), Scalar( 0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar( 0), Scalar(0) ).finished());
        return E3;
      }
      case 4:
      {
        static const LieAlg E4(
                (LieAlg() << Scalar( 0), Scalar(0), Scalar(1), Scalar(0),
                             Scalar( 0), Scalar(0), Scalar(0), Scalar(0),
                             Scalar(-1), Scalar(0), Scalar(0), Scalar(0),
                             Scalar( 0), Scalar(0), Scalar(0), Scalar(0) ).finished());
        return E4;
      }
      case 5:
      {
        static const LieAlg E5(
                (LieAlg() << Scalar(0), Scalar(-1), Scalar(0), Scalar(0),
                             Scalar(1), Scalar( 0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar( 0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar( 0), Scalar(0), Scalar(0) ).finished());
        return E5;
      }
      default:
        MANIF_THROW("Index i must be in [0,5]!", invalid_argument);
        break;
    }

    return LieAlg{};
  }
};

//! @brief Random specialization for SE3TangentBase objects.
template <typename Derived>
struct RandomEvaluatorImpl<SE3TangentBase<Derived>>
{
  static void run(SE3TangentBase<Derived>& m)
  {
    m.coeffs().setRandom();                // in [-1,1]
    m.coeffs().template tail<3>() *= MANIF_PI; // in [-PI,PI]
  }
};

/**
 * @brief Hat operator of SE3.
 * @note See Eq. (169).
 */
template <typename _Derived>
struct ExprEvaluator<HatExpr<SE3TangentBase<_Derived>>>
{
  using Ret = ReturnType<HatExpr<SE3TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    using Scalar = typename traits<_Derived>::Scalar;
    using LieAlg = typename traits<_Derived>::LieAlg;

    return (LieAlg() <<
      Scalar(0)             , Scalar(-m.coeffs()(5)), Scalar( m.coeffs()(4)), Scalar(m.coeffs()(0)),
      Scalar( m.coeffs()(5)), Scalar(0)             , Scalar(-m.coeffs()(3)), Scalar(m.coeffs()(1)),
      Scalar(-m.coeffs()(4)), Scalar( m.coeffs()(3)), Scalar(0)             , Scalar(m.coeffs()(2)),
      Scalar(0)             , Scalar(0)             , Scalar(0)             , Scalar(0)
            ).finished();
  }
};

/**
 * @brief Get the SE3 element.
 * @note This is the exp() map with the argument in vector form.
 * @note See Eq. (172) & Eqs. (179,180).
 */
template <typename _Derived>
struct ExprEvaluator<ExpExpr<SE3TangentBase<_Derived>>>
{
  using Ret = ReturnType<ExpExpr<SE3TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m, typename T::OptJacobianRef& J_m_t)
  {
    if (J_m_t)
    {
      *J_m_t = m.rjac();
    }

    /// @note Eq. 10.93
    return Ret(m.asSO3().ljac()*m.v(), m.asSO3().exp().quat());
  }
};

/**
 * @brief Get the right Jacobian of SE3.
 * @note See note after Eqs. (179,180).
 */
template <typename _Derived>
struct ExprEvaluator<RjacExpr<SE3TangentBase<_Derived>>>
{
  using Ret = ReturnType<RjacExpr<SE3TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    using Jacobian = typename SE3TangentBase<_Derived>::Jacobian;
    /// @note Eq. 10.95
    Jacobian Jr;
    Jr.template bottomLeftCorner<3,3>().setZero();
    Jr.template topLeftCorner<3,3>() = m.asSO3().rjac();
    Jr.template bottomRightCorner<3,3>() = Jr.template topLeftCorner<3,3>();
    internal::fillQ( Jr.template topRightCorner<3,3>(), -m.coeffs() );

    return Jr;
  }
};

/**
 * @brief Get the inverse right Jacobian of SE3.
 * @note See note after Eqs. (179,180).
 */
template <typename _Derived>
struct ExprEvaluator<RjacinvExpr<SE3TangentBase<_Derived>>>
{
  using Ret = ReturnType<RjacinvExpr<SE3TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    using Jacobian = typename SE3TangentBase<_Derived>::Jacobian;
    Jacobian Jr_inv;
    internal::fillQ( Jr_inv.template bottomLeftCorner<3,3>(), -m.coeffs() ); // serves as temporary Q
    Jr_inv.template topLeftCorner<3,3>() = m.asSO3().rjacinv();
    Jr_inv.template bottomRightCorner<3,3>() = Jr_inv.template topLeftCorner<3,3>();
    Jr_inv.template topRightCorner<3,3>().noalias() =
        -Jr_inv.template topLeftCorner<3,3>()    *
         Jr_inv.template bottomLeftCorner<3,3>() *
         Jr_inv.template topLeftCorner<3,3>();
    Jr_inv.template bottomLeftCorner<3,3>().setZero();

    return Jr_inv;
  }
};

/**
 * @brief Get the left Jacobian of SE3.
 * @note See Eqs. (179,180).
 */
template <typename _Derived>
struct ExprEvaluator<LjacExpr<SE3TangentBase<_Derived>>>
{
  using Ret = ReturnType<LjacExpr<SE3TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    using Jacobian = typename SE3TangentBase<_Derived>::Jacobian;
    Jacobian Jl;
    Jl.template bottomLeftCorner<3,3>().setZero();
    Jl.template topLeftCorner<3,3>() = m.asSO3().ljac();
    Jl.template bottomRightCorner<3,3>() = Jl.template topLeftCorner<3,3>();
    internal::fillQ( Jl.template topRightCorner<3,3>(), m.coeffs() );

    return Jl;
  }
};

/**
 * @brief Get the inverse left Jacobian of SE3.
 * @note See Eqs. (179,180).
 */
template <typename _Derived>
struct ExprEvaluator<LjacinvExpr<SE3TangentBase<_Derived>>>
{
  using Ret = ReturnType<LjacinvExpr<SE3TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    using Jacobian = typename SE3TangentBase<_Derived>::Jacobian;
    Jacobian Jl_inv;
    internal::fillQ( Jl_inv.template bottomLeftCorner<3,3>(), m.coeffs() ); // serves as temporary Q
    Jl_inv.template topLeftCorner<3,3>() = m.asSO3().ljacinv();
    Jl_inv.template bottomRightCorner<3,3>() = Jl_inv.template topLeftCorner<3,3>();
    Jl_inv.template topRightCorner<3,3>().noalias() =
        -Jl_inv.template topLeftCorner<3,3>()    *
         Jl_inv.template bottomLeftCorner<3,3>() *
         Jl_inv.template topLeftCorner<3,3>();
    Jl_inv.template bottomLeftCorner<3,3>().setZero();

    return Jl_inv;
  }
};

/**
 * @brief smallAdj
 */
template <typename _Derived>
struct ExprEvaluator<SmallAdjExpr<SE3TangentBase<_Derived>>>
{
  using Ret = ReturnType<SmallAdjExpr<SE3TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    using Jacobian = typename SE3TangentBase<_Derived>::Jacobian;
    /// @note Chirikjian (close to Eq.10.94)
    /// says
    ///       ad(g) = |  Omega  0   |
    ///               |   V   Omega |
    ///
    /// considering vee(log(g)) = (w;v)
    ///
    /// but this is
    ///       ad(g) = |  Omega  V   |
    ///               |   0   Omega |
    ///
    /// considering vee(log(g)) = (v;w)

    Jacobian smallAdj;
    smallAdj.template topRightCorner<3,3>() = skew(m.v());
    smallAdj.template topLeftCorner<3,3>() = skew(m.w());
    smallAdj.template bottomRightCorner<3,3>() = smallAdj.template topLeftCorner<3,3>();
    smallAdj.template bottomLeftCorner<3,3>().setZero();

    return smallAdj;
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_SE3TANGENT_EXPR_H_
