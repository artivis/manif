#ifndef _MANIF_MANIF_SO2TANGENT_EXPR_H_
#define _MANIF_MANIF_SO2TANGENT_EXPR_H_

namespace manif {
namespace internal {

/**
 * @brief Generator specialization for SO2TangentBase objects.
 * E = | 0 -1 |
 *     | 1  0 |
 */
template <typename Derived>
struct GeneratorEvaluator<SO2TangentBase<Derived>>
{
  static typename SO2TangentBase<Derived>::LieAlg
  run(const unsigned int i)
  {
    MANIF_CHECK(i==0,
                "Index i must be 0!",
                invalid_argument);

    const static typename SO2TangentBase<Derived>::LieAlg E0 =
        skew(typename SO2TangentBase<Derived>::Scalar(1));

    return E0;
  }
};

//! @brief Random specialization for SO2TangentBase objects.
template <typename Derived>
struct RandomEvaluatorImpl<SO2TangentBase<Derived>>
{
  static void run(SO2TangentBase<Derived>& m)
  {
    // in [-1,1]  /  in [-PI,PI]
    m.coeffs().setRandom() *= MANIF_PI;
  }
};

/**
 * @brief Hat operator of SO2.
 * @note Hat is an element of the Lie algebra so2 (skew-symmetric matrix).
 * @note See Eqs. (112, 113).
 */
template <typename _Derived>
struct ExprEvaluator<HatExpr<SO2TangentBase<_Derived>>>
{
  using Ret = ReturnType<HatExpr<SO2TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    using Scalar = typename traits<_Derived>::Scalar;
    using LieAlg = typename traits<_Derived>::LieAlg;
    return (LieAlg() << Scalar(0)          ,   Scalar(-m.coeffs()(0)),
                        Scalar(m.coeffs()(0)), Scalar(0)            ).finished();
  }
};

/**
 * @brief Get the SO2 element.
 * @note This is the exp() map with the argument in vector form.
 * @note See Eqs. (114, 116) and Eq. (126).
 */
template <typename _Derived>
struct ExprEvaluator<ExpExpr<SO2TangentBase<_Derived>>>
{
  using Ret = ReturnType<ExpExpr<SO2TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m, typename T::OptJacobianRef& J_m_t)
  {
    using std::cos;
    using std::sin;

    if (J_m_t)
    {
      (*J_m_t) = m.rjac();
    }

    return Ret(cos(m.coeffs()(0)), sin(m.coeffs()(0)));
  }
};

/**
 * @brief Get the right Jacobian of SO2.
 * @note See Eq. (126).
 */
template <typename _Derived>
struct ExprEvaluator<RjacExpr<SO2TangentBase<_Derived>>>
{
  using Ret = ReturnType<RjacExpr<SO2TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    using Jacobian = typename SO2TangentBase<_Derived>::Jacobian;
    using Scalar = typename SO2TangentBase<_Derived>::Scalar;
    static const Jacobian Jr = Jacobian::Constant(Scalar(1));
    return Jr;
  }
};

/**
 * @brief Get the inverse of the right Jacobian of SO2.
 * @note See Eq. (126).
 */
template <typename _Derived>
struct ExprEvaluator<RjacinvExpr<SO2TangentBase<_Derived>>>
{
  using Ret = ReturnType<RjacinvExpr<SO2TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    using Jacobian = typename SO2TangentBase<_Derived>::Jacobian;
    using Scalar = typename SO2TangentBase<_Derived>::Scalar;
    static const Jacobian Jrinv = Jacobian::Constant(Scalar(1));
    return Jrinv;
  }
};

/**
 * @brief Get the left Jacobian of SO2.
 * @note See Eq. (126).
 */
template <typename _Derived>
struct ExprEvaluator<LjacExpr<SO2TangentBase<_Derived>>>
{
  using Ret = ReturnType<LjacExpr<SO2TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    using Jacobian = typename SO2TangentBase<_Derived>::Jacobian;
    using Scalar = typename SO2TangentBase<_Derived>::Scalar;
    static const Jacobian Jl = Jacobian::Constant(Scalar(1));
    return Jl;
  }
};

/**
 * @brief Get the inverse of the right Jacobian of SO2.
 * @note See Eq. (126).
 */
template <typename _Derived>
struct ExprEvaluator<LjacinvExpr<SO2TangentBase<_Derived>>>
{
  using Ret = ReturnType<LjacinvExpr<SO2TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    using Jacobian = typename SO2TangentBase<_Derived>::Jacobian;
    using Scalar = typename SO2TangentBase<_Derived>::Scalar;
    static const Jacobian Jlinv = Jacobian::Constant(Scalar(1));
    return Jlinv;
  }
};

/**
 * @brief smallAdj
 */
template <typename _Derived>
struct ExprEvaluator<SmallAdjExpr<SO2TangentBase<_Derived>>>
{
  using Ret = ReturnType<SmallAdjExpr<SO2TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    using Jacobian = typename SO2TangentBase<_Derived>::Jacobian;
    static const Ret smallAdj = Jacobian::Zero();
    return smallAdj;
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_SO2TANGENT_EXPR_H_
