#ifndef _MANIF_MANIF_SE2TANGENT_EXPR_H_
#define _MANIF_MANIF_SE2TANGENT_EXPR_H_

namespace manif {
namespace internal {

//! @brief Generator specialization for SE2TangentBase objects.
template <typename Derived>
struct GeneratorEvaluator<SE2TangentBase<Derived>>
{
  static typename SE2TangentBase<Derived>::LieAlg
  run(const unsigned int i)
  {
    using LieAlg = typename SE2TangentBase<Derived>::LieAlg;
    using Scalar = typename SE2TangentBase<Derived>::Scalar;

    switch (i)
    {
      case 0:
      {
        const static LieAlg E0(
                (LieAlg() << Scalar(0), Scalar(0), Scalar(1),
                             Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(0) ).finished());
        return E0;
      }
      case 1:
      {
        const static LieAlg E1(
                (LieAlg() << Scalar(0), Scalar(0), Scalar(0),
                             Scalar(0), Scalar(0), Scalar(1),
                             Scalar(0), Scalar(0), Scalar(0) ).finished());
        return E1;
      }
      case 2:
      {
        const static LieAlg E2(
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

//! @brief Inner weight matrix specialization for SE2TangentBase objects.
template <typename Derived>
struct WEvaluator<SE2TangentBase<Derived>>
{
  static typename Derived::Jacobian
  run()
  {
    using Jacobian = typename SE2TangentBase<Derived>::Jacobian;
    using Scalar   = typename SE2TangentBase<Derived>::Scalar;

    const static Jacobian W(
            (Jacobian() << Scalar(1), Scalar(0), Scalar(0),
                           Scalar(0), Scalar(1), Scalar(0),
                           Scalar(0), Scalar(0), Scalar(2) ).finished());

    return W;
  }
};

//! @brief Random specialization for SE2TangentBase objects.
template <typename Derived>
struct RandomEvaluatorImpl<SE2TangentBase<Derived>>
{
  static void run(SE2TangentBase<Derived>& m)
  {
    m.coeffs().setRandom();         // in [-1,1]
    m.coeffs().coeffRef(2) *= MANIF_PI; // in [-PI,PI]
  }
};

/**
 * @brief Hat operator of SE2.
 * @note See Eq. (153).
 */
template <typename _Derived>
struct ExprEvaluator<HatExpr<SE2TangentBase<_Derived>>>
{
  using Ret = ReturnType<HatExpr<SE2TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    using Scalar = typename traits<_Derived>::Scalar;
    using LieAlg = typename traits<_Derived>::LieAlg;

    const Scalar angle = m.angle();

    return ( LieAlg() <<
               Scalar(0), -angle,     m.x(),
               angle,      Scalar(0), m.y(),
               Scalar(0),  Scalar(0), Scalar(0) ).finished();
  }
};

/**
 * @brief Exp map, get the SE2 element.
 * @note This is the exp() map with the argument in vector form.
 * @note See Eqs. (156,158) & Eq. (163).
 */
template <typename _Derived>
struct ExprEvaluator<ExpExpr<SE2TangentBase<_Derived>>>
{
  using Ret = ReturnType<ExpExpr<SE2TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m, typename T::OptJacobianRef& J_m_t)
  {
    using Scalar = typename T::Scalar;

    using std::abs;
    using std::cos;
    using std::sin;

    const Scalar theta = m.angle();
    const Scalar cos_theta = cos(theta);
    const Scalar sin_theta = sin(theta);
    const Scalar theta_sq = theta * theta;

    Scalar A,  // sin_theta_by_theta
           B;  // one_minus_cos_theta_by_theta

    if (theta_sq < Constants<Scalar>::eps_s)
    {
      // Taylor approximation
      A = Scalar(1) - Scalar(1. / 6.) * theta_sq;
      B = Scalar(.5) * theta - Scalar(1. / 24.) * theta * theta_sq;
    }
    else
    {
      // Euler
      A = sin_theta / theta;
      B = (Scalar(1) - cos_theta) / theta;
    }

    if (J_m_t)
    {
      // Jr
      J_m_t->setIdentity();
      (*J_m_t)(0,0) =  A;
      (*J_m_t)(0,1) =  B;
      (*J_m_t)(1,0) = -B;
      (*J_m_t)(1,1) =  A;

      if (theta_sq < Constants<Scalar>::eps_s)
      {
        (*J_m_t)(0,2) = -m.y() / Scalar(2) + theta * m.x() / Scalar(6);
        (*J_m_t)(1,2) =  m.x() / Scalar(2) + theta * m.y() / Scalar(6);
      }
      else
      {
        (*J_m_t)(0,2) = (-m.y() + theta*m.x() + m.y()*cos_theta - m.x()*sin_theta)/theta_sq;
        (*J_m_t)(1,2) = ( m.x() + theta*m.y() - m.x()*cos_theta - m.y()*sin_theta)/theta_sq;
      }
    }

    return Ret( A * m.x() - B * m.y(),
                B * m.x() + A * m.y(),
                cos_theta, sin_theta );
  }
};

/**
 * @brief Get the right Jacobian of SE2.
 * @note See Eq. (163).
 */
template <typename _Derived>
struct ExprEvaluator<RjacExpr<SE2TangentBase<_Derived>>>
{
  using Ret = ReturnType<RjacExpr<SE2TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    //  const Scalar theta = m.angle();
    //  const Scalar cos_theta = cos(theta);
    //  const Scalar sin_theta = sin(theta);
    //  const Scalar theta_sq = theta * theta;

    //  Scalar A,  // sin_theta_by_theta
    //         B;  // one_minus_cos_theta_by_theta

    //  if (abs(theta) < Constants<Scalar>::eps)
    //  {
    //    // Taylor approximation
    //    A = Scalar(1) - Scalar(1. / 6.) * theta_sq;
    //    B = Scalar(.5) * theta - Scalar(1. / 24.) * theta * theta_sq;
    //  }
    //  else
    //  {
    //    // Euler
    //    A = sin_theta / theta;
    //    B = (Scalar(1) - cos_theta) / theta;
    //  }

    //  Ret Jr = Ret::Identity();
    //  Jr(0,0) =  A;
    //  Jr(0,1) =  B;
    //  Jr(1,0) = -B;
    //  Jr(1,1) =  A;

    //  Jr(0,2) = (-m.y() + theta*m.x() + m.y()*cos_theta - m.x()*sin_theta)/theta_sq;
    //  Jr(1,2) = ( m.x() + theta*m.y() - m.x()*cos_theta - m.y()*sin_theta)/theta_sq;

    //  return Jr;

    Ret Jr;
    m.exp(Jr);

    return Jr;
  }
};

/**
 * @brief Get the left Jacobian of SE2.
 * @note See Eq. (164).
 */
template <typename _Derived>
struct ExprEvaluator<LjacExpr<SE2TangentBase<_Derived>>>
{
  using Ret = ReturnType<LjacExpr<SE2TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    using Scalar = typename T::Scalar;

    const Scalar theta = m.angle();
    const Scalar cos_theta = cos(theta);
    const Scalar sin_theta = sin(theta);
    const Scalar theta_sq = theta * theta;

    Scalar A,  // sin_theta_by_theta
           B;  // one_minus_cos_theta_by_theta

    if (theta_sq < Constants<Scalar>::eps_s)
    {
      // Taylor approximation
      A = Scalar(1) - Scalar(1. / 6.) * theta_sq;
      B = Scalar(.5) * theta - Scalar(1. / 24.) * theta * theta_sq;
    }
    else
    {
      // Euler
      A = sin_theta / theta;
      B = (Scalar(1) - cos_theta) / theta;
    }

    Ret Jl = Ret::Identity();
    Jl(0,0) =  A;
    Jl(0,1) = -B;
    Jl(1,0) =  B;
    Jl(1,1) =  A;

    if (theta_sq < Constants<Scalar>::eps_s)
    {
      Jl(0,2) =  m.y() / Scalar(2) + theta * m.x() / Scalar(6);
      Jl(1,2) = -m.x() / Scalar(2) + theta * m.y() / Scalar(6);
    }
    else
    {
      Jl(0,2) = ( m.y() + theta*m.x() - m.y()*cos_theta - m.x()*sin_theta)/theta_sq;
      Jl(1,2) = (-m.x() + theta*m.y() + m.x()*cos_theta - m.y()*sin_theta)/theta_sq;
    }

    return Jl;
  }
};

/**
 * @brief smallAdj
 */
template <typename _Derived>
struct ExprEvaluator<SmallAdjExpr<SE2TangentBase<_Derived>>>
{
  using Ret = ReturnType<SmallAdjExpr<SE2TangentBase<_Derived>>>;

  template <typename T>
  static Ret run(const T& m)
  {
    Ret smallAdj = Ret::Zero();

    smallAdj(0,1) = -m.angle();
    smallAdj(1,0) =  m.angle();
    smallAdj(0,2) =  m.y();
    smallAdj(1,2) = -m.x();

    return smallAdj;
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_SE2TANGENT_EXPR_H_
