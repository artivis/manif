#ifndef _MANIF_MANIF_DHUTANGENT_BASE_H_
#define _MANIF_MANIF_DHUTANGENT_BASE_H_

#include "manif/impl/dhu/DHu_properties.h"
#include "manif/impl/tangent_base.h"

namespace manif {

//
// Tangent
//

/**
 * @brief The base class of the DHu tangent.
 */
template <typename _Derived>
struct DHuTangentBase : TangentBase<_Derived>
{
private:

  using Base = TangentBase<_Derived>;
  using Type = DHuTangentBase<_Derived>;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_OPERATOR

  using Base::data;
  using Base::coeffs;

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(DHuTangentBase)

public:

  MANIF_TANGENT_ML_ASSIGN_OP(DHuTangentBase)

  // Tangent common API

  /**
   * @brief Hat operator of DHu.
   * @return An element of the Lie algebra dhu.
   */
  LieAlg hat() const;

  /**
   * @brief Get the DHu element.
   * @param[out] -optional- J_m_t Jacobian of the DHu element wrt this.
   * @return The DHu element.
   * @note This is the exp() map with the argument in vector form.
   */
  LieGroup exp(OptJacobianRef J_m_t = {}) const;

  /**
   * @brief This function is deprecated.
   * Please considere using
   * @ref exp instead.
   */
  MANIF_DEPRECATED
  LieGroup retract(OptJacobianRef J_m_t = {}) const;

  /**
   * @brief Get the right Jacobian of DHu.
   */
  Jacobian rjac() const;

  /**
   * @brief Get the left Jacobian of DHu.
   */
  Jacobian ljac() const;

  /**
   * @brief Get the inverse right Jacobian of DHu.
   */
  Jacobian rjacinv() const;

  /**
   * @brief Get the inverse left Jacobian of DHu.
   */
  Jacobian ljacinv() const;

  /**
   * @brief
   * @return
   */
  Jacobian smallAdj() const;

  // DHuTangent specific API
};

template <typename _Derived>
typename DHuTangentBase<_Derived>::LieGroup
DHuTangentBase<_Derived>::exp(OptJacobianRef J_m_t) const
{
  if (J_m_t)
  {
    *J_m_t = rjac();
  }

  // @todo
  return LieGroup();
}

template <typename _Derived>
typename DHuTangentBase<_Derived>::LieGroup
DHuTangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  return exp(J_m_t);
}

template <typename _Derived>
typename DHuTangentBase<_Derived>::LieAlg
DHuTangentBase<_Derived>::hat() const
{
  // @todo
  return LieAlg();
}

template <typename _Derived>
typename DHuTangentBase<_Derived>::Jacobian
DHuTangentBase<_Derived>::rjac() const
{
  // @todo
  return Jacobian();
}

template <typename _Derived>
typename DHuTangentBase<_Derived>::Jacobian
DHuTangentBase<_Derived>::ljac() const
{
  // @todo
  return Jacobian();
}

template <typename _Derived>
typename DHuTangentBase<_Derived>::Jacobian
DHuTangentBase<_Derived>::rjacinv() const
{
  // @todo
  return Jacobian();
}

template <typename _Derived>
typename DHuTangentBase<_Derived>::Jacobian
DHuTangentBase<_Derived>::ljacinv() const
{
  // @todo
  return Jacobian();
}

template <typename _Derived>
typename DHuTangentBase<_Derived>::Jacobian
DHuTangentBase<_Derived>::smallAdj() const
{
  // @todo
  return Jacobian();
}

// DHuTangent specific API

namespace internal {

//! @brief Generator specialization for DHuTangentBase objects.
template <typename Derived>
struct GeneratorEvaluator<DHuTangentBase<Derived>>
{
  static typename DHuTangentBase<Derived>::LieAlg
  run(const unsigned int i)
  {
    using LieAlg = typename DHuTangentBase<Derived>::LieAlg;

    // @todo

    return LieAlg{};
  }
};

//! @brief Random specialization for DHuTangentBase objects.
template <typename Derived>
struct RandomEvaluatorImpl<DHuTangentBase<Derived>>
{
  static void run(DHuTangentBase<Derived>& m)
  {
    // @todo check
    m.coeffs().setRandom(); // in [-1,1]
    m.coeffs() *= MANIF_PI; // in [-PI,PI]
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_DHUTANGENT_BASE_H_
