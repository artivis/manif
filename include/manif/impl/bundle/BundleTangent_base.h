#ifndef _MANIF_MANIF_BUNDLETANGENT_BASE_H_
#define _MANIF_MANIF_BUNDLETANGENT_BASE_H_

#include "manif/impl/tangent_base.h"
#include "manif/impl/traits.h"

using manif::internal::intseq;

namespace manif {

/**
 * @brief The base class of the Bundle tangent.
 */
template<typename _Derived>
struct BundleTangentBase : TangentBase<_Derived>
{
private:

  using Base = TangentBase<_Derived>;
  using Type = BundleTangentBase<_Derived>;

  using IdxList = typename internal::traits<_Derived>::IdxList;

  using BegDoF = typename internal::traits<_Derived>::BegDoF;
  using LenDoF = typename internal::traits<_Derived>::LenDoF;

  using BegRep = typename internal::traits<_Derived>::BegRep;
  using LenRep = typename internal::traits<_Derived>::LenRep;

  using BegAlg = typename internal::traits<_Derived>::BegAlg;
  using LenAlg = typename internal::traits<_Derived>::LenAlg;

  template<int _Idx>
  using BlockType = typename internal::traits<_Derived>::template BlockType<_Idx>;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  using Base::data;
  using Base::coeffs;

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(BundleTangentBase)

public:

  MANIF_TANGENT_ML_ASSIGN_OP(BundleTangentBase)

  // Tangent common API

  /**
   * @brief Hat operator.
   * @return An element of the Lie algebra.
   */
  LieAlg hat() const;

  /**
   * @brief Exponential operator.
   * @return An element of the Lie Group.
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
   * @brief Get the right Jacobian.
   */
  Jacobian rjac() const;

  /**
   * @brief Get the left Jacobian.
   */
  Jacobian ljac() const;

  /**
   * @brief Get the inverse of the right Jacobian.
   */
  Jacobian rjacinv() const;

  /**
   * @brief Get the inverse of the left Jacobian.
   */
  Jacobian ljacinv() const;

  /**
   * @brief
   */
  Jacobian smallAdj() const;


  // BundleTangent specific API

  /**
   * @brief Number of blocks in the BundleTangent
   */
  static constexpr std::size_t BundleSize = IdxList::size();

  /**
   * @brief Access BundleTangent block as Map
   * @tparam _Idx block index
   */
  template<int _Idx>
  Eigen::Map<BlockType<_Idx>> block();

  /**
   * @brief Access BundleTangent block as Map to const
   * @tparam _Idx block index
   */
  template<int _Idx>
  Eigen::Map<const BlockType<_Idx>> block() const;

protected:

  template<int ... _Idx, int ... _BegAlg, int ... _LenAlg>
  LieAlg
  hat_impl(intseq<_Idx...>, intseq<_BegAlg...>, intseq<_LenAlg...>) const;

  template<int ... _Idx, int ... _BegDoF, int ... _LenDoF>
  LieGroup
  exp_impl(OptJacobianRef J_m_t, intseq<_Idx...>, intseq<_BegDoF...>, intseq<_LenDoF...>) const;

  template<int ... _Idx, int ... _BegDoF, int ... _LenDoF>
  Jacobian
  rjac_impl(intseq<_Idx...>, intseq<_BegDoF...>, intseq<_LenDoF...>) const;

  template<int ... _Idx, int ... _BegDoF, int ... _LenDoF>
  Jacobian
  ljac_impl(intseq<_Idx...>, intseq<_BegDoF...>, intseq<_LenDoF...>) const;

  template<int ... _Idx, int ... _BegDoF, int ... _LenDoF>
  Jacobian
  rjacinv_impl(intseq<_Idx...>, intseq<_BegDoF...>, intseq<_LenDoF...>) const;

  template<int ... _Idx, int ... _BegDoF, int ... _LenDoF>
  Jacobian
  ljacinv_impl(intseq<_Idx...>, intseq<_BegDoF...>, intseq<_LenDoF...>) const;

  template<int ... _Idx, int ... _BegDoF, int ... _LenDoF>
  Jacobian
  smallAdj_impl(intseq<_Idx...>, intseq<_BegDoF...>, intseq<_LenDoF...>) const;

  friend internal::GeneratorEvaluator<BundleTangentBase<_Derived>>;
  friend internal::RandomEvaluatorImpl<BundleTangentBase<_Derived>>;
};


template<typename _Derived>
typename BundleTangentBase<_Derived>::LieAlg
BundleTangentBase<_Derived>::hat() const
{
  return hat_impl(IdxList{}, BegAlg{}, LenAlg{});
}

template<typename _Derived>
template<int ... _Idx, int ... _BegAlg, int ... _LenAlg>
typename BundleTangentBase<_Derived>::LieAlg
BundleTangentBase<_Derived>::hat_impl(
  intseq<_Idx...>, intseq<_BegAlg...>, intseq<_LenAlg...>) const
{
  LieAlg ret = LieAlg::Zero();
  // c++11 "fold expression"
  auto l = {((ret.template block<_LenAlg, _LenAlg>(_BegAlg, _BegAlg) = block<_Idx>().hat()), 0) ...};
  static_cast<void>(l);  // compiler warning
  return ret;
}

template<typename _Derived>
typename BundleTangentBase<_Derived>::LieGroup
BundleTangentBase<_Derived>::exp(OptJacobianRef J_m_t) const
{
  if (J_m_t) {
    J_m_t->setZero();
  }
  return exp_impl(J_m_t, IdxList{}, BegDoF{}, LenDoF{});
}

template<typename _Derived>
template<int ... _Idx, int ... _BegDoF, int ... _LenDoF>
typename BundleTangentBase<_Derived>::LieGroup
BundleTangentBase<_Derived>::exp_impl(
  OptJacobianRef J_m_t, intseq<_Idx...>, intseq<_BegDoF...>, intseq<_LenDoF...>) const
{
  if (J_m_t) {
    return LieGroup(block<_Idx>().exp(J_m_t->template block<_LenDoF, _LenDoF>(_BegDoF, _BegDoF)) ...);
  }
  return LieGroup(block<_Idx>().exp() ...);
}

template<typename _Derived>
typename BundleTangentBase<_Derived>::LieGroup
BundleTangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  return exp(J_m_t);
}

template<typename _Derived>
typename BundleTangentBase<_Derived>::Jacobian
BundleTangentBase<_Derived>::rjac() const
{
  return rjac_impl(IdxList{}, BegDoF{}, LenDoF{});
}

template<typename _Derived>
typename BundleTangentBase<_Derived>::Jacobian
BundleTangentBase<_Derived>::ljac() const
{
  return ljac_impl(IdxList{}, BegDoF{}, LenDoF{});
}

template<typename _Derived>
typename BundleTangentBase<_Derived>::Jacobian
BundleTangentBase<_Derived>::rjacinv() const
{
  return rjacinv_impl(IdxList{}, BegDoF{}, LenDoF{});
}

template<typename _Derived>
typename BundleTangentBase<_Derived>::Jacobian
BundleTangentBase<_Derived>::ljacinv() const
{
  return ljacinv_impl(IdxList{}, BegDoF{}, LenDoF{});
}

template<typename _Derived>
typename BundleTangentBase<_Derived>::Jacobian
BundleTangentBase<_Derived>::smallAdj() const
{
  return smallAdj_impl(IdxList{}, BegDoF{}, LenDoF{});
}

template<typename _Derived>
template<int ... _Idx, int ... _BegDoF, int ... _LenDoF>
typename BundleTangentBase<_Derived>::Jacobian
BundleTangentBase<_Derived>::rjac_impl(
  intseq<_Idx...>, intseq<_BegDoF...>, intseq<_LenDoF...>) const
{
  Jacobian Jr = Jacobian::Zero();
  // c++11 "fold expression"
  auto l = {((Jr.template block<_LenDoF, _LenDoF>(_BegDoF, _BegDoF) = block<_Idx>().rjac() ), 0) ...};
  static_cast<void>(l);  // compiler warning
  return Jr;
}

template<typename _Derived>
template<int ... _Idx, int ... _BegDoF, int ... _LenDoF>
typename BundleTangentBase<_Derived>::Jacobian
BundleTangentBase<_Derived>::ljac_impl(
  intseq<_Idx...>, intseq<_BegDoF...>, intseq<_LenDoF...>) const
{
  Jacobian Jr = Jacobian::Zero();
  // c++11 "fold expression"
  auto l = {((Jr.template block<_LenDoF, _LenDoF>(_BegDoF, _BegDoF) = block<_Idx>().ljac()), 0) ...};
  static_cast<void>(l);  // compiler warning
  return Jr;
}

template<typename _Derived>
template<int ... _Idx, int ... _BegDoF, int ... _LenDoF>
typename BundleTangentBase<_Derived>::Jacobian
BundleTangentBase<_Derived>::rjacinv_impl(
  intseq<_Idx...>, intseq<_BegDoF...>, intseq<_LenDoF...>) const
{
  Jacobian Jr = Jacobian::Zero();
  // c++11 "fold expression"
  auto l = {
    ((Jr.template block<_LenDoF, _LenDoF>(_BegDoF, _BegDoF) = block<_Idx>().rjacinv()), 0) ...
  };
  static_cast<void>(l);  // compiler warning
  return Jr;
}

template<typename _Derived>
template<int ... _Idx, int ... _BegDoF, int ... _LenDoF>
typename BundleTangentBase<_Derived>::Jacobian
BundleTangentBase<_Derived>::ljacinv_impl(
  intseq<_Idx...>, intseq<_BegDoF...>, intseq<_LenDoF...>) const
{
  Jacobian Jr = Jacobian::Zero();
  // c++11 "fold expression"
  auto l = {
    ((Jr.template block<_LenDoF, _LenDoF>(_BegDoF, _BegDoF) = block<_Idx>().ljacinv()), 0) ...
  };
  static_cast<void>(l);  // compiler warning
  return Jr;
}

template<typename _Derived>
template<int ... _Idx, int ... _BegDoF, int ... _LenDoF>
typename BundleTangentBase<_Derived>::Jacobian
BundleTangentBase<_Derived>::smallAdj_impl(
  intseq<_Idx...>, intseq<_BegDoF...>, intseq<_LenDoF...>) const
{
  Jacobian Jr = Jacobian::Zero();
  // c++11 "fold expression"
  auto l = {
    ((Jr.template block<_LenDoF, _LenDoF>(_BegDoF, _BegDoF) = block<_Idx>().smallAdj()), 0) ...
  };
  static_cast<void>(l);  // compiler warning
  return Jr;
}

template<typename _Derived>
template<int _Idx>
Eigen::Map<typename BundleTangentBase<_Derived>::template BlockType<_Idx>>
BundleTangentBase<_Derived>::block()
{
  return Eigen::Map<BlockType<_Idx>>(
    static_cast<_Derived &>(*this).coeffs().data() +
    internal::intseq_element<_Idx, BegRep>::value);
}

template<typename _Derived>
template<int _Idx>
Eigen::Map<const typename BundleTangentBase<_Derived>::template BlockType<_Idx>>
BundleTangentBase<_Derived>::block() const
{
  return Eigen::Map<const BlockType<_Idx>>(
    static_cast<const _Derived &>(*this).coeffs().data() +
    internal::intseq_element<_Idx, BegRep>::value);
}

namespace internal {

/**
 * @brief Generator specialization for BundleTangentBase objects.
 */
template<typename Derived>
struct GeneratorEvaluator<BundleTangentBase<Derived>>
{
  static typename BundleTangentBase<Derived>::LieAlg
  run(const unsigned int i)
  {
    MANIF_CHECK(
      i < BundleTangentBase<Derived>::DoF,
      "Index i must less than DoF!",
      invalid_argument);

    return run(
      i,
      typename BundleTangentBase<Derived>::IdxList{},
      typename BundleTangentBase<Derived>::BegDoF{},
      typename BundleTangentBase<Derived>::LenDoF{},
      typename BundleTangentBase<Derived>::BegAlg{},
      typename BundleTangentBase<Derived>::LenAlg{});
  }

  template<int ... _Idx, int ... _BegDoF, int ... _LenDoF, int ... _BegAlg, int ... _LenAlg>
  static typename BundleTangentBase<Derived>::LieAlg
  run(
    const unsigned int i, intseq<_Idx...>,
    intseq<_BegDoF...>, intseq<_LenDoF...>,
    intseq<_BegAlg...>, intseq<_LenAlg...>)
  {
    using LieAlg = typename BundleTangentBase<Derived>::LieAlg;
    LieAlg Ei = LieAlg::Constant(0);
    // c++11 "fold expression"
    auto l = {((Ei.template block<_LenAlg, _LenAlg>(_BegAlg, _BegAlg) =
      (i >= _BegDoF && i < _BegDoF + _LenDoF) ?
      BundleTangentBase<Derived>::template BlockType<_Idx>::Generator(i - _BegDoF) :
      BundleTangentBase<Derived>::template BlockType<_Idx>::LieAlg::Zero()
      ), 0) ...};
    static_cast<void>(l);  // compiler warning
    return Ei;
  }
};

/**
 * @brief Random specialization for BundleTangent objects.
 */
template<typename Derived>
struct RandomEvaluatorImpl<BundleTangentBase<Derived>>
{
  static void run(BundleTangentBase<Derived> & m)
  {
    run(m, typename BundleTangentBase<Derived>::IdxList{});
  }

  template<int ... _Idx>
  static void run(BundleTangentBase<Derived> & m, intseq<_Idx...>)
  {
    m = typename BundleTangentBase<Derived>::Tangent(
      BundleTangentBase<Derived>::template BlockType<_Idx>::Random() ...);
  }
};

}  // namespace internal
}  // namespace manif

#endif  // _MANIF_MANIF_BUNDLETANGENT_BASE_H_
