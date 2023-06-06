#ifndef _MANIF_MANIF_BUNDLETANGENT_BASE_H_
#define _MANIF_MANIF_BUNDLETANGENT_BASE_H_

#include "manif/impl/tangent_base.h"
#include "manif/impl/traits.h"

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

public:

  /**
   * @brief Number of elements in the BundleTangent
   */
  static constexpr std::size_t BundleSize = internal::traits<_Derived>::BundleSize;

  using Elements = typename internal::traits<_Derived>::Elements;

  template <int Idx>
  using Element = typename internal::traits<_Derived>::template Element<Idx>;

  template <int Idx>
  using MapElement = typename internal::traits<_Derived>::template MapElement<Idx>;

  template <int Idx>
  using MapConstElement = typename internal::traits<_Derived>::template MapConstElement<Idx>;

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
   * @brief Access BundleTangent element as Map
   * @tparam _Idx element index
   */
  template<int _Idx>
  MapElement<_Idx> element();

  /**
   * @brief Access BundleTangent element as Map to const
   * @tparam _Idx element index
   */
  template<int _Idx>
  MapConstElement<_Idx> element() const;

protected:

  template <int ... _Idx>
  LieAlg hat_impl(internal::intseq<_Idx...>) const;

  template <int ... _Idx>
  LieGroup exp_impl(OptJacobianRef J_m_t, internal::intseq<_Idx...>) const;

  template <int ... _Idx>
  Jacobian rjac_impl(internal::intseq<_Idx...>) const;

  template <int ... _Idx>
  Jacobian ljac_impl(internal::intseq<_Idx...>) const;

  template <int ... _Idx>
  Jacobian rjacinv_impl(internal::intseq<_Idx...>) const;

  template <int ... _Idx>
  Jacobian ljacinv_impl(internal::intseq<_Idx...>) const;

  template <int ... _Idx>
  Jacobian smallAdj_impl(internal::intseq<_Idx...>) const;
};


template<typename _Derived>
typename BundleTangentBase<_Derived>::LieAlg
BundleTangentBase<_Derived>::hat() const
{
  return hat_impl(internal::make_intseq_t<BundleSize>{});
}

template<typename _Derived>
template<int ... _Idx>
typename BundleTangentBase<_Derived>::LieAlg
BundleTangentBase<_Derived>::hat_impl(internal::intseq<_Idx...>) const
{
  LieAlg ret = LieAlg::Zero();
  // c++11 "fold expression"
  auto l = {((ret.template block<
    Element<_Idx>::LieAlg::RowsAtCompileTime,
    Element<_Idx>::LieAlg::RowsAtCompileTime
  >(
    std::get<_Idx>(internal::traits<_Derived>::AlgIdx),
    std::get<_Idx>(internal::traits<_Derived>::AlgIdx)
  ) = element<_Idx>().hat()), 0) ...};
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
  return exp_impl(J_m_t, internal::make_intseq_t<BundleSize>{});
}

template<typename _Derived>
template<int ... _Idx>
typename BundleTangentBase<_Derived>::LieGroup
BundleTangentBase<_Derived>::exp_impl(
  OptJacobianRef J_m_t, internal::intseq<_Idx...>
) const
{
  if (J_m_t) {
    return LieGroup(
      element<_Idx>().exp(
        J_m_t->template block<
          Element<_Idx>::DoF, Element<_Idx>::DoF
        >(
          std::get<_Idx>(internal::traits<_Derived>::DoFIdx),
          std::get<_Idx>(internal::traits<_Derived>::DoFIdx)
        )
      ) ...
    );
  }
  return LieGroup(element<_Idx>().exp() ...);
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
  return rjac_impl(internal::make_intseq_t<BundleSize>{});
}

template<typename _Derived>
typename BundleTangentBase<_Derived>::Jacobian
BundleTangentBase<_Derived>::ljac() const
{
  return ljac_impl(internal::make_intseq_t<BundleSize>{});
}

template<typename _Derived>
typename BundleTangentBase<_Derived>::Jacobian
BundleTangentBase<_Derived>::rjacinv() const
{
  return rjacinv_impl(internal::make_intseq_t<BundleSize>{});
}

template<typename _Derived>
typename BundleTangentBase<_Derived>::Jacobian
BundleTangentBase<_Derived>::ljacinv() const
{
  return ljacinv_impl(internal::make_intseq_t<BundleSize>{});
}

template<typename _Derived>
typename BundleTangentBase<_Derived>::Jacobian
BundleTangentBase<_Derived>::smallAdj() const
{
  return smallAdj_impl(internal::make_intseq_t<BundleSize>{});
}

template<typename _Derived>
template<int ... _Idx>
typename BundleTangentBase<_Derived>::Jacobian
BundleTangentBase<_Derived>::rjac_impl(internal::intseq<_Idx...>) const
{
  Jacobian Jr = Jacobian::Zero();
  // c++11 "fold expression"
  auto l = {((Jr.template block<Element<_Idx>::DoF, Element<_Idx>::DoF>(
    std::get<_Idx>(internal::traits<_Derived>::DoFIdx),
    std::get<_Idx>(internal::traits<_Derived>::DoFIdx)
  ) = element<_Idx>().rjac() ), 0) ...};
  static_cast<void>(l);  // compiler warning
  return Jr;
}

template<typename _Derived>
template<int ... _Idx>
typename BundleTangentBase<_Derived>::Jacobian
BundleTangentBase<_Derived>::ljac_impl(internal::intseq<_Idx...>) const
{
  Jacobian Jr = Jacobian::Zero();
  // c++11 "fold expression"
  auto l = {((Jr.template block<Element<_Idx>::DoF, Element<_Idx>::DoF>(
    std::get<_Idx>(internal::traits<_Derived>::DoFIdx),
    std::get<_Idx>(internal::traits<_Derived>::DoFIdx)
  ) = element<_Idx>().ljac()), 0) ...};
  static_cast<void>(l);  // compiler warning
  return Jr;
}

template<typename _Derived>
template<int ... _Idx>
typename BundleTangentBase<_Derived>::Jacobian
BundleTangentBase<_Derived>::rjacinv_impl(internal::intseq<_Idx...>) const
{
  Jacobian Jr = Jacobian::Zero();
  // c++11 "fold expression"
  auto l = {
    ((Jr.template block<Element<_Idx>::DoF, Element<_Idx>::DoF>(
      std::get<_Idx>(internal::traits<_Derived>::DoFIdx),
      std::get<_Idx>(internal::traits<_Derived>::DoFIdx)
    ) = element<_Idx>().rjacinv()), 0) ...
  };
  static_cast<void>(l);  // compiler warning
  return Jr;
}

template<typename _Derived>
template<int ... _Idx>
typename BundleTangentBase<_Derived>::Jacobian
BundleTangentBase<_Derived>::ljacinv_impl(internal::intseq<_Idx...>) const
{
  Jacobian Jr = Jacobian::Zero();
  // c++11 "fold expression"
  auto l = {
    ((Jr.template block<Element<_Idx>::DoF, Element<_Idx>::DoF>(
      std::get<_Idx>(internal::traits<_Derived>::DoFIdx),
      std::get<_Idx>(internal::traits<_Derived>::DoFIdx)
    ) = element<_Idx>().ljacinv()), 0) ...
  };
  static_cast<void>(l);  // compiler warning
  return Jr;
}

template<typename _Derived>
template<int ... _Idx>
typename BundleTangentBase<_Derived>::Jacobian
BundleTangentBase<_Derived>::smallAdj_impl(internal::intseq<_Idx...>) const
{
  Jacobian Jr = Jacobian::Zero();
  // c++11 "fold expression"
  auto l = {
    ((Jr.template block<Element<_Idx>::DoF, Element<_Idx>::DoF>(
      std::get<_Idx>(internal::traits<_Derived>::DoFIdx),
      std::get<_Idx>(internal::traits<_Derived>::DoFIdx)
    ) = element<_Idx>().smallAdj()), 0) ...
  };
  static_cast<void>(l);  // compiler warning
  return Jr;
}

template<typename _Derived>
template<int _Idx>
auto BundleTangentBase<_Derived>::element() -> MapElement<_Idx>
{
  return MapElement<_Idx>(
    static_cast<_Derived &>(*this).coeffs().data() +
    std::get<_Idx>(internal::traits<_Derived>::RepSizeIdx)
  );
}

template<typename _Derived>
template<int _Idx>
auto BundleTangentBase<_Derived>::element() const -> MapConstElement<_Idx>
{
  return MapConstElement<_Idx>(
    static_cast<const _Derived &>(*this).coeffs().data() +
    std::get<_Idx>(internal::traits<_Derived>::RepSizeIdx)
  );
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
      invalid_argument
    );

    return run(i, make_intseq_t<Derived::BundleSize>{});
  }

  template<int ... _Idx>
  static typename BundleTangentBase<Derived>::LieAlg
  run(const unsigned int i, intseq<_Idx...>)
  {
    using LieAlg = typename BundleTangentBase<Derived>::LieAlg;
    LieAlg Ei = LieAlg::Zero();
    // c++11 "fold expression"
    auto l = {((Ei.template block<
      Derived::template Element<_Idx>::LieAlg::RowsAtCompileTime,
      Derived::template Element<_Idx>::LieAlg::RowsAtCompileTime
    >(
      std::get<_Idx>(internal::traits<Derived>::AlgIdx),
      std::get<_Idx>(internal::traits<Derived>::AlgIdx)
    ) = (
        static_cast<int>(i) >= std::get<_Idx>(internal::traits<Derived>::DoFIdx) &&
        static_cast<int>(i) < std::get<_Idx>(internal::traits<Derived>::DoFIdx) + Derived::template Element<_Idx>::DoF
      ) ?
        Derived::template Element<_Idx>::Generator(
          static_cast<int>(i) - std::get<_Idx>(internal::traits<Derived>::DoFIdx)
        ) :
        Derived::template Element<_Idx>::LieAlg::Zero()
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
    run(m, make_intseq_t<Derived::BundleSize>{});
  }

  template<int ... _Idx>
  static void run(BundleTangentBase<Derived> & m, intseq<_Idx...>)
  {
    m = typename BundleTangentBase<Derived>::Tangent(
      Derived::template Element<_Idx>::Random() ...
    );
  }
};

}  // namespace internal
}  // namespace manif

#endif  // _MANIF_MANIF_BUNDLETANGENT_BASE_H_
