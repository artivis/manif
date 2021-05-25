#ifndef _MANIF_MANIF_BUNDLE_BASE_H_
#define _MANIF_MANIF_BUNDLE_BASE_H_

#include "manif/impl/lie_group_base.h"
#include "manif/impl/traits.h"

namespace manif {

/**
 * @brief The base class of the Bundle group.
 */
template<typename _Derived>
struct BundleBase : LieGroupBase<_Derived>
{
private:

  using Base = LieGroupBase<_Derived>;
  using Type = BundleBase<_Derived>;

public:

  /**
   * @brief Number of elements in bundle
   */
  static constexpr std::size_t BundleSize = internal::traits<_Derived>::BundleSize;

  using Elements = typename internal::traits<_Derived>::Elements;

  template <int Idx>
  using Element = typename internal::traits<_Derived>::template Element<Idx>;

  template <int Idx>
  using MapElement = typename internal::traits<_Derived>::template MapElement<Idx>;

  template <int Idx>
  using MapConstElement = typename internal::traits<_Derived>::template MapConstElement<Idx>;

  MANIF_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_AUTO_API
  MANIF_INHERIT_GROUP_OPERATOR

  using Base::coeffs;

  using Transformation = typename internal::traits<_Derived>::Transformation;

  // LieGroup common API

protected:

  using Base::derived;

  MANIF_DEFAULT_CONSTRUCTOR(BundleBase)

public:

  MANIF_GROUP_ML_ASSIGN_OP(BundleBase)

  /**
   * @brief Get the inverse of this.
   * @param[out] -optional- J_minv_m Jacobian of the inverse wrt this.
   */
  LieGroup inverse(OptJacobianRef J_minv_m = {}) const;

  /**
   * @brief Get the corresponding Lie algebra element.
   * @param[out] -optional- J_t_m Jacobian of the tangent wrt to this.
   * @return The tangent of this.
   */
  Tangent log(OptJacobianRef J_t_m = {}) const;

  /**
   * @brief This function is deprecated.
   * Please consider using
   * @ref log instead.
   */
  MANIF_DEPRECATED
  Tangent lift(OptJacobianRef J_t_m = {}) const;

  /**
   * @brief Composition of this and another Bundle element.
   * @param[in] m Another Bundle element.
   * @param[out] -optional- J_mc_ma Jacobian of the composition wrt this.
   * @param[out] -optional- J_mc_mb Jacobian of the composition wrt m.
   * @return The composition of 'this . m'.
   */
  template<typename _DerivedOther>
  LieGroup compose(
    const LieGroupBase<_DerivedOther> & m,
    OptJacobianRef J_mc_ma = {},
    OptJacobianRef J_mc_mb = {}
  ) const;

  /**
   * @brief Bundle group action
   * @param v vector.
   * @param[out] -optional- J_vout_m The Jacobian of the new object wrt this.
   * @param[out] -optional- J_vout_v The Jacobian of the new object wrt input object.
   * @return The translated vector.
   */
  Vector act(
    const Vector & v,
    tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, DoF>>> J_vout_m = {},
    tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, Dim>>> J_vout_v = {}
  ) const;

  /**
   * @brief Get the adjoint matrix at this.
   */
  Jacobian adj() const;


  // Bundle-specific API

  /**
   * @brief Get the element-diagonal transformation matrix
   */
  Transformation transform() const;

  /**
   * @brief Access Bundle element as Map
   * @tparam _Idx element index
   */
  template<int _Idx>
  MapElement<_Idx> element();

  /**
   * @brief Access Bundle element as Map to const
   * @tparam _Idx element index
   */
  template<int _Idx>
  MapConstElement<_Idx> element() const;

protected:

  template <int ... _Idx>
  LieGroup inverse_impl(OptJacobianRef, internal::intseq<_Idx...>) const;

  template <int ... _Idx>
  Tangent log_impl(OptJacobianRef, internal::intseq<_Idx...>) const;

  template<typename _DerivedOther, int ... _Idx>
  LieGroup compose_impl(
    const LieGroupBase<_DerivedOther> & m,
    OptJacobianRef J_mc_ma,
    OptJacobianRef J_mc_mb,
    internal::intseq<_Idx...>
  ) const;

  template<int ... _Idx>
  Vector act_impl(
    const Vector & v,
    tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, DoF>>> J_vout_m,
    tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, Dim>>> J_vout_v,
    internal::intseq<_Idx...>
  ) const;

  template <int ... _Idx>
  Jacobian adj_impl(internal::intseq<_Idx...>) const;

  template <int ... _Idx>
  Transformation
  transform_impl(internal::intseq<_Idx...>) const;
};


template<typename _Derived>
typename BundleBase<_Derived>::Transformation
BundleBase<_Derived>::transform() const
{
  return transform_impl(internal::make_intseq_t<BundleSize>{});
}

template<typename _Derived>
template<int ... _Idx>
typename BundleBase<_Derived>::Transformation
BundleBase<_Derived>::transform_impl(
  internal::intseq<_Idx...>
) const {
  Transformation ret = Transformation::Zero();
  // cxx11 "fold expression"
  auto l =
  {((ret.template element<
    Element<_Idx>::Dim+1, Element<_Idx>::Dim+1
  >(
    std::get<_Idx>(internal::traits<_Derived>::TraIdx),
    std::get<_Idx>(internal::traits<_Derived>::TraIdx)
  ) = element<_Idx>().transform()), 0) ...};
  static_cast<void>(l);  // compiler warning
  return ret;
}

template<typename _Derived>
typename BundleBase<_Derived>::LieGroup
BundleBase<_Derived>::inverse(OptJacobianRef J_minv_m) const
{
  if (J_minv_m) {
    J_minv_m->setZero();
  }
  return inverse_impl(J_minv_m, internal::make_intseq_t<BundleSize>{});
}

template<typename _Derived>
template<int ... _Idx>
typename BundleBase<_Derived>::LieGroup
BundleBase<_Derived>::inverse_impl(
  OptJacobianRef J_minv_m, internal::intseq<_Idx...>
) const {
  if (J_minv_m) {
    return LieGroup(
      element<_Idx>().inverse(
        J_minv_m->template block<
          Element<_Idx>::DoF,
          Element<_Idx>::DoF
        >(
          std::get<_Idx>(internal::traits<_Derived>::DoFIdx),
          std::get<_Idx>(internal::traits<_Derived>::DoFIdx)
        )
      ) ...
    );
  }
  return LieGroup(element<_Idx>().inverse() ...);
}

template<typename _Derived>
typename BundleBase<_Derived>::Tangent
BundleBase<_Derived>::log(OptJacobianRef J_t_m) const
{
  if (J_t_m) {
    J_t_m->setZero();
  }
  return log_impl(J_t_m, internal::make_intseq_t<BundleSize>{});
}

template<typename _Derived>
template<int ... _Idx>
typename BundleBase<_Derived>::Tangent
BundleBase<_Derived>::log_impl(
  OptJacobianRef J_minv_m,
  internal::intseq<_Idx...>
) const {
  if (J_minv_m) {
    return Tangent(
      element<_Idx>().log(
        J_minv_m->template block<
          Element<_Idx>::DoF,
          Element<_Idx>::DoF
        >(
          std::get<_Idx>(internal::traits<_Derived>::DoFIdx),
          std::get<_Idx>(internal::traits<_Derived>::DoFIdx)
        )
      )...
    );
  }
  return Tangent(element<_Idx>().log() ...);
}

template<typename _Derived>
typename BundleBase<_Derived>::Tangent
BundleBase<_Derived>::lift(OptJacobianRef J_t_m) const
{
  return log(J_t_m);
}

template<typename _Derived>
template<typename _DerivedOther>
typename BundleBase<_Derived>::LieGroup
BundleBase<_Derived>::compose(
  const LieGroupBase<_DerivedOther> & m,
  OptJacobianRef J_mc_ma,
  OptJacobianRef J_mc_mb
) const {
  if (J_mc_ma) {
    J_mc_ma->setZero();
  }
  if (J_mc_mb) {
    J_mc_mb->setZero();
  }
  return compose_impl(m, J_mc_ma, J_mc_mb, internal::make_intseq_t<BundleSize>{});
}

template<typename _Derived>
template<typename _DerivedOther, int ... _Idx>
typename BundleBase<_Derived>::LieGroup
BundleBase<_Derived>::compose_impl(
  const LieGroupBase<_DerivedOther> & m,
  OptJacobianRef J_mc_ma,
  OptJacobianRef J_mc_mb,
  internal::intseq<_Idx...>
) const {
  return LieGroup(
    element<_Idx>().compose(
      static_cast<const _DerivedOther &>(m).template element<_Idx>(),
      J_mc_ma ?
        J_mc_ma->template block<
          Element<_Idx>::DoF, Element<_Idx>::DoF
        >(
          std::get<_Idx>(internal::traits<_Derived>::DoFIdx),
          std::get<_Idx>(internal::traits<_Derived>::DoFIdx)
        ) :
        tl::optional<
          Eigen::Ref<Eigen::Matrix<Scalar, Element<_Idx>::DoF, Element<_Idx>::DoF>>
        >{},
      J_mc_mb ?
        J_mc_mb->template block<
          Element<_Idx>::DoF, Element<_Idx>::DoF
        >(
          std::get<_Idx>(internal::traits<_Derived>::DoFIdx),
          std::get<_Idx>(internal::traits<_Derived>::DoFIdx)
        ) :
        tl::optional<
          Eigen::Ref<Eigen::Matrix<Scalar, Element<_Idx>::DoF, Element<_Idx>::DoF>>
        >{}
    ) ...
  );
}

template<typename _Derived>
typename BundleBase<_Derived>::Vector
BundleBase<_Derived>::act(
  const typename BundleBase<_Derived>::Vector & v,
  tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, DoF>>> J_vout_m,
  tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, Dim>>> J_vout_v) const
{
  if (J_vout_m) {
    J_vout_m->setZero();
  }
  if (J_vout_v) {
    J_vout_v->setZero();
  }

  return act_impl(v, J_vout_m, J_vout_v, internal::make_intseq_t<BundleSize>{});
}

template<typename _Derived>
template<int ... _Idx>
typename BundleBase<_Derived>::Vector
BundleBase<_Derived>::act_impl(
  const typename BundleBase<_Derived>::Vector & v,
  tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, DoF>>> J_vout_m,
  tl::optional<Eigen::Ref<Eigen::Matrix<Scalar, Dim, Dim>>> J_vout_v,
  internal::intseq<_Idx...>
) const {
  Vector ret;
  // cxx11 "fold expression"
  auto l = {((ret.template segment<Element<_Idx>::Dim>(
    std::get<_Idx>(internal::traits<_Derived>::DimIdx)
  ) = element<_Idx>().act(
    v.template segment<Element<_Idx>::Dim>(
        std::get<_Idx>(internal::traits<_Derived>::DimIdx)
    ),
    J_vout_m ?
      J_vout_m->template block<Element<_Idx>::Dim, Element<_Idx>::DoF>(
        std::get<_Idx>(internal::traits<_Derived>::DimIdx),
        std::get<_Idx>(internal::traits<_Derived>::DoFIdx)
      ) :
      tl::optional<
        Eigen::Ref<Eigen::Matrix<Scalar, Element<_Idx>::Dim, Element<_Idx>::DoF>>
      >{},
      J_vout_v ?
        J_vout_v->template block<Element<_Idx>::Dim, Element<_Idx>::Dim>(
          std::get<_Idx>(internal::traits<_Derived>::DimIdx),
          std::get<_Idx>(internal::traits<_Derived>::DimIdx)
        ) :
        tl::optional<
          Eigen::Ref<Eigen::Matrix<Scalar, Element<_Idx>::Dim, Element<_Idx>::Dim>>
        >{}
    )
  ), 0) ...};
  static_cast<void>(l);  // compiler warning
  return ret;
}

template<typename _Derived>
typename BundleBase<_Derived>::Jacobian
BundleBase<_Derived>::adj() const
{
  return adj_impl(internal::make_intseq_t<BundleSize>{});
}

template<typename _Derived>
template<int ... _Idx>
typename BundleBase<_Derived>::Jacobian
BundleBase<_Derived>::adj_impl(internal::intseq<_Idx...>) const
{
  Jacobian adj = Jacobian::Zero();
  // cxx11 "fold expression"
  auto l = {((adj.template block<
    Element<_Idx>::DoF, Element<_Idx>::DoF
  >(
    std::get<_Idx>(internal::traits<_Derived>::DoFIdx),
    std::get<_Idx>(internal::traits<_Derived>::DoFIdx)
  ) = element<_Idx>().adj()), 0) ...};
  static_cast<void>(l);  // compiler warning
  return adj;
}

template<typename _Derived>
template<int _Idx>
auto BundleBase<_Derived>::element() -> MapElement<_Idx>
{
  return MapElement<_Idx>(
    static_cast<_Derived &>(*this).coeffs().data() +
    std::get<_Idx>(internal::traits<_Derived>::RepSizeIdx)
  );
}

template<typename _Derived>
template<int _Idx>
auto BundleBase<_Derived>::element() const -> MapConstElement<_Idx>
{
  return MapConstElement<_Idx>(
    static_cast<const _Derived &>(*this).coeffs().data() +
    std::get<_Idx>(internal::traits<_Derived>::RepSizeIdx)
  );
}

namespace internal {

/**
 * @brief Random specialization for Bundle objects.
 */
template<typename Derived>
struct RandomEvaluatorImpl<BundleBase<Derived>>
{
  static void run(BundleBase<Derived> & m)
  {
    run(m, internal::make_intseq_t<Derived::BundleSize>{});
  }

  template <int ... _Idx>
  static void run(BundleBase<Derived> & m, internal::intseq<_Idx...>)
  {
    m = typename BundleBase<Derived>::LieGroup(
      BundleBase<Derived>::template Element<_Idx>::Random() ...
    );
  }
};

}  // namespace internal
}  // namespace manif

#endif  // _MANIF_MANIF_BUNDLE_BASE_H_
