#ifndef _MANIF_MANIF_BUNDLE_H_
#define _MANIF_MANIF_BUNDLE_H_

#include "manif/impl/bundle/Bundle_base.h"
#include "manif/impl/traits.h"

namespace manif {

// Forward declare for type traits specialization

template<typename _Scalar, template<typename> class ... _T> struct Bundle;
template<typename _Scalar, template<typename> class ... _T> struct BundleTangent;

namespace internal {

//! Traits specialization
template<typename _Scalar, template<typename> class ... _T>
struct traits<Bundle<_Scalar, _T ...>>
{
  // Bundle-specific traits
  using IdxList = typename make_intseq<sizeof...(_T)>::type;

  using LenDim = intseq<_T<_Scalar>::Dim ...>;
  using BegDim = intseq_psum_t<LenDim>;

  using LenDoF = intseq<_T<_Scalar>::DoF ...>;
  using BegDoF = intseq_psum_t<LenDoF>;

  using LenTra = intseq<_T<_Scalar>::Transformation::RowsAtCompileTime ...>;
  using BegTra = intseq_psum_t<LenTra>;

  using LenRep = intseq<_T<_Scalar>::RepSize ...>;
  using BegRep = intseq_psum_t<LenRep>;

  template<std::size_t _Idx>
  using ElementType = typename bundle_element<_Idx, _T<_Scalar>...>::type;

  // Regular traits
  using Scalar = _Scalar;

  using LieGroup = Bundle<_Scalar, _T ...>;
  using Tangent = BundleTangent<_Scalar, _T ...>;

  using Base = BundleBase<Bundle<_Scalar, _T ...>>;

  static constexpr int Dim = intseq_sum<LenDim>::value;
  static constexpr int DoF = intseq_sum<LenDoF>::value;
  static constexpr int RepSize = intseq_sum<LenRep>::value;

  using DataType = Eigen::Matrix<_Scalar, RepSize, 1>;
  using Jacobian = Eigen::Matrix<_Scalar, DoF, DoF>;
  using Transformation = Eigen::Matrix<
    _Scalar, intseq_sum<LenTra>::value, intseq_sum<LenTra>::value
  >;
  using Vector = Eigen::Matrix<_Scalar, Dim, 1>;
};

}  // namespace internal

//
// Bundle LieGroup
//

/**
 * @brief Represents a Bundle (or Composite) element as
 *        described in Section IV of the reference paper
 *        (see also Example 7).
 *
 * A Bundle <G1, ..., Gn> of Lie groups can be utilized as
 * a single group with element-wise operations. This can be
 * convenient when working with aggregate states that consist of
 * multiple Lie group sub-states, like the example in Section VIIb
 * of the reference paper.
 *
 * Example: create an element of the composite <SO3, E3, E3>
 *          using double as the scalar type.
 *
 *   > Bundle<double, SO3, R3, R3> element;
 */
template<typename _Scalar, template<typename> class ... _T>
struct Bundle : BundleBase<Bundle<_Scalar, _T ...>>
{
private:

  static_assert(sizeof...(_T) > 0, "Must have at least one element in Bundle !");

  using Base = BundleBase<Bundle<_Scalar, _T...>>;
  using Type = Bundle<_Scalar, _T...>;

  using BegRep = typename internal::traits<Bundle>::BegRep;
  using LenRep = typename internal::traits<Bundle>::LenRep;

protected:

  using Base::derived;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND

  MANIF_COMPLETE_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_API

  using Base::BundleSize;

  Bundle()  = default;
  ~Bundle() = default;

  MANIF_COPY_CONSTRUCTOR(Bundle);
  MANIF_MOVE_CONSTRUCTOR(Bundle);

  // Copy constructor
  template<typename _DerivedOther>
  Bundle(const LieGroupBase<_DerivedOther> & o);

  MANIF_GROUP_ASSIGN_OP(Bundle);

  // LieGroup common API

  /**
   * @brief Get a reference to the underlying DataType.
   * @param[out] a reference to the underlying Eigen vector
   */
  DataType & coeffs();

  /**
   * @brief Get a const reference to the underlying DataType.
   * @param[out] a const reference to the underlying Eigen vector
   */
  const DataType & coeffs() const;


  // Bundle specific API

  /**
   * @brief Construct from Bundle elements
   */
  Bundle(const _T<_Scalar> & ... elements);

protected:

  // Helper for the elements constructor
  template<int ... _BegRep, int ... _LenRep>
  Bundle(internal::intseq<_BegRep...>, internal::intseq<_LenRep...>, const _T<_Scalar> & ... elements);

protected:

  //! Underlying data (Eigen) vector
  DataType data_;
};


template<typename _Scalar, template<typename> class ... _T>
template<typename _DerivedOther>
Bundle<_Scalar, _T...>::Bundle(const LieGroupBase<_DerivedOther> & o)
: Bundle(o.coeffs())
{}

template<typename _Scalar, template<typename> class ... _T>
Bundle<_Scalar, _T...>::Bundle(const _T<_Scalar> & ... elements)
: Bundle(BegRep{}, LenRep{}, elements ...)
{}

template<typename _Scalar, template<typename> class ... _T>
template<int ... _BegRep, int ... _LenRep>
Bundle<_Scalar, _T...>::Bundle(
  internal::intseq<_BegRep...>, internal::intseq<_LenRep...>, const _T<_Scalar> & ... elements)
{
  // c++11 "fold expression"
  auto l = {((data_.template segment<_LenRep>(_BegRep) = elements.coeffs()), 0) ...};
  static_cast<void>(l);  // compiler warning
}

template<typename _Scalar, template<typename> class ... _T>
typename Bundle<_Scalar, _T...>::DataType &
Bundle<_Scalar, _T...>::coeffs()
{
  return data_;
}

template<typename _Scalar, template<typename> class ... _T>
const typename Bundle<_Scalar, _T...>::DataType &
Bundle<_Scalar, _T...>::coeffs() const
{
  return data_;
}

}  // namespace manif

#endif  // _MANIF_MANIF_BUNDLE_H_
