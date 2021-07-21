#ifndef _MANIF_MANIF_BUNDLE_H_
#define _MANIF_MANIF_BUNDLE_H_

#include "manif/impl/bundle/Bundle_base.h"
#include "manif/impl/traits.h"

namespace manif {

// Forward declare for type traits specialization

template <typename _Scalar, template<typename> class ... _T> struct Bundle;
template <typename _Scalar, template<typename> class ... _T> struct BundleTangent;

namespace internal {

//! Traits specialization
template <typename _Scalar, template<typename> class ... _T>
struct traits<Bundle<_Scalar, _T ...>>
{
  // Bundle-specific traits
  static constexpr std::size_t BundleSize = sizeof...(_T);

  using Elements = std::tuple<_T<_Scalar>...>;

  template <int _N>
  using Element = typename std::tuple_element<_N, Elements>::type;

  template <int _N>
  using MapElement = Eigen::Map<Element<_N>>;

  template <int _N>
  using MapConstElement = Eigen::Map<const Element<_N>>;

  static constexpr std::array<int, sizeof...(_T)> DimIdx = compute_indices<_T<_Scalar>::Dim ...>();
  static constexpr std::array<int, sizeof...(_T)> DoFIdx = compute_indices<_T<_Scalar>::DoF ...>();
  static constexpr std::array<int, sizeof...(_T)> RepSizeIdx = compute_indices<_T<_Scalar>::RepSize ...>();
  static constexpr std::array<int, sizeof...(_T)> TraIdx = compute_indices<_T<_Scalar>::Transformation::RowsAtCompileTime ...>();

  // Regular traits
  using Scalar = _Scalar;

  using LieGroup = Bundle<_Scalar, _T ...>;
  using Tangent = BundleTangent<_Scalar, _T ...>;

  using Base = BundleBase<Bundle<_Scalar, _T ...>>;

  static constexpr int Dim = accumulate(int(_T<_Scalar>::Dim) ...);
  static constexpr int DoF = accumulate(int(_T<_Scalar>::DoF) ...);
  static constexpr int RepSize = accumulate(int(_T<_Scalar>::RepSize) ...);

  using DataType = Eigen::Matrix<_Scalar, RepSize, 1>;
  using Jacobian = Eigen::Matrix<_Scalar, DoF, DoF>;
  using Transformation = SquareMatrix<
    _Scalar,
    accumulate(int(_T<_Scalar>::Transformation::RowsAtCompileTime) ...)
  >;
  using Vector = Eigen::Matrix<_Scalar, Dim, 1>;
};

template <typename _Scalar, template<typename> class ... _T>
const constexpr std::array<int, sizeof...(_T)> traits<Bundle<_Scalar, _T ...>>::DimIdx;
template <typename _Scalar, template<typename> class ... _T>
const constexpr std::array<int, sizeof...(_T)> traits<Bundle<_Scalar, _T ...>>::DoFIdx;
template <typename _Scalar, template<typename> class ... _T>
const constexpr std::array<int, sizeof...(_T)> traits<Bundle<_Scalar, _T ...>>::RepSizeIdx;
template <typename _Scalar, template<typename> class ... _T>
const constexpr std::array<int, sizeof...(_T)> traits<Bundle<_Scalar, _T ...>>::TraIdx;
template <typename _Scalar, template<typename> class ... _T>
const constexpr int traits<Bundle<_Scalar, _T ...>>::Dim;
template <typename _Scalar, template<typename> class ... _T>
const constexpr int traits<Bundle<_Scalar, _T ...>>::DoF;
template <typename _Scalar, template<typename> class ... _T>
const constexpr int traits<Bundle<_Scalar, _T ...>>::RepSize;

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

protected:

  using Base::derived;

public:

  template <int Idx> using Element = typename Base::template Element<Idx>;
  using Base::BundleSize;

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND

  MANIF_COMPLETE_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_API

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
  template <int ... _Idx>
  Bundle(internal::intseq<_Idx...>, const _T<_Scalar> & ... elements);

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
: Bundle(internal::make_intseq_t<BundleSize>{}, elements ...)
{}

template<typename _Scalar, template<typename> class ... _T>
template<int ... _Idx>
Bundle<_Scalar, _T...>::Bundle(
  internal::intseq<_Idx...>, const _T<_Scalar> & ... elements
)
{
  // c++11 "fold expression"
  auto l = {((data_.template segment<Element<_Idx>::RepSize>(
    std::get<_Idx>(internal::traits<Type>::RepSizeIdx)
  ) = elements.coeffs()), 0) ...};
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
