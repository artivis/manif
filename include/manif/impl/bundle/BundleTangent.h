#ifndef _MANIF_MANIF_BUNDLETANGENT_H_
#define _MANIF_MANIF_BUNDLETANGENT_H_

#include "manif/impl/bundle/BundleTangent_base.h"
#include "manif/impl/traits.h"

namespace manif {

// Forward declare for type traits specialization

template<typename _Scalar, template<typename> class ... _T> struct Bundle;
template<typename _Scalar, template<typename> class ... _T> struct BundleTangent;

namespace internal {

//! Traits specialization
template<typename _Scalar, template<typename> class ... _T>
struct traits<BundleTangent<_Scalar, _T...>>
{
  // BundleTangent-specific traits
  static constexpr std::size_t BundleSize = sizeof...(_T);

  using Elements = std::tuple<typename _T<_Scalar>::Tangent...>;

  template <int _N>
  using Element = typename std::tuple_element<_N, Elements>::type;

  template <int _N>
  using MapElement = Eigen::Map<Element<_N>>;

  template <int _N>
  using MapConstElement = Eigen::Map<const Element<_N>>;

  static constexpr std::array<int, sizeof...(_T)> DoFIdx = compute_indices<_T<_Scalar>::Tangent::DoF ...>();
  static constexpr std::array<int, sizeof...(_T)> RepSizeIdx = compute_indices<_T<_Scalar>::Tangent::RepSize ...>();
  static constexpr std::array<int, sizeof...(_T)> AlgIdx = compute_indices<_T<_Scalar>::Tangent::LieAlg::RowsAtCompileTime ...>();

  // Regular traits
  using Scalar = _Scalar;

  using LieGroup = Bundle<_Scalar, _T...>;
  using Tangent = BundleTangent<_Scalar, _T...>;

  using Base = BundleTangentBase<Tangent>;

  static constexpr int Dim = accumulate(int(_T<_Scalar>::Tangent::Dim) ...);
  static constexpr int DoF = accumulate(int(_T<_Scalar>::Tangent::DoF) ...);
  static constexpr int RepSize = accumulate(int(_T<_Scalar>::Tangent::RepSize) ...);

  using DataType = Eigen::Matrix<Scalar, RepSize, 1>;
  using Jacobian = Eigen::Matrix<Scalar, DoF, DoF>;
  using LieAlg = SquareMatrix<
    Scalar,
    accumulate(int(_T<_Scalar>::Tangent::LieAlg::RowsAtCompileTime) ...)
  >;
};

template <typename _Scalar, template<typename> class ... _T>
const constexpr std::array<int, sizeof...(_T)> traits<BundleTangent<_Scalar, _T ...>>::DoFIdx;
template <typename _Scalar, template<typename> class ... _T>
const constexpr std::array<int, sizeof...(_T)> traits<BundleTangent<_Scalar, _T ...>>::RepSizeIdx;
template <typename _Scalar, template<typename> class ... _T>
const constexpr std::array<int, sizeof...(_T)> traits<BundleTangent<_Scalar, _T ...>>::AlgIdx;
template <typename _Scalar, template<typename> class ... _T>
const constexpr int traits<BundleTangent<_Scalar, _T ...>>::Dim;
template <typename _Scalar, template<typename> class ... _T>
const constexpr int traits<BundleTangent<_Scalar, _T ...>>::DoF;
template <typename _Scalar, template<typename> class ... _T>
const constexpr int traits<BundleTangent<_Scalar, _T ...>>::RepSize;

}  // namespace internal

//
// BundleTangent
//

/**
 * @brief Represents a BundleTangent element.
 */
template<typename _Scalar, template<typename> class ... _T>
struct BundleTangent : BundleTangentBase<BundleTangent<_Scalar, _T...>>
{
private:

  static_assert(sizeof...(_T) > 0, "Must have at least one element in BundleTangent !");

  using Base = BundleTangentBase<BundleTangent<_Scalar, _T...>>;
  using Type = BundleTangent<_Scalar, _T...>;

protected:

  using Base::derived;

public:

  template <int Idx> using Element = typename Base::template Element<Idx>;
  using Base::BundleSize;

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  BundleTangent()  = default;
  ~BundleTangent() = default;

  MANIF_COPY_CONSTRUCTOR(BundleTangent)
  MANIF_MOVE_CONSTRUCTOR(BundleTangent)

  // Copy constructors given base
  template<typename _DerivedOther>
  BundleTangent(const TangentBase<_DerivedOther> & o);

  MANIF_TANGENT_ASSIGN_OP(BundleTangent)

  // Tangent common API

  /**
   * @brief Get a reference to the underlying DataType.
   */
  DataType & coeffs();

  /**
   * @brief Get a const reference to the underlying DataType.
   */
  const DataType & coeffs() const;


  // BundleTangent specific API

  /**
   * @brief Construct from BundleTangent elements
   */
  BundleTangent(const typename _T<_Scalar>::Tangent & ... elements);

protected:

  // Helper for the elements constructor
  template <int ... _Idx>
  BundleTangent(
    internal::intseq<_Idx...>,
    const typename _T<_Scalar>::Tangent & ... elements
  );

protected:

  DataType data_;
};

template<typename _Scalar, template<typename> class ... _T>
template<typename _DerivedOther>
BundleTangent<_Scalar, _T...>::BundleTangent(const TangentBase<_DerivedOther> & o)
: data_(o.coeffs())
{}

template<typename _Scalar, template<typename> class ... _T>
BundleTangent<_Scalar, _T...>::BundleTangent(const typename _T<_Scalar>::Tangent & ... elements)
: BundleTangent(internal::make_intseq_t<BundleSize>{}, elements ...)
{}

template<typename _Scalar, template<typename> class ... _T>
template<int ... _Idx>
BundleTangent<_Scalar, _T...>::BundleTangent(
  internal::intseq<_Idx...>,
  const typename _T<_Scalar>::Tangent & ... elements
) {
  // c++11 "fold expression"
  auto l = {((data_.template segment<Element<_Idx>::RepSize>(
    std::get<_Idx>(internal::traits<Type>::RepSizeIdx)
  ) = elements.coeffs()), 0) ...};
  static_cast<void>(l);  // compiler warning
}

template<typename _Scalar, template<typename> class ... _T>
typename BundleTangent<_Scalar, _T...>::DataType &
BundleTangent<_Scalar, _T...>::coeffs()
{
  return data_;
}

template<typename _Scalar, template<typename> class ... _T>
const typename BundleTangent<_Scalar, _T...>::DataType &
BundleTangent<_Scalar, _T...>::coeffs() const
{
  return data_;
}

}  // namespace manif

#endif  // _MANIF_MANIF_BUNDLETANGENT_H_
