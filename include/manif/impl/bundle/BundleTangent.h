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
  using IdxList = typename make_intseq<sizeof...(_T)>::type;

  using LenDim = intseq<_T<_Scalar>::Tangent::Dim ...>;
  using BegDim = intseq_psum_t<LenDim>;

  using LenDoF = intseq<_T<_Scalar>::Tangent::DoF ...>;
  using BegDoF = intseq_psum_t<LenDoF>;

  using LenRep = intseq<_T<_Scalar>::Tangent::RepSize ...>;
  using BegRep = intseq_psum_t<LenRep>;

  using LenAlg = intseq<_T<_Scalar>::Tangent::LieAlg::RowsAtCompileTime ...>;
  using BegAlg = intseq_psum_t<LenAlg>;

  template<std::size_t _Idx>
  using BlockType = typename bundle_element<_Idx, _T<_Scalar>...>::type::Tangent;

  // Regular traits
  using Scalar = _Scalar;

  using LieGroup = Bundle<_Scalar, _T...>;
  using Tangent = BundleTangent<_Scalar, _T...>;

  using Base = BundleTangentBase<Tangent>;

  static constexpr int Dim = intseq_sum<LenDim>::value;
  static constexpr int DoF = intseq_sum<LenDoF>::value;
  static constexpr int RepSize = intseq_sum<LenRep>::value;

  using DataType = Eigen::Matrix<Scalar, RepSize, 1>;
  using Jacobian = Eigen::Matrix<Scalar, DoF, DoF>;
  using LieAlg = Eigen::Matrix<Scalar, intseq_sum<LenAlg>::value,
      intseq_sum<LenAlg>::value>;
};

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

  using BegRep = typename internal::traits<BundleTangent>::BegRep;
  using LenRep = typename internal::traits<BundleTangent>::LenRep;

protected:

  using Base::derived;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  using Base::BundleSize;

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
   * @brief Construct from BundleTangent blocks
   */
  BundleTangent(const typename _T<_Scalar>::Tangent & ... blocks);

protected:

  // Helper for the blocks constructor
  template<int ... _BegRep, int ... _LenRep>
  BundleTangent(
    internal::intseq<_BegRep...>, internal::intseq<_LenRep...>,
    const typename _T<_Scalar>::Tangent & ... blocks);

protected:

  DataType data_;
};


template<typename _Scalar, template<typename> class ... _T>
template<typename _DerivedOther>
BundleTangent<_Scalar, _T...>::BundleTangent(const TangentBase<_DerivedOther> & o)
: data_(o.coeffs())
{}

template<typename _Scalar, template<typename> class ... _T>
BundleTangent<_Scalar, _T...>::BundleTangent(const typename _T<_Scalar>::Tangent & ... blocks)
: BundleTangent(BegRep{}, LenRep{}, blocks ...)
{}

template<typename _Scalar, template<typename> class ... _T>
template<int ... _BegRep, int ... _LenRep>
BundleTangent<_Scalar, _T...>::BundleTangent(
  internal::intseq<_BegRep...>, internal::intseq<_LenRep...>,
  const typename _T<_Scalar>::Tangent & ... blocks)
{
  // c++11 "fold expression"
  auto l = {((data_.template segment<_LenRep>(_BegRep) = blocks.coeffs()), 0) ...};
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
