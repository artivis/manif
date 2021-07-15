#ifndef _MANIF_MANIF_RxSO3TANGENT_MAP_H_
#define _MANIF_MANIF_RxSO3TANGENT_MAP_H_

#include "manif/impl/rxso3/RxSO3Tangent.h"

namespace manif {
namespace internal {

//! @brief traits specialization for Eigen Map
template <typename _Scalar>
struct traits< Eigen::Map<RxSO3Tangent<_Scalar>,0> >
    : public traits<RxSO3Tangent<_Scalar>>
{
  using typename traits<RxSO3Tangent<_Scalar>>::Scalar;
  using traits<RxSO3Tangent<_Scalar>>::DoF;
  using DataType = ::Eigen::Map<Eigen::Matrix<Scalar, DoF, 1>, 0>;
  using Base = RxSO3TangentBase<Eigen::Map<RxSO3Tangent<Scalar>, 0>>;
};

//! @brief traits specialization for Eigen Map const
template <typename _Scalar>
struct traits< Eigen::Map<const RxSO3Tangent<_Scalar>,0> >
    : public traits<const RxSO3Tangent<_Scalar>>
{
  using typename traits<const RxSO3Tangent<_Scalar>>::Scalar;
  using traits<const RxSO3Tangent<_Scalar>>::DoF;
  using DataType = ::Eigen::Map<const Eigen::Matrix<Scalar, DoF, 1>, 0>;
  using Base = RxSO3TangentBase<Eigen::Map<const RxSO3Tangent<Scalar>, 0>>;
};

} // namespace internal
} // namespace manif

namespace Eigen {

/**
 * @brief Specialization of Map for manif::RxSO3Tangent
 */
template <class _Scalar>
class Map<manif::RxSO3Tangent<_Scalar>, 0>
    : public manif::RxSO3TangentBase<Map<manif::RxSO3Tangent<_Scalar>, 0> >
{
  using Base = manif::RxSO3TangentBase<Map<manif::RxSO3Tangent<_Scalar>, 0> >;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  Map(Scalar* coeffs) : data_(coeffs) { }

  MANIF_TANGENT_MAP_ASSIGN_OP(RxSO3Tangent)

  DataType& coeffs() { return data_; }
  const DataType& coeffs() const { return data_; }

protected:

  DataType data_;
};

/**
 * @brief Specialization of Map for const manif::RxSO3Tangent
 */
template <class _Scalar>
class Map<const manif::RxSO3Tangent<_Scalar>, 0>
    : public manif::RxSO3TangentBase<Map<const manif::RxSO3Tangent<_Scalar>, 0> >
{
  using Base = manif::RxSO3TangentBase<Map<const manif::RxSO3Tangent<_Scalar>, 0> >;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  Map(const Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

protected:

  const DataType data_;
};

} // namespace Eigen

#endif // _MANIF_MANIF_RxSO3TANGENT_MAP_H_
