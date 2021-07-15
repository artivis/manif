#ifndef _MANIF_MANIF_RxSO2TANGENT_MAP_H_
#define _MANIF_MANIF_RxSO2TANGENT_MAP_H_

#include "manif/impl/rxso2/RxSO2Tangent.h"

namespace manif {
namespace internal {

//! @brief traits specialization for Eigen Map
template <typename _Scalar>
struct traits< Eigen::Map<RxSO2Tangent<_Scalar>,0> >
    : public traits<RxSO2Tangent<_Scalar>>
{
  using typename traits<RxSO2Tangent<_Scalar>>::Scalar;
  using traits<RxSO2Tangent<_Scalar>>::DoF;
  using DataType = ::Eigen::Map<Eigen::Matrix<Scalar, DoF, 1>, 0>;
  using Base = RxSO2TangentBase<Eigen::Map<RxSO2Tangent<Scalar>, 0>>;
};

//! @brief traits specialization for Eigen Map const
template <typename _Scalar>
struct traits< Eigen::Map<const RxSO2Tangent<_Scalar>,0> >
    : public traits<const RxSO2Tangent<_Scalar>>
{
  using typename traits<const RxSO2Tangent<_Scalar>>::Scalar;
  using traits<const RxSO2Tangent<_Scalar>>::DoF;
  using DataType = ::Eigen::Map<const Eigen::Matrix<Scalar, DoF, 1>, 0>;
  using Base = RxSO2TangentBase<Eigen::Map<const RxSO2Tangent<Scalar>, 0>>;
};

} // namespace internal
} // namespace manif

namespace Eigen {

/**
 * @brief Specialization of Map for manif::RxSO2Tangent
 */
template <class _Scalar>
class Map<manif::RxSO2Tangent<_Scalar>, 0>
    : public manif::RxSO2TangentBase<Map<manif::RxSO2Tangent<_Scalar>, 0> >
{
  using Base = manif::RxSO2TangentBase<Map<manif::RxSO2Tangent<_Scalar>, 0> >;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  Map(Scalar* coeffs) : data_(coeffs) { }

  MANIF_TANGENT_MAP_ASSIGN_OP(RxSO2Tangent)

  DataType& coeffs() { return data_; }
  const DataType& coeffs() const { return data_; }

protected:

  DataType data_;
};

/**
 * @brief Specialization of Map for const manif::RxSO2Tangent
 */
template <class _Scalar>
class Map<const manif::RxSO2Tangent<_Scalar>, 0>
    : public manif::RxSO2TangentBase<Map<const manif::RxSO2Tangent<_Scalar>, 0> >
{
  using Base = manif::RxSO2TangentBase<Map<const manif::RxSO2Tangent<_Scalar>, 0> >;

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

#endif // _MANIF_MANIF_RxSO2TANGENT_MAP_H_
