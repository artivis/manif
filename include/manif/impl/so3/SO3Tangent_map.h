#ifndef _MANIF_MANIF_SO3TANGENT_MAP_H_
#define _MANIF_MANIF_SO3TANGENT_MAP_H_

#include "manif/impl/so3/SO3Tangent.h"

namespace manif {
namespace internal {

//! @brief traits specialization for Eigen Map
template <typename _Scalar>
struct traits< Eigen::Map<SO3Tangent<_Scalar>,0> >
    : public traits<SO3Tangent<_Scalar>>
{
  using typename traits<SO3Tangent<_Scalar>>::Scalar;
  using traits<SO3Tangent<_Scalar>>::DoF;
  using DataType = ::Eigen::Map<Eigen::Matrix<Scalar, DoF, 1>, 0>;
  using Base = SO3TangentBase<Eigen::Map<SO3Tangent<Scalar>, 0>>;
};

//! @brief traits specialization for Eigen Map const
template <typename _Scalar>
struct traits< Eigen::Map<const SO3Tangent<_Scalar>,0> >
    : public traits<const SO3Tangent<_Scalar>>
{
  using typename traits<const SO3Tangent<_Scalar>>::Scalar;
  using traits<const SO3Tangent<_Scalar>>::DoF;
  using DataType = ::Eigen::Map<const Eigen::Matrix<Scalar, DoF, 1>, 0>;
  using Base = SO3TangentBase<Eigen::Map<const SO3Tangent<Scalar>, 0>>;
};

} /* namespace internal */
} /* namespace manif */

namespace Eigen {

/**
 * @brief Specialization of Map for manif::SO3Tangent
 */
template <class _Scalar>
class Map<manif::SO3Tangent<_Scalar>, 0>
    : public manif::SO3TangentBase<Map<manif::SO3Tangent<_Scalar>, 0> >
{
  using Base = manif::SO3TangentBase<Map<manif::SO3Tangent<_Scalar>, 0> >;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  Map(Scalar* coeffs) : data_(coeffs) { }

  MANIF_TANGENT_MAP_ASSIGN_OP(SO3Tangent)

  DataType& coeffs() { return data_; }
  const DataType& coeffs() const { return data_; }

protected:

  DataType data_;
};

/**
 * @brief Specialization of Map for const manif::SO3Tangent
 */
template <class _Scalar>
class Map<const manif::SO3Tangent<_Scalar>, 0>
    : public manif::SO3TangentBase<Map<const manif::SO3Tangent<_Scalar>, 0> >
{
  using Base = manif::SO3TangentBase<Map<const manif::SO3Tangent<_Scalar>, 0> >;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  Map(const Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

protected:

  const DataType data_;
};

} /* namespace Eigen */

#endif /* _MANIF_MANIF_SO3TANGENT_MAP_H_ */
