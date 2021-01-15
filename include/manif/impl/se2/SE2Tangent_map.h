#ifndef _MANIF_MANIF_SE2TANGENT_MAP_H_
#define _MANIF_MANIF_SE2TANGENT_MAP_H_

#include "manif/impl/se2/SE2Tangent.h"

namespace manif {
namespace internal {

//! @brief traits specialization for Eigen Map
template <typename _Scalar>
struct traits< Eigen::Map<SE2Tangent<_Scalar>,0> >
    : public traits<SE2Tangent<_Scalar>>
{
  using typename traits<SE2Tangent<_Scalar>>::Scalar;
  using traits<SE2Tangent<_Scalar>>::DoF;
  using DataType = ::Eigen::Map<Eigen::Matrix<Scalar, DoF, 1>, 0>;
  using Base = SE2TangentBase<Eigen::Map<SE2Tangent<Scalar>, 0>>;
};

//! @brief traits specialization for Eigen Map const
template <typename _Scalar>
struct traits< Eigen::Map<const SE2Tangent<_Scalar>,0> >
    : public traits<const SE2Tangent<_Scalar>>
{
  using typename traits<const SE2Tangent<_Scalar>>::Scalar;
  using traits<SE2Tangent<_Scalar>>::DoF;
  using DataType = ::Eigen::Map<const Eigen::Matrix<Scalar, DoF, 1>, 0>;
  using Base = SE2TangentBase<Eigen::Map<const SE2Tangent<Scalar>, 0>>;
};

} /* namespace internal */
} /* namespace manif */

namespace Eigen {

/**
 * @brief Specialization of Map for manif::SE2
 */
template <class _Scalar>
class Map<manif::SE2Tangent<_Scalar>, 0>
    : public manif::SE2TangentBase<Map<manif::SE2Tangent<_Scalar>, 0> >
{
  using Base = manif::SE2TangentBase<Map<manif::SE2Tangent<_Scalar>, 0> >;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  Map(Scalar* coeffs) : data_(coeffs) { }

  MANIF_TANGENT_MAP_ASSIGN_OP(SE2Tangent)

  DataType& coeffs() { return data_; }
  const DataType& coeffs() const { return data_; }

protected:

  DataType data_;
};

/**
 * @brief Specialization of Map for const manif::SE2
 */
template <class _Scalar>
class Map<const manif::SE2Tangent<_Scalar>, 0>
    : public manif::SE2TangentBase<Map<const manif::SE2Tangent<_Scalar>, 0> >
{
  using Base = manif::SE2TangentBase<Map<const manif::SE2Tangent<_Scalar>, 0> >;

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

#endif /* _MANIF_MANIF_SE2TANGENT_MAP_H_ */
