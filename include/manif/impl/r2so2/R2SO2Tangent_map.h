#ifndef _MANIF_MANIF_R2SO2TANGENT_MAP_H_
#define _MANIF_MANIF_R2SO2TANGENT_MAP_H_

#include "manif/impl/R2SO2/R2SO2Tangent.h"

namespace manif
{
namespace internal
{

template <typename _Scalar>
struct traits< Eigen::Map<R2SO2Tangent<_Scalar>,0> >
    : public traits<R2SO2Tangent<_Scalar>>
{
  using typename traits<R2SO2Tangent<_Scalar>>::Scalar;
  using traits<R2SO2Tangent<_Scalar>>::DoF;
  using DataType = ::Eigen::Map<Eigen::Matrix<Scalar, DoF, 1>, 0>;
};

template <typename _Scalar>
struct traits< Eigen::Map<const R2SO2Tangent<_Scalar>,0> >
    : public traits<const R2SO2Tangent<_Scalar>>
{
  using typename traits<const R2SO2Tangent<_Scalar>>::Scalar;
  using traits<R2SO2Tangent<_Scalar>>::DoF;
  using DataType = ::Eigen::Map<const Eigen::Matrix<Scalar, DoF, 1>, 0>;
};

} /* namespace internal */
} /* namespace manif */

namespace Eigen
{

template <class _Scalar>
class Map<manif::R2SO2Tangent<_Scalar>, 0>
    : public manif::R2SO2TangentBase<Map<manif::R2SO2Tangent<_Scalar>, 0> >
{
  using Base = manif::R2SO2TangentBase<Map<manif::R2SO2Tangent<_Scalar>, 0> >;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  Map(Scalar* coeffs) : data_(coeffs) { }

  DataType& coeffs() { return data_; }
  const DataType& coeffs() const { return data_; }

protected:

  DataType data_;
};

template <class _Scalar>
class Map<const manif::R2SO2Tangent<_Scalar>, 0>
    : public manif::R2SO2TangentBase<Map<const manif::R2SO2Tangent<_Scalar>, 0> >
{
  using Base = manif::R2SO2TangentBase<Map<const manif::R2SO2Tangent<_Scalar>, 0> >;

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

#endif /* _MANIF_MANIF_R2SO2TANGENT_MAP_H_ */
