#ifndef _MANIF_MANIF_SO2TANGENT_MAP_H_
#define _MANIF_MANIF_SO2TANGENT_MAP_H_

#include "manif/impl/so2/SO2Tangent.h"

namespace manif
{
namespace internal
{

template <typename _Scalar/*, int _MapOptions, typename _StrideType*/>
struct traits< Eigen::Map<SO2Tangent<_Scalar>,0> >
    : public traits<SO2Tangent<_Scalar>>
{
  using typename traits<SO2Tangent<_Scalar>>::Scalar;
  using traits<SO2Tangent<_Scalar>>::DoF;
  using DataType = ::Eigen::Map<Eigen::Matrix<Scalar, DoF, 1>, 0>;
};

template <typename _Scalar>
struct traits< Eigen::Map<const SO2Tangent<_Scalar>,0> >
    : public traits<const SO2Tangent<_Scalar>>
{
  using typename traits<const SO2Tangent<_Scalar>>::Scalar;
  using traits<const SO2Tangent<_Scalar>>::DoF;
  using DataType = ::Eigen::Map<const Eigen::Matrix<Scalar, DoF, 1>, 0>;
};

} /* namespace internal */
} /* namespace manif */

namespace Eigen
{

template <class _Scalar>
class Map<manif::SO2Tangent<_Scalar>, 0>
    : public manif::SO2TangentBase<Map<manif::SO2Tangent<_Scalar>, 0> >
{
  using Base = manif::SO2TangentBase<Map<manif::SO2Tangent<_Scalar>, 0> >;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  Map(Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

protected:

  friend class manif::TangentBase<Map<manif::SO2Tangent<_Scalar>, 0>>;
  DataType& coeffs_nonconst() { return data_; }

  DataType data_;
};

template <class _Scalar>
class Map<const manif::SO2Tangent<_Scalar>, 0>
    : public manif::SO2TangentBase<Map<const manif::SO2Tangent<_Scalar>, 0> >
{
  using Base = manif::SO2TangentBase<Map<const manif::SO2Tangent<_Scalar>, 0> >;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  Map(const Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

protected:

  DataType data_;
};

} /* namespace Eigen */

#endif /* _MANIF_MANIF_SO2TANGENT_MAP_H_ */
