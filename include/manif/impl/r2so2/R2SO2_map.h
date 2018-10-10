#ifndef _MANIF_MANIF_R2SO2_MAP_H_
#define _MANIF_MANIF_R2SO2_MAP_H_

#include "manif/impl/R2SO2/R2SO2.h"

namespace manif
{
namespace internal
{

template <typename _Scalar>
struct traits< Eigen::Map<R2SO2<_Scalar>,0> >
    : public traits<R2SO2<_Scalar>>
{
  using typename traits<R2SO2<_Scalar>>::Scalar;
  using traits<R2SO2<_Scalar>>::RepSize;
  using DataType = ::Eigen::Map<Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

template <typename _Scalar>
struct traits< Eigen::Map<const R2SO2<_Scalar>,0> >
    : public traits<const R2SO2<_Scalar>>
{
  using typename traits<const R2SO2<_Scalar>>::Scalar;
  using traits<const R2SO2<_Scalar>>::RepSize;
  using DataType = ::Eigen::Map<const Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

} /* namespace internal */
} /* namespace manif */

namespace Eigen
{

template <class _Scalar>
class Map<manif::R2SO2<_Scalar>, 0>
    : public manif::R2SO2Base<Map<manif::R2SO2<_Scalar>, 0> >
{
  using Base = manif::R2SO2Base<Map<manif::R2SO2<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_MANIFOLD_TYPEDEF
  MANIF_INHERIT_MANIFOLD_API

  Map(Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

  using Base::angle;
  using Base::real;
  using Base::imag;
  using Base::x;
  using Base::y;

protected:

  friend class manif::ManifoldBase<Map<manif::R2SO2<_Scalar>, 0>>;
  DataType& coeffs_nonconst() { return data_; }

  DataType data_;
};

template <class _Scalar>
class Map<const manif::R2SO2<_Scalar>, 0>
    : public manif::R2SO2Base<Map<const manif::R2SO2<_Scalar>, 0> >
{
  using Base = manif::R2SO2Base<Map<const manif::R2SO2<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_MANIFOLD_TYPEDEF
  MANIF_INHERIT_MANIFOLD_API

  Map(const Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

  using Base::angle;
  using Base::real;
  using Base::imag;
  using Base::x;
  using Base::y;

protected:

  const DataType data_;
};

} /* namespace Eigen */

#endif /* _MANIF_MANIF_R2SO2_MAP_H_ */
