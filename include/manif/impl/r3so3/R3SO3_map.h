#ifndef _MANIF_MANIF_R3SO3_MAP_H_
#define _MANIF_MANIF_R3SO3_MAP_H_

#include "manif/impl/R3SO3/R3SO3.h"

namespace manif
{
namespace internal
{

template <typename _Scalar>
struct traits< Eigen::Map<R3SO3<_Scalar>,0> >
    : public traits<R3SO3<_Scalar>>
{
  using typename traits<R3SO3<_Scalar>>::Scalar;
  using traits<R3SO3<_Scalar>>::RepSize;
  using DataType = ::Eigen::Map<Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

template <typename _Scalar>
struct traits< Eigen::Map<const R3SO3<_Scalar>,0> >
    : public traits<const R3SO3<_Scalar>>
{
  using typename traits<const R3SO3<_Scalar>>::Scalar;
  using traits<const R3SO3<_Scalar>>::RepSize;
  using DataType = ::Eigen::Map<const Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

} /* namespace internal */
} /* namespace manif */

namespace Eigen
{

template <class _Scalar>
class Map<manif::R3SO3<_Scalar>, 0>
    : public manif::R3SO3Base<Map<manif::R3SO3<_Scalar>, 0> >
{
  using Base = manif::R3SO3Base<Map<manif::R3SO3<_Scalar>, 0> >;
  using Type = Map<manif::R3SO3<_Scalar>, 0>;

public:

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

  MANIF_INHERIT_MANIFOLD_API

  Map(Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

protected:

  friend class manif::ManifoldBase<Map<manif::R3SO3<_Scalar>, 0>>;
  DataType& coeffs_nonconst() { return data_; }

  DataType data_;
};

template <class _Scalar>
class Map<const manif::R3SO3<_Scalar>, 0>
    : public manif::R3SO3Base<Map<const manif::R3SO3<_Scalar>, 0> >
{
  using Base = manif::R3SO3Base<Map<const manif::R3SO3<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

  MANIF_INHERIT_MANIFOLD_API

  Map(const Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

protected:

  const DataType data_;
};

} /* namespace Eigen */

#endif /* _MANIF_MANIF_R3SO3_MAP_H_ */
