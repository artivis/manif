#ifndef _MANIF_MANIF_SO3_MAP_H_
#define _MANIF_MANIF_SO3_MAP_H_

#include "manif/impl/so3/SO3.h"

namespace manif
{
namespace internal
{

template <typename _Scalar>
struct traits< Eigen::Map<SO3<_Scalar>,0> >
    : public traits<SO3<_Scalar>>
{
  using typename traits<SO3<_Scalar>>::Scalar;
  using DataType = ::Eigen::Map<typename internal::traits<SO3<_Scalar>>::DataType, 0>;
};

template <typename _Scalar>
struct traits< Eigen::Map<const SO3<_Scalar>,0> >
    : public traits<const SO3<_Scalar>>
{
  using typename traits<const SO3<_Scalar>>::Scalar;
  using DataType = ::Eigen::Map<const typename internal::traits<SO3<_Scalar>>::DataType, 0>;
};

} /* namespace internal */
} /* namespace manif */

namespace Eigen
{

template <class _Scalar>
class Map<manif::SO3<_Scalar>, 0>
    : public manif::SO3Base<Map<manif::SO3<_Scalar>, 0> >
{
  using Base = manif::SO3Base<Map<manif::SO3<_Scalar>, 0> >;
  using Type = Map<manif::SO3<_Scalar>, 0>;

public:

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

  MANIF_INHERIT_MANIFOLD_API

  Map(Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

protected:

  friend class manif::ManifoldBase<Map<manif::SO3<_Scalar>, 0>>;
  DataType& coeffs_nonconst() { return data_; }

  DataType data_;
};

template <class _Scalar>
class Map<const manif::SO3<_Scalar>, 0>
    : public manif::SO3Base<Map<const manif::SO3<_Scalar>, 0> >
{
  using Base = manif::SO3Base<Map<const manif::SO3<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

  MANIF_INHERIT_MANIFOLD_API

  Map(const Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

protected:

  friend class manif::ManifoldBase<Map<const manif::SO3<_Scalar>, 0>>;
  DataType& coeffs_nonconst() { return data_; }

  DataType data_;
};

} /* namespace Eigen */

#endif /* _MANIF_MANIF_SO3_MAP_H_ */
