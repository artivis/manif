#ifndef _MANIF_MANIF_SE3_MAP_H_
#define _MANIF_MANIF_SE3_MAP_H_

#include "manif/impl/se3/SE3.h"

namespace manif
{
namespace internal
{

template <typename _Scalar>
struct traits< Eigen::Map<SE3<_Scalar>,0> >
    : public traits<SE3<_Scalar>>
{
  using typename traits<SE3<_Scalar>>::Scalar;
  using traits<SE3<_Scalar>>::DoF;
  using DataType = ::Eigen::Map<Eigen::Matrix<Scalar, 7, 1>, 0>;
};

template <typename _Scalar>
struct traits< Eigen::Map<const SE3<_Scalar>,0> >
    : public traits<const SE3<_Scalar>>
{
  using typename traits<const SE3<_Scalar>>::Scalar;
  using traits<const SE3<_Scalar>>::DoF;
  using DataType = ::Eigen::Map<const Eigen::Matrix<Scalar, 7, 1>, 0>;
};

} /* namespace internal */
} /* namespace manif */

namespace Eigen
{

template <class _Scalar>
class Map<manif::SE3<_Scalar>, 0>
    : public manif::SE3Base<Map<manif::SE3<_Scalar>, 0> >
{
  using Base = manif::SE3Base<Map<manif::SE3<_Scalar>, 0> >;
  using Type = Map<manif::SE3<_Scalar>, 0>;

public:

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

  MANIF_INHERIT_MANIFOLD_API

  Map(Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

protected:

  friend class manif::ManifoldBase<Map<manif::SE3<_Scalar>, 0>>;
  DataType& coeffs_nonconst() { return data_; }

  DataType data_;
};

template <class _Scalar>
class Map<const manif::SE3<_Scalar>, 0>
    : public manif::SE3Base<Map<const manif::SE3<_Scalar>, 0> >
{
  using Base = manif::SE3Base<Map<const manif::SE3<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

  MANIF_INHERIT_MANIFOLD_API

  Map(Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

protected:

  friend class manif::ManifoldBase<Map<const manif::SE3<_Scalar>, 0>>;
  DataType& coeffs_nonconst() { return data_; }

  DataType data_;
};

} /* namespace Eigen */

#endif /* _MANIF_MANIF_SE3_MAP_H_ */
