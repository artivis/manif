#ifndef _MANIF_MANIF_SE2_MAP_H_
#define _MANIF_MANIF_SE2_MAP_H_

#include "manif/impl/so2/SE2.h"

namespace manif {
namespace internal {

template <>
template <typename _Scalar>
struct traits< Eigen::Map<SE2<_Scalar>,0> >
    : public traits<SE2<_Scalar>>
{
  using typename traits<SE2<_Scalar>>::Scalar;
  using traits<SE2<_Scalar>>::RepSize;
  using DataType = ::Eigen::Map<Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

template <>
template <typename _Scalar>
struct traits< Eigen::Map<const SE2<_Scalar>,0> >
    : public traits<const SE2<_Scalar>>
{
  using typename traits<const SE2<_Scalar>>::Scalar;
  using traits<const SE2<_Scalar>>::RepSize;
  using DataType = ::Eigen::Map<const Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

} /* namespace internal */
} /* namespace manif */

namespace Eigen
{

template <class _Scalar>
class Map<manif::SE2<_Scalar>, 0>
    : public manif::SE2Base<Map<manif::SE2<_Scalar>, 0> >
{
  using Base = manif::SE2Base<Map<manif::SE2<_Scalar>, 0> >;
  using Type = Map<manif::SE2<_Scalar>, 0>;

public:

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

  MANIF_INHERIT_MANIFOLD_API

  Map(Scalar* coeffs) : data_(coeffs) { }

  DataType const* data() const { return &data_; }

protected:

  friend class manif::ManifoldBase<Map<manif::SE2<_Scalar>, 0>>;
  DataType* data() { return &data_; }

  DataType data_;
};

template <class _Scalar>
class Map<const manif::SE2<_Scalar>, 0>
    : public manif::SE2Base<Map<const manif::SE2<_Scalar>, 0> >
{
  using Base = manif::SE2Base<Map<const manif::SE2<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

  MANIF_INHERIT_MANIFOLD_API

  Map(Scalar* coeffs) : data_(coeffs) { }

  DataType const* data() const { return &data_; }

protected:

  friend class manif::ManifoldBase<Map<const manif::SE2<_Scalar>, 0>>;
  DataType* data() { return &data_; }

  DataType data_;
};

} /* namespace Eigen */

#endif /* _MANIF_MANIF_SE2_MAP_H_ */
