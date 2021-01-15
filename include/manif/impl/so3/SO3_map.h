#ifndef _MANIF_MANIF_SO3_MAP_H_
#define _MANIF_MANIF_SO3_MAP_H_

#include "manif/impl/so3/SO3.h"

namespace manif {
namespace internal {

//! @brief traits specialization for Eigen Map
template <typename _Scalar>
struct traits< Eigen::Map<SO3<_Scalar>,0> >
    : public traits<SO3<_Scalar>>
{
  using typename traits<SO3<_Scalar>>::Scalar;
  using traits<SO3<Scalar>>::RepSize;
  using Base = SO3Base<Eigen::Map<SO3<Scalar>, 0>>;
  using DataType = Eigen::Map<Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

//! @brief traits specialization for Eigen Map const
template <typename _Scalar>
struct traits< Eigen::Map<const SO3<_Scalar>,0> >
    : public traits<const SO3<_Scalar>>
{
  using typename traits<const SO3<_Scalar>>::Scalar;
  using traits<const SO3<Scalar>>::RepSize;
  using Base = SO3Base<Eigen::Map<const SO3<Scalar>, 0>>;
  using DataType = Eigen::Map<const Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

} /* namespace internal */
} /* namespace manif */

namespace Eigen {

/**
 * @brief Specialization of Map for manif::SO3
 */
template <class _Scalar>
class Map<manif::SO3<_Scalar>, 0>
    : public manif::SO3Base<Map<manif::SO3<_Scalar>, 0> >
{
  using Base = manif::SO3Base<Map<manif::SO3<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_API
  using Base::transform;
  using Base::rotation;

  Map(Scalar* coeffs) : data_(coeffs) { }

  Map(Map&&) = default;

  MANIF_GROUP_MAP_ASSIGN_OP(SO3)

  DataType& coeffs() { return data_; }
  const DataType& coeffs() const { return data_; }

protected:

  DataType data_;
};

/**
 * @brief Specialization of Map for const manif::SO3
 */
template <class _Scalar>
class Map<const manif::SO3<_Scalar>, 0>
    : public manif::SO3Base<Map<const manif::SO3<_Scalar>, 0> >
{
  using Base = manif::SO3Base<Map<const manif::SO3<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_API
  using Base::transform;
  using Base::rotation;

  Map(const Scalar* coeffs) : data_(coeffs) { }

  Map(Map&&) = default;

  const DataType& coeffs() const { return data_; }

protected:

  const DataType data_;
};

} /* namespace Eigen */

#endif /* _MANIF_MANIF_SO3_MAP_H_ */
