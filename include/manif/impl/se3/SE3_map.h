#ifndef _MANIF_MANIF_SE3_MAP_H_
#define _MANIF_MANIF_SE3_MAP_H_

#include "manif/impl/se3/SE3.h"

namespace manif {
namespace internal {

//! @brief traits specialization for Eigen Map
template <typename _Scalar>
struct traits< Eigen::Map<SE3<_Scalar>,0> >
    : public traits<SE3<_Scalar>>
{
  using typename traits<SE3<_Scalar>>::Scalar;
  using traits<SE3<Scalar>>::RepSize;
  using Base = SE3Base<Eigen::Map<SE3<Scalar>, 0>>;
  using DataType = Eigen::Map<Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

//! @brief traits specialization for Eigen Map const
template <typename _Scalar>
struct traits< Eigen::Map<const SE3<_Scalar>,0> >
    : public traits<const SE3<_Scalar>>
{
  using typename traits<const SE3<_Scalar>>::Scalar;
  using traits<const SE3<Scalar>>::RepSize;
  using Base = SE3Base<Eigen::Map<const SE3<Scalar>, 0>>;
  using DataType = Eigen::Map<const Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

} /* namespace internal */
} /* namespace manif */

namespace Eigen {

/**
 * @brief Specialization of Map for manif::SE3
 */
template <class _Scalar>
class Map<manif::SE3<_Scalar>, 0>
    : public manif::SE3Base<Map<manif::SE3<_Scalar>, 0> >
{
  using Base = manif::SE3Base<Map<manif::SE3<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_API
  using Base::transform;
  using Base::rotation;

  Map(Scalar* coeffs) : data_(coeffs) { }

  MANIF_GROUP_MAP_ASSIGN_OP(SE3)

  DataType& coeffs() { return data_; }
  const DataType& coeffs() const { return data_; }

protected:

  DataType data_;
};

/**
 * @brief Specialization of Map for const manif::SE3
 */
template <class _Scalar>
class Map<const manif::SE3<_Scalar>, 0>
    : public manif::SE3Base<Map<const manif::SE3<_Scalar>, 0> >
{
  using Base = manif::SE3Base<Map<const manif::SE3<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_API
  using Base::transform;
  using Base::rotation;

  Map(const Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

protected:

  const DataType data_;
};

} /* namespace Eigen */

#endif /* _MANIF_MANIF_SE3_MAP_H_ */
