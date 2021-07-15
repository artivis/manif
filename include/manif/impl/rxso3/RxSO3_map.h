#ifndef _MANIF_MANIF_RxSO3_MAP_H_
#define _MANIF_MANIF_RxSO3_MAP_H_

#include "manif/impl/rxso3/RxSO3.h"

namespace manif {
namespace internal {

//! @brief traits specialization for Eigen Map
template <typename _Scalar>
struct traits< Eigen::Map<RxSO3<_Scalar>,0> >
  : public traits<RxSO3<_Scalar>>
{
  using typename traits<RxSO3<_Scalar>>::Scalar;
  using traits<RxSO3<Scalar>>::RepSize;
  using Base = RxSO3Base<Eigen::Map<RxSO3<Scalar>, 0>>;
  using DataType = Eigen::Map<Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

//! @brief traits specialization for Eigen Map const
template <typename _Scalar>
struct traits< Eigen::Map<const RxSO3<_Scalar>,0> >
  : public traits<const RxSO3<_Scalar>>
{
  using typename traits<const RxSO3<_Scalar>>::Scalar;
  using traits<const RxSO3<Scalar>>::RepSize;
  using Base = RxSO3Base<Eigen::Map<const RxSO3<Scalar>, 0>>;
  using DataType = Eigen::Map<const Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

} // namespace internal
} // namespace manif

namespace Eigen {

/**
 * @brief Specialization of Map for manif::RxSO3
 */
template <class _Scalar>
class Map<manif::RxSO3<_Scalar>, 0>
  : public manif::RxSO3Base<Map<manif::RxSO3<_Scalar>, 0> >
{
  using Base = manif::RxSO3Base<Map<manif::RxSO3<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_API
  using Base::transform;
  using Base::rotation;

  Map(Scalar* coeffs) : data_(coeffs) { }

  Map(Map&&) = default;

  MANIF_GROUP_MAP_ASSIGN_OP(RxSO3)

  DataType& coeffs() { return data_; }
  const DataType& coeffs() const { return data_; }

protected:

  DataType data_;
};

/**
 * @brief Specialization of Map for const manif::RxSO3
 */
template <class _Scalar>
class Map<const manif::RxSO3<_Scalar>, 0>
  : public manif::RxSO3Base<Map<const manif::RxSO3<_Scalar>, 0> >
{
  using Base = manif::RxSO3Base<Map<const manif::RxSO3<_Scalar>, 0> >;

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

} // namespace Eigen

#endif // _MANIF_MANIF_RxSO3_MAP_H_
