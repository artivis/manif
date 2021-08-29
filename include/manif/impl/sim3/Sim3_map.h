#ifndef _MANIF_MANIF_SIM3_MAP_H_
#define _MANIF_MANIF_SIM3_MAP_H_

#include "manif/impl/sim3/Sim3.h"

namespace manif {
namespace internal {

//! @brief traits specialization for Eigen Map
template <typename _Scalar>
struct traits< Eigen::Map<Sim3<_Scalar>,0> >
  : public traits<Sim3<_Scalar>>
{
  using typename traits<Sim3<_Scalar>>::Scalar;
  using traits<Sim3<Scalar>>::RepSize;
  using Base = Sim3Base<Eigen::Map<Sim3<Scalar>, 0>>;
  using DataType = Eigen::Map<Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

//! @brief traits specialization for Eigen Map const
template <typename _Scalar>
struct traits< Eigen::Map<const Sim3<_Scalar>,0> >
  : public traits<const Sim3<_Scalar>>
{
  using typename traits<const Sim3<_Scalar>>::Scalar;
  using traits<const Sim3<Scalar>>::RepSize;
  using Base = Sim3Base<Eigen::Map<const Sim3<Scalar>, 0>>;
  using DataType = Eigen::Map<const Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

} // namespace internal
} // namespace manif

namespace Eigen {

/**
 * @brief Specialization of Map for manif::Sim3
 */
template <class _Scalar>
class Map<manif::Sim3<_Scalar>, 0>
  : public manif::Sim3Base<Map<manif::Sim3<_Scalar>, 0> >
{
  using Base = manif::Sim3Base<Map<manif::Sim3<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_API
  using Base::transform;
  using Base::rotation;

  Map(Scalar* coeffs) : data_(coeffs) { }

  MANIF_GROUP_MAP_ASSIGN_OP(Sim3)

  DataType& coeffs() { return data_; }
  const DataType& coeffs() const { return data_; }

protected:

  DataType data_;
};

/**
 * @brief Specialization of Map for const manif::Sim3
 */
template <class _Scalar>
class Map<const manif::Sim3<_Scalar>, 0>
  : public manif::Sim3Base<Map<const manif::Sim3<_Scalar>, 0> >
{
  using Base = manif::Sim3Base<Map<const manif::Sim3<_Scalar>, 0> >;

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

} // namespace Eigen

#endif // _MANIF_MANIF_SIM3_MAP_H_
