#ifndef _MANIF_MANIF_SIM2_MAP_H_
#define _MANIF_MANIF_SIM2_MAP_H_

#include "manif/impl/sim2/Sim2.h"

namespace manif {
namespace internal {

//! @brief traits specialization for Eigen Map
template <typename _Scalar>
struct traits< Eigen::Map<Sim2<_Scalar>,0> >
  : public traits<Sim2<_Scalar>>
{
  using typename traits<Sim2<_Scalar>>::Scalar;
  using traits<Sim2<Scalar>>::RepSize;
  using Base = Sim2Base<Eigen::Map<Sim2<Scalar>, 0>>;
  using DataType = Eigen::Map<Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

//! @brief traits specialization for Eigen Map const
template <typename _Scalar>
struct traits< Eigen::Map<const Sim2<_Scalar>,0> >
  : public traits<const Sim2<_Scalar>>
{
  using typename traits<const Sim2<_Scalar>>::Scalar;
  using traits<const Sim2<Scalar>>::RepSize;
  using Base = Sim2Base<Eigen::Map<const Sim2<Scalar>, 0>>;
  using DataType = Eigen::Map<const Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

} // namespace internal
} // namespace manif

namespace Eigen {

/**
 * @brief Specialization of Map for manif::Sim2
 */
template <class _Scalar>
class Map<manif::Sim2<_Scalar>, 0>
  : public manif::Sim2Base<Map<manif::Sim2<_Scalar>, 0> >
{
  using Base = manif::Sim2Base<Map<manif::Sim2<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_API
  using Base::transform;
  using Base::rotation;

  Map(Scalar* coeffs) : data_(coeffs) { }

  MANIF_GROUP_MAP_ASSIGN_OP(Sim2)

  DataType& coeffs() { return data_; }
  const DataType& coeffs() const { return data_; }

protected:

  DataType data_;
};

/**
 * @brief Specialization of Map for const manif::Sim2
 */
template <class _Scalar>
class Map<const manif::Sim2<_Scalar>, 0>
  : public manif::Sim2Base<Map<const manif::Sim2<_Scalar>, 0> >
{
  using Base = manif::Sim2Base<Map<const manif::Sim2<_Scalar>, 0> >;

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

#endif // _MANIF_MANIF_SIM2_MAP_H_
