#ifndef _MANIF_MANIF_SIM3TANGENT_MAP_H_
#define _MANIF_MANIF_SIM3TANGENT_MAP_H_

#include "manif/impl/sim3/Sim3Tangent.h"

namespace manif {
namespace internal {

//! @brief traits specialization for Eigen Map
template <typename _Scalar>
struct traits< Eigen::Map<Sim3Tangent<_Scalar>,0> >
  : public traits<Sim3Tangent<_Scalar>>
{
  using typename traits<Sim3Tangent<_Scalar>>::Scalar;
  using traits<Sim3Tangent<_Scalar>>::DoF;
  using DataType = Eigen::Map<Eigen::Matrix<Scalar, DoF, 1>, 0>;
  using Base = Sim3TangentBase<Eigen::Map<Sim3Tangent<Scalar>, 0>>;
};

//! @brief traits specialization for Eigen Map
template <typename _Scalar>
struct traits< Eigen::Map<const Sim3Tangent<_Scalar>,0> >
  : public traits<const Sim3Tangent<_Scalar>>
{
  using typename traits<const Sim3Tangent<_Scalar>>::Scalar;
  using traits<const Sim3Tangent<_Scalar>>::DoF;
  using DataType = Eigen::Map<const Eigen::Matrix<Scalar, DoF, 1>, 0>;
  using Base = Sim3TangentBase<Eigen::Map<const Sim3Tangent<Scalar>, 0>>;
};

} // namespace internal
} // namespace manif

namespace Eigen {

/**
 * @brief Specialization of Map for manif::Sim3
 */
template <class _Scalar>
class Map<manif::Sim3Tangent<_Scalar>, 0>
  : public manif::Sim3TangentBase<Map<manif::Sim3Tangent<_Scalar>, 0> >
{
  using Base = manif::Sim3TangentBase<Map<manif::Sim3Tangent<_Scalar>, 0> >;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  Map(Scalar* coeffs) : data_(coeffs) { }

  MANIF_TANGENT_MAP_ASSIGN_OP(Sim3Tangent)

  DataType& coeffs() { return data_; }
  const DataType& coeffs() const { return data_; }

protected:

  DataType data_;
};

/**
 * @brief Specialization of Map for const manif::Sim3
 */
template <class _Scalar>
class Map<const manif::Sim3Tangent<_Scalar>, 0>
  : public manif::Sim3TangentBase<Map<const manif::Sim3Tangent<_Scalar>, 0> >
{
  using Base = manif::Sim3TangentBase<Map<const manif::Sim3Tangent<_Scalar>, 0> >;

public:

  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  Map(const Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

protected:

  const DataType data_;
};

} // namespace Eigen

#endif // _MANIF_MANIF_SIM3TANGENT_MAP_H_
