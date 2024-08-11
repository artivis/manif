#ifndef _MANIF_MANIF_SGAL3_MAP_H_
#define _MANIF_MANIF_SGAL3_MAP_H_

#include "manif/impl/sgal3/SGal3.h"

namespace manif {
namespace internal {

//! @brief traits specialization for Eigen Map
template <typename _Scalar>
struct traits<Eigen::Map<SGal3<_Scalar>, 0> > : public traits<SGal3<_Scalar>> {
  using typename traits<SGal3<_Scalar>>::Scalar;
  using traits<SGal3<Scalar>>::RepSize;
  using Base = SGal3Base<Eigen::Map<SGal3<Scalar>, 0>>;
  using DataType = Eigen::Map<Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

//! @brief traits specialization for Eigen Map const
template <typename _Scalar>
struct traits<Eigen::Map<const SGal3<_Scalar>,0> >
  : public traits<const SGal3<_Scalar>> {
  using typename traits<const SGal3<_Scalar>>::Scalar;
  using traits<const SGal3<Scalar>>::RepSize;
  using Base = SGal3Base<Eigen::Map<const SGal3<Scalar>, 0>>;
  using DataType = Eigen::Map<const Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

} // namespace internal
} // namespace manif

namespace Eigen {

/**
 * @brief Specialization of Map for manif::SGal3
 */
template <class _Scalar>
class Map<manif::SGal3<_Scalar>, 0>
  : public manif::SGal3Base<Map<manif::SGal3<_Scalar>, 0> > {
  using Base = manif::SGal3Base<Map<manif::SGal3<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_API
  using Base::rotation;

  Map(Scalar* coeffs) : data_(coeffs) { }

  MANIF_GROUP_MAP_ASSIGN_OP(SGal3)

  DataType& coeffs() { return data_; }
  const DataType& coeffs() const { return data_; }

protected:

  DataType data_;
};

/**
 * @brief Specialization of Map for const manif::SGal3
 */
template <class _Scalar>
class Map<const manif::SGal3<_Scalar>, 0>
  : public manif::SGal3Base<Map<const manif::SGal3<_Scalar>, 0> > {
  using Base = manif::SGal3Base<Map<const manif::SGal3<_Scalar>, 0> >;

public:

  MANIF_COMPLETE_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_API
  // using Base::rotation;

  Map(const Scalar* coeffs) : data_(coeffs) { }

  const DataType& coeffs() const { return data_; }

protected:

  const DataType data_;
};

} // namespace Eigen

#endif // _MANIF_MANIF_SGAL3_MAP_H_
