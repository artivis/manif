#ifndef _MANIF_MANIF_BUNDLETANGENT_MAP_H_
#define _MANIF_MANIF_BUNDLETANGENT_MAP_H_

#include "manif/impl/bundle/BundleTangent.h"

namespace manif
{
namespace internal
{

/**
 * @brief traits specialization for Eigen Map
 */
template<typename _Scalar, template<typename> class ... T>
struct traits<Eigen::Map<BundleTangent<_Scalar, T...>, 0>>
  : public traits<BundleTangent<_Scalar, T...>>
{
  using typename traits<BundleTangent<_Scalar, T...>>::Scalar;
  using traits<BundleTangent<Scalar, T...>>::DoF;
  using Base = BundleTangentBase<Eigen::Map<BundleTangent<Scalar, T...>, 0>>;
  using DataType = Eigen::Map<Eigen::Matrix<Scalar, DoF, 1>, 0>;
};

/**
 * @brief traits specialization for Eigen const Map
 */
template<typename _Scalar, template<typename> class ... T>
struct traits<Eigen::Map<const BundleTangent<_Scalar, T...>, 0>>
  : public traits<const BundleTangent<_Scalar, T...>>
{
  using typename traits<const BundleTangent<_Scalar, T...>>::Scalar;
  using traits<const BundleTangent<Scalar, T...>>::DoF;
  using Base = BundleTangentBase<Eigen::Map<const BundleTangent<Scalar, T...>, 0>>;
  using DataType = Eigen::Map<const Eigen::Matrix<Scalar, DoF, 1>, 0>;
};

}  // namespace internal
}  // namespace manif


namespace Eigen
{

/**
 * @brief Specialization of Map for manif::Bundle
 */
template<class _Scalar, template<typename> class ... T>
class Map<manif::BundleTangent<_Scalar, T...>, 0>
  : public manif::BundleTangentBase<Map<manif::BundleTangent<_Scalar, T...>, 0>>
{
  using Base = manif::BundleTangentBase<Map<manif::BundleTangent<_Scalar, T...>, 0>>;

public:
  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  using Base::BundleSize;

  Map(Scalar * coeffs) : data_(coeffs) { }

  MANIF_TANGENT_MAP_ASSIGN_OP(BundleTangent)

  DataType & coeffs() {return data_;}

  const DataType & coeffs() const {return data_;}

protected:
  DataType data_;
};

/**
 * @brief Specialization of Map for const manif::BundleTangent
 */
template<class _Scalar, template<typename> class ... T>
class Map<const manif::BundleTangent<_Scalar, T...>, 0>
  : public manif::BundleTangentBase<Map<const manif::BundleTangent<_Scalar, T...>, 0>>
{
  using Base = manif::BundleTangentBase<Map<const manif::BundleTangent<_Scalar, T...>, 0>>;

public:
  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  using Base::BundleSize;

  Map(const Scalar * coeffs) : data_(coeffs) { }

  const DataType & coeffs() const {return data_;}

protected:
  const DataType data_;
};

}  // namespace Eigen

#endif  // _MANIF_MANIF_BUNDLETANGENT_MAP_H_
