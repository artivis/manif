#ifndef _MANIF_MANIF_BUNDLE_MAP_H_
#define _MANIF_MANIF_BUNDLE_MAP_H_

#include "manif/impl/bundle/Bundle.h"

namespace manif {
namespace internal {

/**
 * @brief traits specialization for Eigen Map
 */
template<typename _Scalar, template<typename> class ... T>
struct traits<Eigen::Map<Bundle<_Scalar, T...>, 0>>
  : public traits<Bundle<_Scalar, T...>>
{
  using typename traits<Bundle<_Scalar, T...>>::Scalar;
  using traits<Bundle<Scalar, T...>>::RepSize;
  using Base = BundleBase<Eigen::Map<Bundle<Scalar, T...>, 0>>;
  using DataType = Eigen::Map<Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

/**
 * @brief traits specialization for Eigen const Map
 */
template<typename _Scalar, template<typename> class ... T>
struct traits<Eigen::Map<const Bundle<_Scalar, T...>, 0>>
  : public traits<const Bundle<_Scalar, T...>>
{
  using typename traits<const Bundle<_Scalar, T...>>::Scalar;
  using traits<const Bundle<Scalar, T...>>::RepSize;
  using Base = BundleBase<Eigen::Map<const Bundle<Scalar, T...>, 0>>;
  using DataType = Eigen::Map<const Eigen::Matrix<Scalar, RepSize, 1>, 0>;
};

}  // namespace internal
}  // namespace manif


namespace Eigen {

/**
 * @brief Specialization of Map for manif::Bundle
 */
template<class _Scalar, template<typename> class ... T>
class Map<manif::Bundle<_Scalar, T...>, 0>
  : public manif::BundleBase<Map<manif::Bundle<_Scalar, T...>, 0>>
{
  using Base = manif::BundleBase<Map<manif::Bundle<_Scalar, T...>, 0>>;

public:

  MANIF_COMPLETE_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_API

  using Base::BundleSize;

  Map(Scalar * coeffs) : data_(coeffs) { }

  MANIF_GROUP_MAP_ASSIGN_OP(Bundle)

  DataType & coeffs() {return data_;}

  const DataType & coeffs() const {return data_;}

protected:

  DataType data_;
};

/**
 * @brief Specialization of Map for const manif::Bundle
 */
template<class _Scalar, template<typename> class ... T>
class Map<const manif::Bundle<_Scalar, T...>, 0>
  : public manif::BundleBase<Map<const manif::Bundle<_Scalar, T...>, 0>>
{
  using Base = manif::BundleBase<Map<const manif::Bundle<_Scalar, T...>, 0>>;

public:

  MANIF_COMPLETE_GROUP_TYPEDEF
  MANIF_INHERIT_GROUP_API

  using Base::BundleSize;

  Map(const Scalar * coeffs) : data_(coeffs) { }

  const DataType & coeffs() const {return data_;}

protected:

  const DataType data_;
};

}  // namespace Eigen

#endif  // _MANIF_MANIF_BUNDLE_MAP_H_
