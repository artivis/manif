#ifndef _MANIF_MANIF_SIM3_H_
#define _MANIF_MANIF_SIM3_H_

#include "manif/impl/sim3/Sim3_base.h"

namespace manif {

// Forward declare for type traits specialization

template <typename _Scalar> struct Sim3;
template <typename _Scalar> struct Sim3Tangent;

namespace internal {

//! Traits specialization
template <typename _Scalar>
struct traits<Sim3<_Scalar>>
{
  using Scalar = _Scalar;

  using LieGroup = Sim3<_Scalar>;
  using Tangent  = Sim3Tangent<_Scalar>;

  using Base = Sim3Base<Sim3<_Scalar>>;

  static constexpr int Dim = LieGroupProperties<Base>::Dim;
  static constexpr int DoF = LieGroupProperties<Base>::DoF;
  static constexpr int RepSize = 8;

  /// @todo would be nice to concat vec3 + quaternion
  using DataType = Eigen::Matrix<Scalar, RepSize, 1>;

  using Jacobian       = Eigen::Matrix<Scalar, DoF, DoF>;
  using Transformation = Eigen::Matrix<Scalar, 4, 4>;
  using Rotation       = Eigen::Matrix<Scalar, Dim, Dim>;
  using Translation    = Eigen::Matrix<Scalar, Dim, 1>;
  using Vector         = Eigen::Matrix<Scalar, Dim, 1>;
};

} // namespace internal
} // namespace manif

namespace manif {

//
// LieGroup
//

/**
 * @brief Represent an element of Sim3.
 */
template <typename _Scalar>
struct Sim3 : Sim3Base<Sim3<_Scalar>>
{
private:

  using Base = Sim3Base<Sim3<_Scalar>>;
  using Type = Sim3<_Scalar>;

protected:

  using Base::derived;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND

  MANIF_COMPLETE_GROUP_TYPEDEF
  using Translation = typename Base::Translation;
  using Quaternion = typename Base::QuaternionDataType;
  using Scale = typename Base::Scale;

  MANIF_INHERIT_GROUP_API
  using Base::transform;
  using Base::rotation;
  using Base::normalize;

  Sim3()  = default;
  ~Sim3() = default;

  MANIF_COPY_CONSTRUCTOR(Sim3)
  MANIF_MOVE_CONSTRUCTOR(Sim3)

  template <typename _DerivedOther>
  Sim3(const LieGroupBase<_DerivedOther>& o);

  MANIF_GROUP_ASSIGN_OP(Sim3)

  /**
   * @brief Constructor given a translation and a unit quaternion.
   * @param[in] t A translation vector.
   * @param[in] q A unit quaternion.
   * @param[in] scale The scale
   * @throws manif::invalid_argument on un-normalized complex number.
   */
  Sim3(
    const Translation& t, const Eigen::Quaternion<Scalar>& q, const Scale scale
  );

  /**
   * @brief Constructor given a translation and an angle axis.
   * @param[in] t A translation vector.
   * @param[in] angle_axis An angle-axis.
   * @param[in] scale The scale
   */
  Sim3(
    const Translation& t,
    const Eigen::AngleAxis<Scalar>& angle_axis,
    const Scale scale
  );

  /**
   * @brief Constructor given a translation and RxSO3 element.
   * @param[in] t A translation vector.
   * @param[in] RxSO3 An element of RxSO3.
   */
  Sim3(
    const Translation& t, const RxSO3<Scalar>& RxSO3
  );

  /**
   * @brief Constructor given translation components and
   * roll-pitch-yaw angles.
   * @param[in] x The x component of the translation.
   * @param[in] y The y component of the translation.
   * @param[in] z The z component of the translation.
   * @param[in] roll The roll angle.
   * @param[in] pitch The pitch angle.
   * @param[in] yaw The yaw angle.
   * @param[in] scale The scale
   */
  Sim3(
    const Scalar x, const Scalar y, const Scalar z,
    const Scalar roll, const Scalar pitch, const Scalar yaw,
    const Scale scale
  );

  /**
   * @brief Constructor from a 3D Eigen::Isometry<Scalar> and scale
   * @param[in] h an isometry object from Eigen
   * @param[in] scale The scale
   */
  Sim3(
    const Eigen::Transform<_Scalar,3,Eigen::Isometry>& h, const Scale scale
  );

  // LieGroup common API

  DataType& coeffs();
  const DataType& coeffs() const;

  // Sim3 specific API

protected:

  DataType data_;
};

MANIF_EXTRA_GROUP_TYPEDEF(Sim3)

template <typename _Scalar>
template <typename _DerivedOther>
Sim3<_Scalar>::Sim3(const LieGroupBase<_DerivedOther>& o)
  : Sim3(o.coeffs())
{
  //
}

template <typename _Scalar>
Sim3<_Scalar>::Sim3(
  const Translation& t, const Eigen::Quaternion<Scalar>& q, const Scale scale
) : Sim3((DataType() << t, q.coeffs(), scale ).finished()) {
  //
}

template <typename _Scalar>
Sim3<_Scalar>::Sim3(
  const Translation& t, const Eigen::AngleAxis<Scalar>& a, const Scale scale
) : Sim3(t, Quaternion(a), scale) {
  //
}

template <typename _Scalar>
Sim3<_Scalar>::Sim3(
  const Scalar x, const Scalar y, const Scalar z,
  const Scalar roll, const Scalar pitch, const Scalar yaw,
  const Scale scale
) : Sim3(
      Translation(x,y,z),
      Eigen::Quaternion<Scalar>(
        Eigen::AngleAxis<Scalar>(yaw,   Eigen::Matrix<Scalar, 3, 1>::UnitZ()) *
        Eigen::AngleAxis<Scalar>(pitch, Eigen::Matrix<Scalar, 3, 1>::UnitY()) *
        Eigen::AngleAxis<Scalar>(roll,  Eigen::Matrix<Scalar, 3, 1>::UnitX())
      ),
      scale
    ) {
  //
}

template <typename _Scalar>
Sim3<_Scalar>::Sim3(const Translation& t, const RxSO3<Scalar>& rxso3)
  : Sim3((DataType() << t, rxso3.coeffs() ).finished()) {
  //
}

template <typename _Scalar>
Sim3<_Scalar>::Sim3(
  const Eigen::Transform<_Scalar,3,Eigen::Isometry>& h, const Scale scale
) : Sim3(h.translation(), Eigen::Quaternion<_Scalar>(h.rotation()), scale) {
  //
}

template <typename _Scalar>
typename Sim3<_Scalar>::DataType&
Sim3<_Scalar>::coeffs()
{
  return data_;
}

template <typename _Scalar>
const typename Sim3<_Scalar>::DataType&
Sim3<_Scalar>::coeffs() const
{
  return data_;
}

} // namespace manif

#endif // _MANIF_MANIF_SIM3_H_
