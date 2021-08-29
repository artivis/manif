#ifndef _MANIF_MANIF_SIM2_H_
#define _MANIF_MANIF_SIM2_H_

#include "manif/impl/sim2/Sim2_base.h"

namespace manif {

// Forward declare for type traits specialization

template <typename _Scalar> struct Sim2;
template <typename _Scalar> struct Sim2Tangent;

namespace internal {

//! Traits specialization
template <typename _Scalar>
struct traits<Sim2<_Scalar>>
{
  using Scalar = _Scalar;

  using LieGroup = Sim2<_Scalar>;
  using Tangent  = Sim2Tangent<_Scalar>;

  using Base = Sim2Base<Sim2<_Scalar>>;

  static constexpr int Dim = LieGroupProperties<Base>::Dim;
  static constexpr int DoF = LieGroupProperties<Base>::DoF;
  static constexpr int RepSize = 5;

  using DataType = Eigen::Matrix<Scalar, RepSize, 1>;

  using Jacobian       = Eigen::Matrix<Scalar, DoF, DoF>;
  using Transformation = Eigen::Matrix<Scalar, 3, 3>;
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
 * @brief Represent an element of Sim2.
 */
template <typename _Scalar>
struct Sim2 : Sim2Base<Sim2<_Scalar>>
{
private:

  using Base = Sim2Base<Sim2<_Scalar>>;
  using Type = Sim2<_Scalar>;

protected:

  using Base::derived;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND

  MANIF_COMPLETE_GROUP_TYPEDEF
  using Translation = typename Base::Translation;
  using Scale = typename Base::Scale;

  MANIF_INHERIT_GROUP_API
  using Base::transform;
  using Base::rotation;
  using Base::normalize;

  Sim2()  = default;
  ~Sim2() = default;

  MANIF_COPY_CONSTRUCTOR(Sim2)
  MANIF_MOVE_CONSTRUCTOR(Sim2)

  template <typename _DerivedOther>
  Sim2(const LieGroupBase<_DerivedOther>& o);

  MANIF_GROUP_ASSIGN_OP(Sim2)

  /**
   * @brief Constructor given a translation, a complex number and a scalar.
   * @param[in] t A translation vector.
   * @param[in] q A complex number reprensenting the rotation.
   * @param[in] scale The scale
   * @throws manif::invalid_argument on un-normalized complex number.
   */
  Sim2(
    const Translation& t, const std::complex<Scalar>& c, const Scale scale
  );

  /**
   * @brief Constructor given a translation and an angle axis.
   * @param[in] x The x-components of the translational part.
   * @param[in] y The y-components of the translational part.
   * @param[in] theta The rotation angle.
   * @param[in] scale The scale.
   */
  Sim2(
    const Scalar x, const Scalar y, const Scalar theta, const Scalar scale
  );

  /**
   * @brief Constructor given a translation and an angle axis.
   * @param[in] x The x-components of the translational part.
   * @param[in] y The y-components of the translational part.
   * @param[in] real The real of a unitary complex number.
   * @param[in] imag The imaginary of a unitary complex number.
   * @param[in] scale The scale.
   */
  Sim2(
    const Scalar x, const Scalar y,
    const Scalar real, const Scalar imag,
    const Scalar scale
  );

  /**
   * @brief Constructor given a translation and RxSO2 element.
   * @param[in] t A translation vector.
   * @param[in] RxSO2 An element of RxSO2.
   */
  Sim2(
    const Translation& t, const RxSO2<Scalar>& RxSO2
  );

  /**
   * @brief Constructor from a 2D Eigen::Isometry<Scalar> and scale
   * @param[in] h an isometry object from Eigen
   * @param[in] scale The scale
   */
  Sim2(
    const Eigen::Transform<_Scalar,2,Eigen::Isometry>& h, const Scale scale
  );

  // LieGroup common API

  DataType& coeffs();
  const DataType& coeffs() const;

  // Sim2 specific API

protected:

  DataType data_;
};

MANIF_EXTRA_GROUP_TYPEDEF(Sim2)

template <typename _Scalar>
template <typename _DerivedOther>
Sim2<_Scalar>::Sim2(const LieGroupBase<_DerivedOther>& o)
  : Sim2(o.coeffs())
{
  //
}

template <typename _Scalar>
Sim2<_Scalar>::Sim2(
  const Translation& t, const std::complex<Scalar>& c, const Scale scale
) : Sim2((DataType() << t, c.real(), c.imag(), scale).finished()) {
  //
}

template <typename _Scalar>
Sim2<_Scalar>::Sim2(
  const Scalar x, const Scalar y, const Scalar theta, const Scalar scale
) : Sim2(x, y, cos(theta), sin(theta), scale) {
  //
}

template <typename _Scalar>
Sim2<_Scalar>::Sim2(
  const Scalar x, const Scalar y,
  const Scalar real, const Scalar imag,
  const Scalar scale
) : Sim2((DataType() << x, y, real, imag, scale).finished()) {
  //
}

template <typename _Scalar>
Sim2<_Scalar>::Sim2(const Translation& t, const RxSO2<Scalar>& RxSO2)
  : Sim2((DataType() << t, RxSO2.coeffs() ).finished()) {
  //
}

template <typename _Scalar>
Sim2<_Scalar>::Sim2(
  const Eigen::Transform<_Scalar,2,Eigen::Isometry>& h, const Scale scale
) : Sim2(h.translation(), h.angle(), scale) {
  //
}

template <typename _Scalar>
typename Sim2<_Scalar>::DataType&
Sim2<_Scalar>::coeffs()
{
  return data_;
}

template <typename _Scalar>
const typename Sim2<_Scalar>::DataType&
Sim2<_Scalar>::coeffs() const
{
  return data_;
}

} // namespace manif

#endif // _MANIF_MANIF_SIM2_H_
