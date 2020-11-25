#ifndef _MANIF_MANIF_SE2_H_
#define _MANIF_MANIF_SE2_H_

#include "manif/impl/se2/SE2_base.h"

namespace manif {

// Forward declare for type traits specialization

template <typename _Scalar> struct SE2;
template <typename _Scalar> struct SE2Tangent;

namespace internal {

//! traits specialization
template <typename _Scalar>
struct traits<SE2<_Scalar>>
{
  using Scalar = _Scalar;

  using LieGroup = SE2<_Scalar>;
  using Tangent  = SE2Tangent<_Scalar>;

  using Base = SE2Base<SE2<_Scalar>>;

  static constexpr int Dim     = LieGroupProperties<Base>::Dim;
  static constexpr int DoF     = LieGroupProperties<Base>::DoF;
  static constexpr int RepSize = 4;

  using DataType = Eigen::Matrix<Scalar, RepSize, 1>;

  using Jacobian       = Eigen::Matrix<Scalar, DoF, DoF>;
  using Transformation = Eigen::Matrix<Scalar, 3, 3>;
  using Rotation       = Eigen::Matrix<Scalar, Dim, Dim>;
  using Translation    = Eigen::Matrix<Scalar, Dim, 1>;
  using Vector         = Eigen::Matrix<Scalar, Dim, 1>;
};

} /* namespace internal */
} /* namespace manif */

namespace manif {

//
// LieGroup
//

/**
 * @brief Represents an element of SE2.
 */
template <typename _Scalar>
struct SE2 : SE2Base<SE2<_Scalar>>
{
private:

  using Base = SE2Base<SE2<_Scalar>>;
  using Type = SE2<_Scalar>;

protected:

  using Base::derived;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND

  MANIF_COMPLETE_GROUP_TYPEDEF
  using Translation = typename Base::Translation;
  MANIF_INHERIT_GROUP_API
  using Base::transform;
  using Base::rotation;
  using Base::normalize;

  SE2()  = default;
  ~SE2() = default;

  MANIF_COPY_CONSTRUCTOR(SE2)
  MANIF_MOVE_CONSTRUCTOR(SE2)

  // Copy constructor
  template <typename _DerivedOther>
  SE2(const LieGroupBase<_DerivedOther>& o);

  MANIF_GROUP_ASSIGN_OP(SE2)

  /**
   * @brief Constructor given a translation and a unit complex number.
   * @param[in] t A translation vector.
   * @param[in] c A complex number.
   * @throws manif::invalid_argument on un-normalized complex number.
   */
  SE2(const Translation& t, const std::complex<Scalar>& c);

  /**
   * @brief Constructor given the x and y components of the translational part
   * and an angle.
   * @param[in] x The x-components of the translational part.
   * @param[in] y The y-components of the translational part.
   * @param[in] c An angle.
   */
  SE2(const Scalar x, const Scalar y, const Scalar theta);

  /**
   * @brief Constructor given the x and y components of the translational part
   * and the real and imaginary part of a unit complex number.
   * @param[in] x The x-components of the translational part.
   * @param[in] y The y-components of the translational part.
   * @param[in] real The real of a unitary complex number.
   * @param[in] imag The imaginary of a unitary complex number.
   * @throws manif::invalid_argument on un-normalized complex number.
   */
  SE2(const Scalar x, const Scalar y, const Scalar real, const Scalar imag);

  /**
   * @brief Constructor given the x and y components of the translational part
   * and the real and imaginary part of a unit complex number.
   * @param[in] x The x-components of the translational part.
   * @param[in] y The y-components of the translational part.
   * @param[in] c The unitary complex number.
   * @throws manif::invalid_argument on un-normalized complex number.
   */
  SE2(const Scalar x, const Scalar y, const std::complex<Scalar>& c);

  /**
   * @brief Constructor from a 2D Eigen::Isometry<Scalar>
   * @param[in] h an isometry object from Eigen
   *
   * Isometry is a typedef from Eigen::Transform,
   * in which the linear part is assumed a rotation matrix.
   * This is used to speed up certain methods of Transform, especially inverse().
   */
  SE2(const Eigen::Transform<_Scalar,2,Eigen::Isometry>& h);

  // LieGroup common API

  /**
   * @brief Access the underlying data
   * @param[out] a reference to the underlying Eigen vector
   */
  DataType& coeffs();

  /**
   * @brief Access the underlying data
   * @param[out] a const reference to the underlying Eigen vector
   */
  const DataType& coeffs() const;

  // SE2 specific API

  using Base::angle;
  using Base::real;
  using Base::imag;
  using Base::x;
  using Base::y;

protected:

  //! Underlying data (Eigen) vector
  DataType data_;
};

MANIF_EXTRA_GROUP_TYPEDEF(SE2)

template <typename _Scalar>
template <typename _DerivedOther>
SE2<_Scalar>::SE2(const LieGroupBase<_DerivedOther>& o)
  : SE2(o.coeffs())
{
  //
}

template <typename _Scalar>
SE2<_Scalar>::SE2(const Translation& t,
                  const std::complex<Scalar>& c)
  : SE2((DataType() << t, c.real(), c.imag()).finished())
{
  //
}

template <typename _Scalar>
SE2<_Scalar>::SE2(const Scalar x, const Scalar y, const Scalar theta)
  : SE2(DataType(x, y, cos(theta), sin(theta)))
{
  using std::cos;
  using std::sin;
}

template <typename _Scalar>
SE2<_Scalar>::SE2(const Scalar x, const Scalar y,
                  const Scalar real, const Scalar imag)
  : SE2(DataType(x, y, real, imag))
{
  //
}

template <typename _Scalar>
SE2<_Scalar>::SE2(const Scalar x, const Scalar y, const std::complex<Scalar>& c)
  : SE2(x, y, c.real(), c.imag())
{
  //
}

template <typename _Scalar>
SE2<_Scalar>::SE2(const Eigen::Transform<_Scalar, 2, Eigen::Isometry>& h)
  : SE2(h.translation().x(), h.translation().y(), Eigen::Rotation2D<Scalar>(h.rotation()).angle())
{
  //
}

template <typename _Scalar>
typename SE2<_Scalar>::DataType&
SE2<_Scalar>::coeffs()
{
  return data_;
}

template <typename _Scalar>
const typename SE2<_Scalar>::DataType&
SE2<_Scalar>::coeffs() const
{
  return data_;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SE2_H_ */
