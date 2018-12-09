#ifndef _MANIF_MANIF_SO2_BASE_H_
#define _MANIF_MANIF_SO2_BASE_H_

#include "manif/impl/so2/SO2_properties.h"
#include "manif/impl/lie_group_base.h"

namespace manif {

////////////////
///          ///
/// LieGroup ///
///          ///
////////////////

/**
 * @brief The base class of the SO2 group.
 * @note See Appendix A of the paper.
 */
template <typename _Derived>
struct SO2Base : LieGroupBase<_Derived>
{
private:

  using Base = LieGroupBase<_Derived>;
  using Type = SO2Base<_Derived>;

public:

  using Scalar   = typename Base::Scalar;

  using LieGroup = typename Base::LieGroup;
  using Tangent  = typename Base::Tangent;
  using Jacobian = typename Base::Jacobian;

  using OptJacobianRef = typename Base::OptJacobianRef;

  using DataType = typename Base::DataType;

  using Transformation = typename Base::Transformation;
  using Rotation       = typename Base::Rotation;
  using Vector         = typename Base::Vector;

  using Base::coeffs;
  using Base::setRandom;
  using Base::rplus;
  using Base::lplus;
  using Base::rminus;
  using Base::lminus;
  using Base::operator=;

  // LieGroup common API

  /**
   * @brief Get the inverse of this.
   * @param[out] -optional- J_minv_m Jacobian of the inverse wrt this.
   * @note z^-1 = z*
   * @note See Eq. (104).
   */
  LieGroup inverse(OptJacobianRef J_minv_m = {}) const;

  /**
   * @brief Get the SO2 tangent at the point represented by this.
   * @param[out] -optional- J_t_m Jacobian of the tangent wrt to this.
   * @return The SO2 tangent at this.
   * @note See Eq. (95) & Eq. (106).
   * @see SO2Tangent.
   */
  Tangent lift(OptJacobianRef J_t_m = {}) const;

  /**
   * @brief Composition of this and another SO2 element.
   * @param[in] m Another SO2 element.
   * @param[out] -optional- J_mc_ma Jacobian of the composition wrt this.
   * @param[out] -optional- J_mc_mb Jacobian of the composition wrt m.
   * @return The composition of 'this . m'.
   * @note z_c = z_a z_b.
   * @note See Eq. (105).
   */
  template <typename _DerivedOther>
  LieGroup compose(const LieGroupBase<_DerivedOther>& m,
                   OptJacobianRef J_mc_ma = {},
                   OptJacobianRef J_mc_mb = {}) const;

  /**
   * @brief TODO tofix
   * @param  v
   * @param[out] -optional- J_vout_m The Jacobian of the new object wrt this.
   * @param[out] -optional- J_vout_v The Jacobian of the new object wrt input object.
   * @return
   */
  Vector act(const Vector &v,
             OptJacobianRef J_vout_m = {},
             OptJacobianRef J_vout_v = {}) const;

  /**
   * @brief Get the ajoint matrix of SO2 at this.
   * @note See Eq. (103).
   */
  Jacobian adj() const;

  // SO2 specific functions

  /**
   * @brief Get the transformation matrix (2D isometry).
   * @note T = | R 0 |
   *           | 0 1 |
   */
  Transformation transform() const;

  /**
   * @brief Get the rotation matrix R.
   */
  Rotation rotation() const;

  /**
   * @brief Get the real part of the underlying complex number.
   */
  Scalar real() const;

  /**
   * @brief Get the imaginary part of the underlying complex number.
   */
  Scalar imag() const;

  /**
   * @brief Get the angle (rad.).
   */
  Scalar angle() const;

protected:

  using Base::coeffs_nonconst;

  /// @todo given a Eigen::Map<const SO2>
  /// coeffs()->x() return a reference to
  /// temporary ...

//  Scalar& real();
//  Scalar& imag();
};

template <typename _Derived>
typename SO2Base<_Derived>::Transformation
SO2Base<_Derived>::transform() const
{
  Transformation T(Transformation::Identity());
  T.template block<2,2>(0,0) = rotation();
  return T;
}

template <typename _Derived>
typename SO2Base<_Derived>::Rotation
SO2Base<_Derived>::rotation() const
{
  using std::sin;
  using std::cos;
  const Scalar theta = angle();
  return (Rotation() << cos(theta), -sin(theta),
                        sin(theta),  cos(theta)).finished();
}

template <typename _Derived>
typename SO2Base<_Derived>::LieGroup
SO2Base<_Derived>::inverse(OptJacobianRef J_minv_m) const
{
  if (J_minv_m)
    J_minv_m->setConstant(Scalar(-1));

  return LieGroup(real(), -imag());
}

template <typename _Derived>
typename SO2Base<_Derived>::Tangent
SO2Base<_Derived>::lift(OptJacobianRef J_t_m) const
{
  if (J_t_m)
    J_t_m->setConstant(Scalar(1));

  return Tangent(angle());
}

template <typename _Derived>
template <typename _DerivedOther>
typename SO2Base<_Derived>::LieGroup
SO2Base<_Derived>::compose(
    const LieGroupBase<_DerivedOther>& m,
    OptJacobianRef J_mc_ma,
    OptJacobianRef J_mc_mb) const
{
  static_assert(
    std::is_base_of<SO2Base<_DerivedOther>, _DerivedOther>::value,
    "Argument does not inherit from SE2Base !");

  const auto& m_so2 = static_cast<const SO2Base<_DerivedOther>&>(m);

  const Scalar lhs_real = real();
  const Scalar lhs_imag = imag();
  const Scalar rhs_real = m_so2.real();
  const Scalar rhs_imag = m_so2.imag();

  if (J_mc_ma)
    J_mc_ma->setConstant(Scalar(1));

  if (J_mc_mb)
    J_mc_mb->setConstant(Scalar(1));

  return LieGroup(
        lhs_real * rhs_real - lhs_imag * rhs_imag,
        lhs_real * rhs_imag + lhs_imag * rhs_real
        );
}

template <typename _Derived>
typename SO2Base<_Derived>::Vector
SO2Base<_Derived>::act(const Vector &v,
                       OptJacobianRef J_vout_m,
                       OptJacobianRef J_vout_v) const
{
  if (J_vout_m)
  {
    (*J_vout_m) = rotation() * skew(1) * v;
  }

  if (J_vout_v)
  {
    (*J_vout_v) = rotation();
  }

  return rotation() * v;
}

template <typename _Derived>
typename SO2Base<_Derived>::Jacobian
SO2Base<_Derived>::adj() const
{
  static const Jacobian adj = Jacobian::Constant(Scalar(1));
  return adj;
}

/// SO2 specific function

template <typename _Derived>
/*const*/ typename SO2Base<_Derived>::Scalar/*&*/
SO2Base<_Derived>::real() const
{
  return coeffs().x();
}

template <typename _Derived>
/*const*/ typename SO2Base<_Derived>::Scalar/*&*/
SO2Base<_Derived>::imag() const
{
  return coeffs().y();
}

template <typename _Derived>
typename SO2Base<_Derived>::Scalar
SO2Base<_Derived>::angle() const
{
  using std::atan2;
  return atan2(imag(), real());
}

//template <typename _Derived>
//typename SO2Base<_Derived>::Scalar&
//SO2Base<_Derived>::real()
//{
//  return coeffs.x();
//}

//template <typename _Derived>
//typename SO2Base<_Derived>::Scalar&
//SO2Base<_Derived>::imag()
//{
//  return coeffs.y();
//}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO2_BASE_H_ */
