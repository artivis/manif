#ifndef _MANIF_MANIF_SE3_BASE_H_
#define _MANIF_MANIF_SE3_BASE_H_

#include "manif/impl/se2/SE3_properties.h"
#include "manif/impl/manifold_base.h"

namespace manif
{

////////////////
///          ///
/// Manifold ///
///          ///
////////////////

template <typename _Derived>
struct SE3Base : ManifoldBase<_Derived>
{
private:

  using Base = ManifoldBase<_Derived>;
  using Type = SE3Base<_Derived>;

public:

  MANIF_MANIFOLD_PROPERTIES

  MANIF_MANIFOLD_TYPEDEF

  /// @todo find a mechanism to fetch it from base
  /// just like the other typedefs
  using Translation = typename internal::traits<_Derived>::Translation;

  /// Manifold common API

  Transformation transform() const;
  Rotation rotation() const;
  Translation translation() const;

  void identity();

  Manifold inverse(OptJacobianRef J_minv_m = {}) const;
  Tangent lift(OptJacobianRef J_t_m = {}) const;

  template <typename _DerivedOther>
  Manifold compose(const ManifoldBase<_DerivedOther>& m,
                   OptJacobianRef J_mc_ma = {},
                   OptJacobianRef J_mc_mb = {}) const;

  Vector act(const Vector &v,
             OptJacobianRef J_vout_m = {},
             OptJacobianRef J_vout_v = {}) const;

  using Base::coeffs;
  using Base::coeffs_nonconst;
  MANIF_INHERIT_MANIFOLD_AUTO_API
  MANIF_INHERIT_MANIFOLD_OPERATOR

  /// SE3 specific functions

  Scalar x() const;
  Scalar y() const;
  Scalar y() const;

  //Scalar roll() const;
  //Scalar pitch() const;
  //Scalar yaw() const;
};

template <typename _Derived>
typename SE3Base<_Derived>::Transformation
SE3Base<_Derived>::transform() const
{
  Transformation T(Transformation::Identity());
  T.template block<3,3>(0,0) = rotation();
  T(0,2) = x();
  T(1,2) = y();
  T(2,2) = z();
  return T;
}

template <typename _Derived>
typename SE3Base<_Derived>::Rotation
SE3Base<_Derived>::rotation() const
{
  MANIF_NOT_IMPLEMENTED_YET
}

template <typename _Derived>
typename SE3Base<_Derived>::Translation
SE3Base<_Derived>::translation() const
{
  return coeffs().template block<Dim,1>(0,0);
}

template <typename _Derived>
void SE3Base<_Derived>::identity()
{
  coeffs_nonconst().setZero();
  coeffs_nonconst()(6) = 1;
}

template <typename _Derived>
typename SE3Base<_Derived>::Manifold
SE3Base<_Derived>::inverse(OptJacobianRef J_minv_m) const
{
  MANIF_NOT_IMPLEMENTED_YET;

  if (J_minv_m)
  {

  }

  return Manifold();
}

template <typename _Derived>
typename SE3Base<_Derived>::Tangent
SE3Base<_Derived>::lift(OptJacobianRef J_t_m) const
{
  MANIF_NOT_IMPLEMENTED_YET;

  if (J_t_m)
  {

  }

  return Tangent();
}

template <typename _Derived>
template <typename _DerivedOther>
typename SE3Base<_Derived>::Manifold
SE3Base<_Derived>::compose(
    const ManifoldBase<_DerivedOther>& m,
    OptJacobianRef J_mc_ma,
    OptJacobianRef J_mc_mb) const
{
  static_assert(
    std::is_base_of<SE3Base<_DerivedOther>, _DerivedOther>::value,
    "Argument does not inherit from SE3Base !");

  const auto& m_se3 = static_cast<const SE3Base<_DerivedOther>&>(m);

  MANIF_NOT_IMPLEMENTED_YET

  return Manifold();
}

template <typename _Derived>
typename SE3Base<_Derived>::Vector
SE3Base<_Derived>::act(const Vector &v,
                       OptJacobianRef J_vout_m,
                       OptJacobianRef J_vout_v) const
{
  if (J_vout_m)
  {
    MANIF_NOT_IMPLEMENTED_YET
  }

  if (J_vout_v)
  {
    MANIF_NOT_IMPLEMENTED_YET
  }

  return transform() * v;
}

/// SE3 specific function

template <typename _Derived>
typename SE3Base<_Derived>::Scalar
SE3Base<_Derived>::x() const
{
  return coeffs().x();
}

template <typename _Derived>
typename SE3Base<_Derived>::Scalar
SE3Base<_Derived>::y() const
{
  return coeffs().y();
}

template <typename _Derived>
typename SE3Base<_Derived>::Scalar
SE3Base<_Derived>::z() const
{
  return coeffs().z();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SE3_BASE_H_ */
