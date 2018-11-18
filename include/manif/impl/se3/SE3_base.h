#ifndef _MANIF_MANIF_SE3_BASE_H_
#define _MANIF_MANIF_SE3_BASE_H_

#include "manif/impl/se3/SE3_properties.h"
#include "manif/impl/lie_group_base.h"
#include "manif/impl/so3/SO3_map.h"

namespace manif {

////////////////
///          ///
/// LieGroup ///
///          ///
////////////////

template <typename _Derived>
struct SE3Base : LieGroupBase<_Derived>
{
private:

  using Base = LieGroupBase<_Derived>;
  using Type = SE3Base<_Derived>;

public:

  MANIF_GROUP_PROPERTIES

  MANIF_GROUP_TYPEDEF

  /// @todo find a mechanism to fetch it from base
  /// just like the other typedefs
  using Translation = typename internal::traits<_Derived>::Translation;

  /// LieGroup common API

  Transformation transform() const;
  Rotation rotation() const;
  Translation translation() const;

  LieGroup inverse(OptJacobianRef J_minv_m = {}) const;
  Tangent lift(OptJacobianRef J_t_m = {}) const;

  template <typename _DerivedOther>
  LieGroup compose(const LieGroupBase<_DerivedOther>& m,
                   OptJacobianRef J_mc_ma = {},
                   OptJacobianRef J_mc_mb = {}) const;

  Vector act(const Vector &v,
             OptJacobianRef J_vout_m = {},
             OptJacobianRef J_vout_v = {}) const;

  Jacobian adj() const;

  using Base::coeffs;
  using Base::coeffs_nonconst;
  using Base::data;
  MANIF_INHERIT_GROUP_AUTO_API
  MANIF_INHERIT_GROUP_OPERATOR

  /// SE3 specific functions

  Scalar x() const;
  Scalar y() const;
  Scalar z() const;

  //Scalar roll() const;
  //Scalar pitch() const;
  //Scalar yaw() const;

//protected:

  /// Helper

//  auto trapart()
//  -> decltype( std::declval<Type>().coeffs().template block<Dim, 1>(0,0) )
//  {
//    coeffs().block<Dim, 1>(0,0);
//  }

//  auto trapart() const
//  -> decltype( std::declval<const Type>().coeffs().template block<Dim, 1>(0,0) )
//  {
//    coeffs().block<Dim, 1>(0,0);
//  }

//  auto rotpart() const
//  -> decltype( std::declval<const Type>().coeffs().template block<4, 1>(3,0) )
//  {
//    coeffs().block<4, 1>(3,0);
//  }

//  auto rotpart() const
//  -> decltype( std::declval<const Type>().coeffs().template block<4, 1>(3,0) )
//  {
//    coeffs().block<4, 1>(3,0);
//  }

  Eigen::Map<const SO3<Scalar>> asSO3() const
  {
    return Eigen::Map<const SO3<Scalar>>(coeffs().data()+3);
  }

  Eigen::Map<SO3<Scalar>> asSO3()
  {
    return Eigen::Map<SO3<Scalar>>(coeffs_nonconst().data()+3);
  }
};

template <typename _Derived>
typename SE3Base<_Derived>::Transformation
SE3Base<_Derived>::transform() const
{
  Transformation T = Transformation::Identity();
  T.template topLeftCorner<3,3>()  = rotation();
  T.template topRightCorner<3,1>() = translation();
  return T;
}

template <typename _Derived>
typename SE3Base<_Derived>::Rotation
SE3Base<_Derived>::rotation() const
{
  return asSO3().rotation();
}

template <typename _Derived>
typename SE3Base<_Derived>::Translation
SE3Base<_Derived>::translation() const
{
  return coeffs().template head<3>();
}

template <typename _Derived>
typename SE3Base<_Derived>::LieGroup
SE3Base<_Derived>::inverse(OptJacobianRef J_minv_m) const
{
  if (J_minv_m)
  {
    (*J_minv_m) = -adj();
  }

  const SO3<Scalar> so3inv = asSO3().inverse();

  return LieGroup(-so3inv.act(translation()),
                   so3inv);
}

template <typename _Derived>
typename SE3Base<_Derived>::Tangent
SE3Base<_Derived>::lift(OptJacobianRef J_t_m) const
{
  using std::abs;
  using std::sqrt;

  const SO3Tangent<Scalar> so3tan = asSO3().lift();

  Tangent tan((typename Tangent::DataType() <<
               so3tan.ljac().inverse()*translation(),
               so3tan.coeffs()).finished());

  if (J_t_m)
  {
    // Jr^-1
    (*J_t_m) = tan.rjac().inverse();
  }

  return tan;
}

template <typename _Derived>
template <typename _DerivedOther>
typename SE3Base<_Derived>::LieGroup
SE3Base<_Derived>::compose(
    const LieGroupBase<_DerivedOther>& m,
    OptJacobianRef J_mc_ma,
    OptJacobianRef J_mc_mb) const
{
  static_assert(
    std::is_base_of<SE3Base<_DerivedOther>, _DerivedOther>::value,
    "Argument does not inherit from SE3Base !");

  const auto& m_se3 = static_cast<const SE3Base<_DerivedOther>&>(m);

  if (J_mc_ma)
  {
    (*J_mc_ma) = m.adj().inverse();
  }

  if (J_mc_mb)
  {
    J_mc_mb->setIdentity();
  }

  return LieGroup(rotation()*m_se3.translation() + translation(),
                  asSO3().compose(m_se3.asSO3()).quat());
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
    (*J_vout_v) = -skew(v);
  }

  if (J_vout_v)
  {
    (*J_vout_v) = rotation();
  }

  return transform() * v;
}

template <typename _Derived>
typename SE3Base<_Derived>::Jacobian
SE3Base<_Derived>::adj() const
{
  /// @note Chirikjian (close to Eq.10.94)
  /// says
  ///       Ad(g) = |  R  0 |
  ///               | T.R R |
  ///
  /// considering vee(log(g)) = (w;v)
  /// with T = [t]_x
  ///
  /// but this is
  ///       Ad(g) = | R T.R |
  ///               | 0  R  |
  ///
  /// considering vee(log(g)) = (v;w)

  Jacobian Adj = Jacobian::Zero();
  Adj.template topLeftCorner<3,3>() = rotation();
  Adj.template bottomRightCorner<3,3>() =
      Adj.template topLeftCorner<3,3>();
  Adj.template topRightCorner<3,3>() =
    skew(translation()) * Adj.template topLeftCorner<3,3>();

  return Adj;
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
