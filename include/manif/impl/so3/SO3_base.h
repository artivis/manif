#ifndef _MANIF_MANIF_SO3_BASE_H_
#define _MANIF_MANIF_SO3_BASE_H_

#include "manif/impl/so3/SO3_properties.h"
#include "manif/impl/manifold_base.h"

namespace manif
{

////////////////
///          ///
/// Manifold ///
///          ///
////////////////

template <typename _Derived>
struct SO3Base : ManifoldBase<_Derived>
{
private:

  using Base = ManifoldBase<_Derived>;
  using Type = SO3Base<_Derived>;

public:

  static constexpr int Dim = internal::ManifoldProperties<Type>::Dim;
  static constexpr int DoF = internal::ManifoldProperties<Type>::DoF;
  static constexpr int N   = internal::ManifoldProperties<Type>::N;

  using Scalar = typename Base::Scalar;

  using Manifold = typename Base::Manifold;
  using Tangent  = typename Base::Tangent;

  using JacobianMtoM = typename Base::JacobianMtoM;
  using JacobianMtoT = typename Base::JacobianMtoT;

  using ManifoldDataType = typename Base::ManifoldDataType;

  using Transformation  = typename Base::Transformation;
  using Rotation = typename Base::Rotation;

  using Base::data;

  /// Manifold common API

  Transformation matrix() const;

  Rotation rotation() const;

  void identity();

  void random();

  Manifold inverse() const;

  Manifold rplus(const Tangent& t) const;

  Manifold lplus(const Tangent& t) const;

  Manifold rminus(const Manifold& m) const;

  Manifold lminus(const Manifold& m) const;

  Tangent lift() const;

  Manifold compose(const Manifold& m) const;

  /// with Jacs

  void inverse(Manifold& m, JacobianMtoM& j) const;

  void rplus(const Tangent& t, Manifold& m,
             JacobianMtoM& J_c_a, JacobianMtoM& J_c_b) const;

  void lplus(const Tangent& t, Manifold& m,
             JacobianMtoM& J_c_a, JacobianMtoM& J_c_b) const;

  void rminus(const Manifold& min, Manifold& mout,
              JacobianMtoM& J_c_a, JacobianMtoM& J_c_b) const;

  void lminus(const Manifold& min, Manifold& mout,
              JacobianMtoM& J_c_a, JacobianMtoM& J_c_b) const;

  void lift(const Manifold& m, Tangent& t, JacobianMtoT& J_t_m) const;

  void compose(const Manifold& ma, const Manifold& mb,
               Manifold& mout,
               JacobianMtoM& J_c_a, JacobianMtoM& J_c_b) const;

  /// SO3 specific functions
};

template <typename _Derived>
typename SO3Base<_Derived>::Transformation
SO3Base<_Derived>::matrix() const
{
  MANIF_NOT_IMPLEMENTED_YET
  return Transformation();
}

template <typename _Derived>
typename SO3Base<_Derived>::Rotation
SO3Base<_Derived>::rotation() const
{
  MANIF_NOT_IMPLEMENTED_YET
  return Rotation();
}

template <typename _Derived>
void SO3Base<_Derived>::identity()
{
  MANIF_NOT_IMPLEMENTED_YET
}

template <typename _Derived>
void SO3Base<_Derived>::random()
{
  MANIF_NOT_IMPLEMENTED_YET
}

template <typename _Derived>
typename SO3Base<_Derived>::Manifold
SO3Base<_Derived>::inverse() const
{
  MANIF_NOT_IMPLEMENTED_YET
  return Manifold();
}

template <typename _Derived>
typename SO3Base<_Derived>::Manifold
SO3Base<_Derived>::rplus(const Tangent& t) const
{
  MANIF_NOT_IMPLEMENTED_YET
  return Manifold();
}

template <typename _Derived>
typename SO3Base<_Derived>::Manifold
SO3Base<_Derived>::lplus(const Tangent& t) const
{
  MANIF_NOT_IMPLEMENTED_YET
  return Manifold();
}

template <typename _Derived>
typename SO3Base<_Derived>::Manifold
SO3Base<_Derived>::rminus(const Manifold& m) const
{
  MANIF_NOT_IMPLEMENTED_YET
  return Manifold();
}

template <typename _Derived>
typename SO3Base<_Derived>::Manifold
SO3Base<_Derived>::lminus(const Manifold& m) const
{
  MANIF_NOT_IMPLEMENTED_YET
  return Manifold();
}

template <typename _Derived>
typename SO3Base<_Derived>::Tangent
SO3Base<_Derived>::lift() const
{
  MANIF_NOT_IMPLEMENTED_YET
  return Tangent();
}

template <typename _Derived>
typename SO3Base<_Derived>::Manifold
SO3Base<_Derived>::compose(const Manifold& m) const
{
  MANIF_NOT_IMPLEMENTED_YET
  return Manifold();
}

/// with Jacs

template <typename _Derived>
void SO3Base<_Derived>::inverse(Manifold& m, JacobianMtoM& j) const
{
  MANIF_NOT_IMPLEMENTED_YET
}

template <typename _Derived>
void SO3Base<_Derived>::rplus(const Tangent& t,
                              Manifold& m,
                              JacobianMtoM& J_c_a,
                              JacobianMtoM& J_c_b) const
{
  MANIF_NOT_IMPLEMENTED_YET
}

template <typename _Derived>
void SO3Base<_Derived>::lplus(const Tangent& t,
                              Manifold& m,
                              JacobianMtoM& J_c_a,
                              JacobianMtoM& J_c_b) const
{
  MANIF_NOT_IMPLEMENTED_YET
}

template <typename _Derived>
void SO3Base<_Derived>::rminus(const Manifold& min,
                               Manifold& mout,
                               JacobianMtoM& J_c_a,
                               JacobianMtoM& J_c_b) const
{
  MANIF_NOT_IMPLEMENTED_YET
}

template <typename _Derived>
void SO3Base<_Derived>::lminus(const Manifold& min,
                               Manifold& mout,
                               JacobianMtoM& J_c_a,
                               JacobianMtoM& J_c_b) const
{
  MANIF_NOT_IMPLEMENTED_YET
}

template <typename _Derived>
void SO3Base<_Derived>::lift(const Manifold& m,
                             Tangent& t,
                             JacobianMtoT& J_t_m) const
{
  MANIF_NOT_IMPLEMENTED_YET
}

template <typename _Derived>
void SO3Base<_Derived>::compose(const Manifold& ma,
                                const Manifold& mb,
                                Manifold& mout,
                                JacobianMtoM& J_c_a,
                                JacobianMtoM& J_c_b) const
{
  MANIF_NOT_IMPLEMENTED_YET
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO3_BASE_H_ */
