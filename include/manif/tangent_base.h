#ifndef _MANIF_MANIF_TANGENT_BASE_H_
#define _MANIF_MANIF_TANGENT_BASE_H_

#include "manif/fwd.h"

namespace manif
{

template <class _Derived>
struct TangentBase
{
  using Scalar   = typename internal::traits<_Derived>::Scalar;

  using Manifold = typename internal::traits<_Derived>::Manifold;
  using Tangent  = typename internal::traits<_Derived>::Tangent;

  static constexpr int Dim     = internal::traits<_Derived>::Dim;
  static constexpr int RepSize = internal::traits<_Derived>::RepSize;
  static constexpr int DoF     = internal::traits<_Derived>::DoF;

  operator Tangent&() { return tangent(); }
  operator const Tangent& () const { return tangent(); }

  void zero()
  {
    tangent().zero();
  }

  void random()
  {
    tangent().random();
  }

  Manifold retract() const
  {
    return tangent().retract();
  }

  static Manifold Retract(const Tangent& t)
  {
    return t.retract();
  }

private:

  Tangent& tangent() { return *static_cast< Tangent* >(this); }
  const Tangent& tangent() const { return *static_cast< const Tangent* >(this); }
};

} /* namespace manif */

#endif /* _MANIF_MANIF_TANGENT_BASE_H_ */
