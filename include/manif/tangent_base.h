#ifndef _MANIF_MANIF_TANGENT_BASE_H_
#define _MANIF_MANIF_TANGENT_BASE_H_

#include "manif/fwd.h"

namespace manif
{

template <class _Manifold>
struct TangentBase
{
  using Scalar = typename _Manifold::Scalar;

  static constexpr int Dim = _Manifold::Dim;
  static constexpr int RepSize = Dim;

  using Manifold = typename _Manifold::Manifold;

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

  static Manifold Retract(const TangentBase<_Manifold>& t)
  {
    return t.retract();
  }

private:

  using Tangent = typename Manifold::Tangent;

  Tangent& tangent() { return *static_cast< Tangent* >(this); }
  const Tangent& tangent() const { return *static_cast< const Tangent* >(this); }
};

} /* namespace manif */

#endif /* _MANIF_MANIF_TANGENT_BASE_H_ */
