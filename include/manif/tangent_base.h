#ifndef _MANIF_MANIF_TANGENT_BASE_H_
#define _MANIF_MANIF_TANGENT_BASE_H_

namespace manif
{

template <typename T>
struct TangentTraits;

template <class _Derived>
struct TangentBase
{
  using Scalar  = typename TangentTraits<_Derived>::Scalar;

  using Tangent = typename TangentTraits<_Derived>::Tangent;

  static constexpr int Dim     = TangentTraits<_Derived>::Dim;
  static constexpr int RepSize = TangentTraits<_Derived>::RepSize;

  using Manifold = typename TangentTraits<_Derived>::Manifold;

  Manifold retract() const
  {
    return tangent().retract();
  }

protected:

  /*constexpr*/ Tangent& tangent() { return static_cast< Tangent& >(*this); }
};

} /* namespace manif */

#endif /* _MANIF_MANIF_TANGENT_BASE_H_ */
