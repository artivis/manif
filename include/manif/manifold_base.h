#ifndef _MANIF_MANIF_MANIFOLD_BASE_H_
#define _MANIF_MANIF_MANIFOLD_BASE_H_

namespace manif
{

template <typename T>
struct ManifoldTraits;

template <class _Derived>
struct ManifoldBase
{
  using Scalar   = typename ManifoldTraits<_Derived>::Scalar;

  using Manifold = typename ManifoldTraits<_Derived>::Manifold;

  static constexpr int Dim     = ManifoldTraits<_Derived>::Dim;
  static constexpr int RepSize = ManifoldTraits<_Derived>::RepSize;

  using Tangent  = typename ManifoldTraits<_Derived>::Tangent;

  Manifold zero()
  {
    return manifold().zero();
  }

  static Manifold Zero()
  {
    return manifold().Zero();
  }

  void identity()
  {
    manifold().identity();
  }

  static Manifold Identity()
  {
    return manifold().Identity();
  }

  Manifold inverse() const
  {
    return manifold().inverse();
  }

  Tangent lift() const
  {
    return manifold().lift();
  }

  static Tangent lift(const Manifold& m)
  {
    return m.lift();
  }

  static Manifold retract(const Tangent& t)
  {
    return manifold().retract(t);
  }

  Manifold plus(const Manifold& m) const
  {
    return manifold().plus(m);
  }

  Manifold minus(const Manifold& m) const
  {
    return manifold().minus(m);
  }

  /*
  auto compose() -> decltype(std::declval<Manifold>().compose())
  {
    return manifold().compose();
  }

  auto between() -> decltype(std::declval<Manifold>().between())
  {
    return manifold().between();
  }

  auto interpolate() -> decltype(std::declval<Manifold>().interpolate())
  {
    return manifold().interpolate();
  }
  */

protected:

  /*constexpr*/ Manifold& manifold() { return static_cast< Manifold& >(*this); }
};

} /* namespace manif */

#endif /* _MANIF_MANIF_MANIFOLD_BASE_H_ */
