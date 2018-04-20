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

//  using Tangent  = typename ManifoldTraits<_Derived>::Tangent;

  struct Tangent
  {
    using Scalar = typename ManifoldTraits<_Derived>::Scalar;

    static constexpr int Dim = ManifoldTraits<_Derived>::Dim;
    static constexpr int RepSize = Dim;

    using Manifold = typename ManifoldTraits<_Derived>::Manifold;

    Manifold retract() const;
//    {
//      return tangent().retract();
//    }
  };

  void identity()
  {
    manifold().identity();
  }

  void random()
  {
    manifold().random();
  }

  Manifold inverse() const
  {
    return manifold().inverse();
  }

  Manifold plus(const Tangent& t) const
  {
    return manifold().plus(t);
  }

  Manifold rminus(const Manifold& m) const
  {
    return manifold().rminus(m);
  }

  Manifold lminus(const Manifold& m) const
  {
    return manifold().lminus(m);
  }

  Tangent lift() const
  {
    return manifold().lift();
  }

  /*
  LieType lie() const
  {
    return manifold().lie();
  }
  */

  Manifold compose(const Manifold& m)
  {
    return manifold().compose(m);
  }

  Manifold between(const Manifold& m)
  {
    return manifold().inverse().compose(m);
  }

  /*
  Manifold interpolate()
  {
    return manifold().interpolate();
  }
  */

  /// some static helpers

  static Manifold Identity()
  {
    static Manifold m; m.identity();
    return m;
  }

  static Manifold Random()
  {
    static Manifold m; m.random();
    return m;
  }

  static Tangent Inverse(const Manifold& m)
  {
    return m.inverse();
  }

  static Manifold Rplus(const Manifold& m, const Tangent& t)
  {
    return m.plus(t);
  }

  static Manifold Lplus(const Tangent& t, const Manifold& m)
  {
    return t.plus(m);
  }

  static Manifold Rminus(const Manifold& m0, const Manifold& m1)
  {
    return m0.rminus(m1);
  }

  static Manifold Lminus(const Manifold& m0, const Manifold& m1)
  {
    return m0.lminus(m1);
  }

  static Tangent Lift(const Manifold& m)
  {
    return m.lift();
  }

  static Manifold Retract(const Tangent& t)
  {
    return t.retract();
  }

  /*
  static LieType Lie(const Manifold& m)
  {
    return m.lie();
  }

  static LieType Lie(const Tangent& t)
  {
    return t.lie();
  }
  */

  static Manifold Compose(const Manifold& m0, const Manifold& m1)
  {
    return m0.compose(m1);
  }

  static Manifold Between(const Manifold& m0, const Manifold& m1)
  {
    return m0.between(m1);
  }

protected:

  /*constexpr*/ Manifold& manifold() { return static_cast< Manifold& >(*this); }
};

} /* namespace manif */

#endif /* _MANIF_MANIF_MANIFOLD_BASE_H_ */
