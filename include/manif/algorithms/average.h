#ifndef _MANIF_MANIF_AVERAGE_H_
#define _MANIF_MANIF_AVERAGE_H_

#include "manif/impl/lie_group_base.h"
//#include "manif/interpolation.h"
#include <iostream>
namespace manif {

//template <template <typename LieGroup, typename...Args> class Container,
//          typename LieGroup, typename...Args>
//LieGroup
//average_slerp(const Container<LieGroup, Args...>& mans)
//{
//  if (mans.empty())
//    return LieGroup();
//  else if (mans.size() == 1)
//    return *mans.begin();

//  auto it = mans.begin();

//  LieGroup carry = *it;

//  ++it;
//  double i = 2;
//  for (; it != mans.end(); ++it, ++i)
//  {
//    carry = interpolate(carry, *it, (i-1.)/i);
//  }

//  return carry;
//}

/**
 * @brief Compute an average point on Lie groups given a list of
 * 'close' points.
 * @param[in] points A list of 'close' points to compute an average point from.
 * @param[in] eps, update norm threshold to break the iterative averaging.
 * @param[in] max_iterations, max number of iterations.
 * @return The average point of the input points.
 *
 * @note see (a)
 * "Bi-invariant Means in Lie Groups.
 * Application to Left-in variant Polyaffine Transformations" p. 21 Sec. 4.2
 * @link ftp://ftp-sop.inria.fr/epidaure/Publications/Arsigny/arsigny_rr_biinvariant_mean.pdf
 *
 * @note see also (b)
 * "A globally convergent numerical algorithm for computing the center of
 * mass on compact Lie groups."
 * @link http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.721.8010&rep=rep1&type=pdf
 */
template <template <typename LieGroup, typename...Args> class Container,
          typename LieGroup, typename...Args>
LieGroup
average_biinvariant(const Container<LieGroup, Args...>& points,
                    typename LieGroup::Scalar eps =
                      Constants<typename LieGroup::Scalar>::eps_s,
                    int max_iterations = 20)
{
  using Scalar  = typename LieGroup::Scalar;
  using Tangent = typename LieGroup::Tangent;

  MANIF_CHECK(!points.empty(), "Points container is empty !");
  if (points.size() == 1)
    return *points.begin();

  LieGroup avg = *points.begin();

  const Scalar w = Scalar(1) / Scalar(points.size());

  Tangent ts;
  int i=0;
  for (; i<max_iterations; ++i)
  {
    auto it        = points.begin();
    const auto end = points.end();

    ts.setZero();
    for (; it != end; ++it)
    {
      // Update as in (a) & (b)
      ts += ((*it) - avg);
    }
    ts *= w; // doing the common product by 1/N just once

    //////////////
    // Stopping criterion is from (b)
    //////////////

    if (ts.coeffs().squaredNorm() < eps)
      break;

    avg += ts;

    //////////////
    // Stopping criterion is from (a)
    //////////////

//    const LieGroup avg_0 = avg;
//    avg += ts;

//    if (avg.between(avg_0).log().coeffs().squaredNorm() < eps)
//      break;
  }

  //std::cout << "Biinvariant stopped after " << i << " iterations.\n";

  return avg;
}

template <template <typename LieGroup, typename...Args> class Container,
          typename LieGroup, typename...Args>
LieGroup
average(const Container<LieGroup, Args...>& points,
        typename LieGroup::Scalar eps =
          Constants<typename LieGroup::Scalar>::eps_s,
        int max_iterations = 20)
{
  using Scalar  = typename LieGroup::Scalar;
  using Tangent = typename LieGroup::Tangent;

  MANIF_CHECK(!points.empty(), "Points container is empty !");
  if (points.size() == 1)
    return *points.begin();

  LieGroup avg = *points.begin();

  const Scalar w = Scalar(1) / Scalar(points.size());

  Tangent ts, tmp;
  typename LieGroup::Jacobian Jr, G;
  for (int i=0; i<max_iterations; ++i)
  {
    auto it        = points.begin();
    const auto end = points.end();

    ts.setZero();
    for (; it != end; ++it)
    {
      tmp = avg.between(*it).log();

      // Neither (a) nor (b) use G for weighting
      Jr = tmp.rjac();
      G.noalias() = Jr.transpose() * Jr;

      ts += G * tmp;
    }
    ts *= w; // doing the common product by 1/N just once

    // This stopping criterion is derived from (b)
    typename LieGroup::Jacobian G = ts.rjac().transpose() * ts.rjac();
    const Scalar n = ts.coeffs().transpose() * G * ts.coeffs();

    if (n < Constants<Scalar>::eps_s)
      break;

    avg += ts;
  }

  return avg;
}

// ftp://ftp-sop.inria.fr/epidaure/Publications/Arsigny/arsigny_rr_biinvariant_mean.pdf
// page 38
// https://hal.inria.fr/hal-00938320/document#subsection.118
// page 94
template <template <typename LieGroup, typename...Args> class Container,
          typename LieGroup, typename...Args>
LieGroup
average_frechet_left(const Container<LieGroup, Args...>& points,
                     typename LieGroup::Scalar eps =
                       Constants<typename LieGroup::Scalar>::eps_s,
                     int max_iterations = 20)
{
  using Scalar  = typename LieGroup::Scalar;
  using Tangent = typename LieGroup::Tangent;

  MANIF_CHECK(!points.empty(), "Points container is empty !");
  if (points.size() == 1)
    return *points.begin();

  LieGroup avg = *points.begin();

  const Scalar w = Scalar(1) / Scalar(points.size());

  Tangent ts, tmp;
  typename LieGroup::Jacobian Jl;
  int i=0;
  for (; i<max_iterations; ++i)
  {
    auto it        = points.begin();
    const auto end = points.end();

    ts.setZero();
    const LieGroup avg_0 = avg;

    for (; it != end; ++it)
    {
      tmp = (*it) - avg_0; // Log( Avg^-1 . Xi )

      Jl = avg_0.log().ljac(); // Jl(Avg)

      ts += Jl * tmp * w;
    }

    // Avg = Avg . Exp( Jl^-1(Avg) . ts )
    avg = avg_0 + ( avg_0.log().ljacinv() * ts );

    tmp = avg_0.log().ljac() * (avg - avg_0);

    if (tmp.coeffs().squaredNorm() < eps)
      break;
  }

  //std::cout << "Frechet Left stopped after " << i << " iterations.\n";

  return avg;
}

template <template <typename LieGroup, typename...Args> class Container,
          typename LieGroup, typename...Args>
LieGroup
average_frechet_right(const Container<LieGroup, Args...>& points,
                      typename LieGroup::Scalar eps =
                        Constants<typename LieGroup::Scalar>::eps_s,
                      int max_iterations = 20)
{
  using Scalar  = typename LieGroup::Scalar;
  using Tangent = typename LieGroup::Tangent;

  MANIF_CHECK(!points.empty(), "Points container is empty !");
  if (points.size() == 1)
    return *points.begin();

  LieGroup avg = *points.begin();

  const Scalar w = Scalar(1) / Scalar(points.size());

  Tangent ts, tmp;
  typename LieGroup::Jacobian Jr;
  int i=0;
  for (; i<max_iterations; ++i)
  {
    auto it        = points.begin();
    const auto end = points.end();

    ts.setZero();
    const LieGroup avg_0 = avg;

    for (; it != end; ++it)
    {
      tmp = it->lminus(avg_0); // Log( Xi . Avg^-1 )

      Jr = avg_0.log().rjac(); // Jr(Avg)

      ts += Jr * tmp * w;
    }

    // Avg = Exp(Jr^-1(Avg) . ts) * Avg
    avg = avg_0.lplus( avg_0.log().rjacinv() * ts );

    tmp = avg_0.log().rjac() * (avg.lminus(avg_0));

    if (tmp.coeffs().squaredNorm() < eps)
      break;
  }

  //std::cout << "Frechet Right stopped after " << i << " iterations.\n";

  return avg;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_AVERAGE_H_ */
