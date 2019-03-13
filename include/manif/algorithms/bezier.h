#ifndef _MANIF_MANIF_BEZIER_H_
#define _MANIF_MANIF_BEZIER_H_

#include "manif/impl/lie_group_base.h"
#include "manif/algorithms/interpolation.h"

#include <vector>

namespace manif {

/**
 * @brief Curve fitting using the DeCasteljau algorithm
 * on Lie groups.
 *
 * @param trajectory, a discretized trajectory.
 * @param degree, the degree of smoothness of the fitted curve.
 * @param k_interp, the number of points to interpolate
 * between two consecutive points of the trajectory.
 * interpolate k_interp for t in ]0,1].
 * @param closed_curve Whether the input trajectory is closed or not.
 * If true, the first and the last points of the input trajectory are used
 * to interpolate points inbetween. Default false.
 * @return The interpolated smooth trajectory
 *
 * @note A naive implementation of the DeCasteljau algorithm
 * on Lie groups.
 *
 * @link https://www.wikiwand.com/en/De_Casteljau%27s_algorithm
 */

template <typename LieGroup>
std::vector<typename LieGroup::LieGroup>
computeBezierCurve(const std::vector<LieGroup>& control_points,
                   const unsigned int degree,
                   const unsigned int k_interp)
{
  MANIF_CHECK(control_points.size() > 2, "Oups0");
  MANIF_CHECK(degree <= control_points.size(), "Oups1");
  MANIF_CHECK(k_interp > 0, "Oups2");

  const unsigned int n_segments =
       std::floor(double(control_points.size()-degree)/(degree-1)+1);

  std::vector<std::vector<const LieGroup*>> segments_control_points;
  for (unsigned int t=0; t<n_segments; ++t)
  {
    segments_control_points.emplace_back(std::vector<const LieGroup*>());

    // Retrieve control points of the current segment
    for (int n=0; n<degree; ++n)
    {
      if (verbose)
      std::cout << (t*degree+n) << ", ";
      segments_control_points.back().push_back( &control_points[t*(degree-1)+n] );
    }
    if (verbose)
    std::cout << "\n";
  }

  const int segment_k_interp = (degree == 2) ?
         k_interp : k_interp * degree;

  // Actual curve fitting
  std::vector<LieGroup> curve;
  for (unsigned int s=0; s<segments_control_points.size(); ++s)
  {
    for (int t=1; t<=segment_k_interp; ++t)
    {
      // t in [0,1]
      const double t_01 = static_cast<double>(t)/(segment_k_interp);

      LieGroup Qc = LieGroup::Identity();

      // recursive chunk of the algo,
      // compute tmp control points.
      for (int i=0; i<degree-1; ++i)
      {
        Qc = Qc.lplus(segments_control_points[s][i]->log() *
                      polynomialBernstein((double)degree, (double)i, (double)t_01));
      }

      curve.push_back(Qc);
    }
  }

  return curve;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_BEZIER_H_ */
