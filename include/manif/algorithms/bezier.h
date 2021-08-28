#ifndef _MANIF_MANIF_BEZIER_H_
#define _MANIF_MANIF_BEZIER_H_

#include "manif/impl/lie_group_base.h"
#include "manif/algorithms/interpolation.h"

#include <vector>
#include <iostream>

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
bezier(
  const std::vector<LieGroup>& control_points,
  const unsigned int degree,
  const unsigned int k_interp
)
{
  MANIF_CHECK(
    control_points.size() > 2,
    "Input trajectory must have more than two points!"
  );
  MANIF_CHECK(
    degree <= control_points.size(),
    "Degree must be less or equal to the number of input points!"
  );
  MANIF_CHECK(
    k_interp > 0,
    "k_interp must be greater than zero!"
  );

  // std::cout << "trajectory.size: " << control_points.size() << std::endl;
  // std::cout << "degree: " << degree << std::endl;
  // std::cout << "k_interp: " << k_interp << std::endl;

  unsigned int n_segments = static_cast<unsigned int>(
    std::floor(double(control_points.size()-degree)/double((degree-1)+1))
  );

  if (!n_segments) n_segments = 1;

  // std::cout << "n_segments: " << n_segments << std::endl;

  std::vector<std::vector<const LieGroup*>> segments_control_points(
    n_segments,
    std::vector<const LieGroup*>(degree)
  );
  for (unsigned int t=0; t<n_segments; ++t)
  {
    // Retrieve control points of the current segment
    for (int n=0; n<degree; ++n)
    {
      segments_control_points[t][n] = &control_points[t*(degree-1)+n];
    }
  }

  const unsigned int segment_k_interp = (degree == 2) ? k_interp : k_interp * degree;

  // std::cout << "segment_k_interp: " << segment_k_interp << std::endl;

  // Actual curve fitting
  // std::vector<LieGroup> curve(
  //   segments_control_points.size()*segment_k_interp,
  //   LieGroup::Identity()
  // );
  std::vector<LieGroup> curve;
  curve.reserve(segments_control_points.size()*segment_k_interp);
  LieGroup Qc;
  for (unsigned int s=0; s<segments_control_points.size(); ++s)
  {
    // std::cout << "s: " << s << std::endl;

    LieGroup Qc = *segments_control_points[s][0];

    for (unsigned int t=1; t<=segment_k_interp; ++t)
    {
      // std::cout << "t: " << t << std::endl;

      // recursive chunk of the algo, compute tmp control points.
      for (unsigned int i=0; i<degree-1; ++i)
      {
        // std::cout << "i: " << i << std::endl;

        Qc = Qc.rplus(
          // segments_control_points[s][i]->log() *
          segments_control_points[s][i+1]->rminus(*segments_control_points[s][i]) *
          polynomialBernstein(
            (double)degree,
            (double)i,
            static_cast<double>(t)/(segment_k_interp) // t in [0,1]
          )
        );
      }

      curve.push_back(Qc);
    }
  }

  return curve;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_BEZIER_H_ */
