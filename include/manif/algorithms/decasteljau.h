#ifndef _MANIF_MANIF_DECASTELJAU_H_
#define _MANIF_MANIF_DECASTELJAU_H_

#include "manif/impl/lie_group_base.h"

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
decasteljau(
  const std::vector<LieGroup>& trajectory,
  const unsigned int degree,
  const unsigned int k_interp,
  const bool closed_curve = false
)
{
  MANIF_CHECK(
    trajectory.size() > 2,
    "Input trajectory must have more than two points!"
  );
  MANIF_CHECK(
    degree <= trajectory.size(),
    "Degree must be less or equal to the number of input points!"
  );
  MANIF_CHECK(
    k_interp > 0,
    "k_interp must be greater than zero!"
  );

  // std::cout << "trajectory.size: " << trajectory.size() << std::endl;
  // std::cout << "degree: " << degree << std::endl;
  // std::cout << "k_interp: " << k_interp << std::endl;
  // std::cout << "closed_curve: " << closed_curve << std::endl;

  // Number of connected, non-overlapping segments
  unsigned int n_segments = static_cast<unsigned int>(
      // std::floor(double(trajectory.size()-degree)/double((degree-1)+1))
      double(trajectory.size())/double(degree);
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
    for (unsigned int n=0; n<degree; ++n)
    {
      segments_control_points[t][n] = &trajectory[t*(degree-1)+n];
    }
  }

  // Close the curve if there are left-over points
  if (closed_curve && ((n_segments*(degree-1)) <= trajectory.size()-1))
  {
    // std::cout << "Closing: " << n_segments << std::endl;

    const unsigned int last_pts_idx = n_segments*(degree-1);
    const unsigned int left_over = trajectory.size()-1-last_pts_idx;
    segments_control_points.emplace_back(std::vector<const LieGroup*>());

    // Get the left-over points
    for (unsigned int p=last_pts_idx; p<trajectory.size(); ++p)
    {
      segments_control_points.back().push_back( &trajectory[p] );
    }
    // Add a extra points from the beginning of the trajectory
    for (unsigned int p=0; p<degree-left_over-1; ++p)
    {
      segments_control_points.back().push_back( &trajectory[p] );
    }
  }

  // std::cout << "segments_control_points.size(): " << segments_control_points.size() << std::endl;

  const unsigned int segment_k_interp = (degree == 2) ? k_interp : k_interp * degree;

  // std::cout << "segment_k_interp: " << segment_k_interp << std::endl;

  // Actual curve fitting
  std::vector<LieGroup> curve;
  curve.reserve(segments_control_points.size()*segment_k_interp);
  for (unsigned int s=0; s<segments_control_points.size(); ++s)
  {
    for (unsigned int t=1; t<=segment_k_interp; ++t)
    {
      // t in [0,1]
      const double t_01 = static_cast<double>(t)/(segment_k_interp);

      std::vector<LieGroup> Qs, Qs_tmp;

      for (const auto m : segments_control_points[s])
        Qs.emplace_back(*m);

      // recursive chunk of the algo, compute tmp control points.
      for (unsigned int i=0; i<degree-1; ++i)
      {
        for (unsigned int q=0; q<Qs.size()-1; ++q)
        {
          // @todo parameterizable interpolation function
          Qs_tmp.push_back( Qs[q].rplus(Qs[q+1].rminus(Qs[q]) * t_01) );
        }

        Qs = Qs_tmp;
        Qs_tmp.clear();
      }

      curve.push_back(Qs[0]);
    }
  }

  return curve;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_DECASTELJAU_H_ */
