#include "manif/SE2.h"
#include "manif/algorithms/interpolation.h"
#include "se2_points_generator.h"

#include <vector>
#include <iostream>

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    std::cout << "Usage: .se2_interpolation <k> <i> <p>\n";
    std::cout << "\t with k: number of initial points on the 8-shaped curve.\n";
    std::cout << "\t with i: interpolation algorithm to use.\n";
    std::cout << "\t with p: number of points to generate between consecutive points of the initial curve.\n";
    std::cout << "\t Interpolation algorithm are : 0-Slerp / 1-Cubic / 2-CN-Smooth.\n";
    return EXIT_SUCCESS;
  }

  int k, i, p;

  k = atoi(argv[1]);
  i = atoi(argv[2]);
  p = atoi(argv[3]);

  manif::INTERP_METHOD interp_method;
  switch (i) {
  case 0:
    interp_method = manif::INTERP_METHOD::SLERP;
    break;
  case 1:
    interp_method = manif::INTERP_METHOD::CUBIC;
    break;
  case 2:
    interp_method = manif::INTERP_METHOD::CNSMOOTH;
    break;
  default:
    std::cerr << "Interpolation method 'i' must be in [0,2] !\n";
    return EXIT_FAILURE;
    break;
  }

  const auto points = manif::generateSE2PointsOnHeightShape(k);

  // Interpolate between k-points
  // between each consecutive points
  // of the initial curve.

  std::vector<manif::SE2d> interpolated;

  // Initial point with Tangent t0 = 0

  manif::SE2Tangentd t0 = manif::SE2Tangentd::Zero();
  manif::SE2Tangentd t1 = points[1].rminus(points[0]);

  for (int j=1; j<=p; ++j)
  {
    interpolated.push_back(
      interpolate(points[0], points[1],
                  double(j)/double(p+1),
                  interp_method,
                  t0, t1
        )
    );
  }

  // Intermediate points

  for (int i=1; i<points.size()-1; ++i)
  {
    const manif::SE2d& s0 = points[ i ];
    const manif::SE2d& s1 = points[i+1];

    t0 = points[ i ] - points[i-1];
    t1 = points[i+1] - points[ i ];

    for (int j=1; j<=p; ++j)
    {
      interpolated.push_back(
        interpolate(s0, s1,
                    static_cast<double>(j)/(p+1),
                    interp_method,
                    t0, t1
        )
      );
    }
  }

  // Close the loop
  // Final point with Tangent t1 = 0

  const manif::SE2d& s0 = points.back();
  const manif::SE2d& s1 = points[0];

  t0 = points.back() - points[points.size()-2];
  t1 = manif::SE2Tangentd::Zero();

  for (int j=1; j<=p; ++j)
  {
    interpolated.push_back(
      interpolate(s0, s1,
                  static_cast<double>(j)/(p+1),
                  interp_method,
                  t0, t1
      )
    );
  }

  // Print in terminal

  std::cout << k << ", " << i << ", " << p << "\n";

  for (const auto& point : points)
  {
    std::cout << point.x() << ","
              << point.y() << ","
              << point.angle() << "\n";
  }

  for (const auto& interp : interpolated)
  {
    std::cout << interp.x() << ","
              << interp.y() << ","
              << interp.angle() << "\n";
  }

  return EXIT_SUCCESS;
}
