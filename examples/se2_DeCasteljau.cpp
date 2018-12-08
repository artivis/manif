#include "se2_points_generator.h"
#include "manif/algorithms/decasteljau.h"

#include <iostream>

/**
 * @brief An example of the use of the decasteljau algorithm.
 */

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    std::cout << "Usage: .se2_decateljau <k> <d> <p>\n";
    std::cout << "\t with k: number of initial points on the 8-shaped curve.\n";
    std::cout << "\t with d: degree of smoothness of the generated curve.\n";
    std::cout << "\t with p: number of points to generate between consecutive points of the initial curve.\n";
    return 0;
  }

  double n_k_pts, degree, n_pts;

  n_k_pts = atof(argv[1]);
  degree  = atof(argv[2]);
  n_pts   = atof(argv[3]);

  const auto points = manif::generateSE2PointsOnHeightShape(n_k_pts);

  std::cout << "Initial points:\n";
  for (const auto& p : points)
    std::cout << p.x() << ","
              << p.y() << ","
              << p.angle() << "\n";

  std::cout << "Generated points:\n";
  const auto curve = manif::decasteljau(points, degree, n_pts, true);

  for (const auto& p : curve)
    std::cout << p.x() << ","
              << p.y() << ","
              << p.angle() << "\n";

  return 0;
}
