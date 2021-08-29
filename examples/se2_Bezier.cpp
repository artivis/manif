#include "se2_points_generator.h"
#include "manif/algorithms/bezier.h"

#include <iostream>

/**
 * @brief An example of the use of the decasteljau algorithm.
 */
int main(int argc, char** argv)
{
  if (argc < 4)
  {
    std::cout << "Usage: .se2_bezier <k> <d> <p>\n";
    std::cout << "\t with k: number of initial points on the 8-shaped curve.\n";
    std::cout << "\t with d: degree of smoothness of the generated curve.\n";
    std::cout << "\t with p: number of points to generate between consecutive points of the initial curve.\n";
    return EXIT_SUCCESS;
  }

  int k, d, p;

  k = atoi(argv[1]);
  d = atoi(argv[2]);
  p = atoi(argv[3]);

  std::cout << k << "," << d << "," << p << "\n";

  const auto points = manif::generateSE2PointsOnHeightShape(k);

  // std::cout << "Initial points:\n";
  for (const auto& point : points)
    std::cout << point.x() << ","
              << point.y() << ","
              << point.angle() << "\n";

  // std::cout << "Generated points:\n";
  const auto curve = manif::bezier(points, d, p);

  for (const auto& point : curve)
    std::cout << point.x() << ","
              << point.y() << ","
              << point.angle() << "\n";

  return EXIT_SUCCESS;
}