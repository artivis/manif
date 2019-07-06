#include "manif/SO2.h"
#include "manif/algorithms/average.h"

#include <vector>
#include <iostream>

/**
 * @brief An example of the use of the average biinvariant algorithm.
 */

int main(int argc, char** argv)
{
  if (argc > 1)
  {
    std::cout << "Usage: .so2_average\n";
  }

  // Generate 4 points around PI/2

  std::vector<manif::SO2d> points;

  points.emplace_back(   MANIF_PI/4.);
  points.emplace_back(3.*MANIF_PI/8.);
  points.emplace_back(5.*MANIF_PI/8.);
  points.emplace_back(3.*MANIF_PI/4.);

  std::cout << "Initial points:\n";
  for (const auto& p : points)
    std::cout << p.angle() << "\n";

  auto average = manif::average_biinvariant(points);

  std::cout << "Average point:\n";
  std::cout << average.angle() << "\n";

  return 0;
}
