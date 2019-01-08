#include "manif/SE2.h"
#include "manif/algorithms/average.h"

#include <vector>
#include <iostream>

/**
 * @brief An example of the use of the average biinvariant algorithm.
 */

int main(int /*argc*/, char** /*argv*/)
{
  // Generate 4 'close' points

  std::vector<manif::SE2d> points;

  points.emplace_back(1, 1, 3.*M_PI/4.);
  points.emplace_back(1, 3, 5.*M_PI/8.);
  points.emplace_back(3, 1,    M_PI/4.);
  points.emplace_back(3, 3, 3.*M_PI/8.);

  std::cout << "Initial points:\n";
  for (const auto& p : points)
    std::cout << p.x() << ","
              << p.y() << ","
              << p.angle() << "\n";

  auto average = manif::average_biinvariant(points, 1e-5);

  std::cout << "Average point:\n";
  std::cout << average.x() << ","
            << average.y() << ","
            << average.angle() << "\n";

  return EXIT_SUCCESS;
}
