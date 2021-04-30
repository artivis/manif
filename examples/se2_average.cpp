#include "manif/SE2.h"
#include "manif/algorithms/average.h"

#include<Eigen/StdVector>

#include <vector>
#include <iostream>

/**
 * @brief An example of the use of the average algorithms.
 */

int main(int /*argc*/, char** /*argv*/)
{
  // Generate 4 'close' points

  std::vector<manif::SE2d, Eigen::aligned_allocator<manif::SE2d>> points;

//  points.emplace_back(1, 1, 3.*MANIF_PI/4.);
//  points.emplace_back(1, 3, 5.*MANIF_PI/8.);
//  points.emplace_back(3, 1,    MANIF_PI/4.);
//  points.emplace_back(3, 3, 3.*MANIF_PI/8.);

  points.emplace_back(-std::sqrt(2.)/2.,  std::sqrt(2.)/2.,  MANIF_PI/4.);
  points.emplace_back( std::sqrt(2.),     0.,                0.);
  points.emplace_back(-std::sqrt(2.)/2., -std::sqrt(2.)/2., -MANIF_PI/4.);

  std::cout << "Initial points:\n";
  for (const auto& p : points)
    std::cout << p.x() << ","
              << p.y() << ","
              << p.angle() << "\n";
  std::cout << "\n";

  auto average = manif::average_biinvariant(points, 1e-9, 100);

  std::cout << "Average biinvariant:\n";
  std::cout << average.x() << ","
            << average.y() << ","
            << average.angle() << "\n\n";

  std::cout << "Approximate expected average:\n";
  std::cout << "0.2171, 0, 0\n\n";

//  average = manif::average_frechet_left(points, 1e-9, 100);

//  std::cout << "Average Frechet left:\n";
//  std::cout << average.x() << ","
//            << average.y() << ","
//            << average.angle() << "\n\n";

//  average = manif::average_frechet_right(points, 1e-9, 100);

//  std::cout << "Average Frechet right:\n";
//  std::cout << average.x() << ","
//            << average.y() << ","
//            << average.angle() << "\n\n";

  return EXIT_SUCCESS;
}
