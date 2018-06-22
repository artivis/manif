#include "manif/SE2.h"
#include "manif/interpolation.h"

#include <iostream>

template <typename T>
constexpr T binomial_coefficient(const T n, const T k)
{
  return (n >= k) ? (k >= 0) ?
  (k*2 > n) ? binomial_coefficient(n, n-k) :
                     k ? binomial_coefficient(n, k - 1) * (n - k + 1) / k : 1
  // assert n ≥ k ≥ 0
  : (throw std::logic_error("k >= 0 !")) : (throw std::logic_error("n >= k !"));
}

template <typename Manifold>
std::vector<typename Manifold::Manifold>
computeBezierCurve(const std::vector<Manifold>& control_points,
                   const int degree)
{
  MANIF_CHECK(degree <= control_points, "Oups");

  return std::vector<typename Manifold::Manifold>();
}


int main(int argc, char** argv)
{
  manif::INTERP_METHOD interp_method =
      manif::INTERP_METHOD::SLERP;

  if (argc >= 2)
  {
    int selected = atoi(argv[1]);

    switch (selected) {
    case 0:
      interp_method = manif::INTERP_METHOD::SLERP;
      break;
    case 1:
      interp_method = manif::INTERP_METHOD::CUBIC;
      break;
    case 2:
      interp_method = manif::INTERP_METHOD::TWOSTEPS;
      break;
    default:
      interp_method = manif::INTERP_METHOD::SLERP;
      break;
    }
  }

  double n_k_pts = 10;

  if (argc >= 3)
  {
    n_k_pts = atof(argv[2]);
  }

  double n_pts = 10;

  if (argc >= 4)
  {
    n_pts = atof(argv[3]);
  }

  std::cout << n_k_pts << ","
            << n_pts << ","
            << 0   << "\n";

  // Generate some k points on 8-shaped curve

  std::vector<manif::SE2d> states;
  states.reserve(n_k_pts);

  const double x = std::cos(0);
  const double y = std::sin(0)/2;
  states.emplace_back(x,y,M_PI/2);

  double t = 0;
  for (double i=1; i<n_k_pts; ++i)
  {
    t += M_PI*2. / n_k_pts;

    const double x = std::cos(t);
    const double y = std::sin(2*t) / 2;

    const double t = std::atan2(y-states.back().y(),
                                x-states.back().x());

    states.emplace_back(x,y,t);

    std::cout << x << ","
              << y << ","
              << t << "\n";
  }


//  std::cout << 5 << ","
//            << n_pts << ","
//            << 0   << "\n";

//  states.emplace_back(0  ,0  , M_PI/2.);
//  states.emplace_back(2.5,3.5,-M_PI/4.);
//  states.emplace_back(4  ,2.5, 0);
//  states.emplace_back(9  ,  1,-M_PI/2.);
//  states.emplace_back(6  , -1,-M_PI);

//  for (const auto& p : states)
//    std::cout << p.x() << ","
//              << p.y() << ","
//              << p.angle() << "\n";

  // Interpolate between k-points

  manif::SE2d interp;

  for (int i=0; i<states.size()-1; ++i)
  {
    const manif::SE2d& s0 = states[i];
    const manif::SE2d& s1 = states[i+1];

    std::cout << s0.x() << ","
              << s0.y() << ","
              << s0.angle() << "\n";

    for (int j=1; j<=n_pts; ++j)
    {
      interp = interpolate(s0, s1,
                           static_cast<double>(j)/(n_pts+1),
                           interp_method);

      std::cout << interp.x() << ","
                << interp.y() << ","
                << interp.angle() << "\n";
    }

    std::cout << s1.x() << ","
              << s1.y() << ","
              << s1.angle() << "\n";
  }

  // Close the loop

  const manif::SE2d& s0 = states.back();
  const manif::SE2d& s1 = states[0];

  std::cout << s0.x() << ","
            << s0.y() << ","
            << s0.angle() << "\n";

  for (int j=1; j<=n_pts; ++j)
  {
    interp = interpolate(s0, s1,
                         static_cast<double>(j)/(n_pts+1),
                         interp_method);

    std::cout << interp.x() << ","
              << interp.y() << ","
              << interp.angle() << "\n";
  }

  std::cout << s1.x() << ","
            << s1.y() << ","
            << s1.angle() << "\n";

//  manif::SE2d state(0,0,M_PI/2.),
//              state_other(2,0,0);

//  manif::SE2d interp;

//  // Initial Point

//  interp = interpolate(state, state_other,
//                       0, interp_method);

//  std::cout << interp.x() << ","
//            << interp.y() << ","
//            << interp.angle() << "\n";

//  // Interpolated Points

//  for (double i=1; i<n_pts; ++i)
//  {
//    interp = interpolate(state, state_other,
//                         i/n_pts, interp_method);

//    std::cout << interp.x() << ","
//              << interp.y() << ","
//              << interp.angle() << "\n";
//  }

//  // Final Point

//  interp = interpolate(state, state_other,
//                       1, interp_method);

//  std::cout << interp.x() << ","
//            << interp.y() << ","
//            << interp.angle() << "\n";

  return 0;
}
