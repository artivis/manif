#include "manif/SE2.h"
#include "manif/algorithms/interpolation.h"

#include <vector>
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

template <typename T>
constexpr T ipow(const T base, const int exp, T carry = 1) {
  return exp < 1 ? carry : ipow(base*base, exp/2, (exp % 2) ? carry*base : carry);
}

template <typename T>
constexpr T polynomialBernstein(const T n, const T i, const T t)
{
  return binomial_coefficient(n, i) * ipow(T(1)-t, n-i) * ipow(t,i);
}

template <typename Manifold>
std::vector<typename Manifold::Manifold>
computeBezierCurve(const std::vector<Manifold>& control_points,
                   const unsigned int degree,
                   const unsigned int k_interp)
{
  MANIF_CHECK(control_points.size() > 2, "Oups0");
  MANIF_CHECK(degree <= control_points.size(), "Oups1");
  MANIF_CHECK(k_interp > 0, "Oups2");

  bool verbose = false;

  if (verbose)
  std::cout << "Entering Bezier with : \n"
            << "\tcontrol_points : " << control_points.size() << "\n"
            << "\tdegree : " << degree << "\n"
            << "\tk_interp : " << k_interp << "\n";

  const unsigned int n_segments =
      std::floor(double(control_points.size()-degree)/(degree-1)+1);

  if (verbose)
  std::cout << "Will compute " << n_segments << " segments.\n";

  std::vector<std::vector<const Manifold*>> segments_control_points;
  for (unsigned int t=0; t<n_segments; ++t)
  {
    if (verbose)
    std::cout << "Computing segment " << t << " "
              << "from control points : ";

    segments_control_points.emplace_back(std::vector<const Manifold*>());

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
  std::vector<Manifold> curve;
  for (unsigned int s=0; s<segments_control_points.size(); ++s)
  {
    for (int t=1; t<=segment_k_interp; ++t)
    {
      // t in [0,1]
      const double t_01 = static_cast<double>(t)/(segment_k_interp);

      Manifold Qc = Manifold::Identity();

      // recursive chunk of the algo,
      // compute tmp control points.
      for (int i=0; i<degree-1; ++i)
      {
        if (verbose)
          std::cout << "Control point " << i
                    << " has polynomial "
                    << polynomialBernstein((double)degree, (double)i, (double)t_01)
                    << "\n";

        Qc = Qc.lplus(segments_control_points[s][i]->lift() *
                      polynomialBernstein((double)degree, (double)i, (double)t_01));
      }

      curve.push_back(Qc);
    }
  }

////      for (int i=0; i<degree/*-1*/; ++i)
////      {
////        if (verbose)
////        std::cout << "Control point " << i
////                  << " has polynomial "
////                  << polynomialBernstein((double)degree, (double)i, (double)t_01)
////                  << "\n";

////        interp += segment_control_points[i]->lift() *
////                  polynomialBernstein((double)degree, (double)i, (double)t_01);

//////        interp += segment_control_points[i+1]->rminus(*segment_control_points[i]) *
//////                  polynomialBernstein((double)degree, (double)i, (double)t_01);
////      }


  return curve;
}


void twoPointsInterp(const manif::INTERP_METHOD interp_method,
                     const double n_pts)
{
  std::cout << 2 << ","
            << n_pts << ","
            << static_cast<typename std::underlying_type<manif::INTERP_METHOD>::type>(interp_method)   << "\n";

  manif::SE2d state(0,0,M_PI/2.),
              state_other(2,0,0);

  std::cout << state.x() << ","
            << state.y() << ","
            << state.angle() << "\n";

  std::cout << state_other.x() << ","
            << state_other.y() << ","
            << state_other.angle() << "\n";

  manif::SE2d interp;

  // Initial Point

  interp = interpolate(state, state_other,
                       0, interp_method);

  std::cout << interp.x() << ","
            << interp.y() << ","
            << interp.angle() << "\n";

  // Interpolated Points

  for (double i=1; i<n_pts; ++i)
  {
    interp = interpolate(state, state_other,
                         i/n_pts, interp_method);

    std::cout << interp.x() << ","
              << interp.y() << ","
              << interp.angle() << "\n";
  }

  // Final Point

  interp = interpolate(state, state_other,
                       1, interp_method);

  std::cout << interp.x() << ","
            << interp.y() << ","
            << interp.angle() << "\n";
}

void fivePointsInterp(const manif::INTERP_METHOD interp_method,
                      const double n_pts)
{
  std::cout << 5 << "," << n_pts << "," << 0 << "\n";

  std::vector<manif::SE2d> states;
  states.reserve(5);

  states.emplace_back(0  ,0  , M_PI/2.);
  states.emplace_back(2.5,3.5,-M_PI/4.);
  states.emplace_back(4  ,2.5, 0);
  states.emplace_back(9  ,  1,-M_PI/2.);
  states.emplace_back(6  , -1,-M_PI);

  for (const auto& p : states)
    std::cout << p.x() << ","
              << p.y() << ","
              << p.angle() << "\n";

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
}

void heightShapeInterp(const manif::INTERP_METHOD interp_method,
                       const double n_k_pts,
                       const double n_pts)
{
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

//  Close the loop

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
}

void heightShapeBezier(const double degree,
                       const double n_k_pts,
                       const double n_pts)
{
  std::cout << n_k_pts << ","
            << n_pts << ","
            << 0   << std::endl;

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


  const auto curve = computeBezierCurve(states, degree, n_pts);

  for (const auto& p : curve)
    std::cout << p.x() << ","
              << p.y() << ","
              << p.angle() << "\n";
}

int main(int argc, char** argv)
{
  manif::INTERP_METHOD interp_method =
      manif::INTERP_METHOD::SLERP;

  (void)interp_method;

  int selected = 0;

  if (argc >= 2)
  {
    selected = atoi(argv[1]);

    switch (selected) {
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
      interp_method = manif::INTERP_METHOD::SLERP;
      break;
    }
  }

  double n_k_pts = 10;

  if (argc >= 3)
  {
    n_k_pts = atof(argv[2]);
  }
  (void)n_k_pts;

  double n_pts = 10;

  if (argc >= 4)
  {
    n_pts = atof(argv[3]);
  }

//  std::cout << 3 << ","
//            << n_pts << ","
//            << selected << std::endl;

  twoPointsInterp(interp_method, n_pts);
//  fivePointsInterp(interp_method, n_pts);
//  heightShapeInterp(interp_method, n_k_pts, n_pts);

//  heightShapeBezier(5, n_k_pts, n_pts);

//  std::cout << 3 << ","
//            << n_pts << ","
//            << 0   << std::endl;

//  // Generate 3 points
//  std::vector<manif::SE2d> states;
//  states.emplace_back(0,0,-M_PI/2);
//  states.emplace_back(0,2,-M_PI/4);
//  states.emplace_back(2,2,0);

//  for (const auto& p : states)
//    std::cout << p.x() << ","
//              << p.y() << ","
//              << p.angle() << "\n";

////  const auto curve = computeBezierCurve(states, 3, n_pts);
//  const auto curve = computeBezierCurve(states, 3, n_pts);

//  for (const auto& p : curve)
//    std::cout << p.x() << ","
//              << p.y() << ","
//              << p.angle() << "\n";

  return 0;
}
