#include "manif/manif.h"

#include <autodiff/reverse/var.hpp>
#include <autodiff/reverse/var/eigen.hpp>

#include "gtest_common_tester_autodiff.h"

// TEST(autodiff, autodiff_atan2) {

//   auto f = [](const dual& x, const dual& y) -> dual {
//     return atan2(x, y);
//   };

//   dual x{1}, y{0};
//   dual t = f(x, y);

//   double dtdx = derivative(f, wrt(x), at(x, y));
//   double dtdy = derivative(f, wrt(y), at(x, y));

//   std::cout << "x = " << x << "\n";
//   std::cout << "y = " << y << "\n";
//   std::cout << "dtdx = " << dtdx << "\n";
//   std::cout << "dtdy = " << dtdy << "\n";
// }

// TEST(autodiff, autodiff_atan2_real) {

//   auto f = [](const real& x, const real& y) -> real {
//     return atan2(x, y);
//   };

//   real x{1}, y{0};
//   real t = f(x, y);

//   double dtdx = derivative(f, wrt(x), at(x, y));
//   double dtdy = derivative(f, wrt(y), at(x, y));

//   std::cout << "x = " << x << "\n";
//   std::cout << "y = " << y << "\n";
//   std::cout << "dtdx = " << dtdx << "\n";
//   std::cout << "dtdy = " << dtdy << "\n";
// }


// template <typename T> using rreal = autodiff::Real<1, T>;
// template <typename T> using ddual = autodiff::HigherOrderDual<1, T>;
template <typename T> using vvar = autodiff::Variable<T>;

#define __MANIF_MAKE_TEST_AUTODIFF(manifold, type, ad)       \
  using manifold##type = manifold<type>;    \
  using ad##type##manifold = ad<type>;    \
  MANIF_TEST_AUTODIFF(manifold##type, ad##type##manifold)

#define __MANIF_MAKE_TEST_AUTODIFF_ALL_AD(manifold, type)   \
  __MANIF_MAKE_TEST_AUTODIFF(manifold, type, vvar)

  // @todo real seem to have a few issues
  //__MANIF_MAKE_TEST_AUTODIFF(manifold, type, rreal)

#define __MANIF_MAKE_TEST_AUTODIFF_ALL_TYPES(manifold)  \
  __MANIF_MAKE_TEST_AUTODIFF_ALL_AD(manifold, double)

  // @todo float test is too flaky (in [1e-1, 1e-6])
  //__MANIF_MAKE_TEST_AUTODIFF_ALL_AD(manifold, float)

#define MANIF_TEST_AUTODIFF_ALL                 \
  using namespace autodiff;                     \
  using namespace manif;                        \
  __MANIF_MAKE_TEST_AUTODIFF_ALL_TYPES(R2)      \
  __MANIF_MAKE_TEST_AUTODIFF_ALL_TYPES(R5)      \
  __MANIF_MAKE_TEST_AUTODIFF_ALL_TYPES(SO2)     \
  __MANIF_MAKE_TEST_AUTODIFF_ALL_TYPES(SO3)     \
  __MANIF_MAKE_TEST_AUTODIFF_ALL_TYPES(SE2)     \
  __MANIF_MAKE_TEST_AUTODIFF_ALL_TYPES(SE3)     \
  __MANIF_MAKE_TEST_AUTODIFF_ALL_TYPES(SE_2_3)

MANIF_TEST_AUTODIFF_ALL;

MANIF_RUN_ALL_TEST;
