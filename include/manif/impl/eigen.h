#ifndef _MANIF_MANIF_EIGEN_H_
#define _MANIF_MANIF_EIGEN_H_

#include <Eigen/Core>
#include <Eigen/LU> // for mat.inverse()
#include <Eigen/Geometry>

#include <manif/constants.h>

/**
 * @note static_cast<int> to avoid -Wno-enum-compare
 */

//
// Static Asserts
//

// Define some custom static_assert macros

#define static_assert_rows_dim(x, dim) \
  static_assert(static_cast<int>(std::decay<decltype(x)>::type::RowsAtCompileTime) == dim, \
                "x.rows != "#dim" .");

#define static_assert_cols_dim(x, dim) \
  static_assert(static_cast<int>(std::decay<decltype(x)>::type::ColsAtCompileTime) == dim, \
                "x.cols != "#dim" .");

#define static_assert_dim(x, rows, cols) \
  static_assert_rows_dim(x, rows); \
  static_assert_cols_dim(x, cols);

#define static_assert_dim_eq(l,r) \
  static_assert(static_cast<int>(std::decay<decltype(l)>::type::ColsAtCompileTime) == \
                static_cast<int>(std::decay<decltype(r)>::type::ColsAtCompileTime), \
                "lhs.cols != rhs.cols !"); \
  static_assert(static_cast<int>(std::decay<decltype(l)>::type::RowsAtCompileTime) == \
                static_cast<int>(std::decay<decltype(r)>::type::RowsAtCompileTime), \
                "lhs.rows != rhs.rows !");

#define static_assert_is_vector(x) \
  static_assert_cols_dim(x, 1);

#define static_assert_vector_dim(x, dim) \
  static_assert_is_vector(x); \
  static_assert_rows_dim(x, dim);

#define static_assert_is_colmajor_vector(x) \
  static_assert_rows_dim(x, 1);

#define static_assert_colmajor_vector_dim(x, dim) \
  static_assert_is_colmajor_vector(x); \
  static_assert_cols_dim(x, dim);

//
// Asserts
//

// Define some custom assert macros

#define assert_rows_dim(x, dim) \
  static_assert(static_cast<int>(std::decay<decltype(x)>::type::RowsAtCompileTime) == dim || \
                std::decay<decltype(x)>::type::RowsAtCompileTime == Eigen::Dynamic, \
                "x.rows != "#dim" ."); \
  assert(x.rows() == dim && "x.rows != "#dim" .");

#define assert_cols_dim(x, dim) \
  static_assert(static_cast<int>(std::decay<decltype(x)>::type::ColsAtCompileTime) == dim || \
                std::decay<decltype(x)>::type::ColsAtCompileTime == Eigen::Dynamic, \
                "x.cols != "#dim" ."); \
  assert(x.cols() == dim && "x.cols != "#dim" .");

#define assert_dim(x, rows, cols) \
  assert_rows_dim(x, rows); \
  assert_cols_dim(x, cols);

#define assert_dim_eq(l,r) \
  static_assert(static_cast<int>(std::decay<decltype(l)>::type::ColsAtCompileTime) == \
                static_cast<int>(std::decay<decltype(r)>::type::ColsAtCompileTime) || \
                std::decay<decltype(l)>::type::ColsAtCompileTime == Eigen::Dynamic || \
                std::decay<decltype(r)>::type::ColsAtCompileTime == Eigen::Dynamic, \
                "lhs.cols != rhs.cols !"); \
  static_assert(static_cast<int>(std::decay<decltype(l)>::type::RowsAtCompileTime) == \
                static_cast<int>(std::decay<decltype(r)>::type::RowsAtCompileTime) || \
                std::decay<decltype(l)>::type::RowsAtCompileTime == Eigen::Dynamic || \
                std::decay<decltype(r)>::type::RowsAtCompileTime == Eigen::Dynamic, \
                "lhs.rows != rhs.rows !"); \
  assert(l.rows() == r.rows() && "lhs.rows != rhs.rows !"); \
  assert(l.cols() == r.cols() && "lhs.cols != rhs.cols !"); \

#define assert_is_vector(x) \
  static_assert(std::decay<decltype(x)>::type::ColsAtCompileTime ==  1 || \
                std::decay<decltype(x)>::type::ColsAtCompileTime == Eigen::Dynamic, \
                "Expected a vector !"); \
  assert(x.cols() == 1 && "Expected a vector !"); \

#define assert_vector_dim(x, dim) \
  assert_is_vector(x); \
  assert_rows_dim(x, dim);

#define assert_is_colmajor_vector(x) \
  static_assert(std::decay<decltype(x)>::type::RowsAtCompileTime ==  1 || \
                std::decay<decltype(x)>::type::RowsAtCompileTime == Eigen::Dynamic, \
                "Expected a column-major vector !"); \
  assert(x.rows() == 1 && "Expected a column-major vector !"); \

#define assert_colmajor_vector_dim(x, dim) \
  assert_is_colmajor_vector(x); \
  assert_cols_dim(x, dim);

namespace manif {
namespace internal {

template< class Base, class Derived >
constexpr bool is_base_of_v()
{
  return std::is_base_of<Base, Derived>::value;
}

/**
 * @brief traitscast specialization that come handy when writing thing like
 * using Matrix3f = typename traitscast<Matrix3d, float>::cast;
 */
template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols, typename NewScalar>
struct traitscast<Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>, NewScalar>
{
  using cast = Eigen::Matrix<NewScalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>;
};

} /* namespace internal */

/**
 * @brief Return a 2x2 skew matrix given a scalar.
 * @note [x] = | 0 -x |
 *             | x  0 |
 */
template <typename _Scalar>
typename std::enable_if<std::is_arithmetic<_Scalar>::value || internal::is_ad<_Scalar>::value,
                        Eigen::Matrix<_Scalar, 2, 2>>::type
skew(const _Scalar v)
{
  return (Eigen::Matrix<_Scalar, 2, 2>() <<
             _Scalar(0.), -v,
             v, _Scalar(0.) ).finished();
}

/**
 * @brief Return a 3x3 skew matrix given 3-vector.
 * @note [v] = | 0     -v(2) +v(1) |
 *             | +v(2)  0    -v(0) |
 *             | -v(1) +v(0)  0    |
 */
template <typename _Derived>
typename std::enable_if<(internal::is_base_of_v<Eigen::MatrixBase<_Derived>, _Derived>()
                         && _Derived::RowsAtCompileTime == 3),
                        Eigen::Matrix<typename _Derived::Scalar, 3, 3>>::type
skew(const Eigen::MatrixBase<_Derived>& v)
{
  assert_vector_dim(v, 3);

  using T = typename _Derived::Scalar;

  return (Eigen::Matrix<T, 3, 3>() <<
             T(0.),  -v(2),   +v(1),
            +v(2),    T(0.),  -v(0),
            -v(1),   +v(0),    T(0.) ).finished();
}

template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> randPointInBall(Scalar radius)
{
  // See https://stackoverflow.com/a/5408843/9709397

  using std::acos;
  using std::sin;
  using std::cos;
  using std::cbrt;

  // random(0, 2pi)
  Scalar phi = static_cast<Scalar>(rand()) / (static_cast<Scalar>(RAND_MAX / (Scalar(2) * MANIF_PI)));
  // random(-1, 1)
  Scalar costheta = Scalar(-1) + static_cast<Scalar>(rand()) / (static_cast<Scalar>(RAND_MAX / Scalar(2)));
  // random(0, 1)
  Scalar u = static_cast<Scalar>(rand()) / static_cast<Scalar>(RAND_MAX);

  Scalar theta = acos(costheta);
  Scalar r = radius * cbrt(u);
  Scalar rsintheta = r * sin(theta);

  return Eigen::Matrix<Scalar, 3, 1>(
    rsintheta * cos(phi),
    rsintheta * sin(phi),
    r * costheta
  );
}

template <typename Scalar>
Eigen::Quaternion<Scalar> randQuat()
{
#if EIGEN_VERSION_AT_LEAST(3,3,0)

  return Eigen::Quaternion<Scalar>::UnitRandom();

#else

  // @note:
  // Quaternion::UnitRandom is not available in Eigen 3.3-beta1
  // which is the default version in Ubuntu 16.04
  // So we copy its implementation here.

  using std::sqrt;
  using std::sin;
  using std::cos;

  const Scalar u1 = Eigen::internal::random<Scalar>(0, 1),
               u2 = Eigen::internal::random<Scalar>(0, 2.*EIGEN_PI),
               u3 = Eigen::internal::random<Scalar>(0, 2.*EIGEN_PI);
  const Scalar a = sqrt(1. - u1),
               b = sqrt(u1);
  return Eigen::Quaternion<Scalar>(a * sin(u2), a * cos(u2), b * sin(u3), b * cos(u3));

#endif
}

} /* namespace manif */

#endif /* _MANIF_MANIF_EIGEN_H_ */
