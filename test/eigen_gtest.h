#ifndef _MANIF_TEST_EIGEN_GTEST_H_
#define _MANIF_TEST_EIGEN_GTEST_H_

#include <gtest/gtest.h>
#include <Eigen/Dense>

namespace manif {
namespace detail {

template <int... I> struct int_sequence
{
  using type = int_sequence;
  using value_type = int;
  static constexpr unsigned int size() noexcept { return sizeof...(I); }
};

template <class, class, int> struct range_cat;

template <int... H, int... T, int Start>
struct range_cat<int_sequence<H...>, int_sequence<T...>, Start>
{
  using type = int_sequence<H..., Start+T...>;
};

template <int Start, unsigned int N>
struct range_ : range_cat< typename range_<Start, N / 2>::type,
                           typename range_<Start, N - N / 2>::type,
                           N / 2 > { };

template <int Start> struct range_<Start, 1> { using type = int_sequence<Start>; };
template <int Start> struct range_<Start, 0> { using type = int_sequence<>; };

template <int End>
using make_int_sequence = typename range_<0, End + 1>::type;

template<typename F, class T, template <int...I> class S, int... I>
void call_for_each(F f, const T& t, const S<I...>&)
{
  auto l = { (f(std::get<I>(/*std::forward<T>*/(t))), 0)... };
  (void)(l);
}

template<typename F, template <typename...Ts> class C, typename... Ts>
void call_for_each(F&& f, const C<Ts...>& t)
{
  call_for_each(std::forward<F>(f), t, make_int_sequence<sizeof...(Ts)-1>());
}

struct RowSizeGetter {
template <typename... Ts>
auto operator()(const Eigen::MatrixBase<Ts>&... ms)
-> decltype(std::make_tuple(ms.rows()...))
{ return std::make_tuple(ms.rows()...); }
static const char* dim() { constexpr static char dim_arr[] = "row"; return dim_arr; }
};

struct ColSizeGetter {
template <typename... Ts>
auto operator()(const Eigen::MatrixBase<Ts>&... ms)
-> decltype(std::make_tuple(ms.cols()...))
{ return std::make_tuple(ms.cols()...); }
static const char* dim() { constexpr static char dim_arr[] = "col"; return dim_arr; }
};

using EigenIndex = EIGEN_DEFAULT_DENSE_INDEX_TYPE;

} /* namespace detail */

/**
 * @brief Gtest predicate function for matrice same dim.
 * @param dim, the expected dim size
 * @param ms, N Eigen::Matrix to be tested
 * @note This function requires an extra template param helper DimGetter
 * @see detail::RowSizeGetter
 * @see detail::ColSizeGetter
 * @see isEigenMatrixDimSize
 * @see isEigenMatrixColSize
 */
template <typename DimGetter, typename... Ts>
inline ::testing::AssertionResult
isEigenMatrixDimSize(const detail::EigenIndex dim,
                     const Eigen::MatrixBase<Ts>&... ms)
{
  static_assert(sizeof...(Ts)>=1, "No matrix passed !");
  const auto sizes = DimGetter()(ms...);

  bool result = true;
  auto f = [&result, &dim](const detail::EigenIndex i){ result &= (dim == i);};

  detail::call_for_each(f, sizes);

  // cppcheck-suppress knownConditionTrueFalse
  if (!result)
  {
    std::stringstream ss;
    ss << dim;

    auto p = [&ss, &dim](const detail::EigenIndex i)
                        { ss << ((i==dim)?" == ":" != ") << i; };

    detail::call_for_each(p, sizes);

    return ::testing::AssertionFailure() << "Matrice have different "
                                         << DimGetter::dim()
                                         << " size ! " << ss.str();
  }

  return ::testing::AssertionSuccess();
}

/**
 * @brief Gtest predicate function for testing expected matrice row dim.
 * @note This is an helper function for isEigenMatrixDimSize
 */
template <typename... Ts>
inline ::testing::AssertionResult
isEigenMatrixRowSize(const detail::EigenIndex rows,
                     const Eigen::MatrixBase<Ts>&... ms)
{
  return isEigenMatrixDimSize<detail::RowSizeGetter>(rows, ms...);
}

/**
 * @brief Gtest predicate function for testing expected matrice col dim.
 * @note This is an helper function for isEigenMatrixDimSize
 */
template <typename... Ts>
inline ::testing::AssertionResult
isEigenMatrixColSize(const detail::EigenIndex cols,
                     const Eigen::MatrixBase<Ts>&... ms)
{
  return isEigenMatrixDimSize<detail::ColSizeGetter>(cols, ms...);
}

/**
 * @brief Gtest predicate function for testing N matrice have the same row size.
 */
template <typename Derived, typename... Ts>
inline ::testing::AssertionResult
isEigenMatrixSameRowSize(const Eigen::MatrixBase<Derived>& m0,
                         const Eigen::MatrixBase<Ts>&... ms)
{
  static_assert(sizeof...(Ts)>=1, "Only one matrix passed !\n"
                "Please consider using isEigenMatrixRowSize instead.");

  return isEigenMatrixRowSize(m0.rows(), ms...);
}

/**
 * @brief Gtest predicate function for testing
 * N matrice have the same col size.
 */
template <typename Derived, typename... Ts>
inline ::testing::AssertionResult
isEigenMatrixSameColSize(const Eigen::MatrixBase<Derived>& m0,
                         const Eigen::MatrixBase<Ts>&... ms)
{
  static_assert(sizeof...(Ts)>=1, "Only one matrix passed !\n"
                "Please consider using isEigenMatrixColSize instead.");

  return isEigenMatrixColSize(m0.cols(), ms...);
}

/**
 * @brief Gtest predicate function for testing
 * N matrice have the same size.
 */
template <typename Derived, typename... Ts>
inline ::testing::AssertionResult
isEigenMatrixSameSize(const Eigen::MatrixBase<Derived>& m0,
                      const Eigen::MatrixBase<Ts>&... ms)
{
  const ::testing::AssertionResult row_check =
      isEigenMatrixSameRowSize(m0, ms...);
  if (!row_check)
  {
    return row_check;
  }

  const ::testing::AssertionResult col_check =
      isEigenMatrixSameColSize(m0, ms...);
  if (!col_check)
  {
    return col_check;
  }

  return ::testing::AssertionSuccess();
}

/**
 * @brief isZero() is not very suitable for comparing vectors which have norms
 * significantly larger than 0, isApprox(), on the other hand, does not work
 * with small norms.
 * https://eigen.tuxfamily.org/dox/classEigen_1_1DenseBase.html#ae8443357b808cd393be1b51974213f9c
 */
template <class _DerivedA, class _DerivedB>
inline ::testing::AssertionResult isEigenMatrixNear(const Eigen::MatrixBase<_DerivedA>& matrix_a,
                                                    const Eigen::MatrixBase<_DerivedB>& matrix_b,
                                                    const std::string& matrix_a_name = "matrix_a",
                                                    const std::string& matrix_b_name = "matrix_b",
                                                    double tolerance = 1e-8)
{
  const ::testing::AssertionResult size_check =
      isEigenMatrixSameSize(matrix_a, matrix_b);

  if (!size_check)
  {
    return size_check;
  }

  bool result = false;

  if (std::min(matrix_a.norm(), matrix_b.norm()) < tolerance)
  {
    result = (matrix_a - matrix_b).isZero(tolerance);
  }
  else
  {
    result = (matrix_a.isApprox(matrix_b, tolerance));
  }

  return (result ? ::testing::AssertionSuccess()
                 : ::testing::AssertionFailure()
                   << matrix_a_name << " != " << matrix_b_name << "\n"
                   << matrix_a_name << ":\n" << matrix_a << "\n"
                   << matrix_b_name << ":\n" << matrix_b << "\n"
                   << "diff:\n" << (matrix_a - matrix_b) << "\n");
}

} /* namespace manif */

#define __GET_4TH_ARG(arg1,arg2,arg3,arg4, ...) arg4

#define EXPECT_EIGEN_NEAR_DEFAULT_TOL(A,B) \
  EXPECT_TRUE(manif::isEigenMatrixNear(A, B, #A, #B))

#define EXPECT_EIGEN_NEAR_TOL(A,B,tol) \
  EXPECT_TRUE(manif::isEigenMatrixNear(A, B, #A, #B, tol))

#define __EXPECT_EIGEN_NEAR_CHOOSER(...) \
  __GET_4TH_ARG(__VA_ARGS__, EXPECT_EIGEN_NEAR_TOL, \
                EXPECT_EIGEN_NEAR_DEFAULT_TOL, )

#define EXPECT_EIGEN_NEAR(...) \
  __EXPECT_EIGEN_NEAR_CHOOSER(__VA_ARGS__)(__VA_ARGS__)

#define ASSERT_EIGEN_NEAR_DEFAULT_TOL(A,B) \
  ASSERT_TRUE(manif::isEigenMatrixNear(A, B, #A, #B))

#define ASSERT_EIGEN_NEAR_TOL(A,B,tol) \
  ASSERT_TRUE(manif::isEigenMatrixNear(A, B, #A, #B, tol))

#define __ASSERT_EIGEN_NEAR_CHOOSER(...) \
  __GET_4TH_ARG(__VA_ARGS__, ASSERT_EIGEN_NEAR_TOL, \
                ASSERT_EIGEN_NEAR_DEFAULT_TOL, )

#define ASSERT_EIGEN_NEAR(...) \
  __ASSERT_EIGEN_NEAR_CHOOSER(__VA_ARGS__)(__VA_ARGS__)

/*
 * E.g

EXPECT_TRUE(isEigenMatrixSameSize(Eigen::Vector2d::Zero(),
                                  Eigen::Vector2d::Zero(),
                                  Eigen::Vector3d::Zero(),
                                  Eigen::Vector4d::Zero()));
*/

#endif /* _MANIF_TEST_EIGEN_GTEST_H_ */
