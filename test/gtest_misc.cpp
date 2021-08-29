#include "manif/algorithms/interpolation.h"

#include "common_tester.h"

using namespace manif;

TEST(TEST_IPOW, TEST_IPOW)
{
  for (int v=0; v<25; ++v){
    for (int p=0; p<7; ++p) {
      EXPECT_EQ(std::pow(v, p), cpow<int>(v, p));
    }
  }
}

int lbinomialCoeff(int n, int k)
{
  if (k > n) return 0;
  if (k == 0 || k == n) return 1;

  return lbinomialCoeff(n - 1, k - 1) + lbinomialCoeff(n - 1, k);
}

TEST(TEST_BINOMIAL_COEFF, TEST_BINOMIAL_COEFF)
{
  EXPECT_THROW(
    binomialCoeff(1, -1),
    manif::invalid_argument
  );

  EXPECT_THROW(
    binomialCoeff(1, 2),
    manif::invalid_argument
  );

  for (int k=0; k<25; ++k){
    for (int n=k; n<25; ++n) {
      EXPECT_NO_THROW(
        EXPECT_EQ(lbinomialCoeff(n, k), binomialCoeff(n, k))
      ) << n, k;
    }
  }
}

TEST(TEST_BERNSTEIN, TEST_BERNSTEIN)
{
  EXPECT_THROW(
    polynomialBernstein(1, -1, 1),
    manif::invalid_argument
  );

  EXPECT_THROW(
    polynomialBernstein(1, 2, 1),
    manif::invalid_argument
  );

  for (int k=0; k<25; ++k){
    for (int n=k; n<25; ++n) {
      for (double t=0; t<1; t+=0.03) {
        EXPECT_NO_THROW(
          EXPECT_NEAR(
            lbinomialCoeff(n, k) * std::pow(1.-t, n-k) * std::pow(t, k),
            polynomialBernstein((double)n, (double)k, t),
            1e-5
          )
        ) << n << k << t;
      }
    }
  }
}

TEST(TEST_SPHI, TEST_SPHI)
{
  EXPECT_THROW(sphi(0, 0), std::logic_error);
  EXPECT_THROW(sphi(0, 5), std::logic_error);

  for (int i=1; i<5; ++i)
  {
    EXPECT_NEAR(0, sphi(0., i), 1e-10);
    EXPECT_NEAR(1, sphi(1., i), 1e-10);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
