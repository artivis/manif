#include <manif/manif.h>

#include <benchmark/benchmark.h>

template<typename Scalar>
void normalizeQuat(Eigen::Quaternion<Scalar>& q) {
  // see https://github.com/zarathustr/quat_norm
  q.coeffs() /= q.coeffs().cwiseAbs().maxCoeff();
  for (char j = 0; j < 3; ++j) {
    const Scalar N = q.squaredNorm();
    q.coeffs() *= (5.0 + N) / (2.0 + 4.0 * N);
  }
}

Eigen::Quaternion<double> randQuat() {
  // @note: We are using:
  // http://mathworld.wolfram.com/HyperspherePointPicking.html
  using std::sqrt;
  using Scalar = double;
  using Quaternion = Eigen::Quaternion<Scalar>;

  Scalar u1, u2;
  do {
    u1 = Eigen::internal::random<Scalar>(-1, 1),
    u2 = Eigen::internal::random<Scalar>(-1, 1);
  } while (u1 * u1 + u2 * u2 > Scalar(1));

  Scalar u3, u4, n;
  do {
    u3 = Eigen::internal::random<Scalar>(-1, 1),
    u4 = Eigen::internal::random<Scalar>(-1, 1);
    n = u3 * u3 + u4 * u4;
  } while (n > Scalar(1.0));

  const Scalar zw_factor = sqrt((Scalar(1) - u1 * u1 - u2 * u2) / n);
  return Quaternion(u1, u2, u3 * zw_factor, u4 * zw_factor);
}

// Normalization benchmark

static void BM_EigenNormalize(benchmark::State& state) {
  Eigen::Quaternion<double> q;
  // Use the underlying vector random so that
  // the quaternion isn't normalized already
  q.coeffs().setRandom();
  double norm=0;
  for (auto _ : state) {

    q.normalize();

    state.PauseTiming();
    norm += q.norm();
    q.coeffs().setRandom();
    state.ResumeTiming();
  }
  state.counters["normAvg"] = benchmark::Counter(
    norm, benchmark::Counter::kAvgIterations
  );
}
BENCHMARK(BM_EigenNormalize);

static void BM_normalizeQuat(benchmark::State& state) {
  Eigen::Quaternion<double> q;
  // Use the underlying vector random so that
  // the quaternion isn't normalized already
  q.coeffs().setRandom();
  double norm=0;
  for (auto _ : state) {

    normalizeQuat(q);

    state.PauseTiming();
    norm += q.norm();
    q.coeffs().setRandom();
    state.ResumeTiming();
  }
  state.counters["normAvg"] = benchmark::Counter(
    norm, benchmark::Counter::kAvgIterations
  );
}
BENCHMARK(BM_normalizeQuat);

// Random benchmark

static void BM_UnitRandom(benchmark::State& state) {
  Eigen::Quaternion<double> q;
  double norm=0;
  for (auto _ : state) {
    benchmark::DoNotOptimize(q = manif::randQuat<double>());

    state.PauseTiming();
    norm += q.norm();
    state.ResumeTiming();
  }
  state.counters["normAvg"] = benchmark::Counter(
    norm, benchmark::Counter::kAvgIterations
  );
}
BENCHMARK(BM_UnitRandom);

static void BM_RandQuat(benchmark::State& state) {
  Eigen::Quaternion<double> q;
  double norm=0;
  for (auto _ : state) {
    benchmark::DoNotOptimize(q = randQuat());

    state.PauseTiming();
    norm += q.norm();
    state.ResumeTiming();
  }
  state.counters["normAvg"] = benchmark::Counter(
    norm, benchmark::Counter::kAvgIterations
  );
}
BENCHMARK(BM_RandQuat);

BENCHMARK_MAIN();
