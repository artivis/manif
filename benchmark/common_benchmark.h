#ifndef _MANIF_BENCHMARK_COMMON_BENCHMARK_H_
#define _MANIF_BENCHMARK_COMMON_BENCHMARK_H_

#include <benchmark/benchmark.h>

#define MANIF_BENCHMARK(manifold)                                 \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_INVERSE, manifold            \
  )(benchmark::State& st) { for (auto _ : st) evalInverse(); }    \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_INVERSE_JAC, manifold        \
  )(benchmark::State& st) { for (auto _ : st) evalInverseJac(); } \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_RPLUS, manifold              \
  )(benchmark::State& st) { for (auto _ : st) evalRplus(); }      \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_RPLUS_JAC, manifold          \
  )(benchmark::State& st) { for (auto _ : st) evalRplusJac(); }   \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_LPLUS, manifold              \
  )(benchmark::State& st) { for (auto _ : st) evalLplus(); }      \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_LPLUS_JAC, manifold          \
  )(benchmark::State& st) { for (auto _ : st) evalLplusJac(); }   \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_RMINUS, manifold             \
  )(benchmark::State& st) { for (auto _ : st) evalRminus(); }     \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_RMINUS_JAC, manifold         \
  )(benchmark::State& st) { for (auto _ : st) evalRminusJac(); }  \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_LMINUS, manifold             \
  )(benchmark::State& st) { for (auto _ : st) evalLminus(); }     \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_LMINUS_JAC, manifold         \
  )(benchmark::State& st) { for (auto _ : st) evalLminusJac(); }  \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_COMPOSE, manifold            \
  )(benchmark::State& st) { for (auto _ : st) evalCompose(); }    \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_COMPOSE_JAC, manifold        \
  )(benchmark::State& st) { for (auto _ : st) evalComposeJac(); } \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_BETWEEN, manifold            \
  )(benchmark::State& st) { for (auto _ : st) evalBetween(); }    \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_BETWEEN_JAC, manifold        \
  )(benchmark::State& st) { for (auto _ : st) evalBetweenJac(); } \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_ACT, manifold                \
  )(benchmark::State& st) { for (auto _ : st) evalAct(); }        \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_ACT_JAC, manifold            \
  )(benchmark::State& st) { for (auto _ : st) evalActJac(); }     \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_LOG, manifold                \
  )(benchmark::State& st) { for (auto _ : st) evalLog(); }        \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_LOG_JAC, manifold            \
  )(benchmark::State& st) { for (auto _ : st) evalLogJac(); }     \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_EXP, manifold                \
  )(benchmark::State& st) { for (auto _ : st) evalExp(); }        \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_EXP_JAC, manifold            \
  )(benchmark::State& st) { for (auto _ : st) evalExpJac(); }     \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_HAT, manifold                \
  )(benchmark::State& st) { for (auto _ : st) evalHat(); }        \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_ADJ, manifold                \
  )(benchmark::State& st) { for (auto _ : st) evalAdj(); }        \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_SMALL_ADJ, manifold          \
  )(benchmark::State& st) { for (auto _ : st) evalSmallAdj(); }   \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_INNER, manifold              \
  )(benchmark::State& st) { for (auto _ : st) evalInner(); }      \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_RJAC, manifold               \
  )(benchmark::State& st) { for (auto _ : st) evalRJac(); }       \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_RJACINV, manifold            \
  )(benchmark::State& st) { for (auto _ : st) evalRJacInv(); }    \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_LJAC, manifold               \
  )(benchmark::State& st) { for (auto _ : st) evalLJac(); }       \
  BENCHMARK_TEMPLATE_F(                                           \
    CommonBenchmark, BM_##manifold##_LJACINV, manifold            \
  )(benchmark::State& st) { for (auto _ : st) evalLJacInv(); }

template <typename _LieGroup>
class CommonBenchmark : public benchmark::Fixture {

  using LieGroup = _LieGroup;
  using Scalar = typename LieGroup::Scalar;
  using Tangent = typename LieGroup::Tangent;
  using Jacobian = typename LieGroup::Jacobian;
  using LieAlg = typename Tangent::LieAlg;
  using Point = typename LieGroup::Vector;

public:

  void SetUp(const benchmark::State&) override {

    // Maybe is it better to compare the same seed here.
    // std::srand((unsigned int) time(0));

    state       = LieGroup::Random();
    state_other = LieGroup::Random();

    delta = Tangent::Random();
    delta_other = Tangent::Random();

    pa.setRandom();
  }

  void evalInverse() {
    state_new = state.inverse();
  }

  void evalInverseJac() {
    state_new = state.inverse(Ja);
  }

  void evalRplus() {
    state_new = state.rplus(delta);
  }

  void evalRplusJac() {
    state_new = state.rplus(delta, Ja, Jb);
  }

  void evalLplus() {
    state_new = state.lplus(delta);
  }

  void evalLplusJac() {
    state_new = state.lplus(delta, Ja, Jb);
  }

  void evalRminus() {
    delta_new = state.rminus(state_other);
  }

  void evalRminusJac() {
    delta_new = state.rminus(state_other, Ja, Jb);
  }

  void evalLminus() {
    delta_new = state.lminus(state_other);
  }

  void evalLminusJac() {
    delta_new = state.lminus(state_other, Ja, Jb);
  }

  void evalCompose() {
    state_new = state.compose(state_other);
  }

  void evalComposeJac() {
    state_new = state.compose(state_other, Ja, Jb);
  }

  void evalBetween() {
    state_new = state.between(state_other);
  }

  void evalBetweenJac() {
    state_new = state.between(state_other, Ja, Jb);
  }

  void evalAct() {
    pb = state.act(pa);
  }

  void evalActJac() {
    // pb = state.act(pa, Ja, Jb);
  }

  void evalLog() {
    delta_new = state.log();
  }

  void evalLogJac() {
    delta_new = state.log(Ja);
  }

  void evalExp() {
    state_new = delta.exp();
  }

  void evalExpJac() {
    state_new = delta.exp(Ja);
  }

  void evalHat() {
    hat = delta.hat();
  }

  void evalAdj() {
    Ja = state.adj();
  }

  void evalSmallAdj() {
    Ja = delta.smallAdj();
  }

  void evalInner() {
    inner = delta.inner(delta_other);
  }

  void evalRJac() {
    Ja = delta.rjac();
  }

  void evalRJacInv() {
    Ja = delta.rjacinv();
  }

  void evalLJac() {
    Ja = delta.ljac();
  }

  void evalLJacInv() {
    Ja = delta.ljacinv();
  }

protected:

  Scalar inner;

  LieGroup state;
  LieGroup state_other;
  LieGroup state_new;

  Tangent  delta;
  Tangent  delta_other;
  Tangent  delta_new;

  Point pa, pb;

  Jacobian Ja, Jb;
  LieAlg hat;
};

#endif // _MANIF_BENCHMARK_COMMON_BENCHMARK_H_