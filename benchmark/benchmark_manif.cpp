#include "common_benchmark.h"

#include <manif/manif.h>

using namespace manif;

MANIF_BENCHMARK(R5d)
MANIF_BENCHMARK(SO2d)
MANIF_BENCHMARK(SE2d)
MANIF_BENCHMARK(SO3d)
MANIF_BENCHMARK(SE3d)
MANIF_BENCHMARK(SE_2_3d)

BENCHMARK_MAIN();