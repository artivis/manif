find_package(benchmark QUIET)

if(NOT benchmark_FOUND)
  # If not found, download it
  include(FetchContent)
  FetchContent_Declare(
    googlebenchmark
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG        v1.5.2
  )
  set(BENCHMARK_ENABLE_TESTING OFF)
  FetchContent_MakeAvailable(googlebenchmark)
endif()

# small helper function
function(manif_add_benchmark target)
  add_executable(${target} ${ARGN})
  # add_dependencies(${target} gtest)
  target_link_libraries(${target} ${PROJECT_NAME} benchmark::benchmark)

  if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    # GCC is not strict enough by default, so enable most of the warnings.
    target_compile_options(${target} PRIVATE
      -Werror=all
      -Werror=extra
      -Wno-unknown-pragmas
      -Wno-sign-compare
      -Wno-unused-parameter
      -Wno-missing-field-initializers
      # When the library is built using GCC it is necessary to link
      # with the pthread library due to how GCC implements std::thread.
      # See https://github.com/google/benchmark#platform-specific-build-instructions
      -pthread
    )
  endif()
endfunction()

manif_add_benchmark(benchmark_so2 benchmark_so2.cpp)
manif_add_benchmark(benchmark_se2 benchmark_se2.cpp)
manif_add_benchmark(benchmark_so3 benchmark_so3.cpp)
manif_add_benchmark(benchmark_se3 benchmark_se3.cpp)
manif_add_benchmark(benchmark_se_2_3 benchmark_se_2_3.cpp)
manif_add_benchmark(benchmark_rn benchmark_rn.cpp)

manif_add_benchmark(benchmark_manif benchmark_manif.cpp)

manif_add_benchmark(benchmark_quat benchmark_quat.cpp)

set(CXX_17_TEST_TARGETS
  benchmark_so2
  benchmark_se2
  benchmark_so3
  benchmark_se3
  benchmark_se_2_3
  benchmark_rn

  benchmark_manif

  benchmark_quat
)

# Set required C++17 flag
set_property(TARGET ${CXX_17_TEST_TARGETS} PROPERTY CXX_STANDARD 17)
set_property(TARGET ${CXX_17_TEST_TARGETS} PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET ${CXX_17_TEST_TARGETS} PROPERTY CXX_EXTENSIONS OFF)
