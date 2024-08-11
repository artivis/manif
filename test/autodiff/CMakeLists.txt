# autodiff tests

if(NOT MSVC)
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
  if(COMPILER_SUPPORTS_CXX17)
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++17 support.")
  else()
    message(FATAL_ERROR
      "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support."
      "The library 'autodiff' requires a C++17 compliant compiler."
    )
  endif()
endif()

manif_add_gtest(gtest_autodiff gtest_autodiff.cpp)

set(CXX_17_TEST_TARGETS_AUTODIFF
  gtest_autodiff
)

foreach(target ${CXX_17_TEST_TARGETS_AUTODIFF})
  target_link_libraries(${target} autodiff::autodiff)
endforeach()

# Set required C++17 flag
set_property(TARGET ${CXX_17_TEST_TARGETS_AUTODIFF} PROPERTY CXX_STANDARD 17)
set_property(TARGET ${CXX_17_TEST_TARGETS_AUTODIFF} PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET ${CXX_17_TEST_TARGETS_AUTODIFF} PROPERTY CXX_EXTENSIONS OFF)
