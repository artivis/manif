# SO3 tests

if(CMAKE_SIZEOF_VOID_P EQUAL 4)
  add_definitions(-DMANIF_ARCH_32)
endif()

manif_add_gtest(gtest_rn gtest_rn.cpp)

set(CXX_14_TEST_TARGETS

  ${CXX_14_TEST_TARGETS}

  # R^n
  gtest_rn

  PARENT_SCOPE
)
