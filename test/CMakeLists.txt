catkin_add_gtest(gtest_test gtest_test.cpp)
target_link_libraries(gtest_test)




set(CXX_11_TARGETS
  gtest_test
)

# Set required C++11 flag
if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(${CXX_11_TARGETS} PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET ${CXX_11_TARGETS} PROPERTY CXX_STANDARD 11)
  set_property(TARGET ${CXX_11_TARGETS} PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET ${CXX_11_TARGETS} PROPERTY CXX_EXTENSIONS OFF)
endif()
