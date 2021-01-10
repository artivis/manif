set(PYBIND11_CPP_STANDARD -std=c++11)
pybind11_add_module(manifpy

  bindings_eigen_quaternion.cpp

  bindings_rn.cpp
  bindings_so2.cpp
  bindings_so3.cpp
  bindings_se2.cpp
  bindings_se3.cpp
  bindings_se_2_3.cpp
  bindings_manif.cpp
)
target_link_libraries(manifpy PRIVATE ${PROJECT_NAME})
# Eigen and numpy have different default storage order.
# See e.g. https://pybind11.readthedocs.io/en/stable/advanced/cast/eigen.html#storage-orders
target_compile_definitions(manifpy PRIVATE EIGEN_DEFAULT_TO_ROW_MAJOR)

# Set required C++11 flag
if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(manifpy PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET manifpy PROPERTY CXX_STANDARD 11)
  set_property(TARGET manifpy PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET manifpy PROPERTY CXX_EXTENSIONS OFF)
endif()
