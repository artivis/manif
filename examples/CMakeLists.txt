add_executable(se2_interpolation se2_interpolation.cpp)
add_executable(se2_decasteljau se2_DeCasteljau.cpp)
add_executable(se2_cn_smooth se2_cn_smooth.cpp)

add_executable(se2_smooth_trajectory se2_smooth_trajectory.cpp)
target_link_libraries(se2_smooth_trajectory ${CERES_LIBRARIES})

set(CXX_11_EXAMPLE_TARGETS

  # SE2
  se2_interpolation
  se2_decasteljau
  se2_cn_smooth

  # Ceres
  se2_smooth_trajectory

)

# Link to manif
foreach(target ${CXX_11_EXAMPLE_TARGETS})
  target_link_libraries(${target} ${PROJECT_NAME})
endforeach()

# Set required C++11 flag
if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(${CXX_11_EXAMPLE_TARGETS} PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET ${CXX_11_EXAMPLE_TARGETS} PROPERTY CXX_STANDARD 11)
  set_property(TARGET ${CXX_11_EXAMPLE_TARGETS} PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET ${CXX_11_EXAMPLE_TARGETS} PROPERTY CXX_EXTENSIONS OFF)
endif()
