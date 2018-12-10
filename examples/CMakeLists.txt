add_executable(se2_interpolation se2_interpolation.cpp)
add_executable(se2_decasteljau se2_DeCasteljau.cpp)
add_executable(se2_cn_smooth se2_cn_smooth.cpp)
add_executable(se2_localization se2_localization.cpp)

set(CXX_11_EXAMPLE_TARGETS

  # SE2
  se2_interpolation
  se2_decasteljau
  se2_cn_smooth
  se2_localization
  
)

find_package(Ceres QUIET)

if(CERES_FOUND)

  message(STATUS "Found ceres_solver, building ceres examples.")

#  add_executable(none none.cpp)

#  set(CXX_11_CERES_EXAMPLE_TARGETS
#    # Ceres

#  )

#  foreach(target ${CXX_11_CERES_EXAMPLE_TARGETS})
#    target_link_libraries(${target} ${CERES_LIBRARIES})
#    target_include_directories(${target} SYSTEM PRIVATE ${CERES_INCLUDE_DIRS})
#  endforeach()

#  list(APPEND CXX_11_EXAMPLE_TARGETS ${CXX_11_CERES_EXAMPLE_TARGETS})

else()
  message(STATUS "Could not find ceres_solver, ceres examples will not be built.")
endif()

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
