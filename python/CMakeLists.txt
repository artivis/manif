set(PYBIND11_CPP_STANDARD -std=c++11)
pybind11_add_module(PyManif
  bindings_so2.cpp
  bindings_so3.cpp
  bindings_se2.cpp
  bindings_se3.cpp
  bindings_manif.cpp
)
target_link_libraries(PyManif PRIVATE ${PROJECT_NAME})

message(WARNING ${PROJECT_NAME})

# Set required C++11 flag
if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(PyManif PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET PyManif PROPERTY CXX_STANDARD 11)
  set_property(TARGET PyManif PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET PyManif PROPERTY CXX_EXTENSIONS OFF)
endif()
