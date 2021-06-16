set(PYBIND11_CPP_STANDARD -std=c++11)
pybind11_add_module(manifpy MODULE
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
set_property(TARGET manifpy PROPERTY CXX_STANDARD 11)
set_property(TARGET manifpy PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET manifpy PROPERTY CXX_EXTENSIONS OFF)

# Detect the active site-packages folder instead of using CMAKE_INSTALL_PREFIX
option(
  DETECT_ACTIVE_PYTHON_SITEPACKAGES
  "Do you want manif to detect and use the active site-package directory? (it could be a system dir)"
  FALSE)

# Installation in the active site-packages folder (could be a system folder)
if(DETECT_ACTIVE_PYTHON_SITEPACKAGES)
  set(PYTHON_INSTDIR "${Python_SITELIB}/manifpy")
# Installation from the build extension of setup.py
elseif(CALL_FROM_SETUP_PY)
  set(PYTHON_INSTDIR "${CMAKE_INSTALL_PREFIX}")
# Installation in the CMAKE_INSTALL_PREFIX
else()
  execute_process(
    COMMAND
    ${Python_EXECUTABLE} -c
    "import distutils.sysconfig; print(distutils.sysconfig.get_python_lib(plat_specific=True, standard_lib=False, prefix=''))"
    OUTPUT_VARIABLE _PYTHON_INSTDIR)
  string(STRIP ${_PYTHON_INSTDIR} _PYTHON_INSTDIR_CLEAN)
  set(PYTHON_INSTDIR "${_PYTHON_INSTDIR_CLEAN}/manifpy")
endif()

# Setup installation path
install(TARGETS manifpy DESTINATION "${PYTHON_INSTDIR}")

# Create the Python package in the build tree for testing purposes
set(BUILD_TREE_PYTHON_PACKAGE "${CMAKE_BINARY_DIR}/manifpy")
set_target_properties(
  manifpy PROPERTIES
  OUTPUT_NAME _bindings
  LIBRARY_OUTPUT_DIRECTORY "${BUILD_TREE_PYTHON_PACKAGE}")

# Create the __init__.py file
file(
  GENERATE
  OUTPUT "${BUILD_TREE_PYTHON_PACKAGE}/__init__.py"
  CONTENT "from manifpy._bindings import *\n")

# Install the __init__.py file
install(
  FILES "${BUILD_TREE_PYTHON_PACKAGE}/__init__.py"
  DESTINATION ${PYTHON_INSTDIR})
