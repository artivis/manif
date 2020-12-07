#include <pybind11/pybind11.h>

void wrap_SO2(pybind11::module &m);
void wrap_SO3(pybind11::module &m);

void wrap_SE2(pybind11::module &m);
void wrap_SE3(pybind11::module &m);

PYBIND11_MODULE(PyManif, m) {
  m.doc() = "Python bindings for the manif library.";

  wrap_SO3(m);
  wrap_SO2(m);

  wrap_SE3(m);
  wrap_SE2(m);
}
