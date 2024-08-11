#include <pybind11/pybind11.h>

void wrap_Rn(pybind11::module &m);

void wrap_SO2(pybind11::module &m);
void wrap_SO3(pybind11::module &m);

void wrap_SE2(pybind11::module &m);
void wrap_SE3(pybind11::module &m);

void wrap_SE_2_3(pybind11::module &m);

void wrap_SGal3(pybind11::module &m);

PYBIND11_MODULE(_bindings, m) {
  m.doc() = "Python bindings for the manif library, "
            "a small library for Lie theory.";

  wrap_Rn(m);

  wrap_SO2(m);
  wrap_SO3(m);

  wrap_SE2(m);
  wrap_SE3(m);

  wrap_SE_2_3(m);

  wrap_SGal3(m);
}
