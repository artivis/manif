#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "manif/Rn.h"

#include "bindings_optional.h"
#include "bindings_lie_group_base.h"
#include "bindings_tangent_base.h"

namespace py = pybind11;

void wrap_Rn(py::module &m)
{
  // R1
  py::class_<manif::LieGroupBase<manif::R1d>, std::unique_ptr<manif::LieGroupBase<manif::R1d>, py::nodelete>> R1_base(m, "_R1Base");
  py::class_<manif::TangentBase<manif::R1Tangentd>, std::unique_ptr<manif::TangentBase<manif::R1Tangentd>, py::nodelete>> R1_tan_base(m, "_R1TangentBase");

  py::class_<manif::R1d, manif::LieGroupBase<manif::R1d>> R1(m, "R1", "The R^1 group.");
  py::class_<manif::R1Tangentd, manif::TangentBase<manif::R1Tangentd>> R1_tan(m, "R1Tangent");

  wrap_lie_group_base<manif::R1d, manif::LieGroupBase<manif::R1d>>(R1);
  wrap_tangent_base<manif::R1Tangentd, manif::TangentBase<manif::R1Tangentd>>(R1_tan);

  // R2
  py::class_<manif::LieGroupBase<manif::R2d>, std::unique_ptr<manif::LieGroupBase<manif::R2d>, py::nodelete>> R2_base(m, "_R2Base");
  py::class_<manif::TangentBase<manif::R2Tangentd>, std::unique_ptr<manif::TangentBase<manif::R2Tangentd>, py::nodelete>> R2_tan_base(m, "_R2TangentBase");

  py::class_<manif::R2d, manif::LieGroupBase<manif::R2d>> R2(m, "R2");
  py::class_<manif::R2Tangentd, manif::TangentBase<manif::R2Tangentd>> R2_tan(m, "R2Tangent");

  wrap_lie_group_base<manif::R2d, manif::LieGroupBase<manif::R2d>>(R2);
  wrap_tangent_base<manif::R2Tangentd, manif::TangentBase<manif::R2Tangentd>>(R2_tan);

  // R3
  py::class_<manif::LieGroupBase<manif::R3d>, std::unique_ptr<manif::LieGroupBase<manif::R3d>, py::nodelete>> R3_base(m, "_R3Base");
  py::class_<manif::TangentBase<manif::R3Tangentd>, std::unique_ptr<manif::TangentBase<manif::R3Tangentd>, py::nodelete>> R3_tan_base(m, "_R3TangentBase");

  py::class_<manif::R3d, manif::LieGroupBase<manif::R3d>> R3(m, "R3");
  py::class_<manif::R3Tangentd, manif::TangentBase<manif::R3Tangentd>> R3_tan(m, "R3Tangent");

  wrap_lie_group_base<manif::R3d, manif::LieGroupBase<manif::R3d>>(R3);
  wrap_tangent_base<manif::R3Tangentd, manif::TangentBase<manif::R3Tangentd>>(R3_tan);

  // R4
  py::class_<manif::LieGroupBase<manif::R4d>, std::unique_ptr<manif::LieGroupBase<manif::R4d>, py::nodelete>> R4_base(m, "_R4Base");
  py::class_<manif::TangentBase<manif::R4Tangentd>, std::unique_ptr<manif::TangentBase<manif::R4Tangentd>, py::nodelete>> R4_tan_base(m, "_R4TangentBase");

  py::class_<manif::R4d, manif::LieGroupBase<manif::R4d>> R4(m, "R4");
  py::class_<manif::R4Tangentd, manif::TangentBase<manif::R4Tangentd>> R4_tan(m, "R4Tangent");

  wrap_lie_group_base<manif::R4d, manif::LieGroupBase<manif::R4d>>(R4);
  wrap_tangent_base<manif::R4Tangentd, manif::TangentBase<manif::R4Tangentd>>(R4_tan);

  // R5
  py::class_<manif::LieGroupBase<manif::R5d>, std::unique_ptr<manif::LieGroupBase<manif::R5d>, py::nodelete>> R5_base(m, "_R5Base");
  py::class_<manif::TangentBase<manif::R5Tangentd>, std::unique_ptr<manif::TangentBase<manif::R5Tangentd>, py::nodelete>> R5_tan_base(m, "_R5TangentBase");

  py::class_<manif::R5d, manif::LieGroupBase<manif::R5d>> R5(m, "R5");
  py::class_<manif::R5Tangentd, manif::TangentBase<manif::R5Tangentd>> R5_tan(m, "R5Tangent");

  wrap_lie_group_base<manif::R5d, manif::LieGroupBase<manif::R5d>>(R5);
  wrap_tangent_base<manif::R5Tangentd, manif::TangentBase<manif::R5Tangentd>>(R5_tan);

  // R6
  py::class_<manif::LieGroupBase<manif::R6d>, std::unique_ptr<manif::LieGroupBase<manif::R6d>, py::nodelete>> R6_base(m, "_R6Base");
  py::class_<manif::TangentBase<manif::R6Tangentd>, std::unique_ptr<manif::TangentBase<manif::R6Tangentd>, py::nodelete>> R6_tan_base(m, "_R6TangentBase");

  py::class_<manif::R6d, manif::LieGroupBase<manif::R6d>> R6(m, "R6");
  py::class_<manif::R6Tangentd, manif::TangentBase<manif::R6Tangentd>> R6_tan(m, "R6Tangent");

  wrap_lie_group_base<manif::R6d, manif::LieGroupBase<manif::R6d>>(R6);
  wrap_tangent_base<manif::R6Tangentd, manif::TangentBase<manif::R6Tangentd>>(R6_tan);

  // R7
  py::class_<manif::LieGroupBase<manif::R7d>, std::unique_ptr<manif::LieGroupBase<manif::R7d>, py::nodelete>> R7_base(m, "_R7Base");
  py::class_<manif::TangentBase<manif::R7Tangentd>, std::unique_ptr<manif::TangentBase<manif::R7Tangentd>, py::nodelete>> R7_tan_base(m, "_R7TangentBase");

  py::class_<manif::R7d, manif::LieGroupBase<manif::R7d>> R7(m, "R7");
  py::class_<manif::R7Tangentd, manif::TangentBase<manif::R7Tangentd>> R7_tan(m, "R7Tangent");

  wrap_lie_group_base<manif::R7d, manif::LieGroupBase<manif::R7d>>(R7);
  wrap_tangent_base<manif::R7Tangentd, manif::TangentBase<manif::R7Tangentd>>(R7_tan);

  // R8
  py::class_<manif::LieGroupBase<manif::R8d>, std::unique_ptr<manif::LieGroupBase<manif::R8d>, py::nodelete>> R8_base(m, "_R8Base");
  py::class_<manif::TangentBase<manif::R8Tangentd>, std::unique_ptr<manif::TangentBase<manif::R8Tangentd>, py::nodelete>> R8_tan_base(m, "_R8TangentBase");

  py::class_<manif::R8d, manif::LieGroupBase<manif::R8d>> R8(m, "R8");
  py::class_<manif::R8Tangentd, manif::TangentBase<manif::R8Tangentd>> R8_tan(m, "R8Tangent");

  wrap_lie_group_base<manif::R8d, manif::LieGroupBase<manif::R8d>>(R8);
  wrap_tangent_base<manif::R8Tangentd, manif::TangentBase<manif::R8Tangentd>>(R8_tan);

  // R9
  py::class_<manif::LieGroupBase<manif::R9d>, std::unique_ptr<manif::LieGroupBase<manif::R9d>, py::nodelete>> R9_base(m, "_R9Base");
  py::class_<manif::TangentBase<manif::R9Tangentd>, std::unique_ptr<manif::TangentBase<manif::R9Tangentd>, py::nodelete>> R9_tan_base(m, "_R9TangentBase");

  py::class_<manif::R9d, manif::LieGroupBase<manif::R9d>> R9(m, "R9");
  py::class_<manif::R9Tangentd, manif::TangentBase<manif::R9Tangentd>> R9_tan(m, "R9Tangent");

  wrap_lie_group_base<manif::R9d, manif::LieGroupBase<manif::R9d>>(R9);
  wrap_tangent_base<manif::R9Tangentd, manif::TangentBase<manif::R9Tangentd>>(R9_tan);
}
