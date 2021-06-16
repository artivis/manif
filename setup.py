import sys

from cmake_build_extension import BuildExtension, CMakeExtension
from setuptools import setup

setup(
    ext_modules=[
        CMakeExtension(
            name="CMakeProject",
            install_prefix="manifpy",
            cmake_depends_on=["pybind11"],
            disable_editable=True,
            cmake_configure_options=[
                "-DCALL_FROM_SETUP_PY:BOOL=ON",
                "-DBUILD_PYTHON_BINDINGS:BOOL=ON",
                f"-DPython3_EXECUTABLE:PATH={sys.executable}",
            ],
        )
    ],
    cmdclass=dict(build_ext=BuildExtension),
)
