from setuptools import setup, Extension
from pybind11.setup_helpers import Pybind11Extension, build_ext
import sys

if sys.version_info < (3, 9):
    sys.exit("Python 3.9+ required")

ext_modules = [
    Pybind11Extension(
        "kin_wrapper",
        ["kin_wrapper.cpp"],
        include_dirs=["../inc"],
        cxx_std=17,
    ),
]

setup(
    name="kin_wrapper",
    version="0.1.0",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
)
