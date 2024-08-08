#!/usr/bin/env python
# mypy: ignore-errors
"""Setup script for the project."""

import os
import re
import shutil
import subprocess
import sys
import sysconfig
from multiprocessing import cpu_count
from pathlib import Path

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext

PLAT_TO_CMAKE = {
    "win32": "Win32",
    "win-amd64": "x64",
    "win-arm32": "ARM",
    "win-arm64": "ARM64",
}


class CMakeExtension(Extension):
    """CMake extension.

    This is a subclass of setuptools.Extension that allows specifying the
    location of the CMakeLists.txt file.

    Usage:
        setup(
            name="my_package",
            ext_modules=[CMakeExtension("my_package")],
            cmdclass={"build_ext": CMakeBuild},
        )
    """

    def __init__(self, name: str) -> None:
        super().__init__(name, sources=[])

        self.sourcedir = os.fspath(Path(__file__).parent.resolve() / name)


class CMakeBuild(build_ext):
    """CMake build extension.

    This is a subclass of setuptools.command.build_ext.build_ext that runs
    cmake to build the extension.

    Usage:
        setup(
            name="my_package",
            ext_modules=[CMakeExtension("my_package")],
            cmdclass={"build_ext": CMakeBuild},
        )
    """

    def initialize_options(self) -> None:
        super().initialize_options()

        # Set parallel build.
        self.parallel = cpu_count()

    def build_extensions(self) -> None:
        self.check_extensions_list(self.extensions)
        self._build_extensions_serial()

    def build_extension(self, ext: CMakeExtension) -> None:
        import cmake  # noqa: F401
        import pybind11  # noqa: F401

        cmake_path = os.path.join(cmake.CMAKE_BIN_DIR, "cmake")
        ext_fullpath = Path.cwd() / self.get_ext_fullpath(ext.name)  # type: ignore[no-untyped-call]
        extdir = ext_fullpath.parent.resolve()
        debug = int(os.environ.get("DEBUG", 0)) if self.debug is None else self.debug
        cfg = "Debug" if debug else "Release"
        cmake_generator = os.environ.get("CMAKE_GENERATOR", "")

        # Found this necessary for building on Apple M1 machine.
        cmake_cxx_flags = [
            "-fPIC",
            "-Wl,-undefined,dynamic_lookup",
            "-Wno-unused-command-line-argument",
            "-Wall",
        ]

        # System include paths.
        cmake_include_dirs = [pybind11.get_include()]
        python_include_path = sysconfig.get_path("include", scheme="posix_prefix")
        if python_include_path is not None:
            cmake_include_dirs += [python_include_path]
        cmake_cxx_flags += [f"-isystem {dir_name}" for dir_name in cmake_include_dirs]

        # Sets paths to various CMake stuff.
        cmake_prefix_path_str = ";".join([pybind11.get_cmake_dir()])
        cmake_cxx_flags_str = " ".join(cmake_cxx_flags)

        # Gets CMake arguments.
        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}{os.sep}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            f"-DPYTHON_INCLUDE_DIR={sysconfig.get_path('include')}",
            f"-DPYTHON_LIBRARY={sysconfig.get_path('platlib')}",
            f"-DCMAKE_PREFIX_PATH={cmake_prefix_path_str}",
            f"-DCMAKE_CXX_FLAGS={cmake_cxx_flags_str}",
            f"-DCMAKE_BUILD_TYPE={cfg}",  # not used on MSVC, but no harm
        ]

        build_args = []
        if "CMAKE_ARGS" in os.environ:
            cmake_args += [item for item in os.environ["CMAKE_ARGS"].split(" ") if item]

        env = os.environ.copy()

        if self.compiler.compiler_type != "msvc":
            if not cmake_generator or cmake_generator == "Ninja":
                try:
                    import ninja

                    ninja_executable_path = Path(ninja.BIN_DIR) / "ninja"
                    cmake_args += [
                        "-GNinja",
                        f"-DCMAKE_MAKE_PROGRAM:FILEPATH={ninja_executable_path}",
                    ]
                except ImportError:
                    pass

        else:
            single_config = any(x in cmake_generator for x in ("NMake", "Ninja"))
            contains_arch = any(x in cmake_generator for x in ("ARM", "Win64"))
            if not single_config and not contains_arch:
                cmake_args += ["-A", PLAT_TO_CMAKE[self.plat_name]]
            if not single_config:
                cmake_args += [f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{cfg.upper()}={extdir}"]
                build_args += ["--config", cfg]

        if sys.platform.startswith("darwin"):
            archs = re.findall(r"-arch (\S+)", os.environ.get("ARCHFLAGS", ""))
            if archs:
                cmake_args += [f"-DCMAKE_OSX_ARCHITECTURES={';'.join(archs)}"]

        if "CMAKE_BUILD_PARALLEL_LEVEL" not in os.environ:
            if hasattr(self, "parallel") and self.parallel:
                build_args += [f"-j{self.parallel}"]

        build_temp = Path(self.build_temp) / ext.name
        print(f"Building extension {ext.name} in {build_temp}")
        if not build_temp.exists():
            build_temp.mkdir(parents=True)

        def show_and_run(cmd: list[str]) -> None:
            print(" ".join(cmd))
            subprocess.run(cmd, env=env, check=True)

        show_and_run([cmake_path, f"-S{ext.sourcedir}", f"-B{build_temp}"] + cmake_args)
        show_and_run([cmake_path, "--build", f"{build_temp}"] + (["--"] + build_args if build_args else []))

    def copy_extensions_to_source(self) -> None:
        pass

    def run(self) -> None:
        super().run()

        def gen_stubs(ext: Extension) -> None:
            cmd = ["stubgen", "-p", f"{ext.name.replace('/', '.')}", "-o", "."]
            print(" ".join(cmd))
            subprocess.run(cmd, check=True)

        if shutil.which("stubgen") is not None:
            for ext in self.extensions:
                gen_stubs(ext)


with open("README.md", "r", encoding="utf-8") as f:
    long_description: str = f.read()


with open("firmware/requirements.txt", "r", encoding="utf-8") as f:
    requirements: list[str] = f.read().splitlines()


with open("firmware/requirements-dev.txt", "r", encoding="utf-8") as f:
    requirements_dev: list[str] = f.read().splitlines()


with open("firmware/__init__.py", "r", encoding="utf-8") as fh:
    version_re = re.search(r"^__version__ = \"([^\"]*)\"", fh.read(), re.MULTILINE)
assert version_re is not None, "Could not find version in firmware/__init__.py"
version: str = version_re.group(1)


setup(
    name="firmware",
    version=version,
    description="The firmware project",
    author="Benjamin Bolte",
    url="https///github.com/kscalelabs/firmware",
    long_description=long_description,
    long_description_content_type="text/markdown",
    python_requires=">=3.9",
    setup_requires=["cmake", "mypy", "pybind11"],
    install_requires=requirements,
    tests_require=requirements_dev,
    extras_require={"dev": requirements_dev},
    ext_modules=[CMakeExtension("firmware/cpp")],
    cmdclass={"build_ext": CMakeBuild},
    exclude_package_data={
        "firmware": [
            "cpp/**/*.cpp",
        ],
    },
    entry_points={
        "console_scripts": [
            "motor-cli = firmware.scripts.single_motor:cli_entry_point",
        ],
    },
)