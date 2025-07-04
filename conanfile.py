from conan import ConanFile, tools
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.env import VirtualRunEnv
from conan.tools.build import can_run

DEFAULT_GTEST_VERSION = "1.16.0"

class MusicPlayerRecipe(ConanFile):
    name = "variant"
    version = "0.1.0"
    package_type = "library"

    options = {
        "with_tests": [True, False],
        "with_examples": [True, False],
        "shared": [False],
        "fPIC": [False],
    }
    default_options = {
        "with_tests": False,
        "with_examples": False,
        "shared": False,
        "fPIC": False,
    }

    # Optional metadata
    license = "MIT"
    author = "Robert Williamson"
    url = "https://github.com/Robby-cell/variant"
    description = "An implementation of std::variant"
    topics = ("variant", "C++", "STL")

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"

    # Sources are located in the same place as this recipe.
    exports_sources = "CMakeLists.txt", "include/*", "examples/*", "tests/*"

    def requirements(self):
        """Define runtime dependencies."""

    def build_requirements(self):
        """Define build-time dependencies."""
        if self.options.with_tests:
            self.test_requires(f"gtest/{DEFAULT_GTEST_VERSION}")

    def layout(self):
        cmake_layout(self)

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.cache_variables = {
            "CMAKE_EXPORT_COMPILE_COMMANDS": True,

            "VARIANT_BUILD_TESTS": self.options.with_tests,
            "VARIANT_BUILD_EXAMPLES": self.options.with_examples,
            "VARIANT_SHARED": self.options.shared,
            "VARIANT_POSITION_INDEPENDENT_CODE": self.options.fPIC,
        }
        tc.generate()

        # Environment for running executables created during the build
        if can_run(self):
            VirtualRunEnv(self).generate(scope="build")

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        self.copy("LICENSE", dst="licenses")
        self.copy("*.hpp", dst="include", src=".")

    def package_info(self):
        if self.settings.os == "Linux":
            self.cpp_info.libs = ["-Wl,--start-group"]
        self.cpp_info.libs.extend(tools.collect_libs(self))
        if self.settings.os == "Linux":
            self.cpp_info.libs.extend(["-Wl,--end-group"])
