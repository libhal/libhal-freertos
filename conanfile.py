
from conan import ConanFile

required_conan_version = ">=2.0.14"


class libhal_freertos_conan(ConanFile):
    name = "libhal-freertos"
    license = "Apache-2.0"
    homepage = "https://github.com/libhal/libhal-freertos"
    description = ("libhal compatible FreeRTOS support and utility libraries")
    topics = ("freertos", "multithreading", "threads", "mcu")
    settings = "compiler", "build_type", "os", "arch"

    python_requires = "libhal-bootstrap/[^2.0.0]"
    python_requires_extend = "libhal-bootstrap.library"

    options = {
        "freertos_version": ["ANY"],
    }

    default_options = {
        "freertos_version": "10.6.0",
    }

    def requirements(self):
        bootstrap = self.python_requires["libhal-bootstrap"]
        bootstrap.module.add_library_requirements(
            self, override_libhal_version="4.2.0")
        self.requires(
            f"freertos/[^{self.options.freertos_version}]", transitive_headers=True,
            options={
                "configUSE_IDLE_HOOK": True
            })

    def package_info(self):
        self.cpp_info.libs = ["libhal-freertos"]
        self.cpp_info.set_property("cmake_target_name", "libhal::freertos")
