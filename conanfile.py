# Copyright 2024 - 2025 Khalil Estell and the libhal contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from conan import ConanFile

required_conan_version = ">=2.0.14"


class libhal_freertos_conan(ConanFile):
    name = "libhal-freertos"
    license = "Apache-2.0"
    homepage = "https://github.com/libhal/libhal-freertos"
    description = ("libhal compatible FreeRTOS support and utility libraries")
    topics = ("freertos", "multithreading", "threads", "mcu")
    settings = "compiler", "build_type", "os", "arch"

    python_requires = "libhal-bootstrap/[>=4.3.0 <5]"
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

    def package_id(self):
        self.info.python_requires.major_mode()
