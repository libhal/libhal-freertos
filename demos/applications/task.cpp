// Copyright 2024 Khalil Estell
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <FreeRTOS.h>
#include <task.h>

#include <libhal-freertos/task.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include "../resource_list.hpp"

resource_list map;

thread_local int a = 15;
thread_local std::uint8_t b;
int global = 0;

void application(resource_list& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  map = p_map;

  static std::array<hal::byte, 512> task1_stack;
  /* Create the task without using any dynamic memory allocation. */
  static hal::freertos::static_task task1(
    "blinker",
    [](void*) noexcept {
      while (true) {
        a++;
        map.led->level(true);
        vTaskDelay(500);
        b++;
        map.led->level(false);
        vTaskDelay(500);
      }
    },
    task1_stack);

  static std::array<hal::byte, 512> task2_stack;
  /* Create the task without using any dynamic memory allocation. */
  static hal::freertos::static_task task2(
    "iterator",
    [](void*) noexcept {
      while (true) {
        a++;
        vTaskDelay(500);
        b++;
        global++;
      }
    },
    task2_stack);

  /* Start the RTOS scheduler, this function should not return as it causes the
  execution context to change from main() to one of the created tasks. */
  vTaskStartScheduler();
}
