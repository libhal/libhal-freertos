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

#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/interrupt.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-armcortex/systick_timer.hpp>
#include <libhal-stm32f1/clock.hpp>
#include <libhal-stm32f1/constants.hpp>
#include <libhal-stm32f1/interrupt.hpp>
#include <libhal-stm32f1/output_pin.hpp>
#include <libhal-util/enum.hpp>
#include <libhal-util/steady_clock.hpp>

#include "../hardware_map.hpp"

extern "C" void xPortSysTickHandler();
extern "C" void xPortPendSVHandler();
extern "C" void vPortSVCHandler();

using namespace hal::literals;
using namespace std::literals;

void hard_fault_handler()
{
  while (true) {
    continue;
  }
}
void memory_management_handler()
{
  while (true) {
    continue;
  }
}
void bus_fault_handler()
{
  while (true) {
    continue;
  }
}
void usage_fault_handler()
{
  while (true) {
    continue;
  }
}

hal::cortex_m::dwt_counter* global_steady_clock = nullptr;

extern "C"
{
  void vPortSetupTimerInterrupt(void)
  {
    auto cpu_frequency = hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu);
    static hal::cortex_m::systick_timer systick(cpu_frequency);
    try {
      systick.schedule(xPortSysTickHandler, 1ms);
    } catch (...) {
      hal::halt();
    }
  }

  unsigned int _freertos_get_high_resolution_timer_count(void)
  {
    return global_steady_clock->uptime();
  }
  void _freertos_configure_high_resolution_timer(void)
  {
  }

  void vApplicationIdleHook()
  {
    asm volatile("wfi");
  }
}

class stream_dac : public hal::stream_dac<std::uint8_t>
{
public:
  stream_dac()
  {
  }

private:
  void driver_write(samples) override
  {
    // Do nothing
  }
};

std::array<std::uint8_t, 1250> buffer1;
std::array<std::uint8_t, 1250> buffer2;

hardware_map_t initialize_platform()
{
  hal::stm32f1::initialize_interrupts();

  hal::cortex_m::enable_interrupt(hal::cortex_m::irq::software_call,
                                  vPortSVCHandler);
  hal::cortex_m::enable_interrupt(hal::cortex_m::irq::pend_sv,
                                  xPortPendSVHandler);
  hal::cortex_m::enable_interrupt(hal::cortex_m::irq::systick,
                                  xPortSysTickHandler);

  auto cpu_frequency = hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu);
  static hal::cortex_m::dwt_counter steady_clock(cpu_frequency);
  global_steady_clock = &steady_clock;

  static hal::stm32f1::output_pin led('C', 13);

  // stand in dac until a real one is available
  static stream_dac dac;

  return {
    .led = &led,
    .clock = &steady_clock,
    .dac = &dac,
    .reset = []() { hal::cortex_m::reset(); },
    .pcm8_buffer1 = buffer1,
    .pcm8_buffer2 = buffer2,
  };
}
