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
#include <libhal-freertos/io_waiter.hpp>
#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/interrupt.hpp>
#include <libhal-lpc40/output_pin.hpp>
#include <libhal-lpc40/power.hpp>
#include <libhal-lpc40/stream_dac.hpp>
#include <libhal-util/enum.hpp>
#include <libhal-util/static_callable.hpp>
#include <libhal-util/steady_clock.hpp>

#include "../resource_list.hpp"

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
    hal::lpc40::initialize_interrupts();

    hal::cortex_m::enable_interrupt(hal::cortex_m::irq::software_call,
                                    vPortSVCHandler);
    hal::cortex_m::enable_interrupt(hal::cortex_m::irq::pend_sv,
                                    xPortPendSVHandler);

    auto cpu_frequency = hal::lpc40::get_frequency(hal::lpc40::peripheral::cpu);
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

  unsigned int idle_percent = 0;

  void vApplicationIdleHook()
  {
    while (true) {
      idle_percent = ulTaskGetIdleRunTimePercent();
      asm volatile("wfi");
    }
  }
}

std::array<std::uint8_t, 6000> buffer1;
std::array<std::uint8_t, 6000> buffer2;

resource_list initialize_platform()
{
  // Change the input frequency to match the frequency of the crystal attached
  // to the external OSC pins.
  hal::lpc40::maximum(10.0_MHz);

  auto cpu_frequency = hal::lpc40::get_frequency(hal::lpc40::peripheral::cpu);

  static hal::cortex_m::dwt_counter steady_clock(cpu_frequency);
  static hal::lpc40::output_pin led(1, 10);
  static hal::freertos::io_waiter waiter;
  static hal::lpc40::stream_dac_u8 my_dac(waiter);

  global_steady_clock = &steady_clock;

  return {
    .led = &led,
    .clock = &steady_clock,
    .dac = &my_dac,
    .reset = []() { hal::cortex_m::reset(); },
    .pcm8_buffer1 = buffer1,
    .pcm8_buffer2 = buffer2,
  };
}
