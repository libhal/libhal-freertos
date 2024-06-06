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
#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/output_pin.hpp>
#include <libhal-lpc40/pin.hpp>
#include <libhal-lpc40/power.hpp>
#include <libhal-util/enum.hpp>
#include <libhal-util/static_callable.hpp>
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

class dma
{
public:
  static constexpr std::uintptr_t address = 0x2008'0000;
  struct dma_channel
  {
    volatile std::uint32_t source_address;
    volatile std::uint32_t destination_address;
    volatile std::uint32_t linked_list_item;
    volatile std::uint32_t control;
    volatile std::uint32_t config;
  };

  struct gpdma
  {
    volatile std::uint32_t INTSTAT;
    volatile std::uint32_t INTTCSTAT;
    volatile std::uint32_t INTTCCLEAR;
    volatile std::uint32_t INTERRSTAT;
    volatile std::uint32_t INTERRCLR;
    volatile std::uint32_t RAWINTTCSTAT;
    volatile std::uint32_t RAWINTERRSTAT;
    volatile std::uint32_t ENBLDCHNS;
    volatile std::uint32_t SOFTBREQ;
    volatile std::uint32_t SOFTSREQ;
    volatile std::uint32_t SOFTLBREQ;
    volatile std::uint32_t SOFTLSREQ;
    volatile std::uint32_t CONFIG;
    volatile std::uint32_t SYNC;
  };

  enum class transfer_type : uint8_t
  {
    /// Flow Control: DMA controller
    memory_to_memory = 0b000,
    /// Flow Control: DMA controller
    memory_to_peripheral = 0b001,
    /// Flow Control: DMA controller
    peripheral_to_memory = 0b010,
    /// Flow Control: DMA controller
    peripheral_to_peripheral = 0b011,
    /// Flow Control: Destination Peripheral
    peripheral_to_peripheral_dp = 0b100,
    /// Flow Control: Destination Peripheral
    memory_to_peripheral_dp = 0b101,
    /// Flow Control: Source Peripheral
    peripheral_to_memory_sp = 0b110,
    /// Flow Control: Source Peripheral
    peripheral_to_peripheral_sp = 0b111
  };

  static constexpr std::size_t max_transfer_size = (1 << 12) - 1;

  static gpdma* reg()
  {
    return reinterpret_cast<gpdma*>(address);
  }

  static dma_channel* channel(unsigned p_channel)
  {
    auto channel_offset = (p_channel * 0x20);
    return reinterpret_cast<dma_channel*>(address + 0x100 + channel_offset);
  }
};

class stream_dac : public hal::stream_dac<std::uint8_t>
{
public:
  stream_dac(hal::io_waiter& p_waiter)
    : m_waiter(&p_waiter)
  {
    hal::cortex_m::interrupt::initialize<hal::value(hal::lpc40::irq::max)>();
    hal::lpc40::power_on(hal::lpc40::peripheral::gpdma);

    auto isr = [this]() {
      if (dma::reg()->INTSTAT & channel0_mask) {
        m_finished = true;
        dma::reg()->INTTCCLEAR = channel0_mask;
        dma::reg()->INTERRCLR = channel0_mask;
        m_waiter->wake();
      } else {
        dma::reg()->INTTCCLEAR = 0b1111'1111;
        dma::reg()->INTERRCLR = 0b1111'1111;
      }
    };

    auto handler =
      hal::static_callable<stream_dac, 0, void(void)>(isr).get_handler();

    hal::cortex_m::interrupt(hal::value(hal::lpc40::irq::dma)).enable(handler);
    hal::lpc40::pin(0, 26).function(0b010).dac(true);

    // Clear previous interrupts on the same channel
    auto* dma = dma::reg();
    dma->CONFIG = 1;

    auto* dac_reg = reg();
    // Double buffering enabled (1)
    // Count enable (2)
    // DMA enable (3)
    dac_reg->control = (1 << 1) | (1 << 2) | (1 << 3);
  }

  void write_direct(hal::steady_clock& p_clock, samples p_samples)
  {
    using namespace std::chrono_literals;
    for (const auto& sample : p_samples.samples) {
      // Set to half way just for testing purposes
      reg()->conversion_register.parts[1] = sample;
      hal::delay(p_clock, 500us);
    }
  }

private:
  static constexpr std::uintptr_t dac_address = 0x4008'C000;
  static constexpr std::uint8_t dac_dma_request = 9;
  static constexpr auto channel0_mask = 0b1;

  struct dac_registers
  {
    union
    {
      volatile std::uint32_t whole;
      std::array<volatile std::uint8_t, 4> parts;
    } conversion_register;
    volatile std::uint32_t control;
    volatile std::uint32_t count_value;
  };

  dac_registers* reg()
  {
    return reinterpret_cast<dac_registers*>(dac_address);
  }

  void driver_write(samples p_samples) override
  {
    // Setup sampling frequency
    const auto input_clock =
      hal::lpc40::get_frequency(hal::lpc40::peripheral::dac);
    const auto clock_count_value = input_clock / p_samples.sample_rate;

    auto* dac_reg = reg();
    dac_reg->count_value = clock_count_value;

    // Setup DMA channel addresses
    auto* channel0 = dma::channel(0);

    // control masks
    constexpr auto transfer_size = hal::bit_mask::from<0, 11>();
    constexpr auto source_burst_size = hal::bit_mask::from<12, 14>();
    constexpr auto destination_burst_size = hal::bit_mask::from<15, 17>();
    constexpr auto source_transfer_width = hal::bit_mask::from<18, 20>();
    constexpr auto destination_transfer_width = hal::bit_mask::from<21, 23>();
    constexpr auto source_increment = hal::bit_mask::from<26>();
    constexpr auto destination_increment = hal::bit_mask::from<27>();
    constexpr auto enable_terminal_count_interrupt = hal::bit_mask::from<31>();

    // config masks
    constexpr auto enable = hal::bit_mask::from<0>();
    constexpr auto source_peripheral = hal::bit_mask::from<5, 1>();
    constexpr auto destination_peripheral = hal::bit_mask::from<10, 6>();
    constexpr auto transfer_type = hal::bit_mask::from<13, 11>();
    constexpr auto terminal_count_interrupt_mask = hal::bit_mask::from<15>();
    const auto config_value = hal::bit_value()
                                .insert<source_peripheral>(0U)
                                .insert<destination_peripheral>(dac_dma_request)
                                .set<enable>()
                                .insert<transfer_type>(hal::value(
                                  dma::transfer_type::memory_to_peripheral))
                                .set<terminal_count_interrupt_mask>()
                                .to<std::uint32_t>();

    auto data_remaining = p_samples.samples;

    while (not data_remaining.empty()) {
      m_finished = false;
      auto transfer_amount =
        std::min(data_remaining.size(), dma::max_transfer_size - 1);

      auto* source_address = data_remaining.data();
      auto source_address_int = reinterpret_cast<std::uint32_t>(source_address);

      auto* dest_address = &dac_reg->conversion_register.parts[1];
      auto dest_address_int = reinterpret_cast<std::uint32_t>(dest_address);

      channel0->source_address = source_address_int;
      channel0->destination_address = dest_address_int;
      channel0->linked_list_item = 0;

      auto control_value =
        hal::bit_value()
          .insert<transfer_size>(transfer_amount)
          .insert<source_burst_size>(0U)          /* 0b000 = 1x */
          .insert<destination_burst_size>(0U)     /* 0b000 = 1x */
          .insert<source_transfer_width>(0U)      /* 0b000 = 8-bit */
          .insert<destination_transfer_width>(0U) /* 0b000 = 8-bit */
          .set<source_increment>()
          .clear<destination_increment>()
          .set<enable_terminal_count_interrupt>()
          .to<std::uint32_t>();

      channel0->control = control_value;

      // Clear previous interrupts on the same channel
      auto* dma = dma::reg();
      dma->INTTCCLEAR = channel0_mask;
      dma->INTERRCLR = channel0_mask;

      // Start DMA transfer
      channel0->config = config_value;

      while (not m_finished) {
        m_waiter->wait();
      }

      hal::bit_modify(channel0->config).clear<enable>();

      // Move data forward by the amount of data that was transferred
      data_remaining = data_remaining.subspan(transfer_amount);
    }
  }
  hal::io_waiter* m_waiter;
  bool m_finished = false;
};

class freertos_io_waiter : public hal::io_waiter
{
private:
  // Wait for an IO operation to need attention
  void driver_wait() override
  {
    // Save the task handle for the calling task
    m_handle = xTaskGetCurrentTaskHandle();
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
  // Wake up from waiting when IO is ready
  void driver_wake() override
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Notify the task from ISR
    vTaskNotifyGiveFromISR(m_handle, &xHigherPriorityTaskWoken);

    // Force a context switch if a higher priority task was woken up
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }

  // Cancel the waiting IO operation
  void driver_cancel() override
  {
  }

  // Check if the IO operation was cancelled
  bool driver_is_cancelled() override
  {
    return false;
  }

  TaskHandle_t m_handle{};
};

std::array<std::uint8_t, 8000> buffer1;
std::array<std::uint8_t, 8000> buffer2;

hardware_map_t initialize_platform()
{
  // Change the input frequency to match the frequency of the crystal attached
  // to the external OSC pins.
  hal::lpc40::maximum(10.0_MHz);

  auto cpu_frequency = hal::lpc40::get_frequency(hal::lpc40::peripheral::cpu);

  static hal::cortex_m::dwt_counter steady_clock(cpu_frequency);
  static hal::lpc40::output_pin led(1, 10);
  static freertos_io_waiter waiter;
  static stream_dac my_dac(waiter);

  global_steady_clock = &steady_clock;

  hal::cortex_m::interrupt::initialize<hal::value(hal::lpc40::irq::max)>();

  hal::cortex_m::interrupt(hal::value(hal::cortex_m::irq::hard_fault))
    .enable(hard_fault_handler);
  hal::cortex_m::interrupt(
    hal::value(hal::cortex_m::irq::memory_management_fault))
    .enable(memory_management_handler);
  hal::cortex_m::interrupt(hal::value(hal::cortex_m::irq::bus_fault))
    .enable(bus_fault_handler);
  hal::cortex_m::interrupt(hal::value(hal::cortex_m::irq::usage_fault))
    .enable(usage_fault_handler);

  hal::cortex_m::interrupt(hal::value(hal::cortex_m::irq::sv_call))
    .enable(vPortSVCHandler);
  hal::cortex_m::interrupt(hal::value(hal::cortex_m::irq::pend_sv))
    .enable(xPortPendSVHandler);
  hal::cortex_m::interrupt(hal::value(hal::cortex_m::irq::systick))
    .enable(xPortSysTickHandler);

  return {
    .led = &led,
    .clock = &steady_clock,
    .dac = &my_dac,
    .reset = []() { hal::cortex_m::reset(); },
    .pcm8_buffer1 = buffer1,
    .pcm8_buffer2 = buffer2,
  };
}
