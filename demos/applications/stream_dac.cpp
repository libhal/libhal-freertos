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

#include <chrono>
#include <cmath>

#include <array>
#include <concepts>
#include <span>

#include <libhal-armcortex/interrupt.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-armcortex/systick_timer.hpp>
#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/power.hpp>
#include <libhal-lpc40/uart.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/bit_limits.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/static_callable.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pwm.hpp>
#include <libhal/timer.hpp>

#define MINIMP3_IMPLEMENTATION
#define MINIMP3_ONLY_MP3
#define MINIMP3_NO_SIMD

#include <minimp3.h>

#include "mp3.h"
#include "music.h"

namespace hal {

template<std::unsigned_integral data_t>
class stream_dac
{
public:
  struct samples
  {
    /**
     * Sample rate sets the update rate of the dac. For example, if the sample
     * rate is 44.1kHz (typical for consumer audio), then the samples will be
     * read from the data array and written to the dac output at that frequency.
     *
     */
    hal::hertz sample_rate;
    /**
     * Span of unsigned integer data to be written to the dac at the rate
     * specified by the sample rate field.
     *
     * If this is empty (empty() == true), then the write command simply
     * returns without updating anything.
     */
    std::span<const data_t> samples;
  };

  /**
   * @brief Write a sequence of data to the dac
   *
   * This API is blocking call. The expectation is that implementors
   *
   * @param p_samples - sample data to be written to the dac output
   * @throws hal::argument_out_of_domain - when the sample rate is not possible
   * for the driver. This can happen if:
   *     - The sample rate is above the input clock rate of the dac making it
   *       impossible to produce.
   *     - The dac could not reach a sample rate close enough to the desired
   *       sample rate. This can occur if the dac's source clock and the sample
   *       rates are close and as such no divider cleanly reaches a reasonable
   *       sample rate for the device. For example, if the desired sample rate
   *       is 750kHz and the clock rate is 1MHz and divider is an integer. You
   *       would either operate at 1MHz or 500kHz which are very far away from
   *       the desired sample rate. The driver should, but is not required, to
   *       provide a means to control the allowed "error" between the desired
   *       sample rate and the achievable sample rate in order tune what
   *       samples are rejected and an exception thrown.
   *     - Sample rate is too low and cannot be achieved by the dac.
   */
  void write(samples p_samples)
  {
    driver_write(p_samples);
  }

private:
  virtual void driver_write(samples p_samples) = 0;
};

class io_waiter
{
public:
  // Wait for an IO operation to need attention
  void wait()
  {
    return driver_wait();
  }
  // Wake up from waiting when IO is ready
  void wake()
  {
    return driver_wake();
  }
  // Cancel the waiting IO operation
  void cancel()
  {
    return driver_cancel();
  }
  // Check if the IO operation was cancelled
  bool is_cancelled()
  {
    return driver_is_cancelled();
  }

private:
  // Wait for an IO operation to need attention
  virtual void driver_wait() = 0;
  // Wake up from waiting when IO is ready
  virtual void driver_wake() = 0;
  // Cancel the waiting IO operation
  virtual void driver_cancel() = 0;
  // Check if the IO operation was cancelled
  virtual bool driver_is_cancelled() = 0;
};
}  // namespace hal

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

template<std::unsigned_integral T>
class pwm_stream_dac : public hal::stream_dac<T>
{
public:
  pwm_stream_dac(hal::pwm& p_pwm,
                 hal::timer& p_timer,
                 std::uint8_t p_pwm_multiplier = 10)
    : m_pwm(&p_pwm)
    , m_timer(&p_timer)
    , m_pwm_multiplier(p_pwm_multiplier)

  {
  }

private:
  void driver_write(hal::stream_dac<T>::samples p_samples) override
  {
    using namespace std::chrono_literals;
    constexpr float scale_factor = sizeof(T) * 8;

    m_pwm->frequency(p_samples.sample_rate * m_pwm_multiplier);

    auto sample_duration = std::chrono::nanoseconds(1s) /
                           static_cast<std::int32_t>(p_samples.sample_rate);

    for (const auto sample : p_samples.samples) {
      m_transfer_complete = false;

      m_pwm->duty_cycle(static_cast<float>(sample) / scale_factor);
      m_timer->schedule([this]() { m_transfer_complete = true; },
                        sample_duration);

      while (not m_transfer_complete) {
        continue;
      }
    }
  }

  hal::pwm* m_pwm;
  hal::timer* m_timer;
  std::uint8_t m_pwm_multiplier = 1;
  bool m_transfer_complete = true;
};

void hard_fault()
{
  while (true) {
    continue;
  }
}

void bus_fault()
{
  while (true) {
    continue;
  }
}

void usage_fault()
{
  while (true) {
    continue;
  }
}

std::array<std::uint8_t, 128> samples{};

class do_nothing_io_waiter : public hal::io_waiter
{
public:
  do_nothing_io_waiter() = default;

private:
  void driver_wait() override
  {
  }
  void driver_wake() override
  {
  }
  void driver_cancel() override
  {
  }
  bool driver_is_cancelled() override
  {
    return false;
  }
};

class mp3_decoder_waiter : public hal::io_waiter
{
public:
  mp3_decoder_waiter()
  {
    mp3dec_init(&mp3d);
  }

  hal::stream_dac<std::uint8_t>::samples next_sequence()
  {
    m_samples_ready = false;
    return { .sample_rate = static_cast<float>(info.hz) * 0.84375f,
             .samples = { u_pcm, m_sample_count } };
  }

private:
  void driver_wait() override
  {
    if (m_samples_ready) {
      return;
    }

    if (mp3_data.empty()) {
      mp3_data = Amherst_Melonade_8kHz_mp3;
    }

    auto data_length = std::min<std::size_t>(mp3_data.size(), 512);

    m_sample_count =
      mp3dec_decode_frame(&mp3d, mp3_data.data(), data_length, pcm, &info);

    mp3_data = mp3_data.subspan(info.frame_bytes);

    for (size_t i = 0; i < m_sample_count; i++) {
      constexpr auto positive_offset = (UINT16_MAX / 2) + 1;
      const auto unsigned_pcm16 = pcm[i] + positive_offset;
      u_pcm[i] = static_cast<std::uint8_t>(unsigned_pcm16 >> 8);
    }

    m_samples_ready = true;
  }
  void driver_wake() override
  {
  }
  void driver_cancel() override
  {
  }
  bool driver_is_cancelled() override
  {
    return false;
  }

  mp3dec_t mp3d{};
  mp3dec_frame_info_t info{};
  std::span<const hal::byte> mp3_data = Amherst_Melonade_8kHz_mp3;
  std::size_t m_sample_count = 0;
  bool m_samples_ready = false;
  std::int16_t pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];
  std::uint8_t u_pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];
  static constexpr size_t pcm_size = sizeof(pcm);
  static constexpr size_t u_pcm_size = sizeof(u_pcm);
};

enum class music_type
{
  sine,
  raw_pcm,
  mp3,
};

constexpr music_type music = music_type::mp3;

void application()
{
  hal::cortex_m::systick_timer timer(
    hal::lpc40::get_frequency(hal::lpc40::peripheral::cpu));

  hal::cortex_m::interrupt::initialize<hal::value(hal::lpc40::irq::max)>();
  hal::cortex_m::interrupt(hal::value(hal::cortex_m::irq::hard_fault))
    .enable(hard_fault);
  hal::cortex_m::interrupt(hal::value(hal::cortex_m::irq::usage_fault))
    .enable(usage_fault);
  hal::cortex_m::interrupt(hal::value(hal::cortex_m::irq::bus_fault))
    .enable(bus_fault);

  hal::stream_dac<std::uint8_t>* dac;
  mp3_decoder_waiter* mp3_waiter = nullptr;
  if constexpr (music == music_type::mp3) {
    static mp3_decoder_waiter waiter;
    static stream_dac inner_dac(waiter);
    dac = &inner_dac;
    mp3_waiter = &waiter;
  } else {
    static do_nothing_io_waiter waiter;
    static stream_dac inner_dac(waiter);
    dac = &inner_dac;
  }

  if constexpr (music == music_type::sine) {
    // Constants for the sine wave
    // Maximum amplitude for 16-bit unsigned
    const int amplitude = (1 << 7) - 1;
    // Offset to shift the sine wave into the positive range (0-255)
    const int offset = (1 << 7);

    // Fill the array with sine wave values
    for (size_t i = 0; i < samples.size(); ++i) {
      // Generating a sine wave over one complete cycle across the array
      auto angle =
        (static_cast<float>(i) / static_cast<float>(samples.size())) * 2.0 *
        std::numbers::pi_v<float>;
      samples[i] = static_cast<std::uint8_t>(
        std::round(std::sin(angle) * amplitude + offset));
    }

    dac->write({ .sample_rate = 16'000.0f, .samples = samples });
  } else if constexpr (music == music_type::raw_pcm) {
    while (true) {
      // Change to 8'000.0f for lofi
      dac->write({ .sample_rate = 13'500.0f, .samples = Amherst_Melonade_raw });
    }
  } else {
    // Grab the first samples
    mp3_waiter->wait();

    while (true) {
      dac->write(mp3_waiter->next_sequence());
    }

#if 0
    mp3dec_t mp3d;
    mp3dec_init(&mp3d);
    std::span<const hal::byte> mp3_data(Amherst_Melonade_8kHz_mp3);

    mp3dec_frame_info_t info;
    std::int16_t pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];
    std::uint8_t u_pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];

    while (true) {
      if (mp3_data.empty()) {
        mp3_data = Amherst_Melonade_8kHz_mp3;
      }

      std::size_t sample_count = mp3dec_decode_frame(
        &mp3d, mp3_data.data(), mp3_data.size(), pcm, &info);

      mp3_data = mp3_data.subspan(info.frame_bytes);

      for (size_t i = 0; i < sample_count; i++) {
        constexpr auto positive_offset = (UINT16_MAX / 2) - 1;
        u_pcm[i] = static_cast<std::uint8_t>((pcm[i] + positive_offset) >> 8);
      }

      dac->write({ .sample_rate = static_cast<float>(info.hz),
                  .samples = { u_pcm, sample_count } });
    }
#endif
  }
}
