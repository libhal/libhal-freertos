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

#pragma once

#include <libhal/functional.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/steady_clock.hpp>

#include <span>

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

struct hardware_map_t
{
  hal::output_pin* led;
  hal::steady_clock* clock;
  hal::stream_dac<std::uint8_t>* dac;
  hal::callback<void()> reset;
  std::span<std::uint8_t> pcm8_buffer1;
  std::span<std::uint8_t> pcm8_buffer2;
};

// Application function must be implemented by one of the compilation units
// (.cpp) files.
void application(hardware_map_t& p_map);
hardware_map_t initialize_platform();
