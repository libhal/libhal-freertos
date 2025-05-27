#pragma once

#include <FreeRTOS.h>
#include <task.h>

#include <libhal/io_waiter.hpp>

namespace hal::freertos {
class io_waiter : public hal::io_waiter
{
private:
  // Wait for an IO operation to need attention
  void driver_wait() override;
  // Wake up from waiting when IO is ready
  void driver_resume() noexcept override;

  TaskHandle_t m_handle{};
};
}  // namespace hal::freertos
