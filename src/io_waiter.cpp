#include <libhal-freertos/io_waiter.hpp>

namespace hal::freertos {
void io_waiter::driver_wait()
{
  // Save the handler for resume to use
  m_handle = xTaskGetCurrentTaskHandle();
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

void io_waiter::driver_resume() noexcept
{
  BaseType_t higher_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(m_handle, &higher_priority_task_woken);
  // Force a context switch if a higher priority task was woken up
  portYIELD_FROM_ISR(higher_priority_task_woken);
}
}  // namespace hal::freertos
