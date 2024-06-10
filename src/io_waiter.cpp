#include <libhal-freertos/io_waiter.hpp>

namespace hal::freertos {
void io_waiter::driver_wait()
{
  // Save the task handle for the calling task
  m_handle = xTaskGetCurrentTaskHandle();
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}
// Wake up from waiting when IO is ready
void io_waiter::driver_resume()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // Notify the task from ISR
  vTaskNotifyGiveFromISR(m_handle, &xHigherPriorityTaskWoken);

  // Force a context switch if a higher priority task was woken up
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
}  // namespace hal::freertos
