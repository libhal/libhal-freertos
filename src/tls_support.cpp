#include <cstdint>

#include <array>

#include "tls_support.hpp"

extern "C"
{
  std::array<std::uint8_t, 128> corruptible_data{};
  void* global_control;

  void* __emutls_get_address(__emutls_object* p_control)
  {
    // auto* tls_memory = reinterpret_cast<StackType_t*>(
    //   pvTaskGetThreadLocalStoragePointer(nullptr, tls_index));
    // auto* final_address = tls_memory + p_offset;
    global_control = p_control;
    return corruptible_data.data();
  }
}