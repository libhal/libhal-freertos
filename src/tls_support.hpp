#pragma once

#include <cstddef>

extern "C"
{
  extern unsigned __tls_base;
  extern unsigned __tdata_source;
  extern unsigned __tdata_size;
  extern unsigned __tls_size;

  /**
   * @brief libhal definition of the gcc/llvm emutls object/control
   *
   * This structure differs from the gcc/llvm objects in that the 3 field is not
   * a union of an unsigned value and a pointer and in this case is just a
   * pointer.
   *
   */
  struct __emutls_object
  {
    /**
     * @brief The allocation size of the TLS object
     *
     * When this value is initially accessed, the value will be set to the size
     * of the TCB object.
     *
     * If the offset pointer has been set to the appropriate value, then this
     * value will be set to 0. If not, then it will hold
     *
     */
    std::size_t allocation_size;
    /**
     * @brief The alignment of the TLS object
     *
     */
    std::size_t allocation_align;
    /**
     * @brief pointer to the position of the tls object in the TLS memory region
     *
     * At startup the default value is zero and it is up to the
     * `__emutls_get_address` to set this to a valid offset value for all
     * threads that use this tls variable. This is apart of GCC's lazy TLS
     * initialization process. This value is a relative offset to the where the
     * thread local storage region is. The new location must not be zero as it
     * would be reinitialized on each access of the object. THUS, the thread
     * allocation libraries must over allocate the TCB area.
     */
    void* offset_pointer;
    /**
     * @brief pointer to the initialization data of the tls object
     *
     * Will be a nullptr if the tls object was uninitialized.
     *
     */
    void* initial_value;
  };
}