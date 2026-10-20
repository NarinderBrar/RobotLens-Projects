#include <cstddef>
#include <cstdint>

namespace {

constexpr std::uint16_t kBadFileDescriptor = 8U;

}

extern "C" {

__attribute__((weak)) void* __cxa_allocate_exception(std::size_t) {
    __builtin_trap();
}

__attribute__((weak, noreturn)) void __cxa_throw(void*, void*, void (*)(void*)) {
    __builtin_trap();
}

__attribute__((weak, noreturn)) void abort_message(const char*, ...) {
    __builtin_trap();
}

__attribute__((weak)) std::uint16_t __wasi_fd_write(int, const void*, std::size_t, std::size_t*) {
    return kBadFileDescriptor;
}

__attribute__((weak)) std::uint16_t __wasi_fd_close(int) {
    return kBadFileDescriptor;
}

__attribute__((weak)) std::uint16_t __wasi_fd_seek(int, std::int64_t, std::uint8_t,
                                                   std::uint64_t*) {
    return kBadFileDescriptor;
}
}
