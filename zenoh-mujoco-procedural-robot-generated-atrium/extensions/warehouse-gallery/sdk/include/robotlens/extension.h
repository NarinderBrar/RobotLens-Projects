#ifndef ROBOTLENS_EXTENSION_H
#define ROBOTLENS_EXTENSION_H

#include <stdint.h>
#include <stdlib.h>

#define ROBOTLENS_ABI_VERSION 1

#define ROBOTLENS_IMPORT(name) __attribute__((import_module("robotlens"), import_name(name)))
#define ROBOTLENS_EXPORT(name) __attribute__((export_name(name)))

#ifdef __cplusplus
extern "C" {
#endif

enum { ROBOTLENS_LOG_INFO = 0, ROBOTLENS_LOG_WARNING = 1, ROBOTLENS_LOG_ERROR = 2 };

ROBOTLENS_IMPORT("output") int32_t robotlens_output(const void* data, int32_t length);
ROBOTLENS_IMPORT("log") void robotlens_log(int32_t level, const char* text, int32_t length);
ROBOTLENS_IMPORT("asset_size") int64_t robotlens_asset_size(const char* path, int32_t length);
ROBOTLENS_IMPORT("asset_read")
int64_t robotlens_asset_read(const char* path, int32_t path_length, void* destination,
                             int32_t capacity);

#ifdef __cplusplus
}
#endif

#define ROBOTLENS_DEFINE_C_MODULE(handler)                                                         \
    static uint8_t* robotlens_input_buffer = 0;                                                    \
    static int32_t robotlens_input_capacity = 0;                                                   \
    ROBOTLENS_EXPORT("robotlens_abi_version") int32_t robotlens_abi_version(void) {                \
        return ROBOTLENS_ABI_VERSION;                                                              \
    }                                                                                              \
    ROBOTLENS_EXPORT("robotlens_alloc") uint8_t* robotlens_alloc(int32_t size) {                   \
        if (size > robotlens_input_capacity) {                                                     \
            uint8_t* grown = (uint8_t*)realloc(robotlens_input_buffer, (size_t)size);              \
            if (grown == 0) {                                                                      \
                return 0;                                                                          \
            }                                                                                      \
            robotlens_input_buffer = grown;                                                        \
            robotlens_input_capacity = size;                                                       \
        }                                                                                          \
        return robotlens_input_buffer;                                                             \
    }                                                                                              \
    ROBOTLENS_EXPORT("robotlens_call")                                                             \
    int32_t robotlens_call(const uint8_t* input, int32_t length) {                                 \
        return handler(input, length);                                                             \
    }

#endif
