#include "kit.hpp"

namespace {

robotlens::Json handle(const robotlens::Json& message) {
    return warehousekit::handleWith(warehousekit::buildPalletModule, message);
}

}             

#ifndef ROBOTLENS_PROCGEN_NATIVE_TEST
ROBOTLENS_MODULE(handle)
#endif
