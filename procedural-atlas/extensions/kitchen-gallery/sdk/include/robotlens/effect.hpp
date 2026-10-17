#pragma once

#include "robotlens/extension.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>
#include <vector>

namespace robotlens::effects {

inline constexpr uint32_t kStepMagic = 0x31584652U;
inline constexpr uint32_t kForcesMagic = 0x314F4652U;
inline constexpr std::size_t kStepHeaderBytes = 32U;
inline constexpr std::size_t kEnvironmentHeaderBytes = 24U;
inline constexpr std::size_t kBodyDoubles = 13U;

struct Vector3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct BodyState {
    Vector3 position;
    double orientation[4] = {1.0, 0.0, 0.0, 0.0};
    Vector3 linearVelocity;
    Vector3 angularVelocity;
};

class StepInput {
  public:
    StepInput(const uint8_t* bytes, std::size_t length) : bytes_(bytes), length_(length) {}

    [[nodiscard]] bool isValid() const {
        if (length_ < kStepHeaderBytes || read<uint32_t>(0U) != kStepMagic) {
            return false;
        }
        return length_ == kStepHeaderBytes + environmentCount() * environmentStride();
    }
    [[nodiscard]] std::size_t environmentCount() const { return read<uint32_t>(4U); }
    [[nodiscard]] std::size_t bodyCount() const { return read<uint32_t>(8U); }
    [[nodiscard]] double timestep() const { return read<double>(16U); }
    [[nodiscard]] uint64_t runSeed() const { return read<uint64_t>(24U); }
    [[nodiscard]] uint32_t environmentIndex(std::size_t slot) const {
        return read<uint32_t>(environmentOffset(slot));
    }
    [[nodiscard]] uint64_t episodeIndex(std::size_t slot) const {
        return read<uint64_t>(environmentOffset(slot) + 8U);
    }
    [[nodiscard]] double time(std::size_t slot) const {
        return read<double>(environmentOffset(slot) + 16U);
    }
    [[nodiscard]] BodyState body(std::size_t slot, std::size_t bodyIndex) const {
        const std::size_t offset = environmentOffset(slot) + kEnvironmentHeaderBytes +
                                   bodyIndex * kBodyDoubles * sizeof(double);
        double values[kBodyDoubles];
        std::memcpy(values, bytes_ + offset, sizeof(values));
        BodyState state;
        state.position = {values[0], values[1], values[2]};
        for (std::size_t component = 0U; component < 4U; ++component) {
            state.orientation[component] = values[3U + component];
        }
        state.linearVelocity = {values[7], values[8], values[9]};
        state.angularVelocity = {values[10], values[11], values[12]};
        return state;
    }

  private:
    template <typename Value> Value read(std::size_t offset) const {
        Value value{};
        std::memcpy(&value, bytes_ + offset, sizeof(Value));
        return value;
    }
    [[nodiscard]] std::size_t environmentStride() const {
        return kEnvironmentHeaderBytes + bodyCount() * kBodyDoubles * sizeof(double);
    }
    [[nodiscard]] std::size_t environmentOffset(std::size_t slot) const {
        return kStepHeaderBytes + slot * environmentStride();
    }

    const uint8_t* bytes_;
    std::size_t length_;
};

class ForceOutput {
  public:
    ForceOutput() { appendHeader(); }

    void add(uint32_t environmentSlot, uint32_t bodyIndex, Vector3 force, Vector3 torque = {}) {
        addRecord(environmentSlot, bodyIndex, force, torque, nullptr);
    }
    void addAtPoint(uint32_t environmentSlot, uint32_t bodyIndex, Vector3 force, Vector3 point) {
        addRecord(environmentSlot, bodyIndex, force, {}, &point);
    }
    [[nodiscard]] std::size_t count() const { return count_; }
    [[nodiscard]] std::string_view bytes() {
        std::memcpy(bytes_.data() + 4U, &count_, sizeof(count_));
        return {reinterpret_cast<const char*>(bytes_.data()), bytes_.size()};
    }

  private:
    template <typename Value> void append(Value value) {
        const std::size_t offset = bytes_.size();
        bytes_.resize(offset + sizeof(Value));
        std::memcpy(bytes_.data() + offset, &value, sizeof(Value));
    }
    void appendHeader() {
        append(kForcesMagic);
        append(uint32_t{0});
    }
    void appendVector(const Vector3& vector) {
        append(vector.x);
        append(vector.y);
        append(vector.z);
    }
    void addRecord(uint32_t environmentSlot, uint32_t bodyIndex, const Vector3& force,
                   const Vector3& torque, const Vector3* point) {
        append(environmentSlot);
        append(bodyIndex);
        append(uint32_t{point != nullptr ? 1U : 0U});
        append(uint32_t{0});
        appendVector(force);
        appendVector(torque);
        appendVector(point != nullptr ? *point : Vector3{});
        ++count_;
    }

    std::vector<uint8_t> bytes_;
    uint32_t count_ = 0U;
};

namespace detail {

template <Json (*Configure)(const Json&), int32_t (*Step)(const StepInput&, ForceOutput&)>
int32_t dispatch(const uint8_t* input, int32_t length) {
    if (length > 0 && input[0] == '{') {
        const std::optional<Json> message = Json::parse(std::string_view(
            reinterpret_cast<const char*>(input), static_cast<std::size_t>(length)));
        if (!message) {
            return 1;
        }
        const Json reply = Configure(*message);
        if (!reply.isNull()) {
            output(reply.dump());
        }
        return 0;
    }
    const StepInput step(input, static_cast<std::size_t>(length));
    if (!step.isValid()) {
        log(LogLevel::Error, "Effect received a malformed step message");
        return 1;
    }
    ForceOutput forces;
    const int32_t status = Step(step, forces);
    output(forces.bytes());
    return status;
}

}                    

}                                

#define ROBOTLENS_EFFECT_MODULE(configure, step)                                                   \
    [[noreturn]] void std::__libcpp_verbose_abort(const char*, ...) {                              \
        __builtin_trap();                                                                          \
    }                                                                                              \
    static int32_t robotlens_effect_dispatch(const uint8_t* input, int32_t length) {               \
        return ::robotlens::effects::detail::dispatch<configure, step>(input, length);             \
    }                                                                                              \
    ROBOTLENS_DEFINE_C_MODULE(robotlens_effect_dispatch)
