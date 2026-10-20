#pragma once

#include "robotlens/extension.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace robotlens {

enum class LogLevel : int32_t {
    Info = ROBOTLENS_LOG_INFO,
    Warning = ROBOTLENS_LOG_WARNING,
    Error = ROBOTLENS_LOG_ERROR
};

inline void log(LogLevel level, std::string_view message) {
    robotlens_log(static_cast<int32_t>(level), message.data(),
                  static_cast<int32_t>(message.size()));
}

inline void output(std::string_view bytes) {
    robotlens_output(bytes.data(), static_cast<int32_t>(bytes.size()));
}

inline std::optional<std::vector<uint8_t>> readAsset(std::string_view path) {
    const auto length = static_cast<int32_t>(path.size());
    const int64_t size = robotlens_asset_size(path.data(), length);
    if (size < 0) {
        return std::nullopt;
    }
    std::vector<uint8_t> bytes(static_cast<std::size_t>(size));
    const int64_t read =
        robotlens_asset_read(path.data(), length, bytes.data(), static_cast<int32_t>(bytes.size()));
    if (read != size) {
        return std::nullopt;
    }
    return bytes;
}

class Json {
  public:
    using Array = std::vector<Json>;
    using Member = std::pair<std::string, Json>;
    using Object = std::vector<Member>;

    Json() = default;
    Json(std::nullptr_t) {}
    Json(bool value) : value_(value) {}
    Json(int value) : value_(static_cast<double>(value)) {}
    Json(int64_t value) : value_(static_cast<double>(value)) {}
    Json(double value) : value_(value) {}
    Json(float value) : value_(static_cast<double>(value)) {}
    Json(const char* value) : value_(std::string(value)) {}
    Json(std::string value) : value_(std::move(value)) {}
    Json(std::string_view value) : value_(std::string(value)) {}
    Json(Array value) : value_(std::move(value)) {}
    Json(Object value) : value_(std::move(value)) {}

    static Json array(std::initializer_list<Json> items = {}) { return Json(Array(items)); }
    static Json object(std::initializer_list<Member> members = {}) { return Json(Object(members)); }

    [[nodiscard]] bool isNull() const { return std::holds_alternative<std::nullptr_t>(value_); }
    [[nodiscard]] bool isBool() const { return std::holds_alternative<bool>(value_); }
    [[nodiscard]] bool isNumber() const { return std::holds_alternative<double>(value_); }
    [[nodiscard]] bool isString() const { return std::holds_alternative<std::string>(value_); }
    [[nodiscard]] bool isArray() const { return std::holds_alternative<Array>(value_); }
    [[nodiscard]] bool isObject() const { return std::holds_alternative<Object>(value_); }

    [[nodiscard]] bool asBool(bool fallback = false) const {
        const bool* value = std::get_if<bool>(&value_);
        return value == nullptr ? fallback : *value;
    }
    [[nodiscard]] double asNumber(double fallback = 0.0) const {
        const double* value = std::get_if<double>(&value_);
        return value == nullptr ? fallback : *value;
    }
    [[nodiscard]] int asInt(int fallback = 0) const {
        return isNumber() ? static_cast<int>(asNumber()) : fallback;
    }
    [[nodiscard]] std::string_view asString(std::string_view fallback = {}) const {
        const std::string* value = std::get_if<std::string>(&value_);
        return value == nullptr ? fallback : std::string_view(*value);
    }
    [[nodiscard]] const Array& items() const {
        static const Array kEmpty;
        const Array* value = std::get_if<Array>(&value_);
        return value == nullptr ? kEmpty : *value;
    }
    [[nodiscard]] const Object& members() const {
        static const Object kEmpty;
        const Object* value = std::get_if<Object>(&value_);
        return value == nullptr ? kEmpty : *value;
    }

    [[nodiscard]] const Json& operator[](std::string_view key) const {
        for (const Member& member : members()) {
            if (member.first == key) {
                return member.second;
            }
        }
        return null();
    }
    [[nodiscard]] const Json& operator[](std::size_t index) const {
        const Array& array = items();
        return index < array.size() ? array[index] : null();
    }

    Json& set(std::string key, Json value) {
        if (!isObject()) {
            value_ = Object{};
        }
        Object& object = std::get<Object>(value_);
        for (Member& member : object) {
            if (member.first == key) {
                member.second = std::move(value);
                return *this;
            }
        }
        object.emplace_back(std::move(key), std::move(value));
        return *this;
    }

    Json& push(Json value) {
        if (!isArray()) {
            value_ = Array{};
        }
        std::get<Array>(value_).push_back(std::move(value));
        return *this;
    }

    [[nodiscard]] std::string dump() const {
        std::string text;
        write(text);
        return text;
    }

    [[nodiscard]] static std::optional<Json> parse(std::string_view text) {
        Parser parser{text, 0U};
        std::optional<Json> value = parser.parseValue(0U);
        parser.skipSpace();
        if (!value || parser.position != text.size()) {
            return std::nullopt;
        }
        return value;
    }

  private:
    static const Json& null() {
        static const Json kNull;
        return kNull;
    }

    static void writeString(std::string& text, std::string_view value) {
        text += '"';
        for (const char character : value) {
            switch (character) {
            case '"':
                text += "\\\"";
                break;
            case '\\':
                text += "\\\\";
                break;
            case '\n':
                text += "\\n";
                break;
            case '\r':
                text += "\\r";
                break;
            case '\t':
                text += "\\t";
                break;
            default:
                if (static_cast<unsigned char>(character) < 0x20U) {
                    char escaped[8];
                    std::snprintf(escaped, sizeof(escaped), "\\u%04x",
                                  static_cast<unsigned>(static_cast<unsigned char>(character)));
                    text += escaped;
                } else {
                    text += character;
                }
            }
        }
        text += '"';
    }

    static void writeNumber(std::string& text, double value) {
        char buffer[32];
        const bool isFinite = value - value == 0.0;
        if (!isFinite) {
            std::snprintf(buffer, sizeof(buffer), "null");
        } else if (value > -1.0e15 && value < 1.0e15 &&
                   value == static_cast<double>(static_cast<int64_t>(value))) {
            std::snprintf(buffer, sizeof(buffer), "%lld", static_cast<long long>(value));
        } else {
            std::snprintf(buffer, sizeof(buffer), "%.17g", value);
        }
        text += buffer;
    }

    void write(std::string& text) const {
        if (isNull()) {
            text += "null";
        } else if (isBool()) {
            text += asBool() ? "true" : "false";
        } else if (isNumber()) {
            writeNumber(text, asNumber());
        } else if (isString()) {
            writeString(text, asString());
        } else if (isArray()) {
            text += '[';
            bool isFirst = true;
            for (const Json& item : items()) {
                text += isFirst ? "" : ",";
                isFirst = false;
                item.write(text);
            }
            text += ']';
        } else {
            text += '{';
            bool isFirst = true;
            for (const Member& member : members()) {
                text += isFirst ? "" : ",";
                isFirst = false;
                writeString(text, member.first);
                text += ':';
                member.second.write(text);
            }
            text += '}';
        }
    }

    struct Parser {
        static constexpr std::size_t kMaxDepth = 64U;

        std::string_view text;
        std::size_t position;

        void skipSpace() {
            while (position < text.size() && (text[position] == ' ' || text[position] == '\n' ||
                                              text[position] == '\r' || text[position] == '\t')) {
                ++position;
            }
        }

        bool consume(std::string_view literal) {
            if (text.substr(position, literal.size()) != literal) {
                return false;
            }
            position += literal.size();
            return true;
        }

        static void appendUtf8(std::string& out, uint32_t codepoint) {
            if (codepoint < 0x80U) {
                out += static_cast<char>(codepoint);
            } else if (codepoint < 0x800U) {
                out += static_cast<char>(0xC0U | (codepoint >> 6U));
                out += static_cast<char>(0x80U | (codepoint & 0x3FU));
            } else if (codepoint < 0x10000U) {
                out += static_cast<char>(0xE0U | (codepoint >> 12U));
                out += static_cast<char>(0x80U | ((codepoint >> 6U) & 0x3FU));
                out += static_cast<char>(0x80U | (codepoint & 0x3FU));
            } else {
                out += static_cast<char>(0xF0U | (codepoint >> 18U));
                out += static_cast<char>(0x80U | ((codepoint >> 12U) & 0x3FU));
                out += static_cast<char>(0x80U | ((codepoint >> 6U) & 0x3FU));
                out += static_cast<char>(0x80U | (codepoint & 0x3FU));
            }
        }

        std::optional<uint32_t> parseHex4() {
            if (position + 4U > text.size()) {
                return std::nullopt;
            }
            uint32_t value = 0U;
            for (std::size_t index = 0U; index < 4U; ++index) {
                const char digit = text[position++];
                value <<= 4U;
                if (digit >= '0' && digit <= '9') {
                    value |= static_cast<uint32_t>(digit - '0');
                } else if (digit >= 'a' && digit <= 'f') {
                    value |= static_cast<uint32_t>(digit - 'a' + 10);
                } else if (digit >= 'A' && digit <= 'F') {
                    value |= static_cast<uint32_t>(digit - 'A' + 10);
                } else {
                    return std::nullopt;
                }
            }
            return value;
        }

        std::optional<std::string> parseString() {
            if (!consume("\"")) {
                return std::nullopt;
            }
            std::string out;
            while (position < text.size()) {
                const char character = text[position++];
                if (character == '"') {
                    return out;
                }
                if (character != '\\') {
                    out += character;
                    continue;
                }
                if (position >= text.size()) {
                    return std::nullopt;
                }
                const char escape = text[position++];
                switch (escape) {
                case '"':
                case '\\':
                case '/':
                    out += escape;
                    break;
                case 'b':
                    out += '\b';
                    break;
                case 'f':
                    out += '\f';
                    break;
                case 'n':
                    out += '\n';
                    break;
                case 'r':
                    out += '\r';
                    break;
                case 't':
                    out += '\t';
                    break;
                case 'u': {
                    std::optional<uint32_t> codepoint = parseHex4();
                    if (!codepoint) {
                        return std::nullopt;
                    }
                    if (*codepoint >= 0xD800U && *codepoint <= 0xDBFFU && consume("\\u")) {
                        const std::optional<uint32_t> low = parseHex4();
                        if (!low || *low < 0xDC00U || *low > 0xDFFFU) {
                            return std::nullopt;
                        }
                        codepoint = 0x10000U + ((*codepoint - 0xD800U) << 10U) + (*low - 0xDC00U);
                    }
                    appendUtf8(out, *codepoint);
                    break;
                }
                default:
                    return std::nullopt;
                }
            }
            return std::nullopt;
        }

        std::optional<Json> parseNumber() {
            const std::size_t start = position;
            while (position < text.size() &&
                   std::string_view("+-0123456789.eE").find(text[position]) !=
                       std::string_view::npos) {
                ++position;
            }
            if (start == position) {
                return std::nullopt;
            }
            const std::string digits(text.substr(start, position - start));
            char* end = nullptr;
            const double value = std::strtod(digits.c_str(), &end);
            if (end != digits.c_str() + digits.size()) {
                return std::nullopt;
            }
            return Json(value);
        }

        std::optional<Json> parseValue(std::size_t depth) {
            if (depth > kMaxDepth) {
                return std::nullopt;
            }
            skipSpace();
            if (position >= text.size()) {
                return std::nullopt;
            }
            const char next = text[position];
            if (next == '{') {
                ++position;
                Json object = Json::object();
                skipSpace();
                if (consume("}")) {
                    return object;
                }
                while (true) {
                    skipSpace();
                    std::optional<std::string> key = parseString();
                    skipSpace();
                    if (!key || !consume(":")) {
                        return std::nullopt;
                    }
                    std::optional<Json> value = parseValue(depth + 1U);
                    if (!value) {
                        return std::nullopt;
                    }
                    object.set(std::move(*key), std::move(*value));
                    skipSpace();
                    if (consume("}")) {
                        return object;
                    }
                    if (!consume(",")) {
                        return std::nullopt;
                    }
                }
            }
            if (next == '[') {
                ++position;
                Json array = Json::array();
                skipSpace();
                if (consume("]")) {
                    return array;
                }
                while (true) {
                    std::optional<Json> value = parseValue(depth + 1U);
                    if (!value) {
                        return std::nullopt;
                    }
                    array.push(std::move(*value));
                    skipSpace();
                    if (consume("]")) {
                        return array;
                    }
                    if (!consume(",")) {
                        return std::nullopt;
                    }
                }
            }
            if (next == '"') {
                std::optional<std::string> value = parseString();
                return value ? std::optional<Json>(Json(std::move(*value))) : std::nullopt;
            }
            if (consume("true")) {
                return Json(true);
            }
            if (consume("false")) {
                return Json(false);
            }
            if (consume("null")) {
                return Json();
            }
            return parseNumber();
        }
    };

    std::variant<std::nullptr_t, bool, double, std::string, Array, Object> value_;
};

namespace ui {

using Children = std::vector<Json>;

inline Json node(std::string_view type) {
    return Json::object({{"type", Json(type)}});
}

inline Json container(std::string_view type, Children children) {
    return node(type).set("children", Json(std::move(children)));
}

inline Json column(Children children) {
    return container("column", std::move(children));
}

inline Json row(Children children) {
    return container("row", std::move(children));
}

inline Json group(std::string_view label, Children children) {
    return container("group", std::move(children)).set("label", Json(label));
}

inline Json tabs(Children tabNodes) {
    return container("tabs", std::move(tabNodes));
}

inline Json tab(std::string_view label, Children children) {
    return container("tab", std::move(children)).set("label", Json(label));
}

inline Json collapsing(std::string_view label, Children children, bool isOpen = true) {
    return container("collapsing", std::move(children))
        .set("label", Json(label))
        .set("open", Json(isOpen));
}

inline Json separator() {
    return node("separator");
}

inline Json text(std::string_view content, std::string_view style = "normal") {
    return node("text").set("text", Json(content)).set("style", Json(style));
}

inline Json heading(std::string_view content) {
    return text(content, "heading");
}

inline Json widget(std::string_view type, std::string_view id, std::string_view label) {
    return node(type).set("id", Json(id)).set("label", Json(label));
}

inline Json button(std::string_view id, std::string_view label) {
    return widget("button", id, label);
}

inline Json checkbox(std::string_view id, std::string_view label, bool value) {
    return widget("checkbox", id, label).set("value", Json(value));
}

inline Json slider(std::string_view id, std::string_view label, double value, double minimum,
                   double maximum, bool isInteger = false) {
    return widget("slider", id, label)
        .set("value", Json(value))
        .set("min", Json(minimum))
        .set("max", Json(maximum))
        .set("integer", Json(isInteger));
}

inline Json drag(std::string_view id, std::string_view label, double value, double speed = 0.1) {
    return widget("drag", id, label).set("value", Json(value)).set("speed", Json(speed));
}

inline Json optionList(const std::vector<std::string>& options) {
    Json list = Json::array();
    for (const std::string& option : options) {
        list.push(Json(option));
    }
    return list;
}

inline Json combo(std::string_view id, std::string_view label,
                  const std::vector<std::string>& options, int selected) {
    return widget("combo", id, label)
        .set("options", optionList(options))
        .set("selected", Json(selected));
}

inline Json list(std::string_view id, std::string_view label,
                 const std::vector<std::string>& options, int selected) {
    return widget("list", id, label)
        .set("options", optionList(options))
        .set("selected", Json(selected));
}

inline Json textInput(std::string_view id, std::string_view label, std::string_view value) {
    return widget("text_input", id, label).set("value", Json(value));
}

inline Json colorValue(float red, float green, float blue, float alpha = 1.0F) {
    return Json::array({Json(red), Json(green), Json(blue), Json(alpha)});
}

inline Json color(std::string_view id, std::string_view label, float red, float green, float blue) {
    return widget("color", id, label)
        .set("value", Json::array({Json(red), Json(green), Json(blue)}));
}

inline Json table(const std::vector<std::string>& columns,
                  const std::vector<std::vector<std::string>>& rows) {
    Json rowList = Json::array();
    for (const std::vector<std::string>& row : rows) {
        rowList.push(optionList(row));
    }
    return node("table").set("columns", optionList(columns)).set("rows", std::move(rowList));
}

inline Json plot(std::string_view label, const std::vector<float>& values, bool isBars = false) {
    Json list = Json::array();
    for (const float value : values) {
        list.push(Json(value));
    }
    return node("plot")
        .set("label", Json(label))
        .set("values", std::move(list))
        .set("kind", Json(isBars ? "bars" : "lines"));
}

inline Json canvas(float width, float height, Children shapes) {
    return node("canvas")
        .set("width", Json(width))
        .set("height", Json(height))
        .set("shapes", Json(std::move(shapes)));
}

inline Json shape(std::string_view kind, std::initializer_list<float> points, Json shapeColor) {
    Json pointList = Json::array();
    for (const float point : points) {
        pointList.push(Json(point));
    }
    return Json::object({{"shape", Json(kind)}})
        .set("points", std::move(pointList))
        .set("color", std::move(shapeColor));
}

inline Json line(float x0, float y0, float x1, float y1, Json shapeColor, float thickness = 1.0F) {
    return shape("line", {x0, y0, x1, y1}, std::move(shapeColor)).set("thickness", Json(thickness));
}

inline Json rect(float x0, float y0, float x1, float y1, Json shapeColor, bool isFilled = false) {
    return shape("rect", {x0, y0, x1, y1}, std::move(shapeColor)).set("filled", Json(isFilled));
}

inline Json circle(float x, float y, float radius, Json shapeColor, bool isFilled = false) {
    return shape("circle", {x, y}, std::move(shapeColor))
        .set("radius", Json(radius))
        .set("filled", Json(isFilled));
}

inline Json label(float x, float y, std::string_view content, Json shapeColor) {
    return shape("text", {x, y}, std::move(shapeColor)).set("text", Json(content));
}

}                

namespace detail {

template <Json (*Handler)(const Json&)> int32_t dispatch(const uint8_t* input, int32_t length) {
    const std::optional<Json> message = Json::parse(
        std::string_view(reinterpret_cast<const char*>(input), static_cast<std::size_t>(length)));
    if (!message) {
        log(LogLevel::Error, "Module received a message that is not valid JSON");
        return 1;
    }
    const Json reply = Handler(*message);
    if (!reply.isNull()) {
        output(reply.dump());
    }
    return 0;
}

}                    

}                       

#define ROBOTLENS_MODULE(handler)                                                                  \
    [[noreturn]] void std::__libcpp_verbose_abort(const char*, ...) {                              \
        __builtin_trap();                                                                          \
    }                                                                                              \
    static int32_t robotlens_dispatch(const uint8_t* input, int32_t length) {                      \
        return ::robotlens::detail::dispatch<handler>(input, length);                              \
    }                                                                                              \
    ROBOTLENS_DEFINE_C_MODULE(robotlens_dispatch)
