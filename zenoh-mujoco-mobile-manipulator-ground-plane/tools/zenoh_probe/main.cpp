#include <zenoh.hxx>

#include <chrono>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>

int publish(const std::string &key, const std::string &payload, double seconds,
            double rateHz) {
  zenoh::Session session(zenoh::Config::create_default());
  const auto period = std::chrono::duration<double>(1.0 / rateHz);
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (std::chrono::steady_clock::now() < end) {
    session.put(zenoh::KeyExpr(key), zenoh::Bytes(payload));
    std::this_thread::sleep_for(period);
  }
  return 0;
}

int watch(const std::string &keyExpression, double seconds) {
  zenoh::Session session(zenoh::Config::create_default());
  std::mutex mutex;
  const auto start = std::chrono::steady_clock::now();
  auto subscriber = session.declare_subscriber(
      zenoh::KeyExpr(keyExpression),
      [&](const zenoh::Sample &sample) {
        const double t =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        std::lock_guard lock(mutex);
        std::cout << t << ' ' << sample.get_keyexpr().as_string_view() << ' '
                  << sample.get_payload().as_string() << std::endl;
      },
      [] {});
  std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
  return 0;
}

int main(int argc, char **argv) {
  if (argc == 4 && std::string(argv[1]) == "watch") {
    return watch(argv[3], std::stod(argv[2]));
  }
  if (argc == 6 && std::string(argv[1]) == "put") {
    return publish(argv[2], argv[3], std::stod(argv[4]), std::stod(argv[5]));
  }
  if (argc < 2 || argc > 3) {
    std::cerr << "usage: zenoh_probe <seconds> [key-expression] | watch <seconds> <key-expression> | put <key> <json> <seconds> <hz>\n";
    return 2;
  }
  const double seconds = std::stod(argv[1]);
  const std::string keyExpression = argc == 3 ? argv[2] : "**";
  zenoh::Session session(zenoh::Config::create_default());
  std::mutex mutex;
  std::map<std::string, std::pair<int, std::string>> samples;
  auto subscriber = session.declare_subscriber(
      zenoh::KeyExpr(keyExpression),
      [&](const zenoh::Sample &sample) {
        std::lock_guard lock(mutex);
        auto &entry = samples[std::string(sample.get_keyexpr().as_string_view())];
        ++entry.first;
        entry.second = sample.get_payload().as_string();
      },
      [] {});
  std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
  std::lock_guard lock(mutex);
  for (const auto &[key, entry] : samples) {
    std::cout << key << " count=" << entry.first << " last=" << entry.second.substr(0, 4000)
              << '\n';
  }
  return 0;
}
