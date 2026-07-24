#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include <spdlog/spdlog.h>

#include "betaflight_gazebo_bridge/BridgeApp.hh"
#include "betaflight_gazebo_bridge/Config.hh"

namespace
{

std::filesystem::path ExecutableDirectory(const char *argv0)
{
    std::error_code error;
    const auto canonical = std::filesystem::canonical("/proc/self/exe", error);
    if (!error) {
        return canonical.parent_path();
    }
    return std::filesystem::absolute(argv0).parent_path();
}

std::filesystem::path ParseConfigPath(const int argc, char **argv)
{
    std::filesystem::path configPath = ExecutableDirectory(argv[0]) / "bridge.yaml";
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [--config path/to/bridge.yaml]\n";
            std::exit(0);
        }
        if (arg == "--config") {
            if (i + 1 >= argc) {
                throw std::runtime_error("--config requires a file path");
            }
            configPath = argv[++i];
            continue;
        }
        throw std::runtime_error("Unknown argument: " + arg);
    }
    return configPath;
}

void ConfigureLogging(const std::string &level)
{
    const auto parsed = spdlog::level::from_str(level);
    spdlog::set_level(parsed == spdlog::level::off && level != "off" ? spdlog::level::info : parsed);
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
}

}  // namespace

int main(const int argc, char **argv)
{
    try {
        const auto configPath = ParseConfigPath(argc, argv);
        const auto config = betaflight_gazebo_bridge::ConfigLoader::Load(configPath);
        ConfigureLogging(config.logging.level);
        spdlog::info("Using config [{}]", configPath.string());

        betaflight_gazebo_bridge::BridgeApp app(config);
        return app.Run();
    } catch (const std::exception &error) {
        std::cerr << "bridge error: " << error.what() << '\n';
        return 1;
    }
}

