#include "betaflight_gazebo_bridge/Rangefinder.hh"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <mutex>
#include <stdexcept>

#include <arpa/inet.h>
#include <fcntl.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/transport/Node.hh>
#include <spdlog/spdlog.h>
#include <sys/socket.h>
#include <unistd.h>

#include "betaflight_gazebo_bridge/UdpSocket.hh"

extern "C" {
#include <common/mavlink.h>
}

namespace betaflight_gazebo_bridge
{
namespace
{

using Clock = std::chrono::steady_clock;
constexpr auto kStaleAfter = std::chrono::milliseconds(250);

std::uint16_t Centimetres(const double metres)
{
    return static_cast<std::uint16_t>(metres * 100.0);
}

void SendMavlink(UdpSocket &socket, mavlink_message_t &message)
{
    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const auto size = mavlink_msg_to_send_buffer(buffer.data(), &message);
    socket.Send(buffer.data(), size);
}

}  // namespace

std::array<std::uint8_t, 9> EncodeTfmini(const std::optional<double> metres)
{
    const std::uint16_t cm = metres && std::isfinite(*metres) && *metres >= 0.0 && Centimetres(*metres) <= 1200
        ? Centimetres(*metres) : 1201;
    std::array<std::uint8_t, 9> frame{0x59, 0x59, static_cast<std::uint8_t>(cm), static_cast<std::uint8_t>(cm >> 8),
                                      0x64, 0x00, 0x01, 0x00, 0x00};
    for (std::size_t i = 0; i < 8; ++i) frame[8] = static_cast<std::uint8_t>(frame[8] + frame[i]);
    return frame;
}

struct RangefinderManager::Impl
{
    struct Mapping
    {
        RangefinderConfig config;
        mutable std::mutex mutex;
        std::optional<double> metres;
        double minMetres{0.0};
        double maxMetres{0.0};
        Clock::time_point sampleTime{};
        Clock::time_point lastSampleSent{};
        Clock::time_point lastOutput{};
        Clock::time_point lastConnectAttempt{};
        int tcp{-1};
        bool connecting{false};
        bool topicReady{false};
        std::uint64_t outputs{0};
        std::uint64_t invalid{0};

        ~Mapping() { if (tcp >= 0) close(tcp); }
    };

    Impl(const std::vector<RangefinderConfig> &configs, const MavlinkConfig &mavlinkConfig,
         const Clock::time_point bridgeStart)
        : mavlink(mavlinkConfig), start(bridgeStart)
    {
        const bool hasMavlink = std::any_of(configs.begin(), configs.end(), [](const auto &c) { return c.enabled && c.output == "mavlink"; });
        if (hasMavlink) udp.SetDestination(mavlink.address, mavlink.port);
        mappings.reserve(configs.size());
        for (const auto &config : configs) {
            if (!config.enabled) {
                spdlog::info("rangefinder[{}] disabled", config.name);
                continue;
            }
            auto mapping = std::make_unique<Mapping>();
            mapping->config = config;
            Mapping *raw = mapping.get();
            std::function<void(const gz::msgs::LaserScan &)> callback = [raw](const gz::msgs::LaserScan &scan) {
                    std::lock_guard lock(raw->mutex);
                    raw->topicReady = true;
                    raw->sampleTime = Clock::now();
                    raw->minMetres = scan.range_min();
                    raw->maxMetres = scan.range_max();
                    raw->metres = scan.ranges_size() == 1 ? std::optional<double>(scan.ranges(0)) : std::nullopt;
                };
            if (!node.Subscribe<gz::msgs::LaserScan>(config.gazeboTopic, std::move(callback))) {
                throw std::runtime_error("Failed to subscribe to rangefinder topic: " + config.gazeboTopic);
            }
            mappings.push_back(std::move(mapping));
            spdlog::info("rangefinder[{}] output={} subscribed to [{}]", config.name, config.output, config.gazeboTopic);
        }
    }

    void Connect(Mapping &mapping, const Clock::time_point now)
    {
        if (mapping.tcp >= 0 || now - mapping.lastConnectAttempt < std::chrono::seconds(1)) return;
        mapping.lastConnectAttempt = now;
        mapping.tcp = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, IPPROTO_TCP);
        if (mapping.tcp < 0) return;
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(mapping.config.sitlPort);
        if (inet_pton(AF_INET, mapping.config.sitlAddress.c_str(), &address.sin_addr) != 1) {
            close(mapping.tcp); mapping.tcp = -1; return;
        }
        const int result = connect(mapping.tcp, reinterpret_cast<sockaddr *>(&address), sizeof(address));
        if (result == 0) {
            spdlog::info("rangefinder[{}] TCP connected", mapping.config.name);
        } else if (errno == EINPROGRESS) {
            mapping.connecting = true;
        } else {
            close(mapping.tcp); mapping.tcp = -1;
        }
    }

    bool Connected(Mapping &mapping)
    {
        if (!mapping.connecting) return mapping.tcp >= 0;
        int error = 0; socklen_t size = sizeof(error);
        if (getsockopt(mapping.tcp, SOL_SOCKET, SO_ERROR, &error, &size) < 0 || error != 0) {
            if (error == EINPROGRESS) return false;
            close(mapping.tcp); mapping.tcp = -1; mapping.connecting = false;
            return false;
        }
        mapping.connecting = false;
        spdlog::info("rangefinder[{}] TCP connected", mapping.config.name);
        return true;
    }

    std::optional<double> Sample(Mapping &mapping, const Clock::time_point now, double &minimum, double &maximum)
    {
        std::lock_guard lock(mapping.mutex);
        minimum = mapping.minMetres; maximum = mapping.maxMetres;
        if (!mapping.metres || now - mapping.sampleTime > kStaleAfter || !std::isfinite(*mapping.metres) || *mapping.metres < 0.0) {
            return std::nullopt;
        }
        return mapping.metres;
    }

    void UpdateTfmini(Mapping &mapping, const Clock::time_point now)
    {
        Connect(mapping, now);
        if (!Connected(mapping) || now - mapping.lastOutput < std::chrono::milliseconds(10)) return;
        double minimum, maximum;
        auto sample = Sample(mapping, now, minimum, maximum);
        const auto frame = EncodeTfmini(sample);
        const auto sent = send(mapping.tcp, frame.data(), frame.size(), MSG_NOSIGNAL);
        mapping.lastOutput = now;
        if (sent != static_cast<ssize_t>(frame.size())) {
            close(mapping.tcp); mapping.tcp = -1; mapping.connecting = false;
            spdlog::warn("rangefinder[{}] TCP disconnected", mapping.config.name);
            return;
        }
        ++mapping.outputs;
        if (!sample || Centimetres(*sample) > 1200) ++mapping.invalid;
    }

    void UpdateMavlink(Mapping &mapping, const Clock::time_point now)
    {
        double minimum, maximum;
        const auto sample = Sample(mapping, now, minimum, maximum);
        if (mapping.config.mavlinkMessage == "distance_sensor" && (!sample || *sample > maximum)) { ++mapping.invalid; return; }
        const auto minCm = Centimetres(std::max(0.0, minimum));
        const auto maxCm = Centimetres(std::clamp(maximum, 0.0, 655.35));
        mavlink_message_t message{};
        if (mapping.config.mavlinkMessage == "distance_sensor") {
            const float quaternion[4]{};
            mavlink_msg_distance_sensor_pack(mavlink.systemId, mavlink.componentId, &message,
                static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count()),
                minCm, maxCm, Centimetres(std::clamp(*sample, 0.0, 655.35)), MAV_DISTANCE_SENSOR_LASER, *mapping.config.sensorId,
                MAV_SENSOR_ROTATION_NONE, UINT8_MAX, 0, 0, quaternion, 0);
        } else {
            std::array<std::uint16_t, 72> distances{};
            distances.fill(UINT16_MAX);
            if (sample) distances[0] = *sample > maximum ? static_cast<std::uint16_t>(maxCm + 1) : Centimetres(std::clamp(*sample, 0.0, 655.35));
            else ++mapping.invalid;
            mavlink_msg_obstacle_distance_pack(mavlink.systemId, mavlink.componentId, &message,
                std::chrono::duration_cast<std::chrono::microseconds>(now - start).count(), MAV_DISTANCE_SENSOR_LASER,
                distances.data(), 0, minCm, maxCm, 0, 0, MAV_FRAME_BODY_FRD);
        }
        SendMavlink(udp, message);
        mapping.lastOutput = now;
        ++mapping.outputs;
    }

    void Update()
    {
        const auto now = Clock::now();
        for (auto &mapping : mappings) {
            if (mapping->config.output == "tfmini") UpdateTfmini(*mapping, now);
        }
    }

    MavlinkConfig mavlink;
    Clock::time_point start;
    UdpSocket udp;
    gz::transport::Node node;
    std::vector<std::unique_ptr<Mapping>> mappings;
};

RangefinderManager::RangefinderManager(const std::vector<RangefinderConfig> &configs, const MavlinkConfig &mavlink,
                                       const Clock::time_point startTime)
    : impl_(std::make_unique<Impl>(configs, mavlink, startTime)) {}

RangefinderManager::~RangefinderManager() = default;

void RangefinderManager::Update()
{
    impl_->Update();
    const auto now = Clock::now();
    for (auto &mapping : impl_->mappings) {
        if (mapping->config.output != "mavlink") continue;
        bool fresh = false;
        {
            std::lock_guard lock(mapping->mutex);
            fresh = mapping->sampleTime > mapping->lastSampleSent;
            if (fresh) mapping->lastSampleSent = mapping->sampleTime;
        }
        if (!fresh && (mapping->config.mavlinkMessage != "obstacle_distance" ||
                       now - mapping->lastOutput < std::chrono::milliseconds(50))) continue;
        impl_->UpdateMavlink(*mapping, now);
    }
}

void RangefinderManager::LogStatus() const
{
    for (const auto &mapping : impl_->mappings) {
        std::lock_guard lock(mapping->mutex);
        if (mapping->config.output == "tfmini") {
            spdlog::info("rangefinder[{}] output=tfmini topic={} tcp={} frames={} invalid={}", mapping->config.name,
                mapping->topicReady ? "ready" : "waiting", mapping->tcp >= 0 && !mapping->connecting ? "connected" : "disconnected",
                mapping->outputs, mapping->invalid);
        } else {
            spdlog::info("rangefinder[{}] output=mavlink topic={} messages={} invalid={}", mapping->config.name,
                mapping->topicReady ? "ready" : "waiting", mapping->outputs, mapping->invalid);
        }
    }
}

}  // namespace betaflight_gazebo_bridge
