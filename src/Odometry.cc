#include "betaflight_gazebo_bridge/Odometry.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <mutex>
#include <numeric>
#include <stdexcept>

#include <gz/msgs/odometry.pb.h>
#include <gz/transport/Node.hh>
#include <spdlog/spdlog.h>

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

bool Finite(const OdometrySample &sample)
{
    const auto finite = [](const auto &values) {
        return std::all_of(values.begin(), values.end(), [](const auto value) { return std::isfinite(value); });
    };
    const double quaternionNorm = std::sqrt(std::inner_product(sample.orientation.begin(), sample.orientation.end(),
        sample.orientation.begin(), 0.0));
    return finite(sample.position) && finite(sample.orientation) && quaternionNorm > 0.0 &&
           finite(sample.linearVelocity) && finite(sample.angularVelocity);
}

std::array<float, 4> EnuFluToNedFrdQuaternion(const std::array<double, 4> &q)
{
    const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!std::isfinite(norm) || norm == 0.0) return {};
    const double w = q[0] / norm, x = q[1] / norm, y = q[2] / norm, z = q[3] / norm;
    const double r[3][3] = {
        {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
        {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
        {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}};
    const double m[3][3] = {
        {r[1][0], -r[1][1], -r[1][2]},
        {r[0][0], -r[0][1], -r[0][2]},
        {-r[2][0], r[2][1], r[2][2]}};
    const double trace = m[0][0] + m[1][1] + m[2][2];
    std::array<float, 4> out{};
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        out = {static_cast<float>(0.25 * s), static_cast<float>((m[2][1] - m[1][2]) / s),
               static_cast<float>((m[0][2] - m[2][0]) / s), static_cast<float>((m[1][0] - m[0][1]) / s)};
    } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
        const double s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
        out = {static_cast<float>((m[2][1] - m[1][2]) / s), static_cast<float>(0.25 * s),
               static_cast<float>((m[0][1] + m[1][0]) / s), static_cast<float>((m[0][2] + m[2][0]) / s)};
    } else if (m[1][1] > m[2][2]) {
        const double s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
        out = {static_cast<float>((m[0][2] - m[2][0]) / s), static_cast<float>((m[0][1] + m[1][0]) / s),
               static_cast<float>(0.25 * s), static_cast<float>((m[1][2] + m[2][1]) / s)};
    } else {
        const double s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
        out = {static_cast<float>((m[1][0] - m[0][1]) / s), static_cast<float>((m[0][2] + m[2][0]) / s),
               static_cast<float>((m[1][2] + m[2][1]) / s), static_cast<float>(0.25 * s)};
    }
    return out;
}

void SendMavlink(UdpSocket &socket, mavlink_message_t &message)
{
    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const auto size = mavlink_msg_to_send_buffer(buffer.data(), &message);
    socket.Send(buffer.data(), size);
}

}  // namespace

std::optional<ConvertedOdometry> OdometryConverter::Convert(const OdometrySample &sample)
{
    if (!Finite(sample)) return std::nullopt;
    if (lastTimeUsec_ && sample.timeUsec < *lastTimeUsec_) {
        ++resetCounter_;
        origin_.reset();
    }
    lastTimeUsec_ = sample.timeUsec;
    if (!origin_) origin_ = sample.position;

    ConvertedOdometry result;
    result.timeUsec = sample.timeUsec;
    result.position = {static_cast<float>(sample.position[1] - (*origin_)[1]),
                       static_cast<float>(sample.position[0] - (*origin_)[0]),
                       static_cast<float>(-(sample.position[2] - (*origin_)[2]))};
    result.orientation = EnuFluToNedFrdQuaternion(sample.orientation);
    result.linearVelocity = {static_cast<float>(sample.linearVelocity[0]),
                             static_cast<float>(-sample.linearVelocity[1]),
                             static_cast<float>(-sample.linearVelocity[2])};
    result.angularVelocity = {static_cast<float>(sample.angularVelocity[0]),
                              static_cast<float>(-sample.angularVelocity[1]),
                              static_cast<float>(-sample.angularVelocity[2])};
    result.resetCounter = resetCounter_;
    return result;
}

struct OdometryManager::Impl
{
    Impl(const OdometryConfig &odometryConfig, const MavlinkConfig &mavlinkConfig)
        : enabled(odometryConfig.enabled), mavlink(mavlinkConfig)
    {
        if (!enabled) {
            spdlog::info("odometry disabled");
            return;
        }
        udp.SetDestination(mavlink.address, mavlink.port);
        std::function<void(const gz::msgs::Odometry &)> callback = [this](const auto &msg) { OnOdometry(msg); };
        if (!node.Subscribe<gz::msgs::Odometry>(odometryConfig.gazeboTopic, std::move(callback))) {
            throw std::runtime_error("Failed to subscribe to odometry topic: " + odometryConfig.gazeboTopic);
        }
        spdlog::info("odometry subscribed to [{}]", odometryConfig.gazeboTopic);
    }

    void OnOdometry(const gz::msgs::Odometry &msg)
    {
        if (!msg.has_header() || !msg.header().has_stamp() || !msg.has_pose() || !msg.has_twist()) return;
        const auto &stamp = msg.header().stamp();
        if (stamp.sec() < 0 || stamp.nsec() < 0 || stamp.nsec() >= 1000000000) return;
        OdometrySample next;
        next.timeUsec = static_cast<std::uint64_t>(stamp.sec()) * 1000000ULL +
                        static_cast<std::uint64_t>(stamp.nsec()) / 1000ULL;
        const auto &position = msg.pose().position();
        const auto &orientation = msg.pose().orientation();
        const auto &linear = msg.twist().linear();
        const auto &angular = msg.twist().angular();
        next.position = {position.x(), position.y(), position.z()};
        next.orientation = {orientation.w(), orientation.x(), orientation.y(), orientation.z()};
        next.linearVelocity = {linear.x(), linear.y(), linear.z()};
        next.angularVelocity = {angular.x(), angular.y(), angular.z()};
        std::lock_guard lock(mutex);
        sample = next;
        sampleReceived = Clock::now();
        ++generation;
    }

    void Update()
    {
        if (!enabled) return;
        OdometrySample next;
        {
            std::lock_guard lock(mutex);
            if (!sample || generation == sentGeneration || Clock::now() - sampleReceived > kStaleAfter) return;
            next = *sample;
            sentGeneration = generation;
        }
        const auto converted = converter.Convert(next);
        if (!converted) {
            ++invalid;
            return;
        }
        std::array<float, 21> poseCovariance{};
        std::array<float, 21> velocityCovariance{};
        poseCovariance[0] = std::numeric_limits<float>::quiet_NaN();
        velocityCovariance[0] = std::numeric_limits<float>::quiet_NaN();
        mavlink_message_t message{};
        mavlink_msg_odometry_pack(mavlink.systemId, mavlink.componentId, &message, converted->timeUsec,
            MAV_FRAME_LOCAL_NED, MAV_FRAME_BODY_FRD, converted->position[0], converted->position[1], converted->position[2],
            converted->orientation.data(), converted->linearVelocity[0], converted->linearVelocity[1],
            converted->linearVelocity[2], converted->angularVelocity[0], converted->angularVelocity[1],
            converted->angularVelocity[2], poseCovariance.data(), velocityCovariance.data(), converted->resetCounter,
            MAV_ESTIMATOR_TYPE_NAIVE, 0);
        SendMavlink(udp, message);
        ++messages;
    }

    bool enabled{};
    MavlinkConfig mavlink;
    mutable std::mutex mutex;
    std::optional<OdometrySample> sample;
    Clock::time_point sampleReceived{};
    std::uint64_t generation{};
    std::uint64_t sentGeneration{};
    std::uint64_t messages{};
    std::uint64_t invalid{};
    OdometryConverter converter;
    UdpSocket udp;
    gz::transport::Node node;
};

OdometryManager::OdometryManager(const OdometryConfig &config, const MavlinkConfig &mavlink)
    : impl_(std::make_unique<Impl>(config, mavlink)) {}

OdometryManager::~OdometryManager() = default;

void OdometryManager::Update() { impl_->Update(); }

void OdometryManager::LogStatus() const
{
    if (!impl_->enabled) return;
    std::lock_guard lock(impl_->mutex);
    spdlog::info("odometry topic={} messages={} invalid={}", impl_->sample ? "ready" : "waiting", impl_->messages, impl_->invalid);
}

}  // namespace betaflight_gazebo_bridge
