#include "betaflight_gazebo_bridge/BridgeApp.hh"

#include <atomic>
#include <csignal>
#include <cstring>
#include <thread>

#include <spdlog/spdlog.h>

namespace betaflight_gazebo_bridge
{
namespace
{

std::atomic_bool g_stopRequested{false};

void SignalHandler(int)
{
    g_stopRequested.store(true);
}

double SecondsSince(const std::chrono::steady_clock::time_point start)
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

}  // namespace

BridgeApp::BridgeApp(BridgeConfig config)
    : config_(std::move(config)),
      stateSubscriber_(config_.gazebo),
      actuatorPublisher_(config_.gazebo),
      fdmBuilder_(config_.fdm),
      motorMapper_(config_.motors.map),
      velocityConverter_(config_.motors.minRotorVelocityRadS, config_.motors.maxRotorVelocityRadS),
      startTime_(std::chrono::steady_clock::now())
{
    motorSocket_.Bind(config_.sitl.motorPort);
    fdmSocket_.SetDestination(config_.sitl.address, config_.sitl.fdmPort);
    spdlog::info("Listening for Betaflight motors on UDP {}", config_.sitl.motorPort);
    spdlog::info("Sending FDM to {}:{}", config_.sitl.address, config_.sitl.fdmPort);
}

int BridgeApp::Run()
{
    g_stopRequested.store(false);
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    spdlog::info("Bridge started");
    while (!g_stopRequested.load()) {
        ReceiveMotorPackets();
        PublishMotorCommandIfNeeded();
        SendFdmIfNeeded();
        LogStatusIfNeeded();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    actuatorPublisher_.Publish(ZeroMotors());
    spdlog::info("Bridge stopped; published zero motor command");
    return 0;
}

void BridgeApp::ReceiveMotorPackets()
{
    std::array<unsigned char, 1024> buffer{};
    while (true) {
        const auto received = motorSocket_.Receive(buffer.data(), buffer.size());
        if (!received) {
            break;
        }
        if (*received != sizeof(ServoPacket)) {
            ++malformedMotorPackets_;
            continue;
        }

        ServoPacket packet{};
        std::memcpy(&packet, buffer.data(), sizeof(packet));

        std::array<double, 4> packetCommands{};
        bool valid = true;
        for (std::size_t i = 0; i < packetCommands.size(); ++i) {
            packetCommands[i] = packet.motorSpeed[i];
            if (!std::isfinite(packetCommands[i])) {
                valid = false;
                break;
            }
        }
        if (!valid) {
            ++malformedMotorPackets_;
            continue;
        }

        const auto commands = SitlPacketToBetaflightMotorOrder(packetCommands);
        lastMotorCommand_ = commands;
        lastMotorPacketTime_ = std::chrono::steady_clock::now();
        hasMotorPacket_ = true;
        motorTimedOut_ = false;
        ++motorPackets_;
        if (config_.logging.logFirstPackets && !loggedFirstMotor_) {
            spdlog::info("First motor packet received: [{:.3f}, {:.3f}, {:.3f}, {:.3f}]",
                         commands[0], commands[1], commands[2], commands[3]);
            loggedFirstMotor_ = true;
        }
    }
}

void BridgeApp::PublishMotorCommandIfNeeded()
{
    std::array<double, 4> command = ZeroMotors();
    if (hasMotorPacket_) {
        const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - lastMotorPacketTime_).count();
        if (age <= config_.motors.timeoutSeconds) {
            command = lastMotorCommand_;
        } else if (!motorTimedOut_) {
            motorTimedOut_ = true;
            spdlog::warn("Motor command timeout after {:.3f}s; publishing zeros", age);
        }
    }

    const auto mapped = motorMapper_.Apply(command);
    std::array<double, 4> velocities{};
    for (std::size_t i = 0; i < velocities.size(); ++i) {
        velocities[i] = velocityConverter_.Convert(mapped[i]);
    }
    actuatorPublisher_.Publish(velocities);
}

void BridgeApp::SendFdmIfNeeded()
{
    const auto now = std::chrono::steady_clock::now();
    const double period = 1.0 / config_.fdm.rateHz;
    if (std::chrono::duration<double>(now - lastFdmSendTime_).count() < period) {
        return;
    }

    const auto snapshot = stateSubscriber_.Snapshot();
    if (!snapshot) {
        return;
    }

    const FdmPacket packet = fdmBuilder_.Build(*snapshot, SecondsSince(startTime_));
    fdmSocket_.Send(&packet, sizeof(packet));
    lastFdmSendTime_ = now;
    ++fdmPackets_;
    if (config_.logging.logFirstPackets && !loggedFirstFdm_) {
        spdlog::info("First FDM packet sent: altitude={:.3f} pressure={:.1f}", packet.positionXyz[2], packet.pressure);
        loggedFirstFdm_ = true;
    }
}

void BridgeApp::LogStatusIfNeeded()
{
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - lastStatusLogTime_).count() < config_.logging.statusPeriodSeconds) {
        return;
    }
    lastStatusLogTime_ = now;
    spdlog::info("status imu={} altimeter={} fdm_packets={} motor_packets={} malformed_motor_packets={}",
                 stateSubscriber_.HasImu(), stateSubscriber_.HasAltimeter(), fdmPackets_, motorPackets_, malformedMotorPackets_);
}

std::array<double, 4> BridgeApp::ZeroMotors() const
{
    return {0.0, 0.0, 0.0, 0.0};
}

}  // namespace betaflight_gazebo_bridge
