#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include <netinet/in.h>

namespace betaflight_gazebo_bridge
{

class UdpSocket
{
public:
    UdpSocket() = default;
    ~UdpSocket();

    UdpSocket(const UdpSocket &) = delete;
    UdpSocket &operator=(const UdpSocket &) = delete;

    UdpSocket(UdpSocket &&other) noexcept;
    UdpSocket &operator=(UdpSocket &&other) noexcept;

    void Bind(std::uint16_t port);
    void SetDestination(const std::string &address, std::uint16_t port);
    std::optional<std::size_t> Receive(void *buffer, std::size_t size);
    void Send(const void *buffer, std::size_t size);
    void Close();

private:
    int fd_{-1};
    sockaddr_in destination_{};
    bool hasDestination_{false};
};

}  // namespace betaflight_gazebo_bridge

