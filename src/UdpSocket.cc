#include "betaflight_gazebo_bridge/UdpSocket.hh"

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace betaflight_gazebo_bridge
{
namespace
{

int CreateSocket()
{
    const int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) {
        throw std::runtime_error(std::string("socket failed: ") + std::strerror(errno));
    }

    const int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        close(fd);
        throw std::runtime_error(std::string("fcntl O_NONBLOCK failed: ") + std::strerror(errno));
    }

    return fd;
}

}  // namespace

UdpSocket::~UdpSocket()
{
    Close();
}

UdpSocket::UdpSocket(UdpSocket &&other) noexcept
{
    *this = std::move(other);
}

UdpSocket &UdpSocket::operator=(UdpSocket &&other) noexcept
{
    if (this != &other) {
        Close();
        fd_ = other.fd_;
        destination_ = other.destination_;
        hasDestination_ = other.hasDestination_;
        other.fd_ = -1;
        other.hasDestination_ = false;
    }
    return *this;
}

void UdpSocket::Bind(const std::uint16_t port)
{
    if (fd_ < 0) {
        fd_ = CreateSocket();
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(port);

    if (bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
        throw std::runtime_error("bind UDP port " + std::to_string(port) + " failed: " + std::strerror(errno));
    }
}

void UdpSocket::SetDestination(const std::string &address, const std::uint16_t port)
{
    if (fd_ < 0) {
        fd_ = CreateSocket();
    }

    destination_ = {};
    destination_.sin_family = AF_INET;
    destination_.sin_port = htons(port);
    if (inet_pton(AF_INET, address.c_str(), &destination_.sin_addr) != 1) {
        throw std::runtime_error("Invalid IPv4 address: " + address);
    }
    hasDestination_ = true;
}

std::optional<std::size_t> UdpSocket::Receive(void *buffer, const std::size_t size)
{
    if (fd_ < 0) {
        throw std::runtime_error("Receive called on closed UDP socket");
    }

    const ssize_t received = recvfrom(fd_, buffer, size, 0, nullptr, nullptr);
    if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return std::nullopt;
        }
        throw std::runtime_error(std::string("recvfrom failed: ") + std::strerror(errno));
    }
    return static_cast<std::size_t>(received);
}

void UdpSocket::Send(const void *buffer, const std::size_t size)
{
    if (fd_ < 0 || !hasDestination_) {
        throw std::runtime_error("Send called before UDP destination was configured");
    }
    const ssize_t sent = sendto(fd_, buffer, size, 0, reinterpret_cast<sockaddr *>(&destination_), sizeof(destination_));
    if (sent < 0 || static_cast<std::size_t>(sent) != size) {
        throw std::runtime_error(std::string("sendto failed: ") + std::strerror(errno));
    }
}

void UdpSocket::Close()
{
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
    hasDestination_ = false;
}

}  // namespace betaflight_gazebo_bridge

