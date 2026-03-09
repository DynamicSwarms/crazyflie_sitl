#include "communication/firmwarelink.hpp"

#include <fcntl.h>
#include <iostream>
#include <ostream>

constexpr int kMaxPayload = 31;                 // CRTP max data size (excluding header)
constexpr int kMaxDatagram = 1 + kMaxPayload;   // header + payload

// Wait for fd readable, returns true if readable before timeout
bool waitReadable(int fd, int timeoutMs) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

    timeval tv;
    tv.tv_sec = timeoutMs / 1000;
    tv.tv_usec = (timeoutMs % 1000) * 1000;

    int rc = select(fd + 1, &rfds, nullptr, nullptr, &tv);
    return rc > 0 && FD_ISSET(fd, &rfds);
}

Firmwarelink::Firmwarelink(uint16_t port)
{
    m_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (m_fd < 0) {
        throw std::runtime_error("Firmwarelink: socket() failed");
    }

    m_my_address.sin_family = AF_INET;
    m_my_address.sin_addr.s_addr = INADDR_ANY;
    m_my_address.sin_port = htons(port);

    if (bind(m_fd, reinterpret_cast<sockaddr*>(&m_my_address), sizeof(m_my_address)) < 0) {
        close(m_fd);
        m_fd = -1;
        throw std::runtime_error("Firmwarelink: bind() failed");
    }

    // Set socket to non-blocking mode
    int flags = fcntl(m_fd, F_GETFL, 0);
    if (flags < 0) {
        throw std::runtime_error("Firmwarelink: fcntl(F_GETFL) failed");
    }

    if (fcntl(m_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        throw std::runtime_error("Firmwarelink: fcntl(F_SETFL) failed");
    }
}

Firmwarelink::~Firmwarelink()
{
    if (m_fd >= 0) {
        close(m_fd);
        m_fd = -1;
    }
    std::cerr << "Firmwarelink stopped." << std::endl;
}

size_t
Firmwarelink::send(
    const uint8_t* data, size_t size,
    uint8_t* receive_data)
{
    if (m_is_connected) {
        // std::cerr << "UDP Radio: Sending " << size << std::endl;   
        ssize_t sent = sendto(
            m_fd, data, size, 0,
            reinterpret_cast<const sockaddr*>(&m_remote_address), sizeof(m_remote_address)
        );
        //std::cerr << "UDP Radio: sendto() returned " << sent << std::endl;
    
        if (sent < 0) {
            std::cerr << "Firmwarelink : sendto() failed\n";
        } else {
            //std::cerr << "Firmwarelink: Sent " << sent << " bytes\n";
        }
    }

    std::array<uint8_t, kMaxDatagram> rx{};
    
    ssize_t r = recvfrom(
        m_fd, receive_data, kMaxDatagram, 0,
        reinterpret_cast<sockaddr*>(&m_remote_address), &m_address_len
    );
   
    if (r > 0 && !m_is_connected) 
    // If received first message, respond with nullpacket to complete handshake
    {        
        m_is_connected = true;
        uint8_t nullpacket[1] ={0xF3};
        return send(nullpacket, 1, receive_data);
    }
    return r;
}
