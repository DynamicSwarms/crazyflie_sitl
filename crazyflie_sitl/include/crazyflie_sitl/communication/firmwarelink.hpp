#pragma once 
#include <stdint.h>
#include <array>
#include <cstddef>

#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>




class Firmwarelink {
public:
    Firmwarelink(uint16_t port);
    ~Firmwarelink();

    size_t send(const uint8_t* data, size_t size, uint8_t* receive_data);

    bool is_connected() {return m_is_connected;};

private:
    int m_fd{-1};
    
    struct sockaddr_in m_my_address;
    struct sockaddr_in m_remote_address;
    socklen_t m_address_len{sizeof(m_my_address)};

    bool m_is_connected{false};    

};
