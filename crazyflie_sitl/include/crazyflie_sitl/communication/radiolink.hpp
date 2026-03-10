#pragma once

#include <stdint.h>
#include <memory>
#include <queue>
#include "communication/sitl_packets.hpp"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>

class Radiolink {

public: 
    Radiolink(
        uint16_t port,
        std::shared_ptr<std::queue<sitl_communication::packets::queue_packet>> radio_to_firmware_queue,
        std::shared_ptr<std::queue<sitl_communication::packets::queue_packet>> firmware_to_radio_queue);

    ~Radiolink();

    
    void handle_radio_communication();

    bool is_connected() {return m_connected;};

private: 
    void handle_to_radio_packets();
    void handle_from_radio_packets();
private: 
    int m_fd{-1};
        
    struct sockaddr_in m_my_address;
    struct sockaddr_in m_remote_address;
    socklen_t m_address_len{sizeof(m_my_address)};

    bool m_connected = false;

    std::shared_ptr<std::queue<sitl_communication::packets::queue_packet>> m_radio_to_firmware_queue;
    std::shared_ptr<std::queue<sitl_communication::packets::queue_packet>> m_firmware_to_radio_queue;

    size_t m_control_loop_count = 0;
};