#include "communication/radiolink.hpp"
#include <fcntl.h>
#include <sys/socket.h>
#include <stdexcept>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <ostream>

Radiolink::Radiolink(uint16_t port,
    std::shared_ptr<std::queue<sitl_communication::packets::queue_packet>> radio_to_firmware_queue,
    std::shared_ptr<std::queue<sitl_communication::packets::queue_packet>> firmware_to_radio_queue)
    : m_radio_to_firmware_queue(radio_to_firmware_queue)
    , m_firmware_to_radio_queue(firmware_to_radio_queue) 
{
    if ((m_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        throw std::runtime_error("Radiolink: socket() failed!");
    }

    m_my_address.sin_family = AF_INET;
    m_my_address.sin_addr.s_addr = htonl(INADDR_ANY);
    m_my_address.sin_port = htons(port);

    if (bind(m_fd, reinterpret_cast<sockaddr*>(&m_my_address), m_address_len) < 0) {
        close(m_fd);
        m_fd = -1;
        throw std::runtime_error("Radiolink: bind() failed");
    }

    // Set socket to non-blocking mode
    int flags = fcntl(m_fd, F_GETFL, 0);
    if (flags < 0) {
        throw std::runtime_error("Radiolink: fcntl(F_GETFL) failed");
    }

    if (fcntl(m_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        throw std::runtime_error("Radiolink: fcntl(F_SETFL) failed");
    }
}

Radiolink::~Radiolink()
{
    if (m_fd >= 0)
    {
        close(m_fd);
        m_fd = -1;
    }
}

void
Radiolink::handle_radio_communication()
{
   

   // For each received packet we need to send a packet back
   handle_from_radio_packets();
   handle_to_radio_packets();  
}

void Radiolink::handle_to_radio_packets()
{
    sitl_communication::packets::queue_packet out_packet;
    out_packet.data_length = 0;

    if (!is_connected())
    {
        m_control_loop_count++;
        // As long as we are not connected, we send a null packets
        // in order to establish the connection to the radio.
        // We do not clear the queue so radio receives
        // all packets upon connection
        if (m_control_loop_count % 20 == 0)
        {
            out_packet.data[0] = 0xF3;
            out_packet.data_length = 1;
        }
    }
    else if (!m_firmware_to_radio_queue->empty())
    { 
        out_packet = m_firmware_to_radio_queue->front();  
        m_firmware_to_radio_queue->pop();

        // if (out_packet.data[0] != 0xF3)
        //     std::cerr << "Sending to radio: " << std::hex << int(out_packet.data[0]) << " queue: " << int(m_firmware_to_radio_queue->empty()) << std::endl;
      
    }
    
    
    if (out_packet.data_length > 0)
    {    
        sendto(
            m_fd,
            out_packet.data,
            out_packet.data_length,
            0,
            reinterpret_cast<const sockaddr*>(&m_remote_address),
            m_address_len
        ); 

    
    }
}

void
Radiolink::handle_from_radio_packets()
{
    uint8_t buffer[256];
    ssize_t recv_len = recvfrom(m_fd, buffer, 32, 0, (struct sockaddr *)&m_remote_address, &m_address_len);
    if (recv_len > 0)
    {
        sitl_communication::packets::queue_packet packet;
        std::memcpy(packet.data, buffer, recv_len);
        packet.data_length = recv_len;
        m_radio_to_firmware_queue->push(packet);

        m_connected = true;
    }
}