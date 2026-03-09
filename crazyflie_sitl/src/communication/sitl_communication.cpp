#include "communication/sitl_communication.hpp"
#include <ostream>
#include <iostream>


namespace sitl_communication{

SITLCommunication::SITLCommunication(
    uint8_t id,
    const std::string& client_socket_path)
{
    m_radio_to_firmware_queue = std::make_shared<std::queue<sitl_communication::packets::queue_packet>>();
    m_firmware_to_radio_queue = std::make_shared<std::queue<sitl_communication::packets::queue_packet>>();
    
    m_firmware_link = std::make_unique<Firmwarelink>(client_socket_path);
    m_radio_link = std::make_unique<Radiolink>(
        19850 + id,
        m_radio_to_firmware_queue, 
        m_firmware_to_radio_queue);
}

void SITLCommunication::handle_comms()
{
    if (!m_radio_to_firmware_queue->empty())
    {
        auto packet = m_radio_to_firmware_queue->front();
        m_radio_to_firmware_queue->pop();
        send_firmware_packet(packet.data, packet.data_length);
    }

    m_radio_link->handle_radio_communication();
}

void SITLCommunication::send_firmware_packet(const uint8_t* packet, size_t packet_length)
{
    uint8_t in_buffer[256];
    ssize_t recv_len = m_firmware_link->send(packet, packet_length, in_buffer);

    
    if (recv_len > 0) handle_firmware_packet(in_buffer, recv_len);

}

void SITLCommunication::get_motor_pwm_values(uint16_t* pwms)
{
    for (int i = 0; i < 4; i++)
    {
        pwms[i] = m_last_pwms_received[i];
    }
}

void SITLCommunication::handle_firmware_packet(const uint8_t* packet, size_t packet_length)
{
    if (packet_length == sizeof(packets::crtp_pwm_packet_s) &&
        packet[0] == ((CRTP_PORT_SETPOINT_SIM << 4) | 0))
    {
        packets::crtp_pwm_packet_s pwm_packet;
        std::memcpy(&pwm_packet, packet, sizeof(pwm_packet));

        for (int i = 0; i < 4; i++)
        {
            m_last_pwms_received[i] = pwm_packet.motor_pwms[i];
        }
        //  We originally checked if they are all above 1000, otherwise skipped...
    } else if (packet_length > 0) {
        sitl_communication::packets::queue_packet queue_packet;
        std::memcpy(queue_packet.data, packet, packet_length);
        queue_packet.data_length = packet_length;
        m_firmware_to_radio_queue->push(queue_packet);
    }else {
        std::cerr << "Received unknown firmware packet of length " << packet_length << std::endl;
    }

    // for (size_t i = 0; i < packet_length; i++)
    // {
    //     std::cerr << (int)packet[i] << " ";
    // }
    // std::cerr << std::endl;
}

}