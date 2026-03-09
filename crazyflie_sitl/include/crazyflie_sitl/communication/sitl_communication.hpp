#pragma once

#include <stdint.h>
#include <memory>
#include <queue>

#include "communication/sitl_packets.hpp"

#include "communication/firmwarelink.hpp"
#include "communication/radiolink.hpp"



namespace sitl_communication {

class SITLCommunication
{
public: 
    SITLCommunication(
        uint16_t radio_port,
        uint16_t firmware_port
    );

    void handle_comms();

    void send_firmware_packet(const uint8_t* packet, size_t packet_length);

    void get_motor_pwm_values(uint16_t* pwms);
private:
    void handle_firmware_packet(const uint8_t* packet, size_t packet_length);
private:
    std::unique_ptr<Firmwarelink> m_firmware_link;
    std::unique_ptr<Radiolink> m_radio_link;

    uint16_t m_last_pwms_received[4] = {0, 0, 0, 0};

    std::shared_ptr<std::queue<sitl_communication::packets::queue_packet>> m_radio_to_firmware_queue;
    std::shared_ptr<std::queue<sitl_communication::packets::queue_packet>> m_firmware_to_radio_queue;
};

} // namespace sitl_communication