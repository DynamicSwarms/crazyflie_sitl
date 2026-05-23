#pragma once

#include <memory>
#include <array>
#include <vector>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

// Forward declarations of quadcopter classes
#include "quadcopter_model/objects/quadrotor.hpp"

// SITL communication
#include "communication/sitl_communication.hpp"
#include "communication/sitl_packets.hpp"

// Firmware launcher
#include "firmware_launcher.hpp"

class CrazyflieSITL : public rclcpp::Node
{
public:
    explicit CrazyflieSITL(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /// Returns the current position of the quadrotor
    inline std::vector<double> get_position() const
    {
        quadcopter::QuadState state;
        if (m_quadrotor->getState(&state)) {
            return {state.x[quadcopter::QS::POSX], state.x[quadcopter::QS::POSY], state.x[quadcopter::QS::POSZ]};
        } else {
            return {0.0, 0.0, 0.0};
        }
    };

private:
    // Core methods
    void control_loop();
    void update_thrust_command();
    void publish_tf(const quadcopter::QuadState& state);
    void get_pose_packet(const quadcopter::QuadState& state, uint8_t* buffer, size_t& length);
    void get_imu_packet(const quadcopter::QuadState& state, uint8_t* buffer, size_t& length);

    // Parameters
    uint8_t p_id;
    std::vector<double> p_initial_position;
    bool p_publish_tf;

    // Sockets
    std::string m_sitl_socket_path;
    std::string m_client_socket_path;

    // Firmware and communication
    std::unique_ptr<FirmwareLauncher> m_firmware_launcher;
    std::unique_ptr<sitl_communication::SITLCommunication> m_communication;

    // Quadrotor simulation
    quadcopter::Command m_cmd;
    std::shared_ptr<quadcopter::Quadrotor> m_quadrotor;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::string m_world_id = "world";
    std::string m_frame_id;

    // Control loop timer
    rclcpp::TimerBase::SharedPtr m_control_loop_timer;
    size_t m_control_loop_count = 0;

    // Output buffer
    uint8_t m_out_packet_buffer[32];
    size_t m_out_packet_length;
};
