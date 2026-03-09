#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

// Forward declaration - replace with your actual quadcopter model header
#include "quadcopter_model/objects/quadrotor.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "crtp_interfaces/srv/crtp_packet_send.hpp"
#include "crtp_interfaces/msg/crtp_packet.hpp"
#include "crtp_interfaces/msg/crtp_response.hpp"

#include "communication/sitl_communication.hpp"
#include "communication/sitl_packets.hpp"

#include "firmware_launcher.hpp"

using namespace std::chrono_literals;
using namespace flightlib;
using std::placeholders::_1;


#define N 16

float pwm_lookup[N] = {
0,6.25,12.5,18.75,25,31.25,37.5,43.75,
50,56.25,62.5,68.75,75,81.25,87.5,93.75
};

float thrust_lookup[N] = {
  0.000000,
  0.003924,
  0.011772,
  0.019375,
  0.026732,
  0.034090,
  0.042428,
  0.051503,
  0.059841,
  0.070142,
  0.080442,
  0.091433,
  0.102269,
  0.112815,
  0.127300,
  0.141999
};

float pwm_to_thrust(float x)
{
    if (x <= pwm_lookup[0]) return thrust_lookup[0];
    if (x >= pwm_lookup[N-1]) return thrust_lookup[N-1];

    for (int i = 0; i < N-1; i++) {
        if (x >= pwm_lookup[i] && x <= pwm_lookup[i+1]) {

            float t = (x - pwm_lookup[i]) / (pwm_lookup[i+1] - pwm_lookup[i]);

            return thrust_lookup[i] + t * (thrust_lookup[i+1] - thrust_lookup[i]);
        }
    }

    return 0;
}

class CrazyflieSITL : public rclcpp::Node
{
public:
  CrazyflieSITL()
  : Node("crazyflie_sitl")
  , m_cmd()
  , m_firmware_launcher(std::make_unique<FirmwareLauncher>()) // Launch the firmware
  {
    // Crazyflie parameters
    const Scalar mass = 0.02;//0.032;      // kg
    const Scalar arm_length = 0.0325; // super stable  //0.04; // meters

    // Create dynamics model
    QuadrotorDynamics dynamics(mass, arm_length);

    m_quadrotor = std::make_shared<Quadrotor>(dynamics);
    QuadState initial_state;
    initial_state.setZero();
    m_quadrotor->setState(initial_state);
    m_quadrotor->setWorldBox((Matrix<3, 2>() << -10, 10, -10, 10, 0.0, 10).finished());

    std::stringstream dyn_ss;
    dyn_ss << dynamics;
    RCLCPP_INFO(this->get_logger(), "%s", dyn_ss.str().c_str());

    m_cmd.t = 0.0;
    m_cmd.thrusts = flightlib::Vector<4>::Zero();

    m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    client_ = this->create_client<crtp_interfaces::srv::CrtpPacketSend>("crazyradio/send_crtp_packet80");

    uint16_t port = 19950;
    m_communication = std::make_unique<sitl_communication::SITLCommunication>(0,port);

    crtp_response_sub_ = this->create_subscription<crtp_interfaces::msg::CrtpResponse>(
      "crazyradio/crtp_response",
      rclcpp::QoS(100),
      std::bind(&CrazyflieSITL::crtpResponseMsgCallback, this, _1));


    timer_ = this->create_wall_timer(
      1ms,  // 1000 Hz
      std::bind(&CrazyflieSITL::timer_callback, this)
    );
    RCLCPP_INFO(this->get_logger(), "CrazyflieSITL node started");
  }



void get_pose_packet(const QuadState& state, uint8_t* buffer, size_t& length)
{
  sitl_communication::packets::crtp_pose_packet_s packet;
  packet.pose_data.x = state.x[QS::POSX]; // m
  packet.pose_data.y = state.x[QS::POSY]; // m
  packet.pose_data.z = state.x[QS::POSZ]; // m
  packet.pose_data.qx = state.x[QS::ATTX];
  packet.pose_data.qy = state.x[QS::ATTY];
  packet.pose_data.qz = state.x[QS::ATTZ];
  packet.pose_data.qw = state.x[QS::ATTW];
  
  length = sizeof(sitl_communication::packets::crtp_pose_packet_s);
  std::memcpy(buffer, &packet, length);
}

void get_imu_packet(const QuadState& state, uint8_t* buffer, size_t& length)
{
  sitl_communication::packets::crtp_imu_packet_s packet;

  packet.imu_data.acc.x = static_cast<int16_t>(state.ba[0] / 	SENSORS_G_PER_LSB_CFG 		/ GRAVITY_MAGNITUDE_CF);  // 0.0; // m/s^2
  packet.imu_data.acc.y = static_cast<int16_t>(state.ba[1] / 	SENSORS_G_PER_LSB_CFG 		/ GRAVITY_MAGNITUDE_CF);  // 0.0; // m/s^2
  packet.imu_data.acc.z = static_cast<int16_t>(state.ba[2] / 	SENSORS_G_PER_LSB_CFG 		/ GRAVITY_MAGNITUDE_CF);  // 2048.0 = -9.81; // m/s^2
  packet.imu_data.gyro.x = static_cast<int16_t>(state.bw[0] / SENSORS_DEG_PER_LSB_CFG 	/ DEG_TO_RAD_CF); // deg/s
  packet.imu_data.gyro.y = static_cast<int16_t>(state.bw[1] / SENSORS_DEG_PER_LSB_CFG 	/ DEG_TO_RAD_CF); // deg/s
  packet.imu_data.gyro.z = static_cast<int16_t>(state.bw[2] / SENSORS_DEG_PER_LSB_CFG 	/ DEG_TO_RAD_CF); // deg/s

  length = sizeof(sitl_communication::packets::crtp_imu_packet_s);
  std::memcpy(buffer, &packet, length);
}


  void timer_callback()
  {
    m_communication->handle_comms();

    float pwm_norm[4];

    uint16_t pwms_received[4];
    m_communication->get_motor_pwm_values(pwms_received);
    for (int i = 0; i < 4; ++i) {
      pwm_norm[i] = static_cast<float>(pwms_received[i]) / static_cast<float>(std::numeric_limits<uint16_t>::max());
      pwm_norm[i] *= 100.0f;  // scale to 0–100 range expected by pwm_to_thrust
    }

    float thrusts[4];
    for (int i = 0; i < 4; ++i) {
      thrusts[i] = pwm_to_thrust(pwm_norm[i]); // Give a bit more thrust for testting
    }
    m_cmd.thrusts = flightlib::Vector<4>(thrusts[0],thrusts[1], thrusts[2], thrusts[3]); 
  
  //static int tick = 0;
  //tick++;
  //if (tick == 4000) {
  //  m_cmd.thrusts = flightlib::Vector<4>(0.07, 0.07, 0.07, 0.07); // For testing, set a constant thrust
  //  m_quadrotor->setWorldBox((Matrix<3, 2>() << -10, 10, -10, 10, -10.0, 10).finished());
  //}

  static bool world_is_restricted = true;
  if (!world_is_restricted && pwms_received[0] == 0 && pwms_received[1] == 0 && pwms_received[2] == 0 && pwms_received[3] == 0) {
    world_is_restricted = true;
    RCLCPP_INFO(this->get_logger(), "Restricted world box");
    m_quadrotor->setWorldBox((Matrix<3, 2>() << -10, 10, -10, 10, 0.0, 10).finished());
  } else if (world_is_restricted && !(pwms_received[0] == 0 && pwms_received[1] == 0 && pwms_received[2] == 0 && pwms_received[3] == 0)) {
    world_is_restricted = false;
    RCLCPP_INFO(this->get_logger(), "Unrestricted world box");
    m_quadrotor->setWorldBox((Matrix<3, 2>() << -10, 10, -10, 10, -10.0, 10).finished());
  }

//    static bool is_takeoff = false;
//    if (last_pwms_received[0] == 0 && last_pwms_received[1] == 0 && last_pwms_received[2] == 0 && last_pwms_received[3] == 0) {
//      if (!is_takeoff) {
//      
//        QuadState initial_state;
//        initial_state.setZero();
//        m_quadrotor->setState(initial_state);
//        
//        RCLCPP_INFO(this->get_logger(), "No PWM received, resetting quadrotor state to zero");
//      }
//    } else {
//        is_takeoff = true;
//        //m_cmd.thrusts = flightlib::Vector<4>(0.2, 0.2, 0.2, 0.2); 
//
//        RCLCPP_INFO(this->get_logger(), "Received PWM: %d, %d, %d, %d; Normalized: %.2f, %.2f, %.2f, %.2f; Thrusts: %.6f, %.6f, %.6f, %.6f",
//                last_pwms_received[0], last_pwms_received[1], last_pwms_received[2], last_pwms_received[3],
//                pwm_norm[0], pwm_norm[1], pwm_norm[2], pwm_norm[3],
//                thrusts[0], thrusts[1], thrusts[2], thrusts[3]);
//
//    }

    //RCLCPP_INFO(this->get_logger(), "Received PWM: %d, %d, %d, %d; Normalized: %.2f, %.2f, %.2f, %.2f; Thrusts: %.6f, %.6f, %.6f, %.6f",
    //    last_pwms_received[0], last_pwms_received[1], last_pwms_received[2], last_pwms_received[3],
    //    pwm_norm[0], pwm_norm[1], pwm_norm[2], pwm_norm[3],
    //    m_cmd.thrusts[0], m_cmd.thrusts[1], m_cmd.thrusts[2], m_cmd.thrusts[3]);

    m_cmd.t += 0.001;  // time in seconds
    if (!m_quadrotor->run(m_cmd, 0.001)) {
      RCLCPP_WARN(this->get_logger(), "Quadrotor simulation step failed");
    } 

    QuadState state;
    if (m_quadrotor->getState(&state)) {
      static int count = 0;
      if (count++ % 100 == 0) {  // Print state every 100 ms
        publish_tf(state);
      }

      // if (!client_->wait_for_service(std::chrono::seconds(1))) {
      //   RCLCPP_WARN(this->get_logger(), "Service 'crazyradio/send_crtp_packet80' not available.");
      //   //return;
      // }

      //if (!(count % 1 == 0)) return; // 500Hz

      auto request = std::make_shared<crtp_interfaces::srv::CrtpPacketSend::Request>();
      request->link.channel = 80;
      memcpy(request->link.address.data(), "\xE7\xE7\xE7\xE7\xE7", 5); // broadcast address
      

      //request->packet = get_imu(state);
      //auto future = client_->async_send_request(request);

      //request->packet = get_pos(state);
      //future = client_->async_send_request(request);

      uint8_t buffer[32];
      size_t length;
    
      get_imu_packet(state, buffer, length);
      m_communication->send_firmware_packet(buffer, length);

      if (count % 10 == 0) 
      {
        get_pose_packet(state, buffer, length);
        m_communication->send_firmware_packet(buffer, length);
      
      }
      
      std::stringstream ss;
      ss << "State t=" << m_cmd.t
        << state << state.x[QS::ACCZ];

     // RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to get quadrotor state");
    }
  }

  void crtpResponseMsgCallback(const crtp_interfaces::msg::CrtpResponse::SharedPtr msg)
    {
      // Check if we matchh (by address and channel)

      if (msg->packet.port == CRTP_PORT_SETPOINT_SIM && msg->packet.channel == 0) {
          
        // Reconstruct address (same packing convention used elsewhere in this file)
        uint64_t address = 0;
        for (int i = 0; i < 5; i++)
            address |= (uint64_t)msg->address[i] << (8 * (4 - i));

        std::stringstream ss;
        ss << "CRTP response topic: ch=" << (int)msg->channel
           << " addr=0x" << std::hex << (address & 0xFFFFFFFFFFULL) << std::dec
           << " port=" << (int)msg->packet.port
           << " pkt_ch=" << (int)msg->packet.channel
           << " len=" << (int)msg->packet.data_length
           << " data=";

        for (int i = 0; i < msg->packet.data_length; i++)
            ss << (int)msg->packet.data[i] << " ";

        //RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
      }
    }

  void publish_tf(const QuadState& state)
  {
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = this->get_clock()->now();
    transformStamped.header.frame_id = m_world_id;
    transformStamped.child_frame_id = m_frame_id;

    transformStamped.transform.translation.x = state.x[QS::POSX];
    transformStamped.transform.translation.y = state.x[QS::POSY];
    transformStamped.transform.translation.z = state.x[QS::POSZ];

    transformStamped.transform.rotation.w = state.x[QS::ATTW];
    transformStamped.transform.rotation.x = state.x[QS::ATTX];
    transformStamped.transform.rotation.y = state.x[QS::ATTY];
    transformStamped.transform.rotation.z = state.x[QS::ATTZ];

    m_tf_broadcaster->sendTransform(transformStamped);
  }


  flightlib::Command m_cmd;
  std::shared_ptr<Quadrotor> m_quadrotor;
  rclcpp::TimerBase::SharedPtr timer_;
  
  rclcpp::Client<crtp_interfaces::srv::CrtpPacketSend>::SharedPtr client_;
  rclcpp::Subscription<crtp_interfaces::msg::CrtpResponse>::SharedPtr crtp_response_sub_;

  std::unique_ptr<sitl_communication::SITLCommunication> m_communication;

  std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
  std::string m_world_id = "world";
  std::string m_frame_id = "cf231";

  std::unique_ptr<FirmwareLauncher> m_firmware_launcher;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CrazyflieSITL>());
  rclcpp::shutdown();
  return 0;
}
