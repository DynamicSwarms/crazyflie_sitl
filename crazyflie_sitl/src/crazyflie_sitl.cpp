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
  CrazyflieSITL(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("crazyflie_sitl", options)
  , m_id(this->declare_parameter("id", rclcpp::ParameterValue(0xE7)).get<int>())
  , m_initial_position(this->declare_parameter("initial_position", rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0})).get<std::vector<double>>())
  , m_sitl_socket_path("/tmp/crazyflie_sitl_" + std::to_string(m_id) + ".sock")
  , m_client_socket_path("/tmp/crazyflie_client_" + std::to_string(m_id) + ".sock")
  , m_firmware_launcher(std::make_unique<FirmwareLauncher>(m_sitl_socket_path, m_client_socket_path)) // Launch the firmware
  , m_communication(std::make_unique<sitl_communication::SITLCommunication>(m_id, m_client_socket_path))
  , m_cmd()
  , m_frame_id("cf" + std::to_string(m_id))
  {

    // Crazyflie parameters
    const Scalar mass = 0.032;
    const Scalar arm_length = 0.0325; 
    QuadrotorDynamics dynamics(mass, arm_length);

    m_quadrotor = std::make_shared<Quadrotor>(dynamics);
    QuadState initial_state;
    initial_state.setZero();
    initial_state.x[QS::POSX] = m_initial_position[0];
    initial_state.x[QS::POSY] = m_initial_position[1];
    initial_state.x[QS::POSZ] = m_initial_position[2];
    m_quadrotor->setState(initial_state);
    m_quadrotor->setWorldBox((Matrix<3, 2>() << -10, 10, -10, 10, 0.0, 10).finished());

    m_cmd.t = 0.0;
    m_cmd.thrusts = flightlib::Vector<4>::Zero();

    m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    m_control_loop_timer = this->create_wall_timer(
      1ms,  // 1000 Hz
      std::bind(&CrazyflieSITL::control_loop, this)
    );
    RCLCPP_INFO(this->get_logger(), "Started CrazyflieSITL node with ID: %d, and initial position: [%f, %f, %f]", m_id, m_initial_position[0], m_initial_position[1], m_initial_position[2]);
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

  void update_thrust_command()
  {
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
  }


  void control_loop()
  {
    m_control_loop_count++;

    m_communication->handle_comms();

    
    update_thrust_command();
    m_cmd.t += 0.001;  // time in seconds
    if (!m_quadrotor->run(m_cmd, 0.001)) {
      RCLCPP_WARN(this->get_logger(), "Quadrotor simulation step failed");
    } 

    QuadState state;
    if (m_quadrotor->getState(&state)) {
      get_imu_packet(state, m_out_packet_buffer, m_out_packet_length);
      m_communication->send_firmware_packet(m_out_packet_buffer, m_out_packet_length);

      if (m_control_loop_count % 10 == 0) // 100Hz pose publishing
      {
        get_pose_packet(state, m_out_packet_buffer, m_out_packet_length);
        m_communication->send_firmware_packet(m_out_packet_buffer, m_out_packet_length);
      }
      
      if (m_control_loop_count % 100 == 0) publish_tf(state); // 10Hz tf publishing
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to get quadrotor state");
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

private:
  uint8_t m_id;
  std::vector<double> m_initial_position;
  std::string m_sitl_socket_path;
  std::string m_client_socket_path;
  std::unique_ptr<FirmwareLauncher> m_firmware_launcher;
  std::unique_ptr<sitl_communication::SITLCommunication> m_communication;


  flightlib::Command m_cmd;
  std::shared_ptr<Quadrotor> m_quadrotor;
 
  std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
  std::string m_world_id = "world";
  std::string m_frame_id;
  

  rclcpp::TimerBase::SharedPtr m_control_loop_timer;
  size_t m_control_loop_count = 0;

  uint8_t m_out_packet_buffer[32];
  size_t m_out_packet_length;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CrazyflieSITL>());
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(CrazyflieSITL)
