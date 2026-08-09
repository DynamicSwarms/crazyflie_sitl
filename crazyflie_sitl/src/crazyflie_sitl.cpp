#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

// Forward declaration - replace with your actual quadcopter model header
#include "quadcopter_model/objects/quadrotor.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "communication/sitl_communication.hpp"
#include "communication/sitl_packets.hpp"

#include "firmware_launcher.hpp"

#include "crazyflie_sitl/crazyflie_sitl.hpp"


using namespace std::chrono_literals;
using namespace quadcopter;
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

CrazyflieSITL::CrazyflieSITL(const rclcpp::NodeOptions & options)
  : Node("crazyflie_sitl", options)
  , p_id(this->declare_parameter("id", rclcpp::ParameterValue(0xE7)).get<int>())
  , p_initial_position(this->declare_parameter("initial_position", rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0})).get<std::vector<double>>())
  , p_publish_tf(this->declare_parameter("publish_tf", rclcpp::ParameterValue(true)).get<bool>())
  , m_sitl_socket_path("/tmp/crazyflie_sitl_" + std::to_string(p_id) + ".sock")
  , m_client_socket_path("/tmp/crazyflie_client_" + std::to_string(p_id) + ".sock")
  , m_firmware_launcher(std::make_unique<FirmwareLauncher>(m_sitl_socket_path, m_client_socket_path)) // Launch the firmware
  , m_communication(std::make_unique<sitl_communication::SITLCommunication>(p_id, m_client_socket_path))
  , m_cmd()
  , m_frame_id("cf" + std::to_string(p_id))
{

  // Crazyflie parameters
  const Scalar mass = 0.032;
  const Scalar arm_length = 0.0325; 
  QuadrotorDynamics dynamics(mass, arm_length);

  m_quadrotor = std::make_shared<Quadrotor>(dynamics);
  QuadState initial_state;
  initial_state.setZero();
  initial_state.x[QS::POSX] = p_initial_position[0];
  initial_state.x[QS::POSY] = p_initial_position[1];
  initial_state.x[QS::POSZ] = p_initial_position[2];
  m_quadrotor->setState(initial_state);
  m_quadrotor->setWorldBox((Matrix<3, 2>() << -10, 10, -10, 10, 0.0, 10).finished());

  m_cmd.t = 0.0;
  m_cmd.thrusts = quadcopter::Vector<4>::Zero();

  m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  m_control_loop_timer = this->create_wall_timer(
    1ms,  // 1000 Hz
    std::bind(&CrazyflieSITL::control_loop, this)
  );
  const std::string simulation_prefix = "/cf" + std::to_string(p_id) + "/simulation";
  m_crash_service = this->create_service<std_srvs::srv::Trigger>(
    simulation_prefix + "/crash",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      m_crashed = true;
      send_crash_state();
      response->success = true;
      response->message = "SITL motor thrust disabled";
    });
  m_battery_subscription = this->create_subscription<std_msgs::msg::Float32>(
    simulation_prefix + "/set_battery", 1,
    [this](const std_msgs::msg::Float32::SharedPtr message) {
      if (message->data >= 0.0f && message->data <= 5.0f) {
        send_battery_voltage(message->data);
      } else {
        RCLCPP_WARN(get_logger(), "Ignoring invalid simulated battery voltage: %.2f V", message->data);
      }
    });
  RCLCPP_INFO(this->get_logger(), "Started CrazyflieSITL node with ID: %d, and initial position: [%f, %f, %f]", p_id, p_initial_position[0], p_initial_position[1], p_initial_position[2]);
}



void CrazyflieSITL::get_pose_packet(const QuadState& state, uint8_t* buffer, size_t& length)
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

void CrazyflieSITL::get_imu_packet(const QuadState& state, uint8_t* buffer, size_t& length)
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

void CrazyflieSITL::update_thrust_command()
{
  if (m_crashed) {
    m_cmd.thrusts = quadcopter::Vector<4>::Zero();
    return;
  }
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
  m_cmd.thrusts = quadcopter::Vector<4>(thrusts[0],thrusts[1], thrusts[2], thrusts[3]); 
}

void CrazyflieSITL::send_battery_voltage(float voltage)
{
  sitl_communication::packets::crtp_battery_packet_s packet;
  packet.voltage = voltage;
  m_communication->send_firmware_packet(
    reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
}

void CrazyflieSITL::send_crash_state()
{
  sitl_communication::packets::crtp_crash_packet_s packet;
  m_communication->send_firmware_packet(
    reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
}


void CrazyflieSITL::control_loop()
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
    
    if (m_control_loop_count % 100 == 0 && p_publish_tf) publish_tf(state); // 10Hz tf publishing
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to get quadrotor state");
  }
}


void CrazyflieSITL::publish_tf(const QuadState& state)
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



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CrazyflieSITL>());
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(CrazyflieSITL)
