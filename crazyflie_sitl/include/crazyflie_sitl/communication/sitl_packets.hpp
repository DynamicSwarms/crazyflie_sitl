#pragma once

#include <cstdint>

namespace sitl_communication::packets {

#define CRTP_PORT_LOCALIZATION 0x06
#define CRTP_PORT_SETPOINT_SIM 0x09

#define LOCALIZATION_POSITION_CHANNEL 0
#define LOCALIZATION_GENERIC_CHANNEL 1

#define LOCALIZATION_GENERIC_EXT_POSE_TYPE 8

// Conversion coefficient for Crazyflie
#define MAG_GAUSS_PER_LSB                                 666.7
#define SENSORS_DEG_PER_LSB_CFG                           ((2 * 2000.0) / 65536.0)
#define SENSORS_G_PER_LSB_CFG                             ((2 * 16) / 65536.0)

#define PI_CF 3.14159265359
#define DEG_TO_RAD_CF (PI_CF/180.0)
#define RAD_TO_DEG_CF (180.0/PI_CF)
#define GRAVITY_MAGNITUDE_CF (9.81) // we use the magnitude such that the sign/direction is explicit in calculations

enum SensorTypeSim_e {
  SENSOR_GYRO_ACC_SIM   = 0,
  SENSOR_MAG_SIM        = 1,
  SENSOR_BARO_SIM       = 2,
};

union Axis3i16 {
  struct {
    int16_t x;
    int16_t y;
    int16_t z;
  };
  int16_t axis[3];
} __attribute__((packed));

struct imu_s {
  union Axis3i16 acc;
  union Axis3i16 gyro;
} __attribute__((packed));

struct pos_s {
  float x;
  float y;
  float z;
} __attribute__((packed));

struct pose_s {
  float x; // in m
  float y; // in m
  float z; // in m
  float qx;
  float qy;
  float qz;
  float qw;
} __attribute__((packed));

struct crtp_imu_packet_s {
  uint8_t header = (uint8_t)((CRTP_PORT_SETPOINT_SIM << 4) | 0);
  uint8_t type = SENSOR_GYRO_ACC_SIM;
  imu_s imu_data;
} __attribute__((packed));

struct crtp_pose_packet_s {
  uint8_t header = (uint8_t) CRTP_PORT_LOCALIZATION << 4 | LOCALIZATION_GENERIC_CHANNEL;
  uint8_t type = LOCALIZATION_GENERIC_EXT_POSE_TYPE;
  pose_s pose_data;
} __attribute__((packed));

struct crtp_pwm_packet_s {
  uint8_t header;
  uint16_t motor_pwms[4];
} __attribute__((packed));

struct queue_packet
{
  uint8_t data[32];
  uint8_t data_length;
};

} // namespace sitl_communication::packets