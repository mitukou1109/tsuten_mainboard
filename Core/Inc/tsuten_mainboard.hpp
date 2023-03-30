#pragma once

#include <array>
#include <functional>
#include <string>
#include <unordered_map>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <tsuten_msgs/Odometry.h>
#include <tsuten_msgs/ResetOdometry.h>
#include <tsuten_msgs/SensorStates.h>
#include <tsuten_msgs/TapeLEDCommand.h>

class TsutenMainboard
{
public:
  TsutenMainboard(
      UART_HandleTypeDef *motor_driver_uart_handler,
      UART_HandleTypeDef *valve_controller_uart_handler,
      I2C_HandleTypeDef *gyro_i2c_handler,
      TIM_HandleTypeDef *x_odometer_encoder_handler,
      TIM_HandleTypeDef *y_odometer_encoder_handler,
      TIM_HandleTypeDef *publish_timer_handler,
      TIM_HandleTypeDef *tape_led_blink_timer_handler);

  void loop();

  friend void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);

  friend void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

private:
  enum class Axis : size_t
  {
    X,
    Y
  };

  enum class ValveID : uint8_t
  {
    DUAL_TABLE_UPPER_L = 0,
    DUAL_TABLE_UPPER_R = 1,
    DUAL_TABLE_LOWER = 2,
    MOVABLE_TABLE_1200 = 3,
    MOVABLE_TABLE_1500 = 4,
    MOVABLE_TABLE_1800 = 5
  };

  struct ValvePortSet
  {
    uint8_t board_number;
    uint8_t close_port;
    uint8_t open_port;

    static const uint8_t NUM_OF_BOARDS = 2;
  };

  static const std::unordered_map<ValveID, std::string> VALVE_NAMES;

  static const std::unordered_map<ValveID, ValvePortSet> VALVE_PORT_SETS;

  static const uint16_t GYRO_I2C_ADDRESS;

  void publishOdom();

  void publishSensorStates();

  void publishDebugMessage(std::string debug_message);

  double getYaw();

  double getAngularVelocity();

  void toggleTapeLED();

  void cmdVelCallback(const geometry_msgs::Twist &cmd_vel);

  void tapeLEDCommandCallback(const tsuten_msgs::TapeLEDCommand &tape_led_command);

  void valveCommandCallback(const ValveID valve_id, const std_msgs::Bool &valve_command);

  void resetOdometryCallback(const tsuten_msgs::ResetOdometryRequest &request,
                             tsuten_msgs::ResetOdometryResponse &response);

  ros::NodeHandle nh_;

  tsuten_msgs::Odometry odom_;
  ros::Publisher odom_pub_;

  tsuten_msgs::SensorStates sensor_states_;
  ros::Publisher sensor_states_pub_;

  std_msgs::String debug_message_;
  ros::Publisher debug_message_pub_;

  ros::Subscriber<geometry_msgs::Twist, TsutenMainboard> cmd_vel_sub_;

  ros::Subscriber<tsuten_msgs::TapeLEDCommand, TsutenMainboard> tape_led_command_sub_;

  std::unordered_map<ValveID, ros::Subscriber<std_msgs::Bool>> valve_command_subs_;

  ros::ServiceServer<tsuten_msgs::ResetOdometryRequest, tsuten_msgs::ResetOdometryResponse,
                     TsutenMainboard>
      reset_odometry_service_server_;

  std_msgs::ColorRGBA tape_led_color_;

  UART_HandleTypeDef *motor_driver_uart_handler_;

  std::array<uint8_t, ValvePortSet::NUM_OF_BOARDS> valve_controller_port_states_;
  UART_HandleTypeDef *valve_controller_uart_handler_;

  uint8_t gyro_data_;
  I2C_HandleTypeDef *gyro_i2c_handler_;

  std::unordered_map<Axis, TIM_HandleTypeDef *> odometer_encoder_handlers_;
  TIM_HandleTypeDef *publish_timer_handler_;
  TIM_HandleTypeDef *tape_led_blink_timer_handler_;
};