#include <mainpp.h>
#include <main.h>

#include <functional>
#include <string>
#include <unordered_map>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tsuten_msgs/ResetOdometry.h>
#include <tsuten_msgs/SensorStates.h>

enum class Axis
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

static const std::unordered_map<ValveID, std::string> VALVE_NAMES =
    {{ValveID::DUAL_TABLE_UPPER_L, "dual_table_upper_left_shooter"},
     {ValveID::DUAL_TABLE_UPPER_R, "dual_table_upper_right_shooter"},
     {ValveID::DUAL_TABLE_LOWER, "dual_table_lower_shooter"},
     {ValveID::MOVABLE_TABLE_1200, "movable_table_1200_shooter"},
     {ValveID::MOVABLE_TABLE_1500, "movable_table_1500_shooter"},
     {ValveID::MOVABLE_TABLE_1800, "movable_table_1800_shooter"}};

static const uint16_t BNO055_I2C_ADDRESS = 0x28;

static ros::NodeHandle nh;

static nav_msgs::Odometry odom;
static ros::Publisher odom_pub("odom", &odom);

static tsuten_msgs::SensorStates sensor_states;
static ros::Publisher sensor_states_pub("sensor_states", &sensor_states);

static std_msgs::String debug_message;
static ros::Publisher debug_message_pub("debug_message", &debug_message);

void cmdVelCallback(const geometry_msgs::Twist &cmd_vel);
static ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmdVelCallback);

void dualTableUpperLeftShooterCommandCallback(const std_msgs::Bool &valve_command);
void dualTableUpperRightShooterCommandCallback(const std_msgs::Bool &valve_command);
void dualTableLowerShooterCommandCallback(const std_msgs::Bool &valve_command);
void movableTable1200ShooterCommandCallback(const std_msgs::Bool &valve_command);
void movableTable1500ShooterCommandCallback(const std_msgs::Bool &valve_command);
void movableTable1800ShooterCommandCallback(const std_msgs::Bool &valve_command);
static std::unordered_map<ValveID, std::string> valve_command_topic_names;
static std::unordered_map<ValveID, ros::Subscriber<std_msgs::Bool>> valve_command_subs;

void resetOdometryCallback(const tsuten_msgs::ResetOdometryRequest &request,
                           tsuten_msgs::ResetOdometryResponse &response);
static ros::ServiceServer<tsuten_msgs::ResetOdometryRequest, tsuten_msgs::ResetOdometryResponse>
    reset_odometry_service_server("reset_odometry", &resetOdometryCallback);

static UART_HandleTypeDef *motor_driver_uart_handler;

static UART_HandleTypeDef *valve_controller_uart_handler;

static uint8_t gyro_data;
static UART_HandleTypeDef *gyro_uart_handler;
static I2C_HandleTypeDef *gyro_i2c_handler;

static std::unordered_map<Axis, TIM_HandleTypeDef *> odometer_encoder_handlers;
static TIM_HandleTypeDef *publish_timer_handler;

void setup(UART_HandleTypeDef *_motor_driver_uart_handler,
           UART_HandleTypeDef *_valve_controller_uart_handler,
           UART_HandleTypeDef *_gyro_uart_handler,
           I2C_HandleTypeDef *_gyro_i2c_handler,
           TIM_HandleTypeDef *_x_odometer_encoder_handler,
           TIM_HandleTypeDef *_y_odometer_encoder_handler,
           TIM_HandleTypeDef *_publish_timer_handler)
{
  motor_driver_uart_handler = _motor_driver_uart_handler;
  valve_controller_uart_handler = _valve_controller_uart_handler;
  gyro_uart_handler = _gyro_uart_handler;
  gyro_i2c_handler = _gyro_i2c_handler;
  odometer_encoder_handlers = {{Axis::X, _x_odometer_encoder_handler},
                               {Axis::Y, _y_odometer_encoder_handler}};
  publish_timer_handler = _publish_timer_handler;

  std::unordered_map<ValveID, ros::Subscriber<std_msgs::Bool>::CallbackT>
      valve_command_sub_callbacks = {
          {ValveID::DUAL_TABLE_UPPER_L, dualTableUpperLeftShooterCommandCallback},
          {ValveID::DUAL_TABLE_UPPER_R, dualTableUpperRightShooterCommandCallback},
          {ValveID::DUAL_TABLE_LOWER, dualTableLowerShooterCommandCallback},
          {ValveID::MOVABLE_TABLE_1200, movableTable1200ShooterCommandCallback},
          {ValveID::MOVABLE_TABLE_1500, movableTable1500ShooterCommandCallback},
          {ValveID::MOVABLE_TABLE_1800, movableTable1800ShooterCommandCallback}};

  for (auto &valve_name_pair : VALVE_NAMES)
  {
    auto &valve_id = valve_name_pair.first;
    auto &valve_name = valve_name_pair.second;

    valve_command_topic_names.emplace(valve_id, valve_name + "/command");
    valve_command_subs.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(valve_id),
        std::forward_as_tuple(valve_command_topic_names.at(valve_id).c_str(),
                              valve_command_sub_callbacks.at(valve_id)));
  }

  odom.header.frame_id = "odom";
  odom.pose.pose.position.x = 0.;
  odom.pose.pose.position.y = 0.;
  odom.pose.pose.position.z = 0.;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(0.);
  odom.twist.twist.linear.z = 0.;
  odom.twist.twist.angular.x = 0.;
  odom.twist.twist.angular.y = 0.;

  nh.initNode();

  nh.advertise(odom_pub);
  nh.advertise(sensor_states_pub);
  nh.advertise(debug_message_pub);
  nh.subscribe(cmd_vel_sub);
  for (auto &valve_command_sub_pair : valve_command_subs)
  {
    nh.subscribe(valve_command_sub_pair.second);
  }

  nh.advertiseService(reset_odometry_service_server);

  // HAL_UART_Receive_IT(gyro_uart_handler, &gyro_data, 1);

  HAL_I2C_Master_Receive_IT(gyro_i2c_handler, BNO055_I2C_ADDRESS, &gyro_data, 1);

  for (auto &htim_pair : odometer_encoder_handlers)
  {
    HAL_TIM_Encoder_Start(htim_pair.second, TIM_CHANNEL_ALL);
  }

  HAL_TIM_Base_Start_IT(publish_timer_handler);
}

void loop()
{
  nh.spinOnce();

  HAL_Delay(100);
}

double getYaw()
{
  return 0.;
}

double getAngularVelocity()
{
  return 0.;
}

void publishOdom()
{
  static const uint16_t ODOMETER_CPR = 2048;
  static const double ODOMETER_WHEEL_DIAMETER = 50.0e-3;
  static const double CALCULATION_PERIOD =
      static_cast<double>(HAL_RCC_GetPCLK1Freq()) / publish_timer_handler->Init.Prescaler *
      publish_timer_handler->Init.Period;

  std::unordered_map<Axis, int16_t> encoder_counts = {{Axis::X, 0}, {Axis::Y, 0}};
  std::unordered_map<Axis, double> odometer_deltas = {{Axis::X, 0.}, {Axis::Y, 0.}};

  for (auto &encoder_handler_pair : odometer_encoder_handlers)
  {
    auto &axis = encoder_handler_pair.first;
    auto &encoder_handler = encoder_handler_pair.second;
    encoder_counts.at(axis) = static_cast<int16_t>(__HAL_TIM_GET_COUNTER(encoder_handler));
    odometer_deltas.at(axis) =
        M_PI * ODOMETER_WHEEL_DIAMETER * encoder_counts.at(axis) / ODOMETER_CPR;
    __HAL_TIM_SET_COUNTER(encoder_handler, 0);
  }

  double yaw = getYaw();

  double delta_x =
      odometer_deltas.at(Axis::X) * std::cos(yaw) - odometer_deltas.at(Axis::Y) * std::sin(yaw);
  double delta_y =
      odometer_deltas.at(Axis::X) * std::sin(yaw) + odometer_deltas.at(Axis::Y) * std::cos(yaw);

  odom.header.stamp = nh.now();
  odom.pose.pose.position.x += delta_x;
  odom.pose.pose.position.y += delta_y;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(yaw);
  odom.twist.twist.linear.x = delta_x / CALCULATION_PERIOD;
  odom.twist.twist.linear.y = delta_y / CALCULATION_PERIOD;
  odom.twist.twist.angular.z = getAngularVelocity();

  odom_pub.publish(&odom);
}

void publishSensorStates()
{
  sensor_states.bumper_l =
      HAL_GPIO_ReadPin(BUMPER_L_GPIO_Port, BUMPER_L_Pin) == GPIO_PIN_SET;
  sensor_states.bumper_r =
      HAL_GPIO_ReadPin(BUMPER_R_GPIO_Port, BUMPER_R_Pin) == GPIO_PIN_SET;

  sensor_states_pub.publish(&sensor_states);
}

void cmdVelCallback(const geometry_msgs::Twist &cmd_vel)
{
  static const float WHEEL_RADIUS = 100.0e-3;

  uint8_t omega_data[16];
  for (uint8_t i = 0; i < 4; i++)
  {
    float theta = M_PI / 4 + i * M_PI / 2;
    float omega = (-cmd_vel.linear.x * std::sin(theta) + cmd_vel.linear.y * std::cos(theta)) /
                      WHEEL_RADIUS +
                  cmd_vel.angular.z;

    sprintf(reinterpret_cast<char *>(omega_data), "%d:%.3lf", i, omega);
    HAL_UART_Transmit_IT(motor_driver_uart_handler,
                         omega_data, strlen(reinterpret_cast<const char *>(omega_data)));
  }

  static std::string debug_string;
  debug_string = "Received cmd_vel";
  debug_message.data = debug_string.c_str();
  debug_message_pub.publish(&debug_message);
}

void valveCommandCallback(const ValveID valve_id, const std_msgs::Bool &valve_command)
{
  uint8_t valve_command_data = (static_cast<uint8_t>(valve_id) << 1) |
                               static_cast<uint8_t>(valve_command.data);
  HAL_UART_Transmit_IT(valve_controller_uart_handler,
                       &valve_command_data, sizeof(valve_command_data));

  static std::string debug_string;
  debug_string = "Received " + valve_command_topic_names.at(valve_id);
  debug_message.data = debug_string.c_str();
  debug_message_pub.publish(&debug_message);
}

void dualTableUpperLeftShooterCommandCallback(const std_msgs::Bool &command)
{
  valveCommandCallback(ValveID::DUAL_TABLE_UPPER_L, command);
}

void dualTableUpperRightShooterCommandCallback(const std_msgs::Bool &command)
{
  valveCommandCallback(ValveID::DUAL_TABLE_UPPER_R, command);
}

void dualTableLowerShooterCommandCallback(const std_msgs::Bool &command)
{
  valveCommandCallback(ValveID::DUAL_TABLE_LOWER, command);
}

void movableTable1200ShooterCommandCallback(const std_msgs::Bool &command)
{
  valveCommandCallback(ValveID::MOVABLE_TABLE_1200, command);
}

void movableTable1500ShooterCommandCallback(const std_msgs::Bool &command)
{
  valveCommandCallback(ValveID::MOVABLE_TABLE_1500, command);
}

void movableTable1800ShooterCommandCallback(const std_msgs::Bool &command)
{
  valveCommandCallback(ValveID::MOVABLE_TABLE_1800, command);
}

void resetOdometryCallback(const tsuten_msgs::ResetOdometryRequest &request,
                           tsuten_msgs::ResetOdometryResponse &response)
{
  odom.pose.pose.position.x = 0.;
  odom.pose.pose.position.y = 0.;

  static std::string debug_string;
  debug_string = "reset_odometry called";
  debug_message.data = debug_string.c_str();
  debug_message_pub.publish(&debug_message);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == gyro_uart_handler)
  {
    HAL_UART_Receive_IT(gyro_uart_handler, &gyro_data, 1);
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == gyro_i2c_handler)
  {
    HAL_I2C_Master_Receive_IT(gyro_i2c_handler, BNO055_I2C_ADDRESS, &gyro_data, 1);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == publish_timer_handler)
  {
    publishOdom();
    publishSensorStates();
  }
}