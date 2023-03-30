#include <tsuten_mainboard.hpp>

#define _USE_MATH_DEFINES
#include <cmath>

#include <main.h>
#include <tf/tf.h>

const std::unordered_map<TsutenMainboard::ValveID, std::string> TsutenMainboard::VALVE_NAMES =
    {{ValveID::DUAL_TABLE_UPPER_L, "dual_table_upper_left_shooter"},
     {ValveID::DUAL_TABLE_UPPER_R, "dual_table_upper_right_shooter"},
     {ValveID::DUAL_TABLE_LOWER, "dual_table_lower_shooter"},
     {ValveID::MOVABLE_TABLE_1200, "movable_table_1200_shooter"},
     {ValveID::MOVABLE_TABLE_1500, "movable_table_1500_shooter"},
     {ValveID::MOVABLE_TABLE_1800, "movable_table_1800_shooter"}};

const std::unordered_map<TsutenMainboard::ValveID, TsutenMainboard::ValvePortSet>
    TsutenMainboard::VALVE_PORT_SETS =
        {{ValveID::DUAL_TABLE_UPPER_L, ValvePortSet{0, 0, 1}},
         {ValveID::DUAL_TABLE_UPPER_R, ValvePortSet{0, 2, 3}},
         {ValveID::DUAL_TABLE_LOWER, ValvePortSet{0, 4, 5}},
         {ValveID::MOVABLE_TABLE_1200, ValvePortSet{0, 6, 7}},
         {ValveID::MOVABLE_TABLE_1500, ValvePortSet{1, 0, 1}},
         {ValveID::MOVABLE_TABLE_1800, ValvePortSet{1, 2, 3}}};

const uint16_t TsutenMainboard::GYRO_I2C_ADDRESS = 0x28;

TsutenMainboard::TsutenMainboard(UART_HandleTypeDef *motor_driver_uart_handler,
                                 UART_HandleTypeDef *valve_controller_uart_handler,
                                 I2C_HandleTypeDef *gyro_i2c_handler,
                                 TIM_HandleTypeDef *x_odometer_encoder_handler,
                                 TIM_HandleTypeDef *y_odometer_encoder_handler,
                                 TIM_HandleTypeDef *publish_timer_handler,
                                 TIM_HandleTypeDef *tape_led_blink_timer_handler)
    : odom_pub_("odom_raw", &odom_),
      sensor_states_pub_("sensor_states", &sensor_states_),
      debug_message_pub_("debug_message", &debug_message_),
      cmd_vel_sub_("cmd_vel", &TsutenMainboard::cmdVelCallback, this),
      tape_led_command_sub_("tape_led_command", &TsutenMainboard::tapeLEDCommandCallback, this),
      reset_odometry_service_server_("reset_odometry",
                                     &TsutenMainboard::resetOdometryCallback, this),
      motor_driver_uart_handler_(motor_driver_uart_handler),
      valve_controller_uart_handler_(valve_controller_uart_handler),
      gyro_i2c_handler_(gyro_i2c_handler),
      odometer_encoder_handlers_({{Axis::X, x_odometer_encoder_handler},
                                  {Axis::Y, y_odometer_encoder_handler}}),
      publish_timer_handler_(publish_timer_handler),
      tape_led_blink_timer_handler_(tape_led_blink_timer_handler)
{
  for (auto &valve_name_pair : VALVE_NAMES)
  {
    auto &valve_id = valve_name_pair.first;
    auto &valve_name = valve_name_pair.second;

    static std::unordered_map<ValveID, std::string> valve_command_topic_names;
    valve_command_topic_names.emplace(valve_id, valve_name + "/command");

    valve_command_subs_.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(valve_id),
        std::forward_as_tuple(valve_command_topic_names.at(valve_id).c_str(),
                              std::function<void(const std_msgs::Bool &)>(
                                  std::bind(&TsutenMainboard::valveCommandCallback,
                                            this, valve_id, std::placeholders::_1))));
  }

  nh_.initNode();

  nh_.advertise(odom_pub_);
  nh_.advertise(sensor_states_pub_);
  nh_.advertise(debug_message_pub_);
  nh_.subscribe(cmd_vel_sub_);
  nh_.subscribe(tape_led_command_sub_);
  for (auto &valve_command_sub_pair : valve_command_subs_)
  {
    nh_.subscribe(valve_command_sub_pair.second);
  }

  nh_.advertiseService(reset_odometry_service_server_);

  __HAL_UART_DISABLE_IT(&huart2, UART_IT_PE);
  __HAL_UART_DISABLE_IT(&huart2, UART_IT_ERR);

  HAL_I2C_Master_Receive_IT(gyro_i2c_handler_, GYRO_I2C_ADDRESS, &gyro_data_, 1);

  for (auto &htim_pair : odometer_encoder_handlers_)
  {
    HAL_TIM_Encoder_Start(htim_pair.second, TIM_CHANNEL_ALL);
  }

  HAL_TIM_Base_Start_IT(publish_timer_handler_);

  publishDebugMessage("Setup completed");
}

void TsutenMainboard::loop()
{
  nh_.spinOnce();

  HAL_Delay(10);
}

void TsutenMainboard::publishOdom()
{
  static const uint16_t ODOMETER_CPR = 2048;
  static const double ODOMETER_WHEEL_DIAMETER = 50.0e-3;
  static const double CALCULATION_PERIOD =
      publish_timer_handler_->Init.Prescaler / static_cast<double>(HAL_RCC_GetPCLK1Freq()) * publish_timer_handler_->Init.Period;

  std::unordered_map<Axis, int16_t> encoder_counts = {{Axis::X, 0}, {Axis::Y, 0}};
  std::unordered_map<Axis, double> odometer_deltas = {{Axis::X, 0.}, {Axis::Y, 0.}};

  for (auto &htim_pair : odometer_encoder_handlers_)
  {
    auto &axis = htim_pair.first;
    auto &htim = htim_pair.second;
    encoder_counts.at(axis) = static_cast<int16_t>(__HAL_TIM_GET_COUNTER(htim));
    odometer_deltas.at(axis) =
        M_PI * ODOMETER_WHEEL_DIAMETER * encoder_counts.at(axis) / ODOMETER_CPR;
    __HAL_TIM_SET_COUNTER(htim, 0);
  }

  double yaw = getYaw();

  double delta_x =
      -odometer_deltas.at(Axis::X) * std::cos(yaw) + odometer_deltas.at(Axis::Y) * std::sin(yaw);
  double delta_y =
      -odometer_deltas.at(Axis::X) * std::sin(yaw) - odometer_deltas.at(Axis::Y) * std::cos(yaw);

  odom_.stamp = nh_.now();
  odom_.x += std::round(delta_x * 1000);
  odom_.y += std::round(delta_y * 1000);
  odom_.theta = std::round(yaw * 1000);
  odom_.v_x = std::round(delta_x / CALCULATION_PERIOD * 1000);
  odom_.v_y = std::round(delta_y / CALCULATION_PERIOD * 1000);
  odom_.v_theta = std::round(getAngularVelocity() * 1000);

  odom_pub_.publish(&odom_);
}

void TsutenMainboard::publishSensorStates()
{
  sensor_states_.bumper_l =
      HAL_GPIO_ReadPin(BUMPER_L_GPIO_Port, BUMPER_L_Pin) == GPIO_PIN_RESET;
  sensor_states_.bumper_r =
      HAL_GPIO_ReadPin(BUMPER_R_GPIO_Port, BUMPER_R_Pin) == GPIO_PIN_RESET;

  sensor_states_pub_.publish(&sensor_states_);
}

void TsutenMainboard::publishDebugMessage(std::string debug_message)
{
  debug_message_.data = debug_message.c_str();
  debug_message_pub_.publish(&debug_message_);
}

double TsutenMainboard::getYaw()
{
  return 0.;
}

double TsutenMainboard::getAngularVelocity()
{
  return 0.;
}

void TsutenMainboard::toggleTapeLED()
{
  if (tape_led_color_.r > 0.5)
  {
    HAL_GPIO_TogglePin(TAPE_LED_R_GPIO_Port, TAPE_LED_R_Pin);
  }
  if (tape_led_color_.g > 0.5)
  {
    HAL_GPIO_TogglePin(TAPE_LED_G_GPIO_Port, TAPE_LED_G_Pin);
  }
  if (tape_led_color_.b > 0.5)
  {
    HAL_GPIO_TogglePin(TAPE_LED_B_GPIO_Port, TAPE_LED_B_Pin);
  }
}

void TsutenMainboard::cmdVelCallback(const geometry_msgs::Twist &cmd_vel)
{
  publishDebugMessage("Received cmd_vel");

  static const float WHEEL_RADIUS = 53.5e-3;

  for (uint8_t i = 0; i < 4; i++)
  {
    float theta = M_PI / 4 + i * M_PI / 2;
    int16_t velocity = std::round(
                       (-cmd_vel.linear.x * std::sin(theta) + cmd_vel.linear.y * std::cos(theta) +
                        cmd_vel.angular.z * WHEEL_RADIUS) *
        1000. / 2);

    for (int j = 0; j < 4; j++)
    {
      uint8_t velocity_data =
          (i << 5) | (j << 3) |
          ((j == 3) ? ((velocity < 0) << 2) | ((std::abs(velocity) & (0b11 << 9)) >> 9)
                    : (std::abs(velocity) & (0b111 << (j * 3))) >> (j * 3));

      HAL_UART_Transmit_IT(motor_driver_uart_handler_, &velocity_data, 1);
    }
  }
}

void TsutenMainboard::tapeLEDCommandCallback(const tsuten_msgs::TapeLEDCommand &tape_led_command)
{
  publishDebugMessage("Received tape_led_command");

  tape_led_color_ = tape_led_command.color;

  if (tape_led_command.blink)
  {
    HAL_TIM_Base_Start_IT(tape_led_blink_timer_handler_);
  }
  else
  {
    HAL_TIM_Base_Stop_IT(tape_led_blink_timer_handler_);

    HAL_GPIO_WritePin(TAPE_LED_R_GPIO_Port, TAPE_LED_R_Pin,
                      (tape_led_color_.r > 0.5) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TAPE_LED_G_GPIO_Port, TAPE_LED_G_Pin,
                      (tape_led_color_.g > 0.5) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TAPE_LED_B_GPIO_Port, TAPE_LED_B_Pin,
                      (tape_led_color_.b > 0.5) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
}

void TsutenMainboard::valveCommandCallback(const ValveID valve_id,
                                           const std_msgs::Bool &valve_command)
{
  publishDebugMessage("Received " + VALVE_NAMES.at(valve_id) + "/command");

  auto &VALVE_PORT_SET = VALVE_PORT_SETS.at(valve_id);
  auto &valve_controller_port_state =
      valve_controller_port_states_.at(VALVE_PORT_SET.board_number);

  valve_controller_port_state &=
      ~(1 << (valve_command.data ? VALVE_PORT_SET.close_port : VALVE_PORT_SET.open_port));
  valve_controller_port_state |=
      1 << (valve_command.data ? VALVE_PORT_SET.open_port : VALVE_PORT_SET.close_port);

  uint8_t valve_command_data[3] = {0x53, VALVE_PORT_SET.board_number, valve_controller_port_state};
  HAL_UART_Transmit_IT(valve_controller_uart_handler_, valve_command_data,
                       sizeof(valve_command_data) / sizeof(uint8_t));
}

void TsutenMainboard::resetOdometryCallback(const tsuten_msgs::ResetOdometryRequest &request,
                                            tsuten_msgs::ResetOdometryResponse &response)
{
  odom_.x = 0.;
  odom_.y = 0.;
}