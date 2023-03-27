#include <mainpp.h>

#include <tsuten_mainboard.hpp>

TsutenMainboard *tsuten_mainboard;

void setup(UART_HandleTypeDef *motor_driver_uart_handler,
           UART_HandleTypeDef *valve_controller_uart_handler,
           UART_HandleTypeDef *gyro_uart_handler,
           I2C_HandleTypeDef *gyro_i2c_handler,
           TIM_HandleTypeDef *x_odometer_encoder_handler,
           TIM_HandleTypeDef *y_odometer_encoder_handler,
           TIM_HandleTypeDef *publish_timer_handler,
           TIM_HandleTypeDef *tape_led_blink_timer_handler)
{
  tsuten_mainboard = new TsutenMainboard(motor_driver_uart_handler,
                                         valve_controller_uart_handler,
                                         gyro_uart_handler,
                                         gyro_i2c_handler,
                                         x_odometer_encoder_handler,
                                         y_odometer_encoder_handler,
                                         publish_timer_handler,
                                         tape_led_blink_timer_handler);
}

void loop()
{
  tsuten_mainboard->loop();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == tsuten_mainboard->gyro_uart_handler_)
  {
    HAL_UART_Receive_IT(tsuten_mainboard->gyro_uart_handler_, &tsuten_mainboard->gyro_data_, 1);
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == tsuten_mainboard->gyro_i2c_handler_)
  {
    HAL_I2C_Master_Receive_IT(tsuten_mainboard->gyro_i2c_handler_,
                              TsutenMainboard::BNO055_I2C_ADDRESS,
                              &tsuten_mainboard->gyro_data_, 1);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == tsuten_mainboard->publish_timer_handler_)
  {
    tsuten_mainboard->publishOdom();
    tsuten_mainboard->publishSensorStates();
  }
  else if (htim == tsuten_mainboard->tape_led_blink_timer_handler_)
  {
    tsuten_mainboard->toggleTapeLED();
  }
}