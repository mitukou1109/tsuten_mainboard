#pragma once

#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C"
{
#endif
  void setup(UART_HandleTypeDef *_motor_driver_uart_handler,
             UART_HandleTypeDef *_valve_controller_uart_handler,
             UART_HandleTypeDef *_gyro_uart_handler,
             I2C_HandleTypeDef *_gyro_i2c_handler,
             TIM_HandleTypeDef *_x_odometer_encoder_handler,
             TIM_HandleTypeDef *_y_odometer_encoder_handler,
             TIM_HandleTypeDef *_publish_timer_handler);

  void loop();
#ifdef __cplusplus
}
#endif