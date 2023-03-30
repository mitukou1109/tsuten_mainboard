#pragma once

#include <stm32f4xx_hal.h>

#ifdef __cplusplus
extern "C"
{
#endif
  void setup(UART_HandleTypeDef *motor_driver_uart_handler,
             UART_HandleTypeDef *valve_controller_uart_handler,
             I2C_HandleTypeDef *gyro_i2c_handler,
             TIM_HandleTypeDef *x_odometer_encoder_handler,
             TIM_HandleTypeDef *y_odometer_encoder_handler,
             TIM_HandleTypeDef *publish_timer_handler,
             TIM_HandleTypeDef *tape_led_blink_timer_handler);

  void loop();
#ifdef __cplusplus
}
#endif