cmake_minimum_required(VERSION 3.0.2)
project(motor_test)

add_library(${PROJECT_NAME}
  imu_uart.cpp
  motor_pwm.cpp
)

add_executable(${PROJECT_NAME}_node motor_control.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  -lwiringPi
)
