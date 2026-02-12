/**
 * @file api_main.cpp
 * @author Rh
 * @brief 应用层与stm32的接口，放着最直接运行的函数
 * @version 0.1
 * @date 2026-01-20
 *
 * @copyright Copyright (c) 2026
 *
 */


#include "api_main.h"
#include "main.h" // IWYU pragma: keep

#include "stdio.h"
#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"
#include "cmsis_os2.h"

#include "bsp_usart.hpp"
#include "JC2804.hpp"


/**
 * @brief main中初始化（无freertos）
 * @note  也就是在main.c中写了一个函数调用，这样转嫁就可以使用cpp了
 *
 */
void app_init()
{
  printf("\napp_init_ok\n");
}


/* 创建对应句柄 handle */
osThreadId_t can_rx_handler_task_handle;


/**
 * @brief 和freertos有关的初始化
 * @note  也就是在main.c中的MX_FREERTOS_INIT里，写了一个函数调用，这样转嫁就可以使用cpp了
 *
 */
void freertos_init()
{
  // 初始化bsp设备
  bsp_usart6.init();
  bsp_can1.init();


  // 初始化云台双电机
  motor_yaw.init();
  motor_pitch.init();


  // 创建 CAN 接收后处理任务
  const osThreadAttr_t can_rx_handler_task_attributes = {
    .name       = "can_rx_handler_task",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
  };
  can_rx_handler_task_handle = osThreadNew(_can_rx_handler_task, nullptr, &can_rx_handler_task_attributes);


  printf("freertos_init_ok\n");
}


/**
 * 因为CMSIS_OS2这个封装，导致很多东西和原生的FreeRTOS不一样，所以写法也会有的不一样
 * CMSIS_OS2很多句柄不对外声明，如果想用只能extern出来用
 * 虽然说CMSIS_OS2做了层封装，方便使用。但是原生的FreeRTOS在以后要用的时候，还是要花时间适应
 * 默认任务只能weak声明，其他的可以使用外部声明
 * CubeMX提供了FreeRTOS配置的部分，故而使用CubeMX配置了，只有一些内容，必须使用cpp来写
 * 以下均为FreeRTOS的内容定义，因为使用c调用cpp，所以只能在这边定义，然后外部声明让官方接口调用
 * printf要加\n
 *
 * @note 在C++中使用FreeRTOS的Task函数时
 *       需要将任务函数声明为extern "C"格式
 *       同时函数参数必须是void *pvParameters。
 *       所以，我直接把任务放到一个转接文件，然后extern
 */

/* 声明需要使用的句柄 */

/* CMSIS_OS2中使用的是声明，我只需要外部定义同名的函数，然后exteren "C"即可 */


/**
 * @brief 默认任务
 *
 * @param argument 默认参数
 */
extern "C" void _defaultTask(void *argument)
{
  motor_yaw.enter_closed_loop();
  osDelay(10);
  motor_yaw.set_control_mode(1);

  for (;;)
  {
    motor_yaw.set_speed(60);
    osDelay(1000);
    motor_yaw.set_speed(-60);
    osDelay(1000);
  }
}


/**
 * @brief 用于处理CAN接收后的数据处理
 *
 */
// api_main.cpp
extern "C" void _can_rx_handler_task(void *argument)
{
  can_rx_msg_t rx_msg;

  for (;;)
  {
    osStatus_t status = bsp_can1.receive(&rx_msg, osWaitForever);

    if (status == osOK)
    {
      // 根据 ID 判断是哪个设备
      uint32_t device_id = rx_msg.header.Identifier & 0x0F; // 0x600+ID -> ID 是低4位

      // 查找对应的 JC2804 实例
      if (device_id == motor_yaw._device_id)
      {
        motor_yaw.on_can_message(&rx_msg);
      }
      else if (device_id == motor_pitch._device_id)
      {
        motor_pitch.on_can_message(&rx_msg);
      }

      // 打印调试信息
      printf("CAN RX: ID=0x%03X, Data=", rx_msg.header.Identifier);
      for (int i = 0; i < 8; ++i)
      {
        printf("%02X ", rx_msg.data[i]);
      }
      printf("\n");
    }
  }
}