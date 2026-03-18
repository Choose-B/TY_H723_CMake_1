/**
 * @file bsp_can.cpp
 * @author Rh
 * @brief 实现了一个简易的CAN2.0收发驱动。
 * @version 0.1
 * @date 2026-02-11
 *
 * @todo 此处只写了CAN1，以后要是有帮忙写下CAN2,CAN3。以及CANFD的就更好了
 *       以及CAN绑定协议处理 多个CAN的处理 CAN滤波器 的部分写的还是不够好 希望后人能修改
 *
 * @copyright Copyright (c) 2026
 *
 *
 * @brief 使用示例（必须要在freertos的任务中运行收发 中断中不要进行此操作 中断不能阻塞）
 *
 * @note 类的实例化 以及初始化
 *
 *   bsp_can bsp_can1(&hfdcan1); // 全局实例化类
 *   bsp_can1.init();            // 需要在freertos内核开启之后去init
 *   can_rx_msg_t can1_msg;      // 实例化一个接收消息用的msg
 *
 * @note extern好之后，在任务中使用
 *
 *   bsp_can1.send(0x602, data1, 8);           // 发送报头为0x602，数组指针为data1，长度为8（uint8_t）
 *   bsp_can1.receive(&can1_msg,osWaitForever) // 使用此结构体接收数据，等待时间无穷久，使用中断接收
 *
 */
#ifndef __BSP_CAN_HPP__
#define __BSP_CAN_HPP__

#include "fdcan.h"    // IWYU pragma: keep
#include "cmsis_os.h" // IWYU pragma: keep


// CAM接收信息结构体接口
typedef struct
{
  FDCAN_RxHeaderTypeDef header;
  uint8_t               data[8];
} can_rx_msg_t;


// CAN类
class bsp_can
{
public:
  osMessageQueueId_t rx_queue_handle; // CMSIS_OS2的队列
  osMutexId_t        tx_mutex_handle; // CMSIS_OS2的互斥锁

  // 构造函数，增加instance_id参数用于区分不同的实例
  bsp_can(FDCAN_HandleTypeDef* hfdcan, int instance_id);

  // 析构函数
  ~bsp_can();

  // 启动 FDCAN 并配置过滤器
  HAL_StatusTypeDef init();

  // 发送标准数据帧
  HAL_StatusTypeDef send(uint32_t stdId, uint8_t* pData, uint8_t len);

  // 从队列接收数据
  osStatus_t receive(can_rx_msg_t* msg, uint32_t timeout);

private:
  FDCAN_HandleTypeDef* _hfdcan;        // CAN句柄
  int                  _instance_id;    // 实例ID，
  char                 queue_name[32]; // 实例队列的名字，用于调试时看到名字
  char                 mutex_name[32]; // 实例互斥锁的名字，用于调试时看到名字
};


// 外部声明这些类实例化的对象

extern bsp_can bsp_can1;


#endif // __BSP_CAN_HPP__