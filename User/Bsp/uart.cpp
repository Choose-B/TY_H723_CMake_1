#include "uart.hpp"
#include "string.h"
#include "stdio.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;


// 模板实例化实现
template class SerialDriver<256, sizeof(uint32_t)>;


SerialDriver<256, sizeof(uint32_t)> g_serial_driver_uart1(&huart1, SerialDriver<256, sizeof(uint32_t)>::ReceiveMode::LATEST_ONLY, true);
SerialDriver<256, sizeof(uint32_t)> g_serial_driver_uart6(&huart6, SerialDriver<256, sizeof(uint32_t)>::ReceiveMode::LATEST_ONLY, true);


//< @todo: 发现了问题，构造函数这个东西 是在main函数之前就实现的，故而需要专门写一个init函数在main的正确位置调用


/**
 * @brief Construct a new SerialDriver<BUFFER_SIZE, MSG_SIZE>::SerialDriver object
 * 
 * @tparam BUFFER_SIZE 
 * @tparam MSG_SIZE 
 * @param huart 
 * @param rx_mode 
 * @param transmit_signal 
 */
template <size_t BUFFER_SIZE, size_t MSG_SIZE>
SerialDriver<BUFFER_SIZE, MSG_SIZE>::SerialDriver(UART_HandleTypeDef *huart, ReceiveMode rx_mode, bool transmit_signal)
  : _huart(huart), _receive_mode(rx_mode), _rx_active(false), _current_buffer(false), _buffer_size(BUFFER_SIZE), _msg_item_size(MSG_SIZE), _transmit_enable(transmit_signal), _last_received_length(0)
{
  // 创建互斥锁 (使用CMSIS-RTOS2 API)
  const osMutexAttr_t mutex_attr =
    {
      .name      = "SerialMutex",
      .attr_bits = 0,
      .cb_mem    = NULL,
      .cb_size   = 0};

  _mutex_id = osMutexNew(&mutex_attr);

  // 根据接收模式创建消息队列 - 只有LATEST_ONLY模式才创建
  if (_receive_mode == ReceiveMode::LATEST_ONLY)
  {
    const osMessageQueueAttr_t msgq_attr =
      {
        .name      = "SerialMsgQ",
        .attr_bits = 0,
        .cb_mem    = NULL,
        .cb_size   = 0,
        .mq_mem    = NULL,
        .mq_size   = 0};

    // 对于LATEST_ONLY模式，消息队列长度为1，只保留最新数据
    _msg_queue_id = osMessageQueueNew(1, _msg_item_size, &msgq_attr);
  }
  else
  {
    _msg_queue_id = NULL; // 非LATEST_ONLY模式不需要消息队列
  }

  // 初始化接收流缓冲区数组
  for (int i = 0; i < 2; i++)
  {
    _rx_stream_buffers[i] = NULL;
  }

  // 根据接收模式创建相应的缓冲区 (使用FreeRTOS API - 流缓冲区)
  switch (_receive_mode)
  {
    case ReceiveMode::SINGLE_BUFFER:
      _rx_stream_buffers[0] = xStreamBufferCreate(BUFFER_SIZE, 1);
      break;
    case ReceiveMode::DOUBLE_BUFFER:
      // 创建两个流缓冲区用于双缓冲机制
      _rx_stream_buffers[0] = xStreamBufferCreate(BUFFER_SIZE, 1);
      _rx_stream_buffers[1] = xStreamBufferCreate(BUFFER_SIZE, 1);
      break;
    default: // LATEST_ONLY
      // 不需要流缓冲区
      break;
  }

  if (_transmit_enable)
  {
    // 创建发送流缓冲区 (使用FreeRTOS API - 流缓冲区)
    _tx_stream_buffer = xStreamBufferCreate(BUFFER_SIZE, 1);
  }
  else
  {
    _tx_stream_buffer = NULL;
  }

  // 启用IDLE中断
  __HAL_UART_ENABLE_IT(_huart, UART_IT_IDLE);

  // 启动接收
  startReception();
}

// 析构函数实现
template <size_t BUFFER_SIZE, size_t MSG_SIZE>
SerialDriver<BUFFER_SIZE, MSG_SIZE>::~SerialDriver()
{
  stopReception();

  if (_mutex_id != NULL)
  {
    osMutexDelete(_mutex_id);
  }

  // 只有LATEST_ONLY模式才创建了消息队列，需要删除
  if (_msg_queue_id != NULL)
  {
    osMessageQueueDelete(_msg_queue_id);
  }

  // 统一释放接收流缓冲区
  for (int i = 0; i < 2; i++)
  {
    if (_rx_stream_buffers[i] != NULL)
    {
      vStreamBufferDelete(_rx_stream_buffers[i]);
    }
  }

  if (_tx_stream_buffer != NULL)
  {
    vStreamBufferDelete(_tx_stream_buffer);
  }
}

// 发送数据实现
template <size_t BUFFER_SIZE, size_t MSG_SIZE>
int SerialDriver<BUFFER_SIZE, MSG_SIZE>::sendData(const uint8_t *data, size_t size, uint32_t timeout)
{
  if (!_transmit_enable)
  {
    return -1; // 没使能发送 返回错误
  }

  if (_tx_stream_buffer == NULL)
  {
    return -1; // 发送缓冲区未初始化
  }

  osStatus_t status = osMutexAcquire(_mutex_id, timeout);
  if (status != osOK)
  {
    return -1; // 获取互斥锁失败 超时会这样 如果给了forever会一直等待直到成功
  }

  // 将数据写入发送流缓冲区
  size_t bytes_written = xStreamBufferSend(_tx_stream_buffer, data, size, pdMS_TO_TICKS(timeout));

  // 如果发送缓冲区中有数据，启动发送
  if (bytes_written > 0)
  {
    startTransmission();
  }

  osMutexRelease(_mutex_id);
  return bytes_written;
}

// 接收数据实现
template <size_t BUFFER_SIZE, size_t MSG_SIZE>
int SerialDriver<BUFFER_SIZE, MSG_SIZE>::receiveData(uint8_t *buffer, size_t size, uint32_t timeout)
{
  // 根据接收模式进行不同的处理
  if (_receive_mode == ReceiveMode::LATEST_ONLY)
  {
    // 在LATEST_ONLY模式下，从消息邮箱获取最新数据
    osStatus_t status = osMutexAcquire(_mutex_id, timeout);
    if (status != osOK)
    {
      return -1;
    }

    // 获取最新消息
    status = osMessageQueueGet(_msg_queue_id, buffer, NULL, timeout);
    osMutexRelease(_mutex_id);

    if (status == osOK)
    {
      return (size < MSG_SIZE) ? size : MSG_SIZE; // 返回实际读取的字节数
    }
    else
    {
      return -1; // 没有数据或超时
    }
  }
  else
  {
    // 单缓冲和双缓冲的统一处理
    StreamBufferHandle_t target_buffer = NULL;

    if (_receive_mode == ReceiveMode::SINGLE_BUFFER)
    {
      target_buffer = _rx_stream_buffers[0];
    }
    else // DOUBLE_BUFFER
    {
      target_buffer = _current_buffer ? _rx_stream_buffers[1] : _rx_stream_buffers[0];
    }

    if (target_buffer != NULL)
    {
      size_t bytes_read = xStreamBufferReceive(target_buffer, buffer, size, pdMS_TO_TICKS(timeout));
      return bytes_read;
    }
    return -1;
  }
}

// 获取发送缓冲区剩余空间实现
template <size_t BUFFER_SIZE, size_t MSG_SIZE>
size_t SerialDriver<BUFFER_SIZE, MSG_SIZE>::getTxFreeSpace()
{
  if (_tx_stream_buffer != NULL)
  {
    return xStreamBufferSpacesAvailable(_tx_stream_buffer);
  }
  return 0;
}

// 获取接收缓冲区可用数据量实现
template <size_t BUFFER_SIZE, size_t MSG_SIZE>
size_t SerialDriver<BUFFER_SIZE, MSG_SIZE>::getRxAvailableData()
{
  if (_receive_mode == ReceiveMode::LATEST_ONLY)
  {
    // 对于LATEST_ONLY模式，检查消息队列是否有数据
    if (_msg_queue_id != NULL)
    {
      uint32_t count = osMessageQueueGetCount(_msg_queue_id);
      return count * _msg_item_size;
    }
    return 0;
  }
  else
  {
    size_t total_bytes = 0;

    if (_receive_mode == ReceiveMode::SINGLE_BUFFER)
    {
      if (_rx_stream_buffers[0] != NULL)
      {
        total_bytes += xStreamBufferBytesAvailable(_rx_stream_buffers[0]);
      }
    }
    else // DOUBLE_BUFFER
    {
      for (int i = 0; i < 2; i++)
      {
        if (_rx_stream_buffers[i] != NULL)
        {
          total_bytes += xStreamBufferBytesAvailable(_rx_stream_buffers[i]);
        }
      }
    }
    return total_bytes;
  }
}

// 开始接收数据实现
template <size_t BUFFER_SIZE, size_t MSG_SIZE>
void SerialDriver<BUFFER_SIZE, MSG_SIZE>::startReception()
{
  // 启动多字节DMA接收
  HAL_UART_Receive_DMA(_huart, _rx_dma_buffer, BUFFER_SIZE);
  _rx_active = true;
}

// 停止接收数据实现
template <size_t BUFFER_SIZE, size_t MSG_SIZE>
void SerialDriver<BUFFER_SIZE, MSG_SIZE>::stopReception()
{
  _rx_active = false;
  HAL_UART_DMAStop(_huart);
}

// 开始传输数据实现
template <size_t BUFFER_SIZE, size_t MSG_SIZE>
void SerialDriver<BUFFER_SIZE, MSG_SIZE>::startTransmission()
{
  if (_tx_stream_buffer == NULL)
  {
    return; // 发送缓冲区未初始化
  }

  if (!isTransmitting())
  {
    // 从发送缓冲区获取数据准备发送
    size_t bytes_to_send = xStreamBufferReceive(_tx_stream_buffer, _tx_dma_buffer, BUFFER_SIZE, 0);
    if (bytes_to_send > 0)
    {
      HAL_UART_Transmit_DMA(_huart, _tx_dma_buffer, bytes_to_send);
    }
  }
}

// 检查是否正在传输实现
template <size_t BUFFER_SIZE, size_t MSG_SIZE>
bool SerialDriver<BUFFER_SIZE, MSG_SIZE>::isTransmitting()
{
  return (_huart->gState == HAL_UART_STATE_BUSY_TX);
}

// DMA传输完成回调函数实现
template <size_t BUFFER_SIZE, size_t MSG_SIZE>
void SerialDriver<BUFFER_SIZE, MSG_SIZE>::dmaTransferCompleteCallback(UART_HandleTypeDef *huart)
{
  if (huart->gState == HAL_UART_STATE_BUSY_RX)
  {
    // 接收完成处理 - 这种情况不应该发生，因为我们使用IDLE中断
    // 主要是用于错误恢复
    handleReceiveComplete();
  }
  else if (huart->gState == HAL_UART_STATE_BUSY_TX)
  {
    // 发送完成处理
    handleTransmitComplete();
  }
}

// DMA错误回调函数实现
template <size_t BUFFER_SIZE, size_t MSG_SIZE>
void SerialDriver<BUFFER_SIZE, MSG_SIZE>::dmaErrorCallback(UART_HandleTypeDef *huart)
{
  // 记录错误状态并尝试恢复
  handleDmaError();
}

// IDLE中断处理函数
template <size_t BUFFER_SIZE, size_t MSG_SIZE>
void SerialDriver<BUFFER_SIZE, MSG_SIZE>::handleIdleInterrupt(uint32_t received_length)
{
  _last_received_length = received_length;

  if (_receive_mode == ReceiveMode::LATEST_ONLY)
  {
    // 处理整组数据，而非单个字节
    if (received_length >= _msg_item_size)
    {
      // 将整组数据的最后MSG_SIZE个字节作为最新数据
      uint8_t *latest_data_ptr = &_rx_dma_buffer[received_length - _msg_item_size];

      // 清空消息队列中已有的消息，然后发送新消息（确保只保留最新数据）
      uint8_t temp_data[MSG_SIZE];
      while (osMessageQueueGet(_msg_queue_id, temp_data, 0, 0) == osOK)
        ;

      osMessageQueuePut(_msg_queue_id, latest_data_ptr, 0, 0);
    }
    else if (received_length > 0)
    {
      // 如果接收的数据不足MSG_SIZE，将整个数据复制到一个临时缓冲区
      uint8_t temp_latest_data[MSG_SIZE] = {0}; // 初始化为0
      memcpy(temp_latest_data, _rx_dma_buffer, received_length);

      // 清空消息队列中已有的消息，然后发送新消息
      uint8_t temp_data[MSG_SIZE];
      while (osMessageQueueGet(_msg_queue_id, temp_data, 0, 0) == osOK)
        ;

      osMessageQueuePut(_msg_queue_id, temp_latest_data, 0, 0);
    }
  }
  else
  {
    // 对于其他模式，将整个数据包放入流缓冲区
    StreamBufferHandle_t target_buffer = NULL;

    if (_receive_mode == ReceiveMode::SINGLE_BUFFER)
    {
      target_buffer = _rx_stream_buffers[0];
    }
    else // DOUBLE_BUFFER
    {
      target_buffer   = _current_buffer ? _rx_stream_buffers[1] : _rx_stream_buffers[0];
      _current_buffer = !_current_buffer; // 双缓冲切换
    }

    if (target_buffer != NULL)
    {
      xStreamBufferSend(target_buffer, _rx_dma_buffer, received_length, 0);
    }
  }

  // 重新启动DMA接收
  if (_rx_active)
  {
    HAL_UART_Receive_DMA(_huart, _rx_dma_buffer, BUFFER_SIZE);
  }
}

// 处理接收完成事件实现
template <size_t BUFFER_SIZE, size_t MSG_SIZE>
void SerialDriver<BUFFER_SIZE, MSG_SIZE>::handleReceiveComplete()
{
  // 对于多字节DMA+IDLE中断模式，接收完成主要是为了错误恢复
  // 实际的数据处理在IDLE中断中完成
  if (_rx_active)
  {
    HAL_UART_Receive_DMA(_huart, _rx_dma_buffer, BUFFER_SIZE);
  }
}

// 处理发送完成事件实现
template <size_t BUFFER_SIZE, size_t MSG_SIZE>
void SerialDriver<BUFFER_SIZE, MSG_SIZE>::handleTransmitComplete()
{
  // 检查是否还有更多数据需要发送
  if (_tx_stream_buffer != NULL && xStreamBufferSpacesAvailable(_tx_stream_buffer) < BUFFER_SIZE)
  {
    // 有更多数据待发送，继续发送过程
    startTransmission();
  }
}

// 处理DMA错误实现
template <size_t BUFFER_SIZE, size_t MSG_SIZE>
void SerialDriver<BUFFER_SIZE, MSG_SIZE>::handleDmaError()
{
  // 停止当前传输
  HAL_UART_DMAStop(_huart);

  // 记录错误日志（实际应用中应使用适当的日志系统）
  printf("\n\nError \n\n");

  // 尝试重启接收
  if (_rx_active)
  {
    startReception();
  }
}

// IDLE中断处理函数
template <size_t BUFFER_SIZE, size_t MSG_SIZE>
void SerialDriver<BUFFER_SIZE, MSG_SIZE>::handleIdleInterruptInternal(UART_HandleTypeDef *huart)
{
  // 获取DMA剩余计数值，计算已接收的数据长度
  uint32_t dma_counter     = __HAL_DMA_GET_COUNTER(huart->hdmarx);
  uint32_t received_length = BUFFER_SIZE - dma_counter;

  // 处理接收到的数据包
  if (received_length > 0)
  {
    handleIdleInterrupt(received_length);
  }
}

// IDLE中断处理函数
void HAL_BSP_UART_IRQHandler(UART_HandleTypeDef *huart)
{
  // 检查是否是IDLE中断
  if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET) && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE) != RESET))
  {
    // 清除IDLE中断标志
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_IDLE);

    // 使用if-else结构处理不同UART的IDLE中断
    if (huart == &huart1)
    {
      g_serial_driver_uart1.handleIdleInterruptInternal(huart); // 让类内部处理DMA计数器和BUFFER_SIZE
    }
    else if (huart == &huart6)
    {
      g_serial_driver_uart6.handleIdleInterruptInternal(huart); // 让类内部处理DMA计数器和BUFFER_SIZE
    }
    else
    {
    }
  }
}


// C语言回调函数实现
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    g_serial_driver_uart1.dmaTransferCompleteCallback(huart);
  }
  else if (huart == &huart6)
  {
    g_serial_driver_uart6.dmaTransferCompleteCallback(huart);
  }
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // 这个回调主要用于错误恢复，实际数据处理在IDLE中断中
  if (huart == &huart1)
  {
    g_serial_driver_uart1.dmaTransferCompleteCallback(huart);
  }
  else if (huart == &huart6)
  {
    g_serial_driver_uart6.dmaTransferCompleteCallback(huart);
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    g_serial_driver_uart1.dmaErrorCallback(huart);
  }
  else if (huart == &huart6)
  {
    g_serial_driver_uart6.dmaErrorCallback(huart);
  }
}
