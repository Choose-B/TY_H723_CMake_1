// ... existing code ...
#include "JC2804.hpp"
#include "cmsis_os.h"
#include "string.h"
#include "cmsis_os2.h"

/**
 * @todo 明天测试读取接收，把读取最新数据的位置改成结构体，方便存取
 * 
 */

// ========== 静态常量定义 ==========
const float JC2804::VOLTAGE_SCALE     = 0.1f;
const float JC2804::CURRENT_SCALE     = 0.01f;
const float JC2804::SPEED_SCALE       = 0.01f;
const float JC2804::POSITION_SCALE    = 0.01f;
const float JC2804::TEMPERATURE_SCALE = 0.1f;

// ========== 全局实例 ==========
// 注意：在实际初始化时，需要将CAN接口传入
// motor_yaw.init() 会在 freertos_init 中调用
// motor_pitch.init() 会在 freertos_init 中调用
JC2804 motor_yaw(&bsp_can1, 2);
JC2804 motor_pitch(&bsp_can1, 1);

// ========== 构造与初始化 ==========
JC2804::JC2804(bsp_can* can_interface, uint8_t device_id)
  : _can(can_interface), _device_id(device_id), _last_request_type(NONE_REQUEST), _mutex_id(NULL)
{
}

void JC2804::init()
{
  // 创建互斥锁属性
  osMutexAttr_t attr;
  attr.name      = "JC2804_Mutex";
  attr.attr_bits = 0U;
  attr.cb_mem    = NULL;
  attr.cb_size   = 0U;

  // 创建互斥锁
  _mutex_id = osMutexNew(&attr);
}

// ========== 底层发送与请求跟踪 ==========
void JC2804::send_async_command(uint8_t cmd, const uint8_t* data, uint8_t len)
{
  if (!_can || len > 8)
    return;

  lock();                              // 加锁
  uint32_t tx_id = 0x600 | _device_id; // 标准帧ID格式：0x600 + DeviceID
  _can->send(tx_id, const_cast<uint8_t*>(data), len);
  unlock(); // 解锁
}

// 新增：用于发送请求并记录请求类型，以便后续解析响应
void JC2804::send_read_request(uint8_t cmd, uint16_t reg_addr, RequestType req_type)
{
  uint8_t data[8] = {0};
  data[0]         = cmd;                    // 命令字
  data[1]         = (reg_addr >> 8) & 0xFF; // 寄存器地址高字节
  data[2]         = reg_addr & 0xFF;        // 寄存器地址低字节
  // 其余字节保持为0

  lock(); // 加锁
  // 记录本次请求类型，以便响应时能正确解析
  _last_request_type = req_type;

  send_async_command(cmd, data, 8);
  unlock(); // 解锁
}

// ========== 控制指令（写入）==========
void JC2804::set_torque(float torque)
{
  // 文档 5.8: 命令字 0x2B, 寄存器 0x0020, 2字节有符号电流值（单位：A × 100）
  // 注意：文档描述为电流，但函数名为扭矩。这里按电流处理，若需扭矩需查手册确认换算关系。
  int16_t current_raw = static_cast<int16_t>(torque / CURRENT_SCALE);
  uint8_t data[8]     = {
    0x2B,       // Command: Write Register
    0x00, 0x20, // Register Address: 0x0020 (Current/Torque)
    0x00,       // Reserved
    static_cast<uint8_t>(current_raw >> 8),
    static_cast<uint8_t>(current_raw & 0xFF),
    0x00, 0x00 // Padding
  };
  send_async_command(0x2B, data, 8);
}

void JC2804::set_speed(float speed)
{
  // 文档 5.9: 0x23 + reg 0x21, 4字节有符号速度（rpm × 100）
  int32_t spd_raw = static_cast<int32_t>(speed / SPEED_SCALE);
  uint8_t data[8] = {
    0x23,       // Command: Write Register
    0x00, 0x21, // Register Address: 0x0021 (Speed)
    0x00,       // Reserved
    static_cast<uint8_t>(spd_raw >> 24),
    static_cast<uint8_t>(spd_raw >> 16),
    static_cast<uint8_t>(spd_raw >> 8),
    static_cast<uint8_t>(spd_raw & 0xFF)};
  send_async_command(0x23, data, 8);
}

void JC2804::set_absolute_position(float position)
{
  // 文档 5.10: 0x23 + reg 0x23, 4字节有符号位置（度 × 100）
  int32_t pos_raw = static_cast<int32_t>(position / POSITION_SCALE);
  uint8_t data[8] = {
    0x23,       // Command: Write Register
    0x00, 0x23, // Register Address: 0x0023 (Absolute Position)
    0x00,       // Reserved
    static_cast<uint8_t>(pos_raw >> 24),
    static_cast<uint8_t>(pos_raw >> 16),
    static_cast<uint8_t>(pos_raw >> 8),
    static_cast<uint8_t>(pos_raw & 0xFF)};
  send_async_command(0x23, data, 8);
}

void JC2804::set_relative_position(float position)
{
  // 文档 5.11: 0x23 + reg 0x25, 4字节有符号相对位置（度 × 100）
  int32_t pos_raw = static_cast<int32_t>(position / POSITION_SCALE);
  uint8_t data[8] = {
    0x23,       // Command: Write Register
    0x00, 0x25, // Register Address: 0x0025 (Relative Position)
    0x00,       // Reserved
    static_cast<uint8_t>(pos_raw >> 24),
    static_cast<uint8_t>(pos_raw >> 16),
    static_cast<uint8_t>(pos_raw >> 8),
    static_cast<uint8_t>(pos_raw & 0xFF)};
  send_async_command(0x23, data, 8);
}

void JC2804::set_low_speed(float speed)
{
  // 文档 5.12: 0x2B + reg 0x27, 2字节无符号低速（rpm）
  uint16_t spd_raw = static_cast<uint16_t>(speed); // 文档描述不清晰，假设为整数rpm
  uint8_t  data[8] = {
    0x2B,       // Command: Write Register
    0x00, 0x27, // Register Address: 0x0027 (Low Speed)
    0x00,       // Reserved
    static_cast<uint8_t>(spd_raw >> 8),
    static_cast<uint8_t>(spd_raw & 0xFF),
    0x00, 0x00 // Padding
  };
  send_async_command(0x2B, data, 8);
}

void JC2804::pv_command(int32_t position, float speed)
{
  // 文档 5.13: 命令字 0x24
  // 数据格式：[cmd][pos H][M][L][L] (4B signed, ×100), [spd H][L] (2B unsigned rpm), [null]
  int32_t  pos_raw = static_cast<int32_t>(position * 100); // 放大100倍
  uint16_t spd_raw = static_cast<uint16_t>(speed);         // rpm 整数
  uint8_t  data[8] = {
    0x24, // Command: PV Command
    static_cast<uint8_t>(pos_raw >> 24),
    static_cast<uint8_t>(pos_raw >> 16),
    static_cast<uint8_t>(pos_raw >> 8),
    static_cast<uint8_t>(pos_raw & 0xFF),
    static_cast<uint8_t>(spd_raw >> 8),
    static_cast<uint8_t>(spd_raw & 0xFF),
    0x00 // Padding
  };
  send_async_command(0x24, data, 8);
}

void JC2804::pvt_command(int32_t position, float speed, float torque_percent)
{
  // 文档 5.14: 命令字 0x25
  // 格式：[cmd][pos H][M][L][L] (4B pos ×100), [spd H][L] (2B speed rpm), [torque% (1B 0~100)]
  int32_t  pos_raw    = static_cast<int32_t>(position * 100);
  uint16_t spd_raw    = static_cast<uint16_t>(speed);
  uint8_t  torque_raw = static_cast<uint8_t>(torque_percent);
  uint8_t  data[8]    = {
    0x25, // Command: PVT Command
    static_cast<uint8_t>(pos_raw >> 24),
    static_cast<uint8_t>(pos_raw >> 16),
    static_cast<uint8_t>(pos_raw >> 8),
    static_cast<uint8_t>(pos_raw & 0xFF),
    static_cast<uint8_t>(spd_raw >> 8),
    static_cast<uint8_t>(spd_raw & 0xFF),
    torque_raw // Torque Percentage
  };
  send_async_command(0x25, data, 8);
}

void JC2804::set_control_mode(uint8_t mode)
{
  // 文档 5.15: 0x2B + reg 0x60, 写入模式值（0~5）
  uint8_t data[8] = {
    0x2B,             // Command: Write Register
    0x00, 0x60,       // Register Address: 0x0060 (Control Mode)
    0x00,             // Reserved
    0x00, 0x00, 0x00, // Padding
    mode              // Mode Value (0-5)
  };
  send_async_command(0x2B, data, 8);
}

// ========== 系统控制指令（5.16~5.22）==========
void JC2804::idle()
{
  uint8_t data[8] = {0x2B, 0x00, 0xA0, 0x00, 0x00, 0x01, 0x00, 0x00};
  send_async_command(0x2B, data, 8);
}

void JC2804::enter_closed_loop()
{
  uint8_t data[8] = {0x2B, 0x00, 0xA2, 0x00, 0x00, 0x01, 0x00, 0x00};
  send_async_command(0x2B, data, 8);
}

void JC2804::erase()
{
  uint8_t data[8] = {0x2B, 0x00, 0xA3, 0x00, 0x00, 0x01, 0x00, 0x00};
  send_async_command(0x2B, data, 8);
}

void JC2804::save()
{
  uint8_t data[8] = {0x2B, 0x00, 0xA4, 0x00, 0x00, 0x01, 0x00, 0x00};
  send_async_command(0x2B, data, 8);
}

void JC2804::restart()
{
  uint8_t data[8] = {0x2B, 0x00, 0xA5, 0x00, 0x00, 0x01, 0x00, 0x00};
  send_async_command(0x2B, data, 8);
}

void JC2804::set_origin()
{
  uint8_t data[8] = {0x2B, 0x00, 0xA6, 0x00, 0x00, 0x01, 0x00, 0x00};
  send_async_command(0x2B, data, 8);
}

void JC2804::set_temporary_origin()
{
  uint8_t data[8] = {0x2B, 0x00, 0xA7, 0x00, 0x00, 0x01, 0x00, 0x00};
  send_async_command(0x2B, data, 8);
}

// ========== 读取请求（修改：记录请求类型） ==========
void JC2804::request_power_voltage()
{
  send_read_request(0x4B, 0x0004, VOLTAGE_REQUEST);
}

void JC2804::request_bus_current()
{
  send_read_request(0x4B, 0x0005, CURRENT_REQUEST);
}

void JC2804::request_real_time_speed()
{
  send_read_request(0x43, 0x0006, SPEED_REQUEST);
}

void JC2804::request_real_time_position()
{
  send_read_request(0x43, 0x0008, POSITION_REQUEST);
}

void JC2804::request_driver_temperature()
{
  send_read_request(0x4B, 0x000A, DRIVER_TEMP_REQUEST);
}

void JC2804::request_motor_temperature()
{
  send_read_request(0x4B, 0x000B, MOTOR_TEMP_REQUEST);
}

void JC2804::request_error_info()
{
  send_read_request(0x43, 0x000C, ERROR_INFO_REQUEST);
}

// ========== 数据解析（修改：基于请求类型） ==========
void JC2804::store_received_data(uint8_t* data)
{
  // 根据 _last_request_type 解析数据
  // 假设收到的响应数据格式符合文档描述
  lock(); // 加锁
  switch (_last_request_type)
  {
    case VOLTAGE_REQUEST:
      if (data[0] == 0x4B)
      { // 确认是读取响应
        uint16_t raw       = (static_cast<uint16_t>(data[4]) << 8) | data[5];
        latest_voltage     = raw * VOLTAGE_SCALE;
        _last_request_type = NONE_REQUEST; // 清除请求类型
      }
      break;
    case CURRENT_REQUEST:
      if (data[0] == 0x4B)
      {
        uint16_t raw       = (static_cast<uint16_t>(data[4]) << 8) | data[5];
        latest_current     = raw * CURRENT_SCALE;
        _last_request_type = NONE_REQUEST;
      }
      break;
    case SPEED_REQUEST:
      if (data[0] == 0x43)
      {
        int32_t raw        = (static_cast<int32_t>(data[4]) << 24) | (static_cast<int32_t>(data[5]) << 16) | (static_cast<int32_t>(data[6]) << 8) | (static_cast<int32_t>(data[7]));
        latest_speed       = static_cast<float>(raw) * SPEED_SCALE;
        _last_request_type = NONE_REQUEST;
      }
      break;
    case POSITION_REQUEST:
      if (data[0] == 0x43)
      {
        int32_t raw        = (static_cast<int32_t>(data[4]) << 24) | (static_cast<int32_t>(data[5]) << 16) | (static_cast<int32_t>(data[6]) << 8) | (static_cast<int32_t>(data[7]));
        latest_position    = static_cast<float>(raw) * POSITION_SCALE;
        _last_request_type = NONE_REQUEST;
      }
      break;
    case DRIVER_TEMP_REQUEST:
      if (data[0] == 0x4B)
      {
        uint16_t raw       = (static_cast<uint16_t>(data[4]) << 8) | data[5];
        latest_driver_temp = raw * TEMPERATURE_SCALE;
        _last_request_type = NONE_REQUEST;
      }
      break;
    case MOTOR_TEMP_REQUEST:
      if (data[0] == 0x4B)
      {
        uint16_t raw       = (static_cast<uint16_t>(data[4]) << 8) | data[5];
        latest_motor_temp  = raw * TEMPERATURE_SCALE;
        _last_request_type = NONE_REQUEST;
      }
      break;
    case ERROR_INFO_REQUEST:
      if (data[0] == 0x43)
      {
        // 错误信息通常为4字节无符号整数
        uint32_t raw       = (static_cast<uint32_t>(data[4]) << 24) | (static_cast<uint32_t>(data[5]) << 16) | (static_cast<uint32_t>(data[6]) << 8) | (static_cast<uint32_t>(data[7]));
        latest_error_info  = raw;
        _last_request_type = NONE_REQUEST;
      }
      break;
    default:
      // 如果没有匹配的请求类型，或者不是预期的响应命令字，则忽略
      break;
  }
  unlock(); // 解锁
}


// ========== 响应验证（修改：简化逻辑，主要验证ID） ==========
bool JC2804::validate_response(uint8_t expected_cmd, can_rx_msg_t* rx_msg)
{
  uint32_t expected_id = 0x580 | _device_id; // 响应ID格式：0x580 + DeviceID
  if (rx_msg->header.Identifier != expected_id)
  {
    return false;
  }
  // 对于读取请求，响应的命令字应与发送的请求命令字相同 (0x4B or 0x43)
  // 对于写入请求，响应的命令字通常是 0x2A (ACK)
  // 我们在这里主要验证ID，具体命令字的处理交给 store_received_data
  // 简化验证逻辑，只要ID对得上，就认为是本设备的响应
  return true;
}


// ========== CAN 回调（修改：基于请求类型处理） ==========
void JC2804::on_can_message(can_rx_msg_t* rx_msg)
{
  // 验证消息ID是否属于本设备
  if (!validate_response(0, rx_msg))
  { // 不再关心expected_cmd
    return;
  }

  uint8_t received_cmd = rx_msg->data[0];

  // 检查是否是读取响应，并且我们之前确实发送了读取请求
  if (((received_cmd == 0x4B || received_cmd == 0x43) && _last_request_type != NONE_REQUEST))
  {
    // 尝试解析响应数据
    store_received_data(rx_msg->data);
    // 如果解析失败（例如，命令字不匹配请求类型），_last_request_type 不会被清除
    // 可以考虑增加超时机制来清除它，但这里简单地假设响应紧跟请求
  }
  // 注意：写入命令的响应 (0x2A) 通常不需要特殊处理来更新状态，
  // 因为它们只是确认命令已接收。
  // 如果需要确认写入的值，可能需要后续读取相关寄存器。
}


// ========== 获取最新数据 ==========
void JC2804::get_latest_data(
  float* voltage, float* current, float* speed,
  float* position, float* driver_temp, float* motor_temp,
  uint32_t* error_info)
{
  lock(); // 加锁
  if (voltage)
    *voltage = latest_voltage;
  if (current)
    *current = latest_current;
  if (speed)
    *speed = latest_speed;
  if (position)
    *position = latest_position;
  if (driver_temp)
    *driver_temp = latest_driver_temp;
  if (motor_temp)
    *motor_temp = latest_motor_temp;
  if (error_info)
    *error_info = latest_error_info;
  unlock(); // 解锁
}