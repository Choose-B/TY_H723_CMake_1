// ... existing code ...
#include "bsp_can.hpp" // 包含bsp_can的头文件
#include "cmsis_os2.h"

// ========== 类定义 ==========
class JC2804
{
public:
  // ========== 构造与初始化 ==========
  // 注意：构造函数中传入 bsp_can 指针
  JC2804(bsp_can* can_interface, uint8_t device_id);
  void init();

  // ========== 成员变量 ==========
  uint8_t  _device_id;

  // ========== 控制指令（写入） ==========
  void set_torque(float torque);
  void set_speed(float speed);
  void set_absolute_position(float position);
  void set_relative_position(float position);
  void set_low_speed(float speed);
  void pv_command(int32_t position, float speed);
  void pvt_command(int32_t position, float speed, float torque_percent);
  void set_control_mode(uint8_t mode);

  // ========== 系统控制指令 ==========
  void idle();
  void enter_closed_loop();
  void erase();
  void save();
  void restart();
  void set_origin();
  void set_temporary_origin();

  // ========== 读取请求（触发） ==========
  void request_power_voltage();
  void request_bus_current();
  void request_real_time_speed();
  void request_real_time_position();
  void request_driver_temperature();
  void request_motor_temperature();
  void request_error_info();

  // ========== 获取最新数据（获取结果） ==========
  void get_latest_data(
    float* voltage, float* current, float* speed,
    float* position, float* driver_temp, float* motor_temp,
    uint32_t* error_info);

  // ========== CAN消息回调接口 ==========
  void on_can_message(can_rx_msg_t* rx_msg);

private:
  // ========== 静态常量 ==========
  static const float VOLTAGE_SCALE;
  static const float CURRENT_SCALE;
  static const float SPEED_SCALE;
  static const float POSITION_SCALE;
  static const float TEMPERATURE_SCALE;

  // ========== 成员变量 ==========
  bsp_can* _can;

  // ========== 最新数据存储 ==========
  float    latest_voltage     = 0.0f;
  float    latest_current     = 0.0f;
  float    latest_speed       = 0.0f;
  float    latest_position    = 0.0f;
  float    latest_driver_temp = 0.0f;
  float    latest_motor_temp  = 0.0f;
  uint32_t latest_error_info  = 0;

  // ========== 互斥锁 ==========
  osMutexId_t _mutex_id;

  // ========== 新增：读取请求类型枚举 ==========
  enum RequestType
  {
    NONE_REQUEST = 0,
    VOLTAGE_REQUEST,
    CURRENT_REQUEST,
    SPEED_REQUEST,
    POSITION_REQUEST,
    DRIVER_TEMP_REQUEST,
    MOTOR_TEMP_REQUEST,
    ERROR_INFO_REQUEST
  };

  // ========== 新增：跟踪最近的请求类型 ==========
  RequestType _last_request_type;

  // ========== 私有辅助函数 ==========
  void send_async_command(uint8_t cmd, const uint8_t* data, uint8_t len);
  void send_read_request(uint8_t cmd, uint16_t reg_addr, RequestType req_type); // 新增
  bool validate_response(uint8_t expected_cmd, can_rx_msg_t* rx_msg);
  void store_received_data(uint8_t* data); // 修改
  
  // ========== 互斥锁操作辅助函数 ==========
  inline void lock() { if(_mutex_id) osMutexAcquire(_mutex_id, osWaitForever); }
  inline void unlock() { if(_mutex_id) osMutexRelease(_mutex_id); }
};

extern JC2804 motor_yaw;
extern JC2804 motor_pitch;

// ... existing code ...