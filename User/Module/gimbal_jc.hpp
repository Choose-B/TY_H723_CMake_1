#ifndef __GIMBAL_JC_HPP__
#define __GIMBAL_JC_HPP__

#include "jc2804.hpp"

// 云台控制模式枚举
enum class gimbal_position_mode_e
{
  GIMBAL_POSITION_MODE_TRAPEZOID = 2, // 位置梯形模式
  GIMBAL_POSITION_MODE_FILTER    = 3, // 位置滤波模式
  GIMBAL_POSITION_MODE_PASS      = 4  // 位置直通模式
};

// 云台状态枚举
enum class gimbal_state_e
{
  GIMBAL_MOTIONLESS = 0, // 云台静止
  GIMBAL_YAWING,         // yaw轴转动
  GIMBAL_PITCHING,       // pitch轴转动
  GIMBAL_BOTH_MOVING     // 两轴同时转动
} ;

class gimbal_jc
{
public:
  // 构造函数
  gimbal_jc(jc2804* yaw_motor, jc2804* pitch_motor);

  // 析构函数
  ~gimbal_jc();

  /**
   * @brief 云台初始化
   * @note 需要在FreeRTOS内核启动后调用
   */
  void init();

  /**
   * @brief 设置云台位置模式
   * @param mode 位置模式
   */
  void set_position_mode(gimbal_position_mode_e mode);

  /**
   * @brief 控制云台转动到指定角度
   * @param yaw_angle Yaw轴目标角度（度）
   * @param pitch_angle Pitch轴目标角度（度）
   */
  void set_angle(float yaw_angle, float pitch_angle);

  /**
   * @brief 控制云台Yaw轴单独转动
   * @param yaw_angle Yaw轴目标角度（度）
   */
  void set_yaw_angle(float yaw_angle);

  /**
   * @brief 控制云台Pitch轴单独转动
   * @param pitch_angle Pitch轴目标角度（度）
   */
  void set_pitch_angle(float pitch_angle);

  /**
   * @brief 增量控制云台角度
   * @param yaw_offset Yaw轴增量角度（度）
   * @param pitch_offset Pitch轴增量角度（度）
   */
  void increment_angle(float yaw_offset, float pitch_offset);

  /**
   * @brief 获取当前云台状态
   * @return 云台当前状态
   */
  gimbal_state_e get_state();

  /**
   * @brief 进入闭环模式
   */
  void enter_closed_loop();

  /**
   * @brief 设置零点
   */
  void set_zero_point();

  /**
   * @brief 获取电机数据
   * @param yaw_data Yaw电机数据引用
   * @param pitch_data Pitch电机数据引用
   */
  void get_motor_data(MotorData& yaw_data, MotorData& pitch_data);

  /**
   * @brief 停止云台运动
   */
  void stop();

private:
  jc2804*                _yaw_motor;           // Yaw轴电机指针
  jc2804*                _pitch_motor;         // Pitch轴电机指针
  gimbal_position_mode_e _control_mode;        // 当前控制模式
  gimbal_state_e         _state;               // 当前云台状态
  float                  _current_yaw_angle;   // 当前Yaw角度
  float                  _current_pitch_angle; // 当前Pitch角度
  float                  _target_yaw_angle;    // 目标Yaw角度
  float                  _target_pitch_angle;  // 目标Pitch角度

  /**
   * @brief 更新云台状态
   */
  void update_state();
};

#endif // __GIMBAL_JC_HPP__