#include "cmsis_os2.h"
#include "gimbal_jc.hpp"


/*
 * 坐标系统说明：
 * 电机转法 逆时针
 * (pitch电机正转为向上)
 * ^z
 * |   / y(front)(相机朝向)
 * |  /
 * | /
 * . ————————> x(right) (yaw电机反转为向右)
 *
 * 云台控制系统中:
 * - Yaw轴: 水平旋转轴，对应x轴方向的旋转
 * - Pitch轴: 垂直俯仰轴，对应y轴方向的上下移动
 */

gimbal_jc::gimbal_jc(jc2804* yaw_motor, jc2804* pitch_motor) : _yaw_motor(yaw_motor), _pitch_motor(pitch_motor), _control_mode(gimbal_position_mode_e::GIMBAL_POSITION_MODE_FILTER), _state(gimbal_state_e::GIMBAL_MOTIONLESS), _current_yaw_angle(0.0f), _current_pitch_angle(0.0f), _target_yaw_angle(0.0f), _target_pitch_angle(0.0f)
{
}

gimbal_jc::~gimbal_jc()
{
}

void gimbal_jc::init()
{
  // 进入闭环模式
  enter_closed_loop();

  // 设置初始控制模式为位置平滑模式
  set_position_mode(gimbal_position_mode_e::GIMBAL_POSITION_MODE_FILTER);

  // 初始时设置速度为0，防止电机失控
  _yaw_motor->set_speed(0.0f);
  _pitch_motor->set_speed(0.0f);

  // 更新状态
  _state = gimbal_state_e::GIMBAL_MOTIONLESS;
}

void gimbal_jc::set_position_mode(gimbal_position_mode_e mode)
{
  _control_mode = mode;
  osDelay(10);
  _yaw_motor->set_control_mode(static_cast<uint8_t>(mode));
  osDelay(10);
  _pitch_motor->set_control_mode(static_cast<uint8_t>(mode));
}

void gimbal_jc::set_angle(float yaw_angle, float pitch_angle)
{
  _target_yaw_angle   = yaw_angle;
  _target_pitch_angle = pitch_angle;

  // 根据当前控制模式发送相应的位置指令
  switch (_control_mode)
  {
    case gimbal_position_mode_e::GIMBAL_POSITION_MODE_TRAPEZOID:
      _yaw_motor->set_absolute_position(yaw_angle);
      _pitch_motor->set_absolute_position(pitch_angle);
      break;
    case gimbal_position_mode_e::GIMBAL_POSITION_MODE_FILTER:
      _yaw_motor->set_absolute_position(yaw_angle);
      _pitch_motor->set_absolute_position(pitch_angle);
      break;
    case gimbal_position_mode_e::GIMBAL_POSITION_MODE_PASS:
      _yaw_motor->set_absolute_position(yaw_angle);
      _pitch_motor->set_absolute_position(pitch_angle);
      break;
    default:
      _yaw_motor->set_absolute_position(yaw_angle);
      _pitch_motor->set_absolute_position(pitch_angle);
      break;
  }

  update_state();
}

void gimbal_jc::set_yaw_angle(float yaw_angle)
{
  _target_yaw_angle = yaw_angle;

  switch (_control_mode)
  {
    case gimbal_position_mode_e::GIMBAL_POSITION_MODE_TRAPEZOID:
    case gimbal_position_mode_e::GIMBAL_POSITION_MODE_FILTER:
    case gimbal_position_mode_e::GIMBAL_POSITION_MODE_PASS:
      _yaw_motor->set_absolute_position(yaw_angle);
      break;
    default:
      _yaw_motor->set_absolute_position(yaw_angle);
      break;
  }

  update_state();
}

void gimbal_jc::set_pitch_angle(float pitch_angle)
{
  _target_pitch_angle = pitch_angle;

  switch (_control_mode)
  {
    case gimbal_position_mode_e::GIMBAL_POSITION_MODE_TRAPEZOID:
    case gimbal_position_mode_e::GIMBAL_POSITION_MODE_FILTER:
    case gimbal_position_mode_e::GIMBAL_POSITION_MODE_PASS:
      _pitch_motor->set_absolute_position(pitch_angle);
      break;
    default:
      _pitch_motor->set_absolute_position(pitch_angle);
      break;
  }

  update_state();
}

void gimbal_jc::increment_angle(float yaw_offset, float pitch_offset)
{
  float new_yaw_angle   = _current_yaw_angle + yaw_offset;
  float new_pitch_angle = _current_pitch_angle + pitch_offset;

  set_angle(new_yaw_angle, new_pitch_angle);
}

void gimbal_jc::enter_closed_loop()
{
  _yaw_motor->enter_closed_loop();
  osDelay(10); // 等待电机响应
  _pitch_motor->enter_closed_loop();
  osDelay(10); // 等待电机响应
}

void gimbal_jc::set_zero_point()
{
  _yaw_motor->set_origin();
  _pitch_motor->set_origin();
  osDelay(100); // 等待电机响应
}

gimbal_state_e gimbal_jc::get_state()
{
  return _state;
}

void gimbal_jc::get_motor_data(MotorData& yaw_data, MotorData& pitch_data)
{
  yaw_data   = _yaw_motor->get_latest_data_struct();
  pitch_data = _pitch_motor->get_latest_data_struct();
}

void gimbal_jc::stop()
{
  _yaw_motor->set_speed(0.0f);
  _pitch_motor->set_speed(0.0f);
  _state = gimbal_state_e::GIMBAL_MOTIONLESS;
}

void gimbal_jc::update_state()
{
  bool yaw_moving   = (_target_yaw_angle != _current_yaw_angle);
  bool pitch_moving = (_target_pitch_angle != _current_pitch_angle);

  if (yaw_moving && pitch_moving)
  {
    _state = gimbal_state_e::GIMBAL_BOTH_MOVING;
  }
  else if (yaw_moving)
  {
    _state = gimbal_state_e::GIMBAL_YAWING;
  }
  else if (pitch_moving)
  {
    _state = gimbal_state_e::GIMBAL_PITCHING;
  }
  else
  {
    _state = gimbal_state_e::GIMBAL_MOTIONLESS;
  }

  // 更新当前位置（实际应用中可能需要从电机反馈获取精确位置）
  _current_yaw_angle   = _target_yaw_angle;
  _current_pitch_angle = _target_pitch_angle;
}