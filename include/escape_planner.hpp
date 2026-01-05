#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"

#include <vector>
#include <array>
#include <memory>

class EscapePlanner : public rclcpp::Node {
public:
  EscapePlanner();

private:
  // 콜백들
  void emergency_cb(const std_msgs::msg::Bool::SharedPtr msg);
  void timer_cb();
  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void local_wpnts_cb(const f110_msgs::msg::WpntArray::SharedPtr msg);

  double quatToYaw(const geometry_msgs::msg::Quaternion &q);

  // 현재 /safety/trigger 레벨 (HARD STOP용)
  bool safety_trigger_now_{false};

  // 회전 방향 (좌 / 우)
  enum class TurnDirection { LEFT, RIGHT };

  // 경로 + pose 기반 회전 방향 결정
  TurnDirection choose_turn_direction();

  // --- ROS 인터페이스: 구독 / 퍼블리셔 / 타이머 ---
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr local_wpnts_sub_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr esc_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finish_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- 상태 플래그 / 스텝 ---
  bool emergency_active_;
  int step_;
  int tick_count_;

  // --- 마지막 pose / 경로 정보 ---
  geometry_msgs::msg::PoseStamped::SharedPtr last_pose_;
  // [x, y, yaw]
  std::vector<std::array<double,3>> waypoints_;

  // 현재 사용 중인 회전 방향
  TurnDirection turn_dir_;

  // --- lat_err_avg 기반 방향 선택 + 잘못된 방향 보정 로직용 상태 ---

  // choose_turn_direction()에서 계산한 평균 lateral error
  double current_lat_err_avg_{0.0};
  bool   has_current_lat_err_avg_{false};

  // "이 방향이 틀렸다고 판단해서 반대 방향으로 고정"하는 모드
  bool          forced_dir_active_{false};
  TurnDirection forced_dir_{TurnDirection::LEFT};

  // 직전 emergency 트리거 시점에서의 |lat_err|과 사용했던 방향
  double        last_abs_lat_err_at_trigger_{0.0};
  TurnDirection last_dir_at_trigger_{TurnDirection::LEFT};
  bool          has_last_trigger_info_{false};
};

