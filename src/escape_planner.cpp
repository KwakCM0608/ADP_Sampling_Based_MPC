#include "escape_planner.hpp"

#include <chrono>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

EscapePlanner::EscapePlanner()
: Node("escape_planner"),
  emergency_active_(false),
  step_(0),
  tick_count_(0),
  turn_dir_(TurnDirection::LEFT),    // 기본은 좌회전
  safety_trigger_now_(false)         // /safety/trigger 현재 레벨
{
  // /safety/trigger 구독 (위험 감지)
  emergency_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/safety/trigger",
    10,
    std::bind(&EscapePlanner::emergency_cb, this, std::placeholders::_1)
  );

  // 내 차 pose 구독
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/car_state/pose",
    10,
    std::bind(&EscapePlanner::pose_cb, this, std::placeholders::_1)
  );

  // local waypoints 구독
  local_wpnts_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
    "/local_waypoints",
    10,
    std::bind(&EscapePlanner::local_wpnts_cb, this, std::placeholders::_1)
  );

  // /escape/cmd 퍼블리시 (탈출용 명령)
  esc_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/escape/cmd", 10);
  // /escape/finished 퍼블리시 (탈출 종료 신호)
  finish_pub_ = create_publisher<std_msgs::msg::Bool>("/escape/finished", 10);

  // 20 Hz 타이머
  timer_ = create_wall_timer(
    50ms,  // 20Hz
    std::bind(&EscapePlanner::timer_cb, this)
  );

  RCLCPP_INFO(get_logger(), "[EscapePlanner] Ready.");
}

// /car_state/pose 콜백
void EscapePlanner::pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  last_pose_ = msg;
}

// /local_waypoints 콜백
void EscapePlanner::local_wpnts_cb(const f110_msgs::msg::WpntArray::SharedPtr msg)
{
  waypoints_.clear();
  waypoints_.reserve(msg->wpnts.size());
  for (const auto &w : msg->wpnts) {
    // [x, y, yaw] = [x_m, y_m, psi_rad]
    waypoints_.push_back({ w.x_m, w.y_m, w.psi_rad });
  }
}

double EscapePlanner::quatToYaw(const geometry_msgs::msg::Quaternion &q)
{
  // 표준적인 quaternion → yaw 변환
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

// lat_err_car 기반 경로 기준 회전 방향 선택 (앞쪽 여러 점 평균)
EscapePlanner::TurnDirection EscapePlanner::choose_turn_direction()
{
  if (!last_pose_ || waypoints_.empty()) {
    RCLCPP_WARN(get_logger(),
      "[EscapePlanner] No pose or waypoints yet. Use previous turn_dir_ = %s",
      (turn_dir_ == TurnDirection::LEFT ? "LEFT" : "RIGHT"));
    // 정보가 없으면 이전 방향 유지
    return turn_dir_;
  }

  const auto &pose = last_pose_->pose;
  double px = pose.position.x;
  double py = pose.position.y;
  double yaw_car = quatToYaw(pose.orientation);

  const double c = std::cos(yaw_car);
  const double s = std::sin(yaw_car);

  // 1) 차량 "앞쪽"에 있는 wp 중에서 가장 가까운 것을 찾는다
  int best_idx = -1;
  double best_d2 = 1e18;

  for (int i = 0; i < static_cast<int>(waypoints_.size()); ++i) {
    double wx = waypoints_[i][0];
    double wy = waypoints_[i][1];

    double dx = wx - px;
    double dy = wy - py;

    // 차량 진행 방향 기준 앞/뒤 판단 (앞쪽만 사용)
    double forward_dot = dx * c + dy * s;
    if (forward_dot < 0.0) {
      // 차량 뒤쪽에 있는 점은 무시
      continue;
    }

    double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_idx = i;
    }
  }

  if (best_idx < 0) {
    // 앞쪽에 있는 점을 못 찾은 경우: 그냥 이전 방향 유지
    RCLCPP_WARN(get_logger(),
      "[EscapePlanner] No forward waypoint found. Keep previous turn_dir_ = %s",
      (turn_dir_ == TurnDirection::LEFT ? "LEFT" : "RIGHT"));
    return turn_dir_;
  }

  // 2) best_idx 근처 여러 개를 보면서 lat_err_car 평균
  int lookahead_start = 2;   // 바로 앞은 노이즈가 클 수 있으니 약간 띄우기
  int lookahead_end   = 10;  // 상황 보고 튜닝 (5~15 정도)

  int idx_start = std::min(best_idx + lookahead_start,
                           static_cast<int>(waypoints_.size()) - 1);
  int idx_end   = std::min(best_idx + lookahead_end,
                           static_cast<int>(waypoints_.size()) - 1);

  double sum_lat_err = 0.0;
  int    cnt_lat_err = 0;

  for (int idx = idx_start; idx <= idx_end; ++idx) {
    double wx = waypoints_[idx][0];
    double wy = waypoints_[idx][1];

    double dx = wx - px;
    double dy = wy - py;

    // 차량 좌표계: x_forward, y_left
    double x_car =  c * dx + s * dy;
    double y_car = -s * dx + c * dy;

    // 너무 뒤쪽이면 제외 (이론상 여기선 거의 없겠지만)
    if (x_car < 0.0) continue;

    sum_lat_err += y_car;
    cnt_lat_err++;
  }

  double lat_err_avg;
  if (cnt_lat_err > 0) {
    lat_err_avg = sum_lat_err / static_cast<double>(cnt_lat_err);
  } else {
    // fallback: 예전 방식처럼 best_idx + 5 하나만 사용
    int idx = std::min(best_idx + 5,
                       static_cast<int>(waypoints_.size()) - 1);

    double wx = waypoints_[idx][0];
    double wy = waypoints_[idx][1];
    double dx = wx - px;
    double dy = wy - py;

    double x_car =  c * dx + s * dy;
    double y_car = -s * dx + c * dy;
    (void)x_car;

    lat_err_avg = y_car;
    cnt_lat_err = 1;
    idx_start = idx_end = idx;
  }

  const double eps = 0.10;  // deadband 10cm
  TurnDirection dir = turn_dir_;  // 기본은 직전 방향 유지

  if (lat_err_avg > eps) {
    dir = TurnDirection::LEFT;
  } else if (lat_err_avg < -eps) {
    dir = TurnDirection::RIGHT;
  }

  RCLCPP_INFO(get_logger(),
    "[EscapePlanner] lat_err_avg=%.3f (best_idx=%d, range=[%d..%d], cnt=%d) → turn %s",
    lat_err_avg, best_idx, idx_start, idx_end, cnt_lat_err,
    (dir == TurnDirection::LEFT ? "LEFT" : "RIGHT"));

  return dir;
}

// /safety/trigger 콜백
void EscapePlanner::emergency_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
  // level 정보는 HARD STOP용으로 항상 저장
  safety_trigger_now_ = msg->data;

  // rising edge (false → true) 에서만 시퀀스 시작
  if (msg->data && !emergency_active_) {
    // 현재 pose + 경로 기반으로 회전 방향 결정
    turn_dir_ = choose_turn_direction();

    RCLCPP_WARN(get_logger(),
      "[EscapePlanner] Emergency TRIGGER rising edge. Start escape sequence. turn_dir=%s",
      (turn_dir_ == TurnDirection::LEFT ? "LEFT" : "RIGHT"));

    emergency_active_ = true;
    step_ = 0;
    tick_count_ = 0;
  }
}

// 타이머 콜백: 탈출 시퀀스 실행
void EscapePlanner::timer_cb()
{
  if (!emergency_active_) {
    return;
  }

  ackermann_msgs::msg::AckermannDriveStamped cmd;
  cmd.header.stamp = now();
  cmd.header.frame_id = "base_link";

  if (step_ == 0) {
    // STEP 0: STOP 0.2s
    cmd.drive.speed = 0.0;
    cmd.drive.steering_angle = 0.0;

    if (++tick_count_ > 4) { // 20Hz → 0.2초
      RCLCPP_INFO(get_logger(), "[EscapePlanner] STEP 0 done. Go to STEP 1 (BACKWARD).");
      step_ = 1;
      tick_count_ = 0;
    }
  }
  else if (step_ == 1) {
    // STEP 1: BACKWARD 0.5s (직진 후진)
    cmd.drive.speed = -0.5;
    cmd.drive.steering_angle = 0.0;

    if (++tick_count_ > 10) { // 0.5초
      RCLCPP_INFO(get_logger(), "[EscapePlanner] STEP 1 done. Go to STEP 2 (TURN).");
      step_ = 2;
      tick_count_ = 0;
    }
  }
  else if (step_ == 2) {
    // STEP 2: 선택된 방향으로 1.0s 회전 (전진)
    cmd.drive.speed = 0.5;

    double steer_angle = 0.4;         // LEFT 기본
    if (turn_dir_ == TurnDirection::RIGHT) {
      steer_angle = -0.4;            // RIGHT
    }
    cmd.drive.steering_angle = steer_angle;

    if (++tick_count_ > 20) { // 1초
      RCLCPP_INFO(get_logger(), "[EscapePlanner] STEP 2 done. Escape finished.");
      step_ = 3;
      tick_count_ = 0;
    }
  }
  else if (step_ == 3) {
    // STEP 3: FINISH
    std_msgs::msg::Bool done;
    done.data = true;
    finish_pub_->publish(done);

    RCLCPP_INFO(get_logger(), "[EscapePlanner] Publishing /escape/finished = true, reset state.");
    emergency_active_ = false;
    return;
  }

  // -----------------------------
  // HARD STOP 레이어 (safety_monitor 연동)
  // -----------------------------
  // safety_monitor 기준으로 위험(safety_trigger_now_ == true)일 때
  // 전진 명령은 강제로 STOP
  if (safety_trigger_now_ && cmd.drive.speed > 0.0) {
    cmd.drive.speed = 0.0;
    cmd.drive.steering_angle = 0.0;

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 500,
      "[EscapePlanner] HARD STOP by /safety/trigger (level=true) while moving forward.");
  }

  esc_pub_->publish(cmd);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EscapePlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

