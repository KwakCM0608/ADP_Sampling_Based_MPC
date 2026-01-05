#pragma once

#include <array>
#include <vector>
#include <optional>
#include <utility>
#include <string>
#include <cmath>
#include <limits>

class MPC_Controller
{
public:
  MPC_Controller(
      double wheelbase,
      double dt,
      int horizon_steps,
      int num_candidates);

  /// state, pose, local waypoints 등을 받아서 (speed, steer) 반환
  std::pair<double, double> main_loop(
      const std::string &state,
      const std::optional<std::array<double, 3>> &position_in_map,
      const std::vector<std::array<double, 8>> &waypoint_array_in_map,
      double speed_now,
      const std::optional<std::array<double, 5>> &opponent,
      const std::optional<std::array<double, 4>> &position_in_map_frenet,
      const std::vector<double> &acc_now,
      double track_length);

private:
  double wheelbase_;
  double dt_;
  int horizon_steps_;
  int num_candidates_;

  // 조향 연속성 보존용
  double last_steer_{0.0};

  // nearest waypoint 인덱스 안정화용 (진행 방향이 갑자기 뒤로 튀지 않도록)
  int last_nearest_idx_{-1};

  // 한 번에 너무 크게 꺾지 않도록 제한 (조금 완화)
  const double max_steer_rad_   = 0.75;  // 약 43 deg (조금 더 여유)
  const double max_speed_mps_   = 5.0;   // 필요시 조절
  const double min_speed_mps_   = 0.5;   // 너무 느리게 가지 않도록

  // 조향 속도 제한 (rate limit)
  const double max_steer_rate_rad_per_s_ = 3.0;  // 조향을 조금 더 빠르게 추종

  // pure-pursuit 기본 조향 gain
  const double steer_gain_base_ = 1.0;   // 기본은 1.0

  // 곡률에 따른 추가 조향 gain (steer *= (1 + k * kappa_local))
  const double k_steer_curv_gain_ = 1.0; // 0.9 → 1.0 (곡률 클수록 더 많이 꺾도록)

  // 곡률 기반 속도 제한에 사용할 목표 횡가속 (m/s^2)
  const double ay_target_curve_  = 2.7;  // 3.0 → 2.7 (코너 속도 더 보수적으로)

  // cost weight (센터라인 집착도 ↑, 스무딩 약간 ↓)
  const double w_lat_err_       = 8.0;   // 경로에서 벗어나면 더 강하게 페널티
  const double w_yaw_err_       = 4.0;   // 헤딩 정렬
  const double w_steer_smooth_  = 0.18;  // 조향 변화 허용 조금 증가
  const double w_speed_err_     = 0.3;
  const double w_lat_acc_       = 0.08;  // 횡가속 페널티

  // 곡률과 lookahead 연관 weight
  const double k_Ld_curvature_  = 3.0;   // 곡률이 클수록 Ld를 줄이는 계수
  const double w_curv_Ld_       = 0.8;   // 큰 Ld + 큰 곡률에 더 강한 벌점

  // lookahead 거리 관련
  const double min_Ld_base_     = 0.6;   // 최소 lookahead 거리 (m)
  const double max_Ld_base_     = 3.0;   // 최대 lookahead 거리 (m)
  const double k_Ld_speed_      = 0.2;   // 속도에 따른 Ld 증가 계수

  // pseudo-MPC에서 몇 스텝 앞까지 예측해서 cost를 볼지
  const int sim_steps_          = 3;     // dt * 3 만큼 미래를 보고 판단

  // 곡률을 볼 window (양쪽 half-window)
  const int curvature_half_window_ = 15; // nearest 주변 ±15포인트 정도

  // ----- 상대 차량(opp)을 장애물로 취급하기 위한 파라미터 -----
  // Frenet 좌표계에서 ego / opponent 가 같은 레인이라고 볼 d 차이 한계
  const double opp_same_lane_d_thresh_ = 1.5;  // |d_ego - d_opp| < 1.5m → 같은 레인

  // 앞차와 유지하고 싶은 최소 / 소프트 거리 (Frenet s 축 기준)
  const double opp_hard_min_s_        = 3.0;   // 절대 접근 금지 영역 (m)
  const double opp_soft_min_s_        = 6.0;   // 이 안으로 들어가면 강하게 감속 유도 (m)

  // opponent gap에 대한 cost weight
  const double w_opp_gap_             = 12.0;  // gap이 짧을수록 큰 페널티

  // opponent 를 따라갈 때 ego 속도를 얼마나 보수적으로 제한할지
  const double opp_rel_speed_margin_  = 0.5;   // 앞차보다 약간 느리게 (m/s)

  // 현재 pose와 waypoint 배열에서, "전방"에 있는 최근접 인덱스
  int nearest_wp_idx_(
      const std::array<double, 3> &pose,
      const std::vector<std::array<double, 8>> &wps) const;

  // 단일 waypoint에 대한 pure-pursuit (속도/조향 계산)
  std::pair<double,double> pure_pursuit_to_point_(
      const std::array<double, 3> &pose,
      const std::array<double, 8> &wp,
      double speed_now) const;

  // 자전거 모델로 dt 동안 1-step 예측 (pseudo-MPC)
  std::array<double, 3> forward_simulate_pose_(
      const std::array<double, 3> &pose,
      double v,
      double steer) const;

  // start_idx부터 앞으로 경로를 따라 Ld (meter)만큼 간 위치에 해당하는 wp index 찾기
  int find_lookahead_index_by_distance_(
      int start_idx,
      const std::vector<std::array<double, 8>> &wps,
      double Ld) const;

  // 단일 index 주변 세 점으로 곡률 계산
  double curvature_at_index_(
      int idx,
      const std::vector<std::array<double, 8>> &wps) const;

  // center_idx 주변 window에서 최대 곡률 (S-코너 대비용)
  double max_curvature_in_window_(
      int center_idx,
      const std::vector<std::array<double, 8>> &wps,
      int half_window) const;
};
