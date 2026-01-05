#include "hybrid.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace controller_cpp {

using Pose3  = std::array<double, 3>;
using WpRow  = std::array<double, 8>;

static int nearest_idx_(const Pose3& pose,
                        const std::vector<WpRow>& wps)
{
  int best = 0;
  double best_d2 = 1e18;
  for (int i = 0; i < (int)wps.size(); ++i) {
    double dx = wps[i][0] - pose[0];
    double dy = wps[i][1] - pose[1];
    double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) { best_d2 = d2; best = i; }
  }
  return best;
}

// ==========================================
// [최종 튜닝 파라미터: 속도 기반 적응형]
// ==========================================
// 속도가 낮을수록(코너) 게인을 키우고, 높을수록(직선) 줄입니다.
static constexpr double k_low_speed  = 1.8; // 저속(코너)에서 강력한 보정
static constexpr double k_high_speed = 0.8; // 고속(직선)에서 부드러운 보정
static constexpr double low_speed_thresh  = 2.0; // 2m/s 이하
static constexpr double high_speed_thresh = 6.0; // 6m/s 이상

PP_Controller::Output hybrid_main_loop(
  PP_Controller& pp,
  const std::string& state,
  const std::optional<std::array<double,3>>& position_in_map,
  const std::vector<std::array<double,8>>& waypoint_array_in_map,
  double speed_now,
  const std::optional<std::array<double,5>>& opponent,
  const std::optional<std::array<double,4>>& position_in_map_frenet,
  const std::vector<double>& acc_now,
  double track_length
)
{
  // [핵심] 1. 속도에 따라 PP의 Lookahead 거리부터 조절 (PP 내부 로직이 지원한다면 좋겠지만, 여기서는 출력만 보정)
  // *참고: PP 클래스 내부 파라미터를 여기서 직접 바꿀 수 있다면 speed_lookahead를 줄이는 게 베스트입니다.
  // 현재 구조상 PP 실행 결과를 받아오므로, PP 실행 후 보정에 집중합니다.

  auto out_pp = pp.main_loop(
    state, position_in_map, waypoint_array_in_map, speed_now,
    opponent, position_in_map_frenet, acc_now, track_length
  );
  auto [speed, t_clip, lat_err_pp, steer_pp, l1_point, l1_dist, idx_nearest] = out_pp;

  if (!position_in_map || waypoint_array_in_map.empty()) return out_pp;

  const Pose3& pose = *position_in_map;
  int idx = idx_nearest;
  if (idx < 0 || idx >= (int)waypoint_array_in_map.size()) {
    idx = nearest_idx_(pose, waypoint_array_in_map);
  }
  const auto& wp = waypoint_array_in_map[idx];

  // 2. Cross Track Error 계산
  double road_yaw = wp[6];
  double dx = pose[0] - wp[0];
  double dy = pose[1] - wp[1];
  double cross_track_error = -std::sin(road_yaw) * dx + std::cos(road_yaw) * dy;

  // 3. [핵심] 속도 기반 적응형 게인 (Adaptive Gain)
  // 속도가 느리면(코너) -> 오차를 강력하게 잡음 (k = 2.5)
  // 속도가 빠르면(직선) -> 오차를 부드럽게 잡음 (k = 0.8)
 
  double k_adaptive = k_low_speed;
  if (speed_now >= high_speed_thresh) {
    k_adaptive = k_high_speed;
  } else if (speed_now > low_speed_thresh) {
    // 중간 속도 영역에서는 선형 보간 (Linear Interpolation)
    double ratio = (speed_now - low_speed_thresh) / (high_speed_thresh - low_speed_thresh);
    k_adaptive = k_low_speed - ratio * (k_low_speed - k_high_speed);
  }

  // 4. Stanley 보정
  // 속도가 0에 가까울 때 분모가 작아져서 게인이 폭발하는 것을 방지하기 위해 soft term 추가
  double v_eff = std::max(speed_now, 1.0);
  double soft_term = 1.0;

  double steer_correction = -std::atan2(k_adaptive * cross_track_error, v_eff + soft_term);
 
  // 보정량 제한 (안전장치)
  steer_correction = std::clamp(steer_correction, -0.4, 0.4);

  // 5. 최종 조향
  double steer_final = steer_pp + steer_correction;
  steer_final = std::clamp(steer_final, -0.45, 0.45);

  return PP_Controller::Output(
    speed, t_clip, cross_track_error, steer_final, l1_point, l1_dist, idx
  );
}

} // namespace controller_cpp
