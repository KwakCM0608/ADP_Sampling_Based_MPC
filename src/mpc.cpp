#include "mpc.hpp"
#include <limits>
#include <cmath>
#include <algorithm>

MPC_Controller::MPC_Controller(
    double wheelbase,
    double dt,
    int horizon_steps,
    int num_candidates)
  : wheelbase_(wheelbase),
    dt_(dt),
    horizon_steps_(horizon_steps),
    num_candidates_(num_candidates)
{
}

// "전방"에 있는 최근접 waypoint 인덱스
int MPC_Controller::nearest_wp_idx_(
    const std::array<double, 3> &pose,
    const std::vector<std::array<double, 8>> &wps) const
{
  const int N = static_cast<int>(wps.size());
  if (N == 0) return 0;

  const double px  = pose[0];
  const double py  = pose[1];
  const double yaw = pose[2];

  int best_ahead   = -1;
  double best_d2_a = std::numeric_limits<double>::max();

  int best_all     = 0;
  double best_d2   = std::numeric_limits<double>::max();

  for (int i = 0; i < N; ++i) {
    const double wx = wps[i][0];
    const double wy = wps[i][1];

    const double dx = wx - px;
    const double dy = wy - py;

    const double cos_y = std::cos(-yaw);
    const double sin_y = std::sin(-yaw);
    const double x_body = cos_y * dx - sin_y * dy;
    const double y_body = sin_y * dx + cos_y * dy;

    const double d2 = dx * dx + dy * dy;

    // 전체 중에서 제일 가까운 점 (fallback 용)
    if (d2 < best_d2) {
      best_d2 = d2;
      best_all = i;
    }

    // "전방"에 있는 점만 따로 관리 (x_body > 0)
    if (x_body > 0.0 && d2 < best_d2_a) {
      best_d2_a = d2;
      best_ahead = i;
    }
  }

  if (best_ahead >= 0) {
    return best_ahead;
  } else {
    return best_all;
  }
}

// 단일 waypoint를 향한 pure-pursuit
std::pair<double,double> MPC_Controller::pure_pursuit_to_point_(
    const std::array<double, 3> &pose,
    const std::array<double, 8> &wp,
    double speed_now) const
{
  const double px = pose[0];
  const double py = pose[1];
  const double yaw = pose[2];

  const double wx = wp[0];
  const double wy = wp[1];
  const double v_ref = wp[2];  // waypoint 목표 속도라고 가정

  // world → 차량 좌표계
  const double dx = wx - px;
  const double dy = wy - py;

  const double cos_y = std::cos(-yaw);
  const double sin_y = std::sin(-yaw);
  const double x_body = cos_y * dx - sin_y * dy;
  const double y_body = sin_y * dx + cos_y * dy;

  const double Ld = std::sqrt(x_body * x_body + y_body * y_body);
  if (Ld < 1e-3) {
    return {speed_now, 0.0};
  }

  const double alpha = std::atan2(y_body, x_body);

  // 기본 pure-pursuit 조향
  double steer = std::atan2(2.0 * wheelbase_ * std::sin(alpha), Ld);

  // 기본 조향 gain
  steer *= steer_gain_base_;

  // 속도는 waypoint v_ref + clamp
  double target_speed = v_ref;
  if (!std::isfinite(target_speed)) target_speed = speed_now;
  if (target_speed > max_speed_mps_) target_speed = max_speed_mps_;
  if (target_speed < min_speed_mps_) target_speed = min_speed_mps_;

  // 조향 saturate
  if (steer > max_steer_rad_)  steer = max_steer_rad_;
  if (steer < -max_steer_rad_) steer = -max_steer_rad_;

  return {target_speed, steer};
}

std::array<double, 3> MPC_Controller::forward_simulate_pose_(
    const std::array<double, 3> &pose,
    double v,
    double steer) const
{
  double x = pose[0];
  double y = pose[1];
  double yaw = pose[2];

  // 단순 자전거 모델
  x   += v * std::cos(yaw) * dt_;
  y   += v * std::sin(yaw) * dt_;
  yaw += v * std::tan(steer) / wheelbase_ * dt_;

  const double pi = 3.14159265358979323846;
  while (yaw >  pi) yaw -= 2.0 * pi;
  while (yaw < -pi) yaw += 2.0 * pi;

  return {x, y, yaw};
}

// start_idx에서부터 앞으로 누적 거리 Ld에 해당하는 waypoint index 찾기
int MPC_Controller::find_lookahead_index_by_distance_(
    int start_idx,
    const std::vector<std::array<double, 8>> &wps,
    double Ld) const
{
  const int N = static_cast<int>(wps.size());
  if (N == 0) return 0;
  if (start_idx < 0) start_idx = 0;
  if (start_idx >= N) start_idx = N - 1;

  double accum = 0.0;
  int idx = start_idx;
  for (int i = start_idx; i < N - 1; ++i) {
    double dx = wps[i+1][0] - wps[i][0];
    double dy = wps[i+1][1] - wps[i][1];
    double ds = std::sqrt(dx*dx + dy*dy);
    accum += ds;
    idx = i + 1;
    if (accum >= Ld) {
      break;
    }
  }
  return idx;
}

// 단일 인덱스 주변 세 점으로 곡률 계산
double MPC_Controller::curvature_at_index_(
    int idx,
    const std::vector<std::array<double, 8>> &wps) const
{
  const int N = static_cast<int>(wps.size());
  if (N < 3) return 0.0;
  if (idx <= 0) idx = 1;
  if (idx >= N - 1) idx = N - 2;

  const double x0 = wps[idx - 1][0];
  const double y0 = wps[idx - 1][1];
  const double x1 = wps[idx    ][0];
  const double y1 = wps[idx    ][1];
  const double x2 = wps[idx + 1][0];
  const double y2 = wps[idx + 1][1];

  const double a = std::hypot(x1 - x0, y1 - y0);
  const double b = std::hypot(x2 - x1, y2 - y1);
  const double c = std::hypot(x2 - x0, y2 - y0);

  const double denom = a * b * c + 1e-6;
  const double cross =
      (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0);

  const double kappa = 2.0 * std::fabs(cross) / denom;
  return kappa;  // 부호는 안 쓰고 크기만 사용
}

// center_idx 주변 window에서 최대 곡률
double MPC_Controller::max_curvature_in_window_(
    int center_idx,
    const std::vector<std::array<double, 8>> &wps,
    int half_window) const
{
  const int N = static_cast<int>(wps.size());
  if (N < 3) return 0.0;
  if (half_window <= 0) return curvature_at_index_(center_idx, wps);

  int start = std::max(1, center_idx - half_window);
  int end   = std::min(N - 2, center_idx + half_window);

  double kappa_max = 0.0;
  for (int i = start; i <= end; ++i) {
    double k = curvature_at_index_(i, wps);
    if (k > kappa_max) {
      kappa_max = k;
    }
  }
  return kappa_max;
}

std::pair<double, double> MPC_Controller::main_loop(
    const std::string &state,
    const std::optional<std::array<double, 3>> &position_in_map,
    const std::vector<std::array<double, 8>> &waypoint_array_in_map,
    double speed_now,
    const std::optional<std::array<double, 5>> &opponent,
    const std::optional<std::array<double, 4>> &position_in_map_frenet,
    const std::vector<double> &acc_now,
    double track_length)
{
  (void)state;
  (void)acc_now;

  if (!position_in_map.has_value() || waypoint_array_in_map.empty()) {
    return {0.0, 0.0};
  }

  const auto pose = position_in_map.value();
  const int N = static_cast<int>(waypoint_array_in_map.size());
  if (N == 0) {
    return {0.0, 0.0};
  }

  // ---------------------------------------------------------
  // 0) Frenet 상의 ego / opponent 정보 정리 (가능한 경우에만 사용)
  // ---------------------------------------------------------
  bool has_ego_sd = false;
  double ego_s = 0.0;
  double ego_d = 0.0;

  if (position_in_map_frenet.has_value()) {
    const auto &f = position_in_map_frenet.value();
    ego_s = f[0];
    ego_d = f[1];
    has_ego_sd = true;
  }

  bool have_opp = false;
  double opp_s = 0.0;
  double opp_d = 0.0;
  double opp_vs = 0.0;
  bool opp_is_static = false;
  bool opp_is_visible = false;

  if (opponent.has_value()) {
    const auto &o = opponent.value();
    opp_s        = o[0];
    opp_d        = o[1];
    opp_vs       = o[2];
    opp_is_static  = (o[3] > 0.5);
    opp_is_visible = (o[4] > 0.5);
    have_opp = true;
  }

  // opponent 를 실제로 "고려할지" 여부
  bool use_opp = has_ego_sd && have_opp && opp_is_visible && track_length > 1.0;

  // 1) 현재 위치에서 "전방" 기준 최근접 waypoint (raw)
  int nearest_raw = nearest_wp_idx_(pose, waypoint_array_in_map);
  int nearest_idx = nearest_raw;

  // 1-0) progress 기반 smoothing: 뒤로 너무 많이 튀는 건 제한
  if (last_nearest_idx_ >= 0) {
    // 예: 한 번에 최대 3 인덱스까지만 뒤로 가도록 허용
    if (nearest_idx < last_nearest_idx_ - 3) {
      nearest_idx = last_nearest_idx_ - 3;
    }
  }
  if (nearest_idx < 0)       nearest_idx = 0;
  if (nearest_idx >= N)      nearest_idx = N - 1;
  last_nearest_idx_ = nearest_idx;

  // 1-1) 주변 window에서 최대 곡률 계산 (S-코너 대비용)
  const double kappa_max =
      max_curvature_in_window_(nearest_idx,
                               waypoint_array_in_map,
                               curvature_half_window_);

  // 2) 로컬 적으로 waypoint 간 평균 간격 대략 추정 (horizon_steps_ 활용)
  double avg_ds = 0.2;  // fallback
  int ds_count = 0;
  const int max_ds_samples = 10;
  for (int i = nearest_idx; i < std::min(N - 1, nearest_idx + max_ds_samples); ++i) {
    double dx = waypoint_array_in_map[i+1][0] - waypoint_array_in_map[i][0];
    double dy = waypoint_array_in_map[i+1][1] - waypoint_array_in_map[i][1];
    double ds = std::sqrt(dx*dx + dy*dy);
    if (ds > 1e-4) {
      avg_ds += ds;
      ds_count++;
    }
  }
  if (ds_count > 0) {
    avg_ds /= (ds_count + 1);
  }

  // horizon_steps_를 "평균 간격 * step 수" ≈ 기본 Ld 로 사용
  double Ld_base = avg_ds * std::max(1, horizon_steps_);

  // 곡률이 클수록 Ld_base를 줄임 (첫 코너 이후 바로 다음 코너 대비)
  double curv_factor = 1.0 / (1.0 + k_Ld_curvature_ * kappa_max);
  Ld_base *= curv_factor;

  Ld_base = std::clamp(Ld_base, min_Ld_base_, max_Ld_base_);

  // 속도에 따라 Ld를 조금 키우기 (하지만 위에서 이미 곡률로 줄여놓음)
  double v_eff = std::clamp(speed_now, 0.0, max_speed_mps_);
  Ld_base += k_Ld_speed_ * v_eff;
  Ld_base = std::clamp(Ld_base, min_Ld_base_, max_Ld_base_);

  // 3) 여러 개의 lookahead 거리 후보 생성
  const int max_candidates = std::max(1, num_candidates_);
  std::vector<double> Ld_candidates;
  Ld_candidates.reserve(max_candidates);
  for (int i = 0; i < max_candidates; ++i) {
    // 예: 0.8*Ld_base, 1.1*Ld_base, 1.4*Ld_base ...
    double scale = 0.8 + 0.3 * i;
    double Ld = Ld_base * scale;
    // 너무 멀어지지 않도록 클램프
    Ld = std::clamp(Ld, min_Ld_base_, max_Ld_base_ * 1.2);
    Ld_candidates.push_back(Ld);
  }

  double best_cost  = std::numeric_limits<double>::max();
  double best_speed = 0.0;
  double best_steer = 0.0;

  // 4) 각 lookahead 거리별로 pseudo-MPC + cost 계산
  for (double Ld : Ld_candidates) {
    // (1) 해당 거리만큼 앞에 있는 waypoint index 찾기
    int idx_target = find_lookahead_index_by_distance_(
        nearest_idx, waypoint_array_in_map, Ld);
    const auto &wp_target = waypoint_array_in_map[idx_target];

    // (1-1) 이 후보 구간의 로컬 곡률
    double kappa_local = curvature_at_index_(idx_target, waypoint_array_in_map);

    // (2) 현재 pose에서 그 지점을 향한 (v, steer) 계산
    auto [v_candidate_raw, steer_candidate_raw] =
        pure_pursuit_to_point_(pose, wp_target, speed_now);

    double v_candidate     = v_candidate_raw;
    double steer_candidate = steer_candidate_raw;

    // (2-1) 곡률 기반 속도 제한 (v^2 * kappa <= ay_target_curve_)
    if (kappa_local > 1e-4) {
      double v_curve_limit = std::sqrt(ay_target_curve_ / kappa_local);
      v_curve_limit = std::clamp(v_curve_limit, min_speed_mps_, max_speed_mps_);
      if (v_candidate > v_curve_limit) {
        v_candidate = v_curve_limit;
      }
    }

    // (2-2) 곡률이 클수록 조향 gain을 조금 더 키움
    double steer_gain_local = 1.0 + k_steer_curv_gain_ * kappa_local;
    // 너무 과격해지지 않게 상한 (약간 상향)
    steer_gain_local = std::clamp(steer_gain_local, 1.0, 1.8);
    steer_candidate *= steer_gain_local;
    steer_candidate = std::clamp(steer_candidate, -max_steer_rad_, max_steer_rad_);

    // (3) sim_steps_ 만큼 자전거 모델로 pose 예측 (멀리까지 한 번 더 본다)
    auto pose_pred = pose;
    for (int k = 0; k < sim_steps_; ++k) {
      pose_pred = forward_simulate_pose_(pose_pred, v_candidate, steer_candidate);
    }

    // (4) 예측된 pose 기준으로 "경로를 얼마나 잘 따라가고 있는지" 평가
    int idx_cost = nearest_wp_idx_(pose_pred, waypoint_array_in_map);
    const auto &wp_cost = waypoint_array_in_map[idx_cost];

    const double px2 = pose_pred[0];
    const double py2 = pose_pred[1];
    const double yaw2 = pose_pred[2];

    const double wx2 = wp_cost[0];
    const double wy2 = wp_cost[1];

    const double dx2 = wx2 - px2;
    const double dy2 = wy2 - py2;

    const double cos_y2 = std::cos(-yaw2);
    const double sin_y2 = std::sin(-yaw2);
    const double x_body2 = cos_y2 * dx2 - sin_y2 * dy2;
    const double y_body2 = sin_y2 * dx2 + cos_y2 * dy2;

    const double Ld2 = std::sqrt(x_body2 * x_body2 + y_body2 * y_body2);
    if (Ld2 < 1e-3) {
      continue;
    }

    const double alpha2 = std::atan2(y_body2, x_body2);
    const double lat_err = y_body2;

    double cost = 0.0;
    cost += w_lat_err_      * lat_err * lat_err;
    cost += w_yaw_err_      * alpha2  * alpha2;

    const double d_steer = steer_candidate - last_steer_;
    cost += w_steer_smooth_ * d_steer * d_steer;

    const double dv = v_candidate - speed_now;
    cost += w_speed_err_    * dv * dv;

    // 횡가속 페널티 (a_y ≈ v^2 / L * tan(delta))
    const double ay = v_candidate * v_candidate * std::tan(steer_candidate) / wheelbase_;
    const double ay_max = 6.0;  // m/s^2
    if (std::fabs(ay) > ay_max) {
      double over = std::fabs(ay) - ay_max;
      cost += w_lat_acc_ * over * over;
    }

    // -------------------------------------------------
    // 4-1) 상대 차량(opp)을 장애물처럼 고려하는 cost/제약
    //      (Frenet s,d 기반 front-car follow + 충돌 회피)
    // -------------------------------------------------
    if (use_opp) {
      // ego Frenet s 예상값 (아주 단순한 근사: 현재 s + v * dt * sim_steps_)
      double ego_s_pred = ego_s + v_candidate * dt_ * static_cast<double>(sim_steps_);
      if (track_length > 1.0) {
        // 트랙 길이 기준 wrap
        ego_s_pred = std::fmod(ego_s_pred, track_length);
        if (ego_s_pred < 0.0) ego_s_pred += track_length;
      }

      // opponent 와의 s 차이 (앞/뒤 구분, wrap 고려)
      double ds = opp_s - ego_s_pred;
      if (track_length > 1.0) {
        // [-L/2, L/2] 범위로 정규화
        if (ds >  0.5 * track_length) ds -= track_length;
        if (ds < -0.5 * track_length) ds += track_length;
      }

      double dd = std::fabs(opp_d - ego_d);

      // 같은 레인에서, 앞쪽에 있고, 너무 가까워지면 강한 페널티
      if (dd < opp_same_lane_d_thresh_ && ds > 0.0) {
        // gap 이 아주 작아지면 cost 폭발 → MPC가 그런 후보를 버리게 됨
        if (ds < opp_hard_min_s_) {
          double gap = std::max(ds, 0.1); // 0에 수렴하면 매우 큰 cost
          cost += w_opp_gap_ * (1.0 / (gap * gap));
        } else if (ds < opp_soft_min_s_) {
          // 소프트 영역에서는 부드러운 벌점
          double ratio = (opp_soft_min_s_ - ds) / (opp_soft_min_s_ - opp_hard_min_s_);
          cost += w_opp_gap_ * ratio * ratio;
        }

        // 속도도 앞차에 맞춰서 강제로 줄이기 (follow 모드)
        double v_follow = opp_vs - opp_rel_speed_margin_;
        if (v_follow < min_speed_mps_) v_follow = min_speed_mps_;
        if (v_candidate > v_follow) {
          // 속도 제한 자체
          v_candidate = v_follow;

          // 제한으로 인한 dv 변화가 cost에 반영되도록 다시 계산
          double dv_new = v_candidate - speed_now;
          cost -= w_speed_err_ * dv * dv;      // 이전 속도 cost 제거
          cost += w_speed_err_ * dv_new * dv_new;
        }
      }
    }

    // 곡률이 큰데 Ld가 큰 후보는 추가 벌점 → 자연스럽게 짧은 Ld, 더 큰 조향 선택
    if (kappa_max > 1e-4) {
      const double Ld_norm = Ld / (max_Ld_base_ + 1e-6);
      cost += w_curv_Ld_ * kappa_max * Ld_norm * Ld_norm;
    }

    if (cost < best_cost) {
      best_cost   = cost;
      best_speed  = v_candidate;
      best_steer  = steer_candidate;
    }
  }

  // 후보가 전부 스킵된 경우 → fallback: 그냥 Ld_base로 PP + 곡률 기반 속도/조향 보정
  if (!std::isfinite(best_cost) || best_cost == std::numeric_limits<double>::max()) {
    int idx_fallback = find_lookahead_index_by_distance_(
        nearest_idx, waypoint_array_in_map, Ld_base);
    auto [v_pp_raw, steer_pp_raw] =
        pure_pursuit_to_point_(pose, waypoint_array_in_map[idx_fallback], speed_now);

    double kappa_fb = curvature_at_index_(idx_fallback, waypoint_array_in_map);

    double v_pp = v_pp_raw;
    if (kappa_fb > 1e-4) {
      double v_curve_limit = std::sqrt(ay_target_curve_ / kappa_fb);
      v_curve_limit = std::clamp(v_curve_limit, min_speed_mps_, max_speed_mps_);
      if (v_pp > v_curve_limit) v_pp = v_curve_limit;
    }

    double steer_pp = steer_pp_raw;
    double steer_gain_local = 1.0 + k_steer_curv_gain_ * kappa_fb;
    steer_gain_local = std::clamp(steer_gain_local, 1.0, 1.8);
    steer_pp *= steer_gain_local;
    steer_pp = std::clamp(steer_pp, -max_steer_rad_, max_steer_rad_);

    double max_delta = max_steer_rate_rad_per_s_ * dt_;
    double d = steer_pp - last_steer_;
    double steer_limited = steer_pp;
    if (d > max_delta)        steer_limited = last_steer_ + max_delta;
    else if (d < -max_delta)  steer_limited = last_steer_ - max_delta;

    last_steer_ = steer_limited;
    return {v_pp, steer_limited};
  }

  // 선택된 best_steer에 rate limit 적용
  double max_delta = max_steer_rate_rad_per_s_ * dt_;
  double d = best_steer - last_steer_;
  double steer_limited = best_steer;
  if (d > max_delta)        steer_limited = last_steer_ + max_delta;
  else if (d < -max_delta)  steer_limited = last_steer_ - max_delta;

  last_steer_ = steer_limited;
  return {best_speed, steer_limited};
}
