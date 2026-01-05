// src/controller_manager_cpp.cpp
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <type_traits>
#include <future>
#include <typeinfo>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>  // 추가: wrapToPi, sin, cos, atan2 등

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rclcpp/parameter_events_filter.hpp"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "f110_msgs/msg/wpnt_array.hpp"
#include "f110_msgs/msg/obstacle_array.hpp"
#include "f110_msgs/msg/pid_data.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <yaml-cpp/yaml.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "map.hpp"
#include "pp.hpp"
#include "ftg.hpp"
#include "hybrid.hpp"
#include "mpc.hpp"   // 새 샘플링 기반 MPC 컨트롤러

#include "rclcpp/executors/single_threaded_executor.hpp"

using namespace std::chrono_literals;

class Controller : public rclcpp::Node {
public:
  Controller()
  : rclcpp::Node(
        "controller_manager",
        rclcpp::NodeOptions()
          .allow_undeclared_parameters(true)
          .automatically_declare_parameters_from_overrides(true))
  {
    map_path_          = get_remote_parameter<std::string>("global_parameters", "map_path");
    racecar_version_   = get_remote_parameter<std::string>("global_parameters", "racecar_version");
    sim_               = get_remote_parameter<bool>("global_parameters", "sim");
    state_machine_rate_= get_remote_parameter<double>("state_machine", "rate_hz");

    LUT_name_ = this->get_parameter("LU_table").as_string();
    RCLCPP_INFO(get_logger(), "Using LUT: %s", LUT_name_.c_str());
    mode_     = this->get_parameter("mode").as_string();
    mapping_  = this->get_parameter("mapping").as_bool();

    if (this->has_parameter("ctrl_algo")) {
      ctrl_algo_ = this->get_parameter("ctrl_algo").as_string();
    } else {
      ctrl_algo_ = mode_;
    }

    // ctrl_algo 가 MPC면 mode를 MPC로 강제 전환
    if (ctrl_algo_ == "MPC") {
      RCLCPP_INFO(get_logger(),
                  "ctrl_algo == MPC → overriding mode to MPC");
      mode_ = "MPC";
    }

    RCLCPP_INFO(get_logger(),
                "mode=%s, ctrl_algo=%s",
                mode_.c_str(), ctrl_algo_.c_str());

    // ===========================
    //   track_error TXT 파일 설정
    // ===========================
    eval_start_time_ = this->get_clock()->now();
    std::string txt_name = "track_error_" + ctrl_algo_ + ".txt";
    eval_txt_.open(txt_name, std::ios::out | std::ios::trunc);
    if (!eval_txt_.is_open()) {
      RCLCPP_ERROR(
        get_logger(),
        "[EVAL] Failed to open TXT file: %s",
        txt_name.c_str()
      );
    } else {
      RCLCPP_INFO(
        get_logger(),
        "[EVAL] Logging track_error to TXT: %s",
        txt_name.c_str()
      );
      // 헤더 한 줄 (공백 구분)
      // x_m y_m track_error heading_error_rad speed_mps speed_cmd_mps steer_cmd_rad mode ctrl_algo
      eval_txt_ << "x_m y_m track_error heading_error_rad speed_mps speed_cmd_mps steer_cmd_rad mode ctrl_algo\n";
      eval_txt_.flush();
    }

    rate_hz_ = 40.0;
    state_ = "GB_TRACK";

    drive_pub_      = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    steering_pub_   = create_publisher<visualization_msgs::msg::Marker>("steering", 10);
    lookahead_pub_  = create_publisher<visualization_msgs::msg::Marker>("lookahead_point", 10);
    trailing_pub_   = create_publisher<visualization_msgs::msg::Marker>("trailing_opponent_marker", 10);
    waypoint_pub_   = create_publisher<visualization_msgs::msg::MarkerArray>("my_waypoints", 10);
    l1_dist_pub_    = create_publisher<geometry_msgs::msg::Point>("l1_distance", 10);
    gap_data_pub_   = create_publisher<f110_msgs::msg::PidData>("/trailing/gap_data", 10);

    if (mode_ == "MAP") {
      RCLCPP_INFO(get_logger(), "Initializing MAP controller");
      init_map_controller();
      prioritize_dyn_ = l1_params_["prioritize_dyn"].as<bool>();
    } else if (mode_ == "PP") {
      RCLCPP_INFO(get_logger(), "Initializing PP controller");
      init_pp_controller();
      prioritize_dyn_ = l1_params_["prioritize_dyn"].as<bool>();
    } else if (mode_ == "HYBRID") {
      RCLCPP_INFO(get_logger(), "Initializing HYBRID controller");
      init_pp_controller();
      prioritize_dyn_ = l1_params_["prioritize_dyn"].as<bool>();
    } else if (mode_ == "FTG") {
      RCLCPP_INFO(get_logger(), "Initializing FTG controller");
      init_ftg_controller();
      prioritize_dyn_ = false;
    } else if (mode_ == "MPC") {
      RCLCPP_INFO(get_logger(), "Initializing MPC controller");
      init_mpc_controller();
      prioritize_dyn_ = false;
    } else {
      RCLCPP_ERROR(get_logger(), "Invalid mode: %s", mode_.c_str());
      throw std::runtime_error("Invalid mode parameter");
    }

    state_sub_ = create_subscription<std_msgs::msg::String>(
        "/state", 10,
        [this](std_msgs::msg::String::SharedPtr msg){ state_ = msg->data; });

    glb_wpnts_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
        "/global_waypoints", 10,
        std::bind(&Controller::track_length_cb, this, std::placeholders::_1));

    local_wpnts_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
        "/local_waypoints", 10,
        std::bind(&Controller::local_waypoint_cb, this, std::placeholders::_1));

    obstacles_sub_ = create_subscription<f110_msgs::msg::ObstacleArray>(
        "/perception/obstacles", 10,
        std::bind(&Controller::obstacle_cb, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/car_state/odom", 10,
        [this](nav_msgs::msg::Odometry::SharedPtr msg){ speed_now_ = msg->twist.twist.linear.x; });

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/car_state/pose", 10,
        std::bind(&Controller::car_state_cb, this, std::placeholders::_1));

    frenet_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/car_state/frenet/odom", 10,
        std::bind(&Controller::car_state_frenet_cb, this, std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&Controller::scan_cb, this, std::placeholders::_1));

    // ===========================
    //   Emergency / Escape hooks
    // ===========================
    const bool use_escape = (mode_ == "HYBRID" || mode_ == "MPC");  // <<< CHANGED
    if (use_escape) {
      emergency_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/safety/trigger",
        10,
        [this](std_msgs::msg::Bool::SharedPtr msg) {
          // false → true 로 바뀌는 순간(상승 에지) 에서만 EMERGENCY_STOP 진입
          if (msg->data && !last_safety_trigger_) {
            RCLCPP_WARN(this->get_logger(),
                        "[Controller] /safety/trigger RISING EDGE → EMERGENCY_STOP");
            emergency_active_ = true;
            escape_mode_ = false;
            state_ = "EMERGENCY_STOP";
          }
          // 현재 상태 저장
          last_safety_trigger_ = msg->data;
        });

      escape_cmd_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "/escape/cmd",
        10,
        [this](ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
          escape_cmd_ = msg;
          escape_mode_ = true;
        });

      escape_finished_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/escape/finished",
        10,
        [this](std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data) {
            RCLCPP_INFO(this->get_logger(),
                        "[Controller] /escape/finished TRUE → exit emergency/escape, back to GB_TRACK");
            emergency_active_ = false;
            escape_mode_ = false;
            escape_cmd_.reset();
            state_ = "GB_TRACK";
          }
        });
    }

    wait_for_messages_();
    declare_update_params_and_watch_();

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_hz_),
        std::bind(&Controller::control_loop, this));

    RCLCPP_INFO(get_logger(), "Controller ready");
  }

private:
  template<typename T>
  T get_remote_parameter(const std::string &remote_node, const std::string &name) {
    using Service = rcl_interfaces::srv::GetParameters;

    const std::string service_name = "/" + remote_node + "/get_parameters";

    auto client = this->create_client<Service>(service_name);
    if (!client->wait_for_service(std::chrono::seconds(5))) {
      throw std::runtime_error("Timeout waiting for service: " + service_name);
    }

    auto req = std::make_shared<Service::Request>();
    req->names = {name};

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(this->get_node_base_interface());

    auto future = client->async_send_request(req);
    auto rc = exec.spin_until_future_complete(future, std::chrono::seconds(5));
    if (rc != rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error("Failed to get parameter '" + name + "' from " + remote_node);
    }

    auto res = future.get();
    if (res->values.empty()) {
      throw std::runtime_error("Empty response for parameter '" + name + "'");
    }

    const auto &p = res->values[0];
    using PT = rcl_interfaces::msg::ParameterType;

    if constexpr (std::is_same_v<T, std::string>) {
      if (p.type == PT::PARAMETER_STRING) return p.string_value;
    } else if constexpr (std::is_same_v<T, bool>) {
      if (p.type == PT::PARAMETER_BOOL)   return p.bool_value;
      if (p.type == PT::PARAMETER_STRING) return (p.string_value == "true");
    } else if constexpr (std::is_same_v<T, double>) {
      if (p.type == PT::PARAMETER_DOUBLE)  return p.double_value;
      if (p.type == PT::PARAMETER_INTEGER) return static_cast<double>(p.integer_value);
    } else if constexpr (std::is_integral_v<T>) {
      if (p.type == PT::PARAMETER_INTEGER) return static_cast<T>(p.integer_value);
      if (p.type == PT::PARAMETER_DOUBLE)  return static_cast<T>(p.double_value);
    }

    std::stringstream ss;
    ss << "Type mismatch getting '" << name << "' from " << remote_node
      << " (got=" << static_cast<int>(p.type) << ")";
    throw std::runtime_error(ss.str());
  }

  void init_map_controller() {
    load_l1_params_from_yaml_();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/sensors/imu/raw", 10, std::bind(&Controller::imu_cb, this, std::placeholders::_1));
    acc_now_.assign(10, 0.0);

    map_controller_ = std::make_unique<MAP_Controller>(
      l1_params_["t_clip_min"].as<double>(),
      l1_params_["t_clip_max"].as<double>(),
      l1_params_["m_l1"].as<double>(),
      l1_params_["q_l1"].as<double>(),
      l1_params_["speed_lookahead"].as<double>(),
      l1_params_["lat_err_coeff"].as<double>(),
      l1_params_["acc_scaler_for_steer"].as<double>(),
      l1_params_["dec_scaler_for_steer"].as<double>(),
      l1_params_["start_scale_speed"].as<double>(),
      l1_params_["end_scale_speed"].as<double>(),
      l1_params_["downscale_factor"].as<double>(),
      l1_params_["speed_lookahead_for_steer"].as<double>(),
      l1_params_["prioritize_dyn"].as<bool>(),
      l1_params_["trailing_gap"].as<double>(),
      l1_params_["trailing_p_gain"].as<double>(),
      l1_params_["trailing_i_gain"].as<double>(),
      l1_params_["trailing_d_gain"].as<double>(),
      l1_params_["blind_trailing_speed"].as<double>(),
      l1_params_["trailing_to_gbtrack_speed_scale"].as<double>(),
      rate_hz_,
      LUT_name_,
      [this](const std::string &s){ RCLCPP_INFO(this->get_logger(), "%s", s.c_str());},
      [this](const std::string &s){ RCLCPP_WARN(this->get_logger(), "%s", s.c_str());}
    );
  }

  void init_pp_controller() {
    load_l1_params_from_yaml_();

    double wheelbase = 0.33;
    const auto share_dir = ament_index_cpp::get_package_share_directory("stack_master");
    if (sim_) {
      const std::string cfg = share_dir + "/config/" + racecar_version_ + "/sim_params.yaml";
      YAML::Node y = YAML::LoadFile(cfg);
      wheelbase = y["lr"].as<double>() + y["lf"].as<double>();
    } else {
      const std::string cfg = share_dir + "/config/" + racecar_version_ + "/vesc.yaml";
      YAML::Node y = YAML::LoadFile(cfg);
      wheelbase = y["vesc_to_odom_node"]["ros__parameters"]["wheelbase"].as<double>();
    }

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/sensors/imu/raw", 10, std::bind(&Controller::imu_cb, this, std::placeholders::_1));
    acc_now_.assign(10, 0.0);

    pp_controller_ = std::make_unique<PP_Controller>(
      l1_params_["t_clip_min"].as<double>(),
      l1_params_["t_clip_max"].as<double>(),
      l1_params_["m_l1"].as<double>(),
      l1_params_["q_l1"].as<double>(),
      l1_params_["speed_lookahead"].as<double>(),
      l1_params_["lat_err_coeff"].as<double>(),
      l1_params_["acc_scaler_for_steer"].as<double>(),
      l1_params_["dec_scaler_for_steer"].as<double>(),
      l1_params_["start_scale_speed"].as<double>(),
      l1_params_["end_scale_speed"].as<double>(),
      l1_params_["downscale_factor"].as<double>(),
      l1_params_["speed_lookahead_for_steer"].as<double>(),
      l1_params_["prioritize_dyn"].as<bool>(),
      l1_params_["trailing_gap"].as<double>(),
      l1_params_["trailing_p_gain"].as<double>(),
      l1_params_["trailing_i_gain"].as<double>(),
      l1_params_["trailing_d_gain"].as<double>(),
      l1_params_["blind_trailing_speed"].as<double>(),
      l1_params_["trailing_to_gbtrack_speed_scale"].as<double>(),
      rate_hz_,
      wheelbase,
      [this](const std::string &s){ RCLCPP_INFO(this->get_logger(), "%s", s.c_str());},
      [this](const std::string &s){ RCLCPP_WARN(this->get_logger(), "%s", s.c_str());}
    );
  }

  void init_ftg_controller() {
    state_machine_debug_         = get_remote_parameter<bool>("state_machine", "debug");
    state_machine_safety_radius_ = get_remote_parameter<double>("state_machine", "safety_radius");
    state_machine_max_lidar_dist_= get_remote_parameter<double>("state_machine", "max_lidar_dist");
    state_machine_max_speed_     = get_remote_parameter<double>("state_machine", "max_speed");
    state_machine_range_offset_  = get_remote_parameter<double>("state_machine", "range_offset");
    state_machine_track_width_   = get_remote_parameter<double>("state_machine", "track_width");

    RCLCPP_INFO(get_logger(),
      "FTG params: debug=%d, safety=%.3f, maxLidar=%.3f, maxSpeed=%.3f, offset=%.3f, trackWidth=%.3f",
      state_machine_debug_, state_machine_safety_radius_, state_machine_max_lidar_dist_,
      state_machine_max_speed_, state_machine_range_offset_, state_machine_track_width_);

    ftg_controller_ = std::make_unique<FTG_Controller>(
      this,
      mapping_,
      state_machine_debug_,
      state_machine_safety_radius_,
      state_machine_max_lidar_dist_,
      state_machine_max_speed_,
      state_machine_range_offset_,
      state_machine_track_width_
    );
  }

  void init_mpc_controller() {
    load_l1_params_from_yaml_();

    double wheelbase = 0.33;
    const auto share_dir = ament_index_cpp::get_package_share_directory("stack_master");
    if (sim_) {
      const std::string cfg = share_dir + "/config/" + racecar_version_ + "/sim_params.yaml";
      YAML::Node y = YAML::LoadFile(cfg);
      wheelbase = y["lr"].as<double>() + y["lf"].as<double>();
    } else {
      const std::string cfg = share_dir + "/config/" + racecar_version_ + "/vesc.yaml";
      YAML::Node y = YAML::LoadFile(cfg);
      wheelbase = y["vesc_to_odom_node"]["ros__parameters"]["wheelbase"].as<double>();
    }

    const double dt_mpc       = 1.0 / rate_hz_;
    const int    horizon_step = 4;   // nearest + 4, 8, 12, ... 번째 wp
    const int    num_cands    = 7;   // 총 7개 lookahead 후보

    mpc_controller_ = std::make_unique<MPC_Controller>(
      wheelbase,
      dt_mpc,
      horizon_step,
      num_cands
    );
  }

  void load_l1_params_from_yaml_() {
    const std::string share_dir = ament_index_cpp::get_package_share_directory("stack_master");
    const std::string cfg = share_dir + "/config/" + racecar_version_ + "/l1_params.yaml";
    YAML::Node y = YAML::LoadFile(cfg);
    const auto node = y["controller"]["ros__parameters"];

    const std::vector<std::string> keys = {
      "t_clip_min","t_clip_max","m_l1","q_l1","speed_lookahead","lat_err_coeff",
      "acc_scaler_for_steer","dec_scaler_for_steer","start_scale_speed","end_scale_speed",
      "downscale_factor","speed_lookahead_for_steer","prioritize_dyn","trailing_gap",
      "trailing_p_gain","trailing_i_gain","trailing_d_gain","blind_trailing_speed", "trailing_to_gbtrack_speed_scale"
    };
    l1_params_.clear();
    for (const auto &k : keys) l1_params_[k] = node[k];
  }

  void wait_for_messages_() {
    RCLCPP_INFO(get_logger(), "Controller Manager waiting for messages...");
    bool track_length_ok = false, wpnts_ok = false, state_ok = false;

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(this->get_node_base_interface());

    rclcpp::Rate r(50.0);
    while (rclcpp::ok() && (!track_length_ok || !wpnts_ok || !state_ok)) {
      exec.spin_some();
      if (track_length_ && !track_length_ok) {
        RCLCPP_INFO(get_logger(), "Received track length");
        track_length_ok = true;
      }
      if (!waypoint_array_in_map_.empty() && !wpnts_ok) {
        RCLCPP_INFO(get_logger(), "Received waypoint array");
        wpnts_ok = true;
      }
      if (speed_now_.has_value() && position_in_map_.has_value() &&
          position_in_map_frenet_.has_value() && !state_ok) {
        RCLCPP_INFO(get_logger(), "Received car state messages");
        state_ok = true;
      }
      r.sleep();
    }
    RCLCPP_INFO(get_logger(), "All required messages received. Continuing...");
  }

  void declare_update_params_and_watch_() {
    using rcl_interfaces::msg::ParameterDescriptor;
    using rcl_interfaces::msg::FloatingPointRange;

    auto fp_range = [](double a, double b, double step){
      FloatingPointRange r; r.from_value=a; r.to_value=b; r.step=step; return r;
    };
    auto decl = [&](const std::string& name, const YAML::Node& v, const ParameterDescriptor& d){
      if (v.IsScalar()) {
        if (v.Tag() == "!" || v.as<std::string>().find_first_not_of("0123456789.-") != std::string::npos) {
          if (v.as<std::string>()=="true" || v.as<std::string>()=="false")
            this->declare_parameter(name, v.as<bool>(), d);
          else
            this->declare_parameter(name, v.as<std::string>(), d);
        } else {
          this->declare_parameter(name, v.as<double>(), d);
        }
      } else {
        this->declare_parameter(name, v.as<bool>(), d);
      }
    };

    decl("t_clip_min", l1_params_["t_clip_min"], param_desc(fp_range(0.0,1.5,0.01)));
    decl("t_clip_max", l1_params_["t_clip_max"], param_desc(fp_range(0.0,10.0,0.01)));
    decl("m_l1", l1_params_["m_l1"],         param_desc(fp_range(0.0,1.0,0.001)));
    decl("q_l1", l1_params_["q_l1"],         param_desc(fp_range(-1.0,1.0,0.001)));
    decl("speed_lookahead", l1_params_["speed_lookahead"], param_desc(fp_range(0.0,1.0,0.01)));
    decl("lat_err_coeff",   l1_params_["lat_err_coeff"],   param_desc(fp_range(0.0,1.0,0.01)));
    decl("acc_scaler_for_steer", l1_params_["acc_scaler_for_steer"], param_desc(fp_range(0.0,1.5,0.01)));
    decl("dec_scaler_for_steer", l1_params_["dec_scaler_for_steer"], param_desc(fp_range(0.0,1.5,0.01)));
    decl("start_scale_speed", l1_params_["start_scale_speed"], param_desc(fp_range(0.0,10.0,0.01)));
    decl("end_scale_speed",   l1_params_["end_scale_speed"],   param_desc(fp_range(0.0,10.0,0.01)));
    decl("downscale_factor",  l1_params_["downscale_factor"],  param_desc(fp_range(0.0,0.5,0.01)));
    decl("speed_lookahead_for_steer", l1_params_["speed_lookahead_for_steer"], param_desc(fp_range(0.0,0.2,0.01)));
    declare_parameter("prioritize_dyn", prioritize_dyn_);
    decl("trailing_gap", l1_params_["trailing_gap"], param_desc(fp_range(0.0,3.0,0.1)));
    decl("trailing_p_gain", l1_params_["trailing_p_gain"], param_desc(fp_range(0.0,3.0,0.01)));
    decl("trailing_i_gain", l1_params_["trailing_i_gain"], param_desc(fp_range(0.0,0.5,0.001)));
    decl("trailing_d_gain", l1_params_["trailing_d_gain"], param_desc(fp_range(0.0,1.0,0.01)));
    decl("blind_trailing_speed", l1_params_["blind_trailing_speed"], param_desc(fp_range(0.0,3.0,0.01)));
    decl("trailing_to_gbtrack_speed_scale", l1_params_["trailing_to_gbtrack_speed_scale"], param_desc(fp_range(0.0,1.0,0.01)));

    param_event_sub_ = create_subscription<rcl_interfaces::msg::ParameterEvent>(
      "/parameter_events", 10,
      std::bind(&Controller::on_parameter_event_, this, std::placeholders::_1));
  }

  rcl_interfaces::msg::ParameterDescriptor param_desc(
      const rcl_interfaces::msg::FloatingPointRange &rng) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    d.floating_point_range = {rng};
    return d;
  }

  void on_parameter_event_(const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
    if (event->node != this->get_fully_qualified_name() || mode_ == "FTG")
      return;

    if (mode_ == "MAP" && map_controller_) {
      map_controller_->t_clip_min               = get_parameter("t_clip_min").as_double();
      map_controller_->t_clip_max               = get_parameter("t_clip_max").as_double();
      map_controller_->m_l1                     = get_parameter("m_l1").as_double();
      map_controller_->q_l1                     = get_parameter("q_l1").as_double();
      map_controller_->speed_lookahead          = get_parameter("speed_lookahead").as_double();
      map_controller_->lat_err_coeff            = get_parameter("lat_err_coeff").as_double();
      map_controller_->acc_scaler_for_steer     = get_parameter("acc_scaler_for_steer").as_double();
      map_controller_->dec_scaler_for_steer     = get_parameter("dec_scaler_for_steer").as_double();
      map_controller_->start_scale_speed        = get_parameter("start_scale_speed").as_double();
      map_controller_->end_scale_speed          = get_parameter("end_scale_speed").as_double();
      map_controller_->downscale_factor         = get_parameter("downscale_factor").as_double();
      map_controller_->speed_lookahead_for_steer= get_parameter("speed_lookahead_for_steer").as_double();
      map_controller_->prioritize_dyn           = get_parameter("prioritize_dyn").as_bool();
      map_controller_->trailing_gap             = get_parameter("trailing_gap").as_double();
      map_controller_->trailing_p_gain          = get_parameter("trailing_p_gain").as_double();
      map_controller_->trailing_i_gain          = get_parameter("trailing_i_gain").as_double();
      map_controller_->trailing_d_gain          = get_parameter("trailing_d_gain").as_double();
      map_controller_->blind_trailing_speed     = get_parameter("blind_trailing_speed").as_double();
      map_controller_->trailing_to_gbtrack_speed_scale = get_parameter("trailing_to_gbtrack_speed_scale").as_double();
    } else if ((mode_ == "PP" || mode_ == "HYBRID") && pp_controller_) {
      pp_controller_->t_clip_min                = get_parameter("t_clip_min").as_double();
      pp_controller_->t_clip_max                = get_parameter("t_clip_max").as_double();
      pp_controller_->m_l1                      = get_parameter("m_l1").as_double();
      pp_controller_->q_l1                      = get_parameter("q_l1").as_double();
      pp_controller_->speed_lookahead           = get_parameter("speed_lookahead").as_double();
      pp_controller_->lat_err_coeff             = get_parameter("lat_err_coeff").as_double();
      pp_controller_->acc_scaler_for_steer      = get_parameter("acc_scaler_for_steer").as_double();
      pp_controller_->dec_scaler_for_steer      = get_parameter("dec_scaler_for_steer").as_double();
      pp_controller_->start_scale_speed         = get_parameter("start_scale_speed").as_double();
      pp_controller_->end_scale_speed           = get_parameter("end_scale_speed").as_double();
      pp_controller_->downscale_factor          = get_parameter("downscale_factor").as_double();
      pp_controller_->speed_lookahead_for_steer = get_parameter("speed_lookahead_for_steer").as_double();
      pp_controller_->prioritize_dyn            = get_parameter("prioritize_dyn").as_bool();
      pp_controller_->trailing_gap              = get_parameter("trailing_gap").as_double();
      pp_controller_->trailing_p_gain           = get_parameter("trailing_p_gain").as_double();
      pp_controller_->trailing_i_gain           = get_parameter("trailing_i_gain").as_double();
      pp_controller_->trailing_d_gain           = get_parameter("trailing_d_gain").as_double();
      pp_controller_->blind_trailing_speed      = get_parameter("blind_trailing_speed").as_double();
      pp_controller_->trailing_to_gbtrack_speed_scale = get_parameter("trailing_to_gbtrack_speed_scale").as_double();
    }
    RCLCPP_INFO(get_logger(), "Updated parameters");
  }

  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) { scan_ = msg; }

  void track_length_cb(const f110_msgs::msg::WpntArray::SharedPtr msg) {
    if (!msg->wpnts.empty()) {
      track_length_ = msg->wpnts.back().s_m;
      waypoints_.clear();
      waypoints_.reserve(msg->wpnts.size());
      for (const auto &w : msg->wpnts) {
        waypoints_.push_back({w.x_m, w.y_m, w.psi_rad});
      }
    }
  }

  void obstacle_cb(const f110_msgs::msg::ObstacleArray::SharedPtr msg) {
    if (!msg->obstacles.empty() && position_in_map_frenet_.has_value() && track_length_.has_value()) {
      bool static_flag = false;
      double closest_opp = track_length_.value();
      std::optional<std::array<double,5>> best;
      for (const auto &ob : msg->obstacles) {
        const double opponent_dist = std::fmod(ob.s_start - position_in_map_frenet_.value()[0] + track_length_.value(), track_length_.value());
        if (opponent_dist < closest_opp || (static_flag && !ob.is_static)) {
          closest_opp = opponent_dist;
          best = {ob.s_center, ob.d_center, ob.vs, static_cast<double>(ob.is_static), static_cast<double>(ob.is_visible)};
          static_flag = ob.is_static ? l1_params_["prioritize_dyn"].as<bool>() : false;
        }
      }
      opponent_ = best;
    } else {
      opponent_.reset();
    }
  }

  void car_state_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    const auto &p = msg->pose.position;
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    position_in_map_ = std::array<double,3>{p.x, p.y, yaw};
  }

  void local_waypoint_cb(const f110_msgs::msg::WpntArray::SharedPtr msg) {
    waypoint_array_in_map_.clear();
    waypoint_array_in_map_.reserve(msg->wpnts.size());
    for (const auto &w : msg->wpnts) {
      double share = 0.0;
      if (w.d_right + w.d_left != 0.0)
        share = std::min(w.d_left, w.d_right) / (w.d_right + w.d_left);

      waypoint_array_in_map_.push_back({
        w.x_m, w.y_m, w.vx_mps, share, w.s_m, w.kappa_radpm, w.psi_rad, w.ax_mps2
      });
    }
    waypoint_safety_counter_ = 0;
  }

  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) {
    for (size_t i = acc_now_.size() - 1; i > 0; --i) acc_now_[i] = acc_now_[i-1];
    acc_now_[0] = msg->linear_acceleration.x;
  }

  void car_state_frenet_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    position_in_map_frenet_ = std::array<double,4>{
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y
    };
  }

  int nearest_wp_idx_(
      const std::array<double,3>& pose,
      const std::vector<std::array<double,8>>& wps) const
  {
    int best = 0;
    double best_d2 = 1e18;
    for (int i = 0; i < static_cast<int>(wps.size()); ++i) {
      double dx = wps[i][0] - pose[0]; // wps[i][0] = x, wps[i][1] = y
      double dy = wps[i][1] - pose[1];
      double d2 = dx*dx + dy*dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        best = i;
      }
    }
    return best;
  }

  // [-pi, pi] 범위로 각도를 래핑
  double wrapToPi_(double a) const {
    return std::atan2(std::sin(a), std::cos(a));
  }

  // track_error와 heading_error를 함께 계산하는 함수
  double compute_track_error_and_heading_(
      const std::array<double,3>& pose,
      const std::vector<std::array<double,8>>& wps,
      double &heading_err_out) const
  {
    if (wps.empty()) {
      heading_err_out = 0.0;
      return 0.0;
    }

    const int idx = nearest_wp_idx_(pose, wps);
    const auto& wp = wps[idx];

    const double yaw_path = wp[6];   // wp[6] = psi_rad
    const double dx = pose[0] - wp[0];
    const double dy = pose[1] - wp[1];

    // 경로 좌표계에서 lateral 방향 성분 (m)
    const double lat_err = -std::sin(yaw_path) * dx + std::cos(yaw_path) * dy;

    // 헤딩 오차 (차량 헤딩 - 경로 헤딩), [-pi, pi]로 래핑
    const double yaw_car = pose[2];
    heading_err_out = wrapToPi_(yaw_car - yaw_path);

    return std::fabs(lat_err);
  }

  // 기존 인터페이스도 유지 (필요시 사용)
  double compute_track_error_(
      const std::array<double,3>& pose,
      const std::vector<std::array<double,8>>& wps) const
  {
    double dummy_heading = 0.0;
    return compute_track_error_and_heading_(pose, wps, dummy_heading);
  }

  void visualize_steering(double theta) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);

    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = "car_state/base_link";
    mk.header.stamp = now();
    mk.type = visualization_msgs::msg::Marker::CUBE;
    mk.id = 50;
    mk.scale.x = 0.6;
    mk.scale.y = 0.05;
    mk.scale.z = 0.01;
    mk.color.r = 1.0f; mk.color.g = 0.0f; mk.color.b = 0.0f; mk.color.a = 1.0f;
    mk.pose.orientation = tf2::toMsg(q);
    steering_pub_->publish(mk);
  }

  void set_waypoint_markers_(const std::vector<std::array<double,8>>& wps) {
    visualization_msgs::msg::MarkerArray arr;
    int id = 1;
    for (const auto &w : wps) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = now();
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.id = id++;
      m.scale.x = m.scale.y = m.scale.z = 0.1;
      m.color.b = 1.0f; m.color.a = 1.0f;
      m.pose.position.x = w[0];
      m.pose.position.y = w[1];
      m.pose.orientation.w = 1.0;
      arr.markers.push_back(m);
    }
    waypoint_pub_->publish(arr);
  }

  void set_lookahead_marker_(const std::array<double,2>& pt, int id) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = now();
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = id;
    m.scale.x = m.scale.y = m.scale.z = 0.35;
    m.color.g = 1.0f; m.color.a = 1.0f;
    m.pose.position.x = pt[0];
    m.pose.position.y = pt[1];
    m.pose.orientation.w = 1.0;
    lookahead_pub_->publish(m);
  }

  void control_loop() {
    // =======================
    // Emergency / Escape 우선
    // =======================
    const bool use_escape = (mode_ == "HYBRID" || mode_ == "MPC");  // <<< CHANGED
    if (use_escape) {
      if (emergency_active_ && !escape_mode_) {
        // 그냥 완전 정지
        ackermann_msgs::msg::AckermannDriveStamped out;
        out.header.stamp = now();
        out.header.frame_id = "base_link";
        out.drive.speed = 0.0;
        out.drive.steering_angle = 0.0;
        drive_pub_->publish(out);
        return;
      }

      if (escape_mode_ && escape_cmd_) {
        // EscapePlanner가 보내준 cmd 그대로 사용
        drive_pub_->publish(*escape_cmd_);
        return;
      }
    }

    double speed = 0.0, steer = 0.0;

    if (mode_ == "MAP" && map_controller_) {
      std::tie(speed, steer) = map_cycle_();
    } else if (mode_ == "PP" && pp_controller_) {
      std::tie(speed, steer) = pp_cycle_();
    } else if (mode_ == "HYBRID" && pp_controller_) {
      std::tie(speed, steer) = hybrid_cycle_();
    } else if (mode_ == "MPC" && mpc_controller_) {
      std::tie(speed, steer) = mpc_cycle_();
    } else if (mode_ == "FTG" && ftg_controller_) {
      std::tie(speed, steer) = ftg_cycle_();
    }

    // ===========================
    //   성능 평가용 TXT 로깅
    // ===========================
    if ((mode_ == "PP" || mode_ == "HYBRID" || mode_ == "MPC") &&
        position_in_map_.has_value() &&
        !waypoint_array_in_map_.empty())
    {
      const auto& pose = position_in_map_.value();
      const double x = pose[0];
      const double y = pose[1];

      double heading_err = 0.0;
      const double track_err = compute_track_error_and_heading_(pose, waypoint_array_in_map_, heading_err);

      const double speed_meas = speed_now_.value_or(0.0);

      // ==== track_error 및 추가 성능 지표를 TXT 파일에 저장 ====
      if (eval_txt_.is_open()) {
        // 공백으로 구분된 한 줄:
        // x_m y_m track_error heading_error_rad speed_mps speed_cmd_mps steer_cmd_rad mode ctrl_algo
        eval_txt_ << std::fixed << std::setprecision(6)
                  << x << " "
                  << y << " "
                  << track_err << " "
                  << heading_err << " "
                  << speed_meas << " "
                  << speed << " "
                  << steer << " "
                  << mode_ << " "
                  << ctrl_algo_
                  << "\n";
        eval_txt_.flush();
      }

      // 너무 자주 찍히지 않게 0.2초(200ms) 단위로 throttle
      if (eval_log_count_ < 5000) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 200,
          "[EVAL][%s] track_error=%.3f, heading_error=%.3f, v=%.2f, cmd_v=%.2f, cmd_steer=%.3f (x=%.2f, y=%.2f)",
          ctrl_algo_.c_str(),
          track_err,
          heading_err,
          speed_meas,
          speed,
          steer,
          x, y
        );
        eval_log_count_++;

        if (eval_log_count_ == 5000) {
            RCLCPP_INFO(
              get_logger(),
              "[EVAL] reached 500 logs → stop printing further EVAL logs"
            );
        }
      }
    }

    ackermann_msgs::msg::AckermannDriveStamped ack;
    ack.header.stamp = this->get_clock()->now();
    ack.header.frame_id = "base_link";
    ack.drive.steering_angle = steer;
    ack.drive.speed = speed;
    drive_pub_->publish(ack);
  }

  std::pair<double,double> map_cycle_() {
    auto out = map_controller_->main_loop(
      state_,
      position_in_map_,
      waypoint_array_in_map_,
      speed_now_.value_or(0.0),
      opponent_,
      position_in_map_frenet_,
      acc_now_,
      track_length_.value_or(0.0)
    );
    double speed = std::get<0>(out);
    double steer = std::get<3>(out);

    waypoint_safety_counter_++;
    if (waypoint_safety_counter_ >= static_cast<int>(rate_hz_ / state_machine_rate_ * 10.0)) {
      RCLCPP_WARN(get_logger(), "[controller_manager] Received no local wpnts. STOPPING!!");
      speed = 0.0; steer = 0.0;
    }
    return {speed, steer};
  }

  std::pair<double,double> pp_cycle_() {
    auto out = pp_controller_->main_loop(
      state_,
      position_in_map_,
      waypoint_array_in_map_,
      speed_now_.value_or(0.0),
      opponent_,
      position_in_map_frenet_,
      acc_now_,
      track_length_.value_or(0.0)
    );
    double speed = std::get<0>(out);
    double steer = std::get<3>(out);

    waypoint_safety_counter_++;
    if (waypoint_safety_counter_ >= static_cast<int>(rate_hz_ / state_machine_rate_ * 10.0)) {
      RCLCPP_WARN(get_logger(), "[controller_manager] Received no local wpnts. STOPPING!!");
      speed = 0.0; steer = 0.0;
    }
    return {speed, steer};
  }

  std::pair<double,double> hybrid_cycle_() {
    auto out = controller_cpp::hybrid_main_loop(
      *pp_controller_,
      state_,
      position_in_map_,
      waypoint_array_in_map_,
      speed_now_.value_or(0.0),
      opponent_,
      position_in_map_frenet_,
      acc_now_,
      track_length_.value_or(0.0)
    );

    double speed = std::get<0>(out);
    double steer = std::get<3>(out);

    waypoint_safety_counter_++;
    if (waypoint_safety_counter_ >= static_cast<int>(rate_hz_ / state_machine_rate_ * 10.0)) {
      RCLCPP_WARN(get_logger(), "[controller_manager] Received no local wpnts. STOPPING!!");
      speed = 0.0; steer = 0.0;
    }
    return {speed, steer};
  }

  std::pair<double,double> mpc_cycle_() {
    if (!mpc_controller_) {
      return {0.0, 0.0};
    }

    auto out = mpc_controller_->main_loop(
      state_,
      position_in_map_,
      waypoint_array_in_map_,
      speed_now_.value_or(0.0),
      opponent_,
      position_in_map_frenet_,
      acc_now_,
      track_length_.value_or(0.0)
    );

    double speed = out.first;
    double steer = out.second;

    waypoint_safety_counter_++;
    if (waypoint_safety_counter_ >= static_cast<int>(rate_hz_ / state_machine_rate_ * 10.0)) {
      RCLCPP_WARN(get_logger(), "[controller_manager] Received no local wpnts. STOPPING!!");
      speed = 0.0; steer = 0.0;
    }
    return {speed, steer};
  }

  std::pair<double,double> ftg_cycle_() {
    if (!scan_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "FTGONLY!!! (no scan yet)");
      return {0.0, 0.0};
    }
    const auto &ranges = scan_->ranges;
    auto out = ftg_controller_->process_lidar(ranges);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[STATE MACHINE] FTGONLY!!!");
    return out;
  }

private:
  std::string map_path_;
  std::string racecar_version_;
  bool sim_{false};
  double state_machine_rate_{10.0};

  std::string LUT_name_;
  std::string mode_;
  bool mapping_{false};
  double rate_hz_{30.0};
  std::string state_;

  std::string ctrl_algo_{"PP"};
  std::size_t eval_log_count_{0};

  std::optional<double> track_length_;
  std::vector<std::array<double,3>> waypoints_;
  std::vector<std::array<double,8>> waypoint_array_in_map_;
  int waypoint_safety_counter_{0};

  std::optional<double> speed_now_;
  std::optional<std::array<double,3>> position_in_map_;
  std::optional<std::array<double,4>> position_in_map_frenet_;
  std::optional<std::array<double,5>> opponent_;

  std::vector<double> acc_now_;
  bool prioritize_dyn_{false};

  bool   state_machine_debug_{false};
  double state_machine_safety_radius_{0.0};
  double state_machine_max_lidar_dist_{0.0};
  double state_machine_max_speed_{0.0};
  double state_machine_range_offset_{0.0};
  double state_machine_track_width_{0.0};

  std::unordered_map<std::string, YAML::Node> l1_params_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr steering_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trailing_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr l1_dist_pub_;
  rclcpp::Publisher<f110_msgs::msg::PidData>::SharedPtr gap_data_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr glb_wpnts_sub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr local_wpnts_sub_;
  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obstacles_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr frenet_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_event_sub_;

  // Emergency / Escape 관련
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr escape_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr escape_finished_sub_;
  ackermann_msgs::msg::AckermannDriveStamped::SharedPtr escape_cmd_;
  bool emergency_active_ = false;
  bool escape_mode_ = false;
  bool last_safety_trigger_ = false;

  sensor_msgs::msg::LaserScan::SharedPtr scan_;

  std::unique_ptr<MAP_Controller> map_controller_;
  std::unique_ptr<PP_Controller>  pp_controller_;
  std::unique_ptr<FTG_Controller> ftg_controller_;
  std::unique_ptr<MPC_Controller> mpc_controller_;

  // track_error TXT 로깅용
  std::ofstream eval_txt_;
  rclcpp::Time eval_start_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}

