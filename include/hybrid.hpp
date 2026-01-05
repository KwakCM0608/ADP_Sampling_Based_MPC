#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "pp.hpp"

namespace controller_cpp {

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
);

}  // namespace controller_cpp

