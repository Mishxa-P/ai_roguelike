#pragma once
#include <vector>
#include <flecs.h>
#include <string>

namespace dmaps
{
  void gen_enemy_approach_map(flecs::world &ecs, std::vector<float> &map, int team);
  void gen_enemy_flee_map(flecs::world &ecs, std::vector<float> &map, int team);
  void gen_enemy_range_map(flecs::world& ecs, std::vector<float>& map, int team, float desiredDistance, float invalidDistance);
  void gen_hive_pack_map(flecs::world &ecs, std::vector<float> &map);
  inline std::string gen_name(const char* origin, int team) { return std::string(origin) + std::to_string(team); }
  void gen_explore_map(flecs::world& ecs, std::vector<float>& map);
};

