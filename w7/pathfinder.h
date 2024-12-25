#pragma once
#include <flecs.h>
#include <vector>
#include "math.h"
#include "ecsTypes.h"

struct PortalConnection
{
  size_t connIdx;
  float score;
  std::vector<IVec2> path;
};

struct PathPortal
{
  size_t startX, startY;
  size_t endX, endY;
  std::vector<PortalConnection> conns;
};

struct DungeonPortals
{
  size_t tileSplit;
  std::vector<PathPortal> portals;
  std::vector<std::vector<size_t>> tilePortalsIndices;
};

void prebuild_map(flecs::world &ecs);
std::vector<IVec2> find_path(const DungeonData& dd, const DungeonPortals& dp, IVec2 from, IVec2 to);
