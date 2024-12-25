#pragma once

#include <flecs.h>

constexpr float tile_size = 512.f;

enum EntityTeam : int
{
    TEAM_PLAYER = 0,
    TEAM_ROGUES,
    TEAM_HIVE
};

void init_roguelike(flecs::world &ecs);
void init_dungeon(flecs::world &ecs, char *tiles, size_t w, size_t h);
void process_turn(flecs::world &ecs);
void print_stats(flecs::world &ecs);
