#pragma once

#include "stateMachine.h"
#include "behaviourTree.h"

// states
State *create_attack_enemy_state();
State *create_move_to_enemy_state();
State *create_flee_from_enemy_state();
State *create_patrol_state(float patrol_dist);
State *create_nop_state();

// transitions
StateTransition *create_enemy_available_transition(float dist);
StateTransition *create_enemy_reachable_transition();
StateTransition *create_hitpoints_less_than_transition(float thres);
StateTransition *create_negate_transition(StateTransition *in);
StateTransition *create_and_transition(StateTransition *lhs, StateTransition *rhs);

BehNode *sequence(const std::vector<BehNode*> &nodes);
BehNode *selector(const std::vector<BehNode*> &nodes);

BehNode *move_to_entity(flecs::entity entity, const char *bb_name);
BehNode *is_low_hp(float thres);
BehNode *find_enemy(flecs::entity entity, float dist, const char *bb_name);
BehNode *flee(flecs::entity entity, const char *bb_name);
BehNode *patrol(flecs::entity entity, float patrol_dist, const char *bb_name);
BehNode *node_not(BehNode* node);
BehNode *node_xor(BehNode* nodeFirst, BehNode* nodeSecond);
BehNode *node_repeat_n(BehNode* nodeFirst, int32_t numRepeats);
BehNode *find_powerup_or_heal(flecs::entity entity, float in_dist, const char* bb_name);
BehNode *find_waypoint(flecs::entity entity, flecs::entity start_waypoint, const char* bb_name);
BehNode *move_to_spawn();
BehNode *spawn_items();
