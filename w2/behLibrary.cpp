#include "aiLibrary.h"
#include "ecsTypes.h"
#include "aiUtils.h"
#include "math.h"
#include "raylib.h"
#include "blackboard.h"

struct CompoundNode : public BehNode
{
  std::vector<BehNode*> nodes;

  virtual ~CompoundNode()
  {
    for (BehNode *node : nodes)
      delete node;
    nodes.clear();
  }

  CompoundNode &pushNode(BehNode *node)
  {
    nodes.push_back(node);
    return *this;
  }
};

struct Not : public BehNode
{
    BehNode* node;
    Not(BehNode* node) : node(node) {}

    virtual ~Not()
    {
        delete node;
    }

    BehResult update(flecs::world& ecs, flecs::entity entity, Blackboard& bb) override
    {
        BehResult res = node->update(ecs, entity, bb);
        if (res == BEH_RUNNING)
        {
            return res;
        }
        return res == BEH_SUCCESS ? BEH_FAIL : BEH_SUCCESS;
    }
};

struct Xor : public BehNode
{
    BehNode* nodeFirst;
    BehNode* nodeSecond;
    Xor(BehNode* nodeFirst, BehNode* nodeSecond) : nodeFirst(nodeFirst), nodeSecond(nodeSecond) {}

    virtual ~Xor()
    {
        delete nodeFirst;
        delete nodeSecond;
    }

    BehResult update(flecs::world& ecs, flecs::entity entity, Blackboard& bb) override
    {
        BehResult firstNodeRes = nodeFirst->update(ecs, entity, bb);
        if (firstNodeRes == BEH_RUNNING)
        {
            return firstNodeRes;
        }
        BehResult secondNodeRes = nodeSecond->update(ecs, entity, bb);
        if (secondNodeRes == BEH_RUNNING)
        {
            return secondNodeRes;
        }
        return firstNodeRes == secondNodeRes ? BEH_FAIL : BEH_SUCCESS;
    }
};

struct RepeatN : public BehNode
{
    BehNode* node;
    int32_t repeatCount;
    RepeatN(BehNode* node, int32_t count) : node(node), repeatCount(count) {}

    virtual ~RepeatN()
    {
        delete node;
    }

    BehResult update(flecs::world& ecs, flecs::entity entity, Blackboard& bb) override
    {
        for (int32_t i = 0; i < repeatCount; ++i)
        {
            BehResult res = node->update(ecs, entity, bb);
            if (res != BEH_SUCCESS)
                return res;
        }
        return BEH_SUCCESS;
    }
};

struct Sequence : public CompoundNode
{
  BehResult update(flecs::world &ecs, flecs::entity entity, Blackboard &bb) override
  {
    for (BehNode *node : nodes)
    {
      BehResult res = node->update(ecs, entity, bb);
      if (res != BEH_SUCCESS)
        return res;
    }
    return BEH_SUCCESS;
  }
};

struct Selector : public CompoundNode
{
  BehResult update(flecs::world &ecs, flecs::entity entity, Blackboard &bb) override
  {
    for (BehNode *node : nodes)
    {
      BehResult res = node->update(ecs, entity, bb);
      if (res != BEH_FAIL)
        return res;
    }
    return BEH_FAIL;
  }
};

struct MoveToEntity : public BehNode
{
  size_t entityBb = size_t(-1); // wraps to 0xff...
  MoveToEntity(flecs::entity entity, const char *bb_name)
  {
    entityBb = reg_entity_blackboard_var<flecs::entity>(entity, bb_name);
  }

  BehResult update(flecs::world &, flecs::entity entity, Blackboard &bb) override
  {
    BehResult res = BEH_RUNNING;
    entity.insert([&](Action &a, const Position &pos)
    {
      flecs::entity targetEntity = bb.get<flecs::entity>(entityBb);
      if (!targetEntity.is_alive())
      {
        res = BEH_FAIL;
        return;
      }
      targetEntity.get([&](const Position &target_pos)
      {
        if (pos != target_pos)
        {
          a.action = move_towards(pos, target_pos);
          res = BEH_RUNNING;
        }
        else
          res = BEH_SUCCESS;
      });
    });
    return res;
  }
};

//struct FindWaypoint : public BehNode
//{
//    size_t waypointEntityBb = size_t(-1);
//    FindWaypoint(flecs::entity entity, flecs::entity startWaypoint, const char* bb_name)
//    {
//        waypointEntityBb = reg_entity_blackboard_var<flecs::entity>(entity, bb_name);
//        entity.insert([&](Blackboard& bb)
//            {
//                bb.set<flecs::entity>(waypointEntityBb, startWaypoint);
//            });
//    }
//    BehResult update(flecs::world& ecs, flecs::entity entity, Blackboard& bb) override
//    {
//        BehResult res = BEH_FAIL;
//        entity.insert([&](Blackboard& bb)
//            {
//                flecs::entity currentWaypointEntity = bb.get<flecs::entity>(waypointEntityBb);
//                currentWaypointEntity.insert([&](const Waypoint& waypoint)
//                    {
//                        if (ecs.is_valid(waypoint.nextWaypoint)) {
//                            bb.set<flecs::entity>(waypointEntityBb, waypoint.nextWaypoint);
//                            res = BEH_SUCCESS;
//                        }
//                    });
//            });
//        return res;
//    }
//};

struct FindWaypoint : public BehNode
{
    size_t waypointBb = size_t(-1);
    FindWaypoint(flecs::entity entity, flecs::entity wp_entity, const char* bb_name)
    {
        waypointBb = reg_entity_blackboard_var<flecs::entity>(entity, bb_name);
        entity.insert([&](Blackboard& bb)
            {
                bb.set<flecs::entity>(waypointBb, wp_entity);
            });
    }
    BehResult update(flecs::world&, flecs::entity entity, Blackboard& bb) override
    {
        entity.insert([&](const Position& pos)
            {
                flecs::entity wp_entity = bb.get<flecs::entity>(waypointBb);
                if (pos == *wp_entity.get<Position>())
                {
                    bb.set<flecs::entity>(waypointBb, wp_entity.get<Waypoint>()->nextWaypoint);
                    Position newPos = *wp_entity.get<Waypoint>()->nextWaypoint.get<Position>();
                    printf("New waypoint: {%i, %i}\n", newPos.x, newPos.y);
                }
            });
        return BEH_SUCCESS;
    }
};


struct IsLowHp : public BehNode
{
  float threshold = 0.f;
  IsLowHp(float thres) : threshold(thres) {}

  BehResult update(flecs::world &, flecs::entity entity, Blackboard &) override
  {
    BehResult res = BEH_SUCCESS;
    entity.get([&](const Hitpoints &hp)
    {
      res = hp.hitpoints < threshold ? BEH_SUCCESS : BEH_FAIL;
    });
    return res;
  }
};

struct FindEnemy : public BehNode
{
  size_t entityBb = size_t(-1);
  float distance = 0;
  FindEnemy(flecs::entity entity, float in_dist, const char *bb_name) : distance(in_dist)
  {
    entityBb = reg_entity_blackboard_var<flecs::entity>(entity, bb_name);
  }
  BehResult update(flecs::world &ecs, flecs::entity entity, Blackboard &bb) override
  {
    BehResult res = BEH_FAIL;
    static auto enemiesQuery = ecs.query<const Position, const Team>();
    entity.insert([&](const Position &pos, const Team &t)
    {
      flecs::entity closestEnemy;
      float closestDist = FLT_MAX;
      Position closestPos;
      enemiesQuery.each([&](flecs::entity enemy, const Position &epos, const Team &et)
      {
        if (t.team == et.team)
          return;
        float curDist = dist(epos, pos);
        if (curDist < closestDist)
        {
          closestDist = curDist;
          closestPos = epos;
          closestEnemy = enemy;
        }
      });
      if (ecs.is_valid(closestEnemy) && closestDist <= distance)
      {
        bb.set<flecs::entity>(entityBb, closestEnemy);
        res = BEH_SUCCESS;
      }
    });
    return res;
  }
};

struct FindPowerupOrHeal : public BehNode
{
    size_t entityBb = size_t(-1);
    float distance = 0;
    FindPowerupOrHeal(flecs::entity entity, float in_dist, const char* bb_name) : distance(in_dist)
    {
        entityBb = reg_entity_blackboard_var<flecs::entity>(entity, bb_name);
    }
    BehResult update(flecs::world& ecs, flecs::entity entity, Blackboard& bb) override
    {
        BehResult res = BEH_FAIL;
        static auto healsQuery = ecs.query<const Position, const HealAmount>();
        static auto powerupsQuery = ecs.query<const Position, const PowerupAmount>();
        entity.insert([&](const Position& pos)
            {
                flecs::entity closestEntity;
                float closestDist = FLT_MAX;
                Position closestPos;
                healsQuery.each([&](flecs::entity heal, const Position& epos, const HealAmount&)
                    {
                        float curDist = dist(epos, pos);
                        if (curDist < closestDist)
                        {
                            closestDist = curDist;
                            closestPos = epos;
                            closestEntity = heal;
                        }
                    });
                powerupsQuery.each([&](flecs::entity powerup, const Position& epos, const PowerupAmount)
                    {
                        float curDist = dist(epos, pos);
                        if (curDist < closestDist)
                        {
                            closestDist = curDist;
                            closestPos = epos;
                            closestEntity = powerup;
                        }
                    });
                if (ecs.is_valid(closestEntity) && closestDist <= distance)
                {
                    bb.set<flecs::entity>(entityBb, closestEntity);
                    res = BEH_SUCCESS;
                }
            });
        return res;
    }
};

struct SpawnItems : public BehNode
{
    BehResult update(flecs::world&, flecs::entity entity, Blackboard& bb) override
    {
        BehResult res = BEH_RUNNING;
        entity.insert([&](Action& a, const Gatherer& gatherer)
            {
                if (gatherer.itemsCollected > 0) 
                {
                    a.action = EA_SPAWN_ITEMS;
                }
                else 
                {
                    res = BEH_FAIL;
                }
            });
        return res;
    }
};

struct Flee : public BehNode
{
  size_t entityBb = size_t(-1);
  Flee(flecs::entity entity, const char *bb_name)
  {
    entityBb = reg_entity_blackboard_var<flecs::entity>(entity, bb_name);
  }

  BehResult update(flecs::world &, flecs::entity entity, Blackboard &bb) override
  {
    BehResult res = BEH_RUNNING;
    entity.insert([&](Action &a, const Position &pos)
    {
      flecs::entity targetEntity = bb.get<flecs::entity>(entityBb);
      if (!targetEntity.is_alive())
      {
        res = BEH_FAIL;
        return;
      }
      targetEntity.get([&](const Position &target_pos)
      {
        a.action = inverse_move(move_towards(pos, target_pos));
      });
    });
    return res;
  }
};

struct Patrol : public BehNode
{
  size_t pposBb = size_t(-1);
  float patrolDist = 1.f;
  Patrol(flecs::entity entity, float patrol_dist, const char *bb_name)
    : patrolDist(patrol_dist)
  {
    pposBb = reg_entity_blackboard_var<Position>(entity, bb_name);
    entity.insert([&](Blackboard &bb, const Position &pos)
    {
      bb.set<Position>(pposBb, pos);
    });
  }

  BehResult update(flecs::world &, flecs::entity entity, Blackboard &bb) override
  {
    BehResult res = BEH_RUNNING;
    entity.insert([&](Action &a, const Position &pos)
    {
      Position patrolPos = bb.get<Position>(pposBb);
      if (dist(pos, patrolPos) > patrolDist)
        a.action = move_towards(pos, patrolPos);
      else
        a.action = GetRandomValue(EA_MOVE_START, EA_MOVE_END - 1); // do a random walk
    });
    return res;
  }
};

struct MoveToSpawn : public BehNode
{
    BehResult update(flecs::world&, flecs::entity entity, Blackboard& bb) override
    {
        BehResult res = BEH_RUNNING;
        entity.insert([&](Action& a, const Position& pos, const SpawnPos& spawnPos)
            {    
                if (pos.x != spawnPos.x || pos.y != spawnPos.y)
                    a.action = move_towards(pos, spawnPos);
                else
                    res = BEH_SUCCESS;
            });
        return res;
    }
};

BehNode *sequence(const std::vector<BehNode*> &nodes)
{
  Sequence *seq = new Sequence;
  for (BehNode *node : nodes)
    seq->pushNode(node);
  return seq;
}

BehNode *selector(const std::vector<BehNode*> &nodes)
{
  Selector *sel = new Selector;
  for (BehNode *node : nodes)
    sel->pushNode(node);
  return sel;
}

BehNode *move_to_entity(flecs::entity entity, const char *bb_name)
{
  return new MoveToEntity(entity, bb_name);
}

BehNode *is_low_hp(float thres)
{
  return new IsLowHp(thres);
}

BehNode *find_enemy(flecs::entity entity, float dist, const char *bb_name)
{
  return new FindEnemy(entity, dist, bb_name);
}

BehNode *flee(flecs::entity entity, const char *bb_name)
{
  return new Flee(entity, bb_name);
}

BehNode *patrol(flecs::entity entity, float patrol_dist, const char *bb_name)
{
  return new Patrol(entity, patrol_dist, bb_name);
}

BehNode *node_not(BehNode* node) 
{
    return new Not(node);
}

BehNode *node_xor(BehNode* nodeFirst, BehNode* nodeSecond)
{
    return new Xor(nodeFirst, nodeSecond);
}

BehNode *node_repeat_n(BehNode* nodeFirst, int32_t numRepeats)
{
    return new RepeatN(nodeFirst, numRepeats);
}

BehNode *find_powerup_or_heal(flecs::entity entity, float in_dist, const char* bb_name)
{
    return new FindPowerupOrHeal(entity, in_dist, bb_name);
}

BehNode* find_waypoint(flecs::entity entity, flecs::entity start_waypoint, const char* bb_name)
{
    return new FindWaypoint(entity, start_waypoint, bb_name);
}

BehNode *move_to_spawn() 
{
    return new MoveToSpawn();
}

BehNode *spawn_items() 
{
    return new SpawnItems();
}


