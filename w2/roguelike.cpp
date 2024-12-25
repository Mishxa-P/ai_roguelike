#include "roguelike.h"
#include "ecsTypes.h"
#include "raylib.h"
#include "stateMachine.h"
#include "aiLibrary.h"
#include "blackboard.h"


static void create_minotaur_beh(flecs::entity e)
{
  e.set(Blackboard{});
  BehNode *root =
    selector({
      sequence({
        is_low_hp(50.f),
        find_enemy(e, 4.f, "flee_enemy"),
        flee(e, "flee_enemy")
      }),
      sequence({
        find_enemy(e, 3.f, "attack_enemy"),
        move_to_entity(e, "attack_enemy")
      }),
      patrol(e, 2.f, "patrol_pos")
    });
  e.set(BehaviourTree{root});
}

static flecs::entity create_monster(flecs::world &ecs, int x, int y, Color col, const char *texture_src)
{
  flecs::entity textureSrc = ecs.entity(texture_src);
  return ecs.entity()
    .set(Position{x, y})
    .set(MovePos{x, y})
    .set(Hitpoints{100.f})
    .set(Action{EA_NOP})
    .set(Color{col})
    .add<TextureSource>(textureSrc)
    .set(StateMachine{})
    .set(Team{1})
    .set(NumActions{1, 0})
    .set(MeleeDamage{20.f})
    .set(Blackboard{});
}

static void create_player(flecs::world &ecs, int x, int y, const char *texture_src)
{
  flecs::entity textureSrc = ecs.entity(texture_src);
  ecs.entity("player")
    .set(Position{x, y})
    .set(MovePos{x, y})
    .set(Hitpoints{100.f})
    //.set(Color{0xee, 0xee, 0xee, 0xff})
    .set(Action{EA_NOP})
    .add<IsPlayer>()
    .set(Team{0})
    .set(PlayerInput{})
    .set(NumActions{2, 0})
    .set(Color{255, 255, 255, 255})
    .add<TextureSource>(textureSrc)
    .set(MeleeDamage{50.f});
}

static void create_gatherer_beh(flecs::world& ecs, flecs::entity e)
{
    e.set(Gatherer{ 0 });
    e.set(SpawnPos{ e.get<Position>()->x, e.get<Position>()->y });
    e.set(Blackboard{});
    BehNode* root =
        selector({
          sequence({
            find_powerup_or_heal(e, 10.f, "item"),
            move_to_entity(e, "item")
          }),
          sequence({
            move_to_spawn(),
            spawn_items()
          })
            });
    e.set(BehaviourTree{ root });
}

static void create_guard_beh(flecs::entity e, flecs::entity start_waypoint)
{
    e.set(Blackboard{});
    BehNode* root =
        selector({
          sequence({ 
            find_enemy(e, 2.f, "attack_enemy"),
            move_to_entity(e, "attack_enemy")
          }),
          sequence({ 
            find_waypoint(e, start_waypoint, "next_waypoint"),
            move_to_entity(e, "next_waypoint")
          })
            });
    e.set(BehaviourTree{ root });
}


static void create_gatherer_visual_spawn_point(flecs::world& ecs, Color color, int x, int y)
{
     ecs.entity()
        .set(Position{ x , y })
        .set(Color{ color });
}

static flecs::entity create_waypoint(flecs::world& ecs, Color color, int x, int y)
{
    return ecs.entity()
        .set(Position{ x , y })
        .set(Waypoint{ flecs::entity::null() })
        .set(Color{ color });
}
static flecs::entity create_waypoints(flecs::world& ecs, Color color, int start_point_x, int start_point_y, int wp_count, int max_x, int max_y)
{
    flecs::entity start_waypoint = create_waypoint(ecs, color, start_point_x, start_point_y);
    flecs::entity last_waypoint = start_waypoint;
    for (int i = 1; i < wp_count; ++i)
    {
        flecs::entity next_waypoint = create_waypoint(ecs, color, GetRandomValue(-max_x, max_x), GetRandomValue(-max_y, max_y));
        last_waypoint.get_mut<Waypoint>()->nextWaypoint = next_waypoint;
        last_waypoint = next_waypoint;
    }
    last_waypoint.get_mut<Waypoint>()->nextWaypoint = start_waypoint;
    return start_waypoint;
}

static void create_heal(flecs::world &ecs, int x, int y, float amount)
{
  ecs.entity()
    .set(Position{x, y})
    .set(HealAmount{amount})
    .set(Color{0xff, 0x44, 0x44, 0xff});
}

static void create_powerup(flecs::world &ecs, int x, int y, float amount)
{
  ecs.entity()
    .set(Position{x, y})
    .set(PowerupAmount{amount})
    .set(Color{0xff, 0xff, 0x00, 0xff});
}

static void register_roguelike_systems(flecs::world &ecs)
{
  ecs.system<PlayerInput, Action, const IsPlayer>()
    .each([&](PlayerInput &inp, Action &a, const IsPlayer)
    {
      bool left = IsKeyDown(KEY_LEFT);
      bool right = IsKeyDown(KEY_RIGHT);
      bool up = IsKeyDown(KEY_UP);
      bool down = IsKeyDown(KEY_DOWN);
      if (left && !inp.left)
        a.action = EA_MOVE_LEFT;
      if (right && !inp.right)
        a.action = EA_MOVE_RIGHT;
      if (up && !inp.up)
        a.action = EA_MOVE_UP;
      if (down && !inp.down)
        a.action = EA_MOVE_DOWN;
      inp.left = left;
      inp.right = right;
      inp.up = up;
      inp.down = down;
    });
  ecs.system<const Position, const Color>()
    .without<TextureSource>(flecs::Wildcard)
    .each([&](const Position &pos, const Color color)
    {
      const Rectangle rect = {float(pos.x), float(pos.y), 1, 1};
      DrawRectangleRec(rect, color);
    });
  ecs.system<const Position, const Color>()
    .with<TextureSource>(flecs::Wildcard)
    .each([&](flecs::entity e, const Position &pos, const Color color)
    {
      const auto textureSrc = e.target<TextureSource>();
      DrawTextureQuad(*textureSrc.get<Texture2D>(),
          Vector2{1, 1}, Vector2{0, 0},
          Rectangle{float(pos.x), float(pos.y), 1, 1}, color);
    });
}


void init_roguelike(flecs::world &ecs)
{
  register_roguelike_systems(ecs);

  ecs.entity("swordsman_tex")
    .set(Texture2D{LoadTexture("assets/swordsman.png")});
  ecs.entity("minotaur_tex")
    .set(Texture2D{LoadTexture("assets/minotaur.png")});

  ecs.observer<Texture2D>()
    .event(flecs::OnRemove)
    .each([](Texture2D texture)
      {
        UnloadTexture(texture);
      });


  create_gatherer_beh(ecs, create_monster(ecs, -5, 5, Color{ 0xee, 0x00, 0xee, 0xff }, "minotaur_tex"));
  create_gatherer_visual_spawn_point(ecs, Color{ 0xee, 0x00, 0xee, 0xff }, -5, 5);

  create_guard_beh(create_monster(ecs, 2, 2, Color{ 0x05, 0x44, 0x44, 0xff }, "minotaur_tex"), create_waypoints(ecs, Color{ 0x05, 0x44, 0x44, 0xff }, 2, 2, 4, 5, 5));

  create_player(ecs, 0, 0, "swordsman_tex");

  create_powerup(ecs, 7, 7, 10.f);
  create_powerup(ecs, 10, -6, 10.f);
  create_powerup(ecs, 10, -4, 10.f);

  create_heal(ecs, -5, -5, 50.f);
  create_heal(ecs, -5, 5, 50.f);
}

static bool is_player_acted(flecs::world &ecs)
{
  static auto processPlayer = ecs.query<const IsPlayer, const Action>();
  bool playerActed = false;
  processPlayer.each([&](const IsPlayer, const Action &a)
  {
    playerActed = a.action != EA_NOP;
  });
  return playerActed;
}

static bool upd_player_actions_count(flecs::world &ecs)
{
  static auto updPlayerActions = ecs.query<const IsPlayer, NumActions>();
  bool actionsReached = false;
  updPlayerActions.each([&](const IsPlayer, NumActions &na)
  {
    na.curActions = (na.curActions + 1) % na.numActions;
    actionsReached |= na.curActions == 0;
  });
  return actionsReached;
}

static Position move_pos(Position pos, int action)
{
  if (action == EA_MOVE_LEFT)
    pos.x--;
  else if (action == EA_MOVE_RIGHT)
    pos.x++;
  else if (action == EA_MOVE_UP)
    pos.y--;
  else if (action == EA_MOVE_DOWN)
    pos.y++;
  return pos;
}

static void process_actions(flecs::world &ecs)
{
  static auto processActions = ecs.query<Action, Position, MovePos, const MeleeDamage, const Team>();
  static auto checkAttacks = ecs.query<const MovePos, Hitpoints, const Team>();
  static auto gathererSpawnItems = ecs.query<Gatherer, const Position, Action>();
  // Process all actions
  ecs.defer([&]
  {
    processActions.each([&](flecs::entity entity, Action &a, Position &pos, MovePos &mpos, const MeleeDamage &dmg, const Team &team)
    {
      Position nextPos = move_pos(pos, a.action);
      bool blocked = false;
      checkAttacks.each([&](flecs::entity enemy, const MovePos &epos, Hitpoints &hp, const Team &enemy_team)
      {
        if (entity != enemy && epos == nextPos)
        {
          blocked = true;
          if (team.team != enemy_team.team)
            hp.hitpoints -= dmg.damage;
        }
      });
      if (blocked)
        a.action = EA_NOP;
      else
        mpos = nextPos;
    });
    gathererSpawnItems.each([&](Gatherer& gatherer, const Position& pos, Action& a) {
        if (a.action == EA_SPAWN_ITEMS) 
        {
            for (int32_t i = 0; i < gatherer.itemsCollected * 2; ++i) 
            {
                float dist = (static_cast<float>(rand()) / RAND_MAX) * 20.0f;
                float angle = static_cast<float>(rand());
                float randPointX = std::cos(angle) * dist;
                float randPointY = std::sin(angle) * dist;
                int intRandPointX = pos.x < randPointX ? std::floor(randPointX) : std::ceil(randPointX);
                int intRandPointY = pos.y < randPointY ? std::floor(randPointY) : std::ceil(randPointY);
                if (rand() % 2 == 0) 
                {
                    create_powerup(ecs, intRandPointX, intRandPointY, 10.f);
                }
                else 
                {
                    create_heal(ecs, intRandPointX, intRandPointY, 50.f);
                }
            }
            gatherer.itemsCollected = 0;
            a.action = EA_NOP;
        }
        });
    // now move
    processActions.each([&](Action &a, Position &pos, MovePos &mpos, const MeleeDamage &, const Team&)
    {
      pos = mpos;
      a.action = EA_NOP;
    });
  });

  static auto deleteAllDead = ecs.query<const Hitpoints>();
  ecs.defer([&]
  {
    deleteAllDead.each([&](flecs::entity entity, const Hitpoints &hp)
    {
      if (hp.hitpoints <= 0.f)
        entity.destruct();
    });
  });

  static auto playerPickup = ecs.query<const IsPlayer, const Position, Hitpoints, MeleeDamage>();
  static auto gathererPickup = ecs.query<Gatherer, const Position, Hitpoints, MeleeDamage>();
  static auto healPickup = ecs.query<const Position, const HealAmount>();
  static auto powerupPickup = ecs.query<const Position, const PowerupAmount>();
  ecs.defer([&]
  {
    playerPickup.each([&](const IsPlayer&, const Position &pos, Hitpoints &hp, MeleeDamage &dmg)
    {
      healPickup.each([&](flecs::entity entity, const Position &ppos, const HealAmount &amt)
      {
        if (pos == ppos)
        {
          hp.hitpoints += amt.amount;
          entity.destruct();
        }
      });
      powerupPickup.each([&](flecs::entity entity, const Position &ppos, const PowerupAmount &amt)
      {
        if (pos == ppos)
        {
          dmg.damage += amt.amount;
          entity.destruct();
        }
      });
    });
    gathererPickup.each([&](Gatherer& gatherer, const Position& pos, Hitpoints& hp, MeleeDamage& dmg)
        {
            healPickup.each([&](flecs::entity entity, const Position& ppos, const HealAmount& amt)
                {
                    if (pos == ppos)
                    {
                        hp.hitpoints += amt.amount;
                        gatherer.itemsCollected += 1;
                        entity.destruct();
                    }
                });
            powerupPickup.each([&](flecs::entity entity, const Position& ppos, const PowerupAmount& amt)
                {
                    if (pos == ppos)
                    {
                        dmg.damage += amt.amount;
                        gatherer.itemsCollected += 1;
                        entity.destruct();
                    }
                });
        });
  });
}

void process_turn(flecs::world &ecs)
{
  static auto stateMachineAct = ecs.query<StateMachine>();
  static auto behTreeUpdate = ecs.query<BehaviourTree, Blackboard>();
  if (is_player_acted(ecs))
  {
    if (upd_player_actions_count(ecs))
    {
      // Plan action for NPCs
      ecs.defer([&]
      {
        stateMachineAct.each([&](flecs::entity e, StateMachine &sm)
        {
          sm.act(0.f, ecs, e);
        });
        behTreeUpdate.each([&](flecs::entity e, BehaviourTree &bt, Blackboard &bb)
        {
          bt.update(ecs, e, bb);
        });
      });
    }
    process_actions(ecs);
  }
}

void print_stats(flecs::world &ecs)
{
  static auto playerStatsQuery = ecs.query<const IsPlayer, const Hitpoints, const MeleeDamage>();
  playerStatsQuery.each([&](const IsPlayer &, const Hitpoints &hp, const MeleeDamage &dmg)
  {
    DrawText(TextFormat("hp: %d", int(hp.hitpoints)), 20, 20, 20, WHITE);
    DrawText(TextFormat("power: %d", int(dmg.damage)), 20, 40, 20, WHITE);
  });
}

