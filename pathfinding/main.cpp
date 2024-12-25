#include "raylib.h"
#include <functional>
#include <vector>
#include <limits>
#include <float.h>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include "math.h"
#include "dungeonGen.h"
#include "dungeonUtils.h"

enum class Algorithm
{
    A,
    AWA
};

static const int AWA_MAX_HITS = 10;

template<typename T>
static size_t coord_to_idx(T x, T y, size_t w)
{
  return size_t(y) * w + size_t(x);
}

template <typename T>
static T idx_to_coord(size_t idx, size_t w)
{
    return T{ (decltype(T{}.x))(idx % w), (decltype(T{}.y))(idx / w) };
}

static void draw_nav_grid(const char *input, size_t width, size_t height)
{
  for (size_t y = 0; y < height; ++y)
    for (size_t x = 0; x < width; ++x)
    {
      char symb = input[coord_to_idx(x, y, width)];
      Color color = GetColor(symb == ' ' ? 0xeeeeeeff : symb == 'o' ? 0x7777ffff : 0x222222ff);
      const Rectangle rect = {float(x), float(y), 1.f, 1.f};
      DrawRectangleRec(rect, color);
      //DrawPixel(int(x), int(y), color);
    }
}

static void draw_path(std::vector<Position> path)
{
  for (const Position &p : path)
  {
    const Rectangle rect = {float(p.x), float(p.y), 1.f, 1.f};
    DrawRectangleRec(rect, GetColor(0x44000088));
  }
}

static std::vector<Position> reconstruct_path(std::vector<Position> prev, Position to, size_t width)
{
  Position curPos = to;
  std::vector<Position> res = {curPos};
  while (prev[coord_to_idx(curPos.x, curPos.y, width)] != Position{-1, -1})
  {
    curPos = prev[coord_to_idx(curPos.x, curPos.y, width)];
    res.insert(res.begin(), curPos);
  }
  return res;
}

float heuristic(Position lhs, Position rhs)
{
  return sqrtf(square(float(lhs.x - rhs.x)) + square(float(lhs.y - rhs.y)));
};

static float ida_star_search(const char *input, size_t width, size_t height, std::vector<Position> &path, const float g, const float bound, Position to)
{
  const Position &p = path.back();
  const float f = g + heuristic(p, to);
  if (f > bound)
    return f;
  if (p == to)
    return -f;
  float min = FLT_MAX;
  auto checkNeighbour = [&](Position p) -> float
  {
    // out of bounds
    if (p.x < 0 || p.y < 0 || p.x >= int(width) || p.y >= int(height))
      return 0.f;
    size_t idx = coord_to_idx(p.x, p.y, width);
    // not empty
    if (input[idx] == '#')
      return 0.f;
    if (std::find(path.begin(), path.end(), p) != path.end())
      return 0.f;
    path.push_back(p);
    float weight = input[idx] == 'o' ? 10.f : 1.f;
    float gScore = g + 1.f * weight; // we're exactly 1 unit away
    const float t = ida_star_search(input, width, height, path, gScore, bound, to);
    if (t < 0.f)
      return t;
    if (t < min)
      min = t;
    path.pop_back();
    return t;
  };
  float lv = checkNeighbour({p.x + 1, p.y + 0});
  if (lv < 0.f) return lv;
  float rv = checkNeighbour({p.x - 1, p.y + 0});
  if (rv < 0.f) return rv;
  float tv = checkNeighbour({p.x + 0, p.y + 1});
  if (tv < 0.f) return tv;
  float bv = checkNeighbour({p.x + 0, p.y - 1});
  if (bv < 0.f) return bv;
  return min;
}

static std::vector<Position> find_ida_star_path(const char *input, size_t width, size_t height, Position from, Position to)
{
  float bound = heuristic(from, to);
  std::vector<Position> path = {from};
  while (true)
  {
    const float t = ida_star_search(input, width, height, path, 0.f, bound, to);
    if (t < 0.f)
      return path;
    if (t == FLT_MAX)
      return {};
    bound = t;
    printf("new bound %0.1f\n", bound);
  }
  return {};
}

static std::vector<Position> find_path_a_star(const char *input, size_t width, size_t height, Position from, Position to, float weight)
{
  if (from.x < 0 || from.y < 0 || from.x >= int(width) || from.y >= int(height))
    return std::vector<Position>();
  size_t inpSize = width * height;

  std::vector<float> g(inpSize, std::numeric_limits<float>::max());
  std::vector<float> f(inpSize, std::numeric_limits<float>::max());
  std::vector<Position> prev(inpSize, {-1,-1});

  auto getG = [&](Position p) -> float { return g[coord_to_idx(p.x, p.y, width)]; };
  auto getF = [&](Position p) -> float { return f[coord_to_idx(p.x, p.y, width)]; };

  g[coord_to_idx(from.x, from.y, width)] = 0;
  f[coord_to_idx(from.x, from.y, width)] = weight * heuristic(from, to);

  std::vector<Position> openList = {from};
  std::vector<Position> closedList;

  while (!openList.empty())
  {
    size_t bestIdx = 0;
    float bestScore = getF(openList[0]);
    for (size_t i = 1; i < openList.size(); ++i)
    {
      float score = getF(openList[i]);
      if (score < bestScore)
      {
        bestIdx = i;
        bestScore = score;
      }
    }
    if (openList[bestIdx] == to)
      return reconstruct_path(prev, to, width);
    Position curPos = openList[bestIdx];
    openList.erase(openList.begin() + bestIdx);
    if (std::find(closedList.begin(), closedList.end(), curPos) != closedList.end())
      continue;
    size_t idx = coord_to_idx(curPos.x, curPos.y, width);
    const Rectangle rect = {float(curPos.x), float(curPos.y), 1.f, 1.f};
    DrawRectangleRec(rect, Color{uint8_t(g[idx]), uint8_t(g[idx]), 0, 100});
    closedList.emplace_back(curPos);
    auto checkNeighbour = [&](Position p)
    {
      // out of bounds
      if (p.x < 0 || p.y < 0 || p.x >= int(width) || p.y >= int(height))
        return;
      size_t idx = coord_to_idx(p.x, p.y, width);
      // not empty
      if (input[idx] == '#')
        return;
      float edgeWeight = input[idx] == 'o' ? 10.f : 1.f;
      float gScore = getG(curPos) + 1.f * edgeWeight; // we're exactly 1 unit away
      if (gScore < getG(p))
      {
        prev[idx] = curPos;
        g[idx] = gScore;
        f[idx] = gScore + weight * heuristic(p, to);
      }
      bool found = std::find(openList.begin(), openList.end(), p) != openList.end();
      if (!found)
        openList.emplace_back(p);
    };
    checkNeighbour({curPos.x + 1, curPos.y + 0});
    checkNeighbour({curPos.x - 1, curPos.y + 0});
    checkNeighbour({curPos.x + 0, curPos.y + 1});
    checkNeighbour({curPos.x + 0, curPos.y - 1});
  }
  // empty path
  return std::vector<Position>();
}

std::vector<Position> find_path_awa_star(const char* input, size_t width, size_t height, Position from, Position to, float weight) 
{
    if (from.x < 0 || from.y < 0 || from.x >= int(width) || from.y >= int(height) ||
        input[coord_to_idx(from.x, from.y, width)] == '#' ||
        to.x < 0 || to.y < 0 || to.x >= int(width) || to.y >= int(height) ||
        input[coord_to_idx(to.x, to.y, width)] == '#') 
    {
        return {};
    }

    size_t inpSize = width * height;
    std::vector<float> g(inpSize, std::numeric_limits<float>::max());
    std::vector<float> f(inpSize, std::numeric_limits<float>::max());
    std::vector<float> fw(inpSize, std::numeric_limits<float>::max());
    std::vector<Position> prev(inpSize, { -1, -1 });

    auto get_idx = [&](const Position& p) { return coord_to_idx(p.x, p.y, width); };
    auto get_g = [&](const Position& p) { return g[get_idx(p)]; };
    auto get_f = [&](const Position& p) { return f[get_idx(p)]; };
    auto get_fw = [&](const Position& p) { return fw[get_idx(p)]; };

    g[get_idx(from)] = 0;
    f[get_idx(from)] = heuristic(from, to);
    fw[get_idx(from)] = weight * heuristic(from, to);

    size_t incumbent = -1;
    size_t hits = 0;
    std::vector<Position> openList = { from };
    std::vector<Position> closedList;

    while (!openList.empty())
    {
        auto bestIt = std::min_element(openList.begin(), openList.end(), [&](const Position& a, const Position& b) 
            {
            return get_fw(a) < get_fw(b);
            });

        Position curPos = *bestIt;
        openList.erase(bestIt);

        if (incumbent != -1 && get_f(curPos) >= get_f(Position{ static_cast<int>(incumbent % width), static_cast<int>(incumbent / width) })) 
        {
            continue;
        }

        size_t idx = coord_to_idx(curPos.x, curPos.y, width);
        const Rectangle rect = { float(curPos.x), float(curPos.y), 1.f, 1.f };
        DrawRectangleRec(rect, Color{ uint8_t(g[idx]), uint8_t(g[idx]), 0, 100 });

        if (std::find(closedList.begin(), closedList.end(), curPos) != closedList.end()) {
            continue;
        }
        closedList.push_back(curPos);

        auto check_neighbor = [&](Position p)
            {
            if (p.x < 0 || p.y < 0 || p.x >= int(width) || p.y >= int(height) || input[get_idx(p)] == '#') 
            {
                return;
            }

            size_t idx = get_idx(p);
            float edgeWeight = input[idx] == 'o' ? 10.f : 1.f;
            float gScore = get_g(curPos) + 1.0f * edgeWeight;

            if (p == to)
            {
                g[idx] = f[idx] = fw[idx] = gScore;
                prev[idx] = curPos;
                incumbent = idx;
                ++hits;
                return;
            }

            auto in_open = std::find(openList.begin(), openList.end(), p) != openList.end();
            auto in_closed = std::find(closedList.begin(), closedList.end(), p) != closedList.end();

            if ((in_open || in_closed) && g[idx] > gScore) 
            {
                g[idx] = gScore;
                f[idx] = gScore + heuristic(p, to);
                fw[idx] = gScore + weight * heuristic(p, to);
                prev[idx] = curPos;
                if (in_closed) 
                {
                    closedList.erase(std::remove(closedList.begin(), closedList.end(), p), closedList.end());
                }
                if (!in_open) 
                {
                    openList.push_back(p);
                }
            }
            else if (!in_open && !in_closed) 
            {
                g[idx] = gScore;
                f[idx] = gScore + heuristic(p, to);
                fw[idx] = gScore + weight * heuristic(p, to);
                prev[idx] = curPos;
                openList.push_back(p);
            }
            };

        check_neighbor({ curPos.x + 1, curPos.y });
        check_neighbor({ curPos.x - 1, curPos.y });
        check_neighbor({ curPos.x, curPos.y + 1 });
        check_neighbor({ curPos.x, curPos.y - 1 });

        if (hits >= AWA_MAX_HITS) 
        {
            break;
        }
    }
    return hits > 0 ? reconstruct_path(prev, to, width) : std::vector<Position>{};
}

std::vector<float> choose_best_weights(size_t width, size_t height, Algorithm alg, int nmaps, const std::vector<float>& ws,
    const std::vector<float>& percs)
{
    std::vector<char> navGrid(width * height);
    auto findPath = [&alg](const std::vector<char>& grid, size_t w, size_t h, const Position& from, const Position& to, float weight) -> std::vector<Position> 
        {
        switch (alg) 
        {
        case Algorithm::A:
            return find_path_a_star(grid.data(), w, h, from, to, weight);
        case Algorithm::AWA:
            return find_path_awa_star(grid.data(), w, h, from, to, weight);
        }
        };

    std::vector<float> freqs(ws.size(), 0.f);
    for (int imap = 0; imap < nmaps; imap++)
    {
        gen_drunk_dungeon(navGrid.data(), width, height, 24, 150, false);
        spill_drunk_water(navGrid.data(), width, height, 8, 10);

        Position from = dungeon::find_walkable_tile(navGrid.data(), width, height);
        Position to = dungeon::find_walkable_tile(navGrid.data(), width, height);

        std::vector<Position> basePath = findPath(navGrid, width, height, from, to, 1.f);

        for (size_t i = 0; i < ws.size(); i++)
        {
            std::vector<Position> path = findPath(navGrid, width, height, from, to, ws[i]);
            if (path == basePath) 
            {
                freqs[i] += 1.f / static_cast<float>(nmaps); 
            }
        }
    }

    std::vector<float> bestWs(percs.size(), 1.f);
    for (size_t i = 0; i < ws.size(); i++) 
    {
        for (size_t j = 0; j < percs.size(); j++)
        {
            if (freqs[i] >= percs[j] && ws[i] > bestWs[j]) 
            {
                bestWs[j] = ws[i];
            }
        }
    }

    return bestWs;
}

void draw_nav_data(const char *input, size_t width, size_t height, Position from, Position to, float weight, Algorithm alg)
{
  draw_nav_grid(input, width, height);
  std::vector<Position> path;
  switch (alg)
  {
    case Algorithm::A: 
        path = find_path_a_star(input, width, height, from, to, weight); 
        break;
    case Algorithm::AWA: 
        path = find_path_awa_star(input, width, height, from, to, weight); 
        break;
  }
    
  //std::vector<Position> path = find_ida_star_path(input, width, height, from, to);
  draw_path(path);
}

int main(int /*argc*/, const char ** /*argv*/)
{
  int width = 1920;
  int height = 1080;
  InitWindow(width, height, "w3 AI MIPT");

  const int scrWidth = GetMonitorWidth(0);
  const int scrHeight = GetMonitorHeight(0);
  if (scrWidth < width || scrHeight < height)
  {
    width = std::min(scrWidth, width);
    height = std::min(scrHeight - 150, height);
    SetWindowSize(width, height);
  }

  constexpr size_t dungWidth = 100;
  constexpr size_t dungHeight = 100;
  char *navGrid = new char[dungWidth * dungHeight];
  gen_drunk_dungeon(navGrid, dungWidth, dungHeight, 24, 100);
  spill_drunk_water(navGrid, dungWidth, dungHeight, 8, 10);
  float weight = 1.f;

  Position from = dungeon::find_walkable_tile(navGrid, dungWidth, dungHeight);
  Position to = dungeon::find_walkable_tile(navGrid, dungWidth, dungHeight);

  Camera2D camera = { {0, 0}, {0, 0}, 0.f, 1.f };
  //camera.offset = Vector2{ width * 0.5f, height * 0.5f };
  camera.zoom = float(height) / float(dungHeight);

  Algorithm alg = Algorithm::AWA;

  std::vector<float> ws{};
  for (int i = 0; i < 20; ++i)
      ws.push_back(float(1.1f + 0.1f * float(i)));
  std::vector<float> percs = { 0.99f, 0.95f, 0.9f, 0.75f };
  auto bestWs = choose_best_weights(dungWidth, dungHeight, alg, 20, ws, percs);
  for (size_t i = 0; i < percs.size(); ++i)
      printf("Best weight for %f = %f\n", percs[i], bestWs[i]);

  SetTargetFPS(60);               // Set our game to run at 60 frames-per-second
  while (!WindowShouldClose())
  {
    // pick pos
    Vector2 mousePosition = GetScreenToWorld2D(GetMousePosition(), camera);
    Position p{int(mousePosition.x), int(mousePosition.y)};
    if (IsMouseButtonPressed(2) || IsKeyPressed(KEY_Q))
    {
      size_t idx = coord_to_idx(p.x, p.y, dungWidth);
      if (idx < dungWidth * dungHeight)
        navGrid[idx] = navGrid[idx] == ' ' ? '#' : navGrid[idx] == '#' ? 'o' : ' ';
    }
    else if (IsMouseButtonPressed(0))
    {
      Position &target = from;
      target = p;
    }
    else if (IsMouseButtonPressed(1))
    {
      Position &target = to;
      target = p;
    }
    if (IsKeyPressed(KEY_SPACE))
    {
      gen_drunk_dungeon(navGrid, dungWidth, dungHeight, 24, 100);
      spill_drunk_water(navGrid, dungWidth, dungHeight, 8, 10);
      from = dungeon::find_walkable_tile(navGrid, dungWidth, dungHeight);
      to = dungeon::find_walkable_tile(navGrid, dungWidth, dungHeight);
    }
    if (IsKeyPressed(KEY_UP))
    {
      weight += 0.1f;
      printf("new weight %f\n", weight);
    }
    if (IsKeyPressed(KEY_DOWN))
    {
      weight = std::max(1.f, weight - 0.1f);
      printf("new weight %f\n", weight);
    }
    if (IsKeyPressed(KEY_ONE))
    {
        alg = Algorithm::A;
        printf("using A_star\n");
    }
    if (IsKeyPressed(KEY_TWO))
    {
        alg = Algorithm::AWA;
        printf("using AWA_star\n");
    }
    BeginDrawing();
      ClearBackground(BLACK);
      BeginMode2D(camera);
        draw_nav_data(navGrid, dungWidth, dungHeight, from, to, weight, alg);
      EndMode2D();
    EndDrawing();
  }
  CloseWindow();
  return 0;
}
