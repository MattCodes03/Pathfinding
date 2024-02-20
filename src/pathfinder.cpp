#include "vec.hpp"
#include "draw-triangle-pro.hpp"
#include "raylib-cpp.hpp"
#include "graph.hpp"
#include "graph-utils.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>

std::vector<node_t> astar_pathfind(const Graph &g, node_t start, node_t goal)
{
  std::unordered_map<node_t, node_t> came_from;
  std::unordered_map<node_t, double> cost_so_far;
  a_star_search(g, start, goal, came_from, cost_so_far);
  std::vector<node_t> path = reconstruct_path(start, goal, came_from);
  return path;
}

unsigned int path_cost(const std::vector<node_t> &path)
{
  double dcost = 0;
  if (path.size() >= 2) // then we have some lines to draw
  {
    const int num_edges = path.size() - 1;
    for (int i = 0; i < num_edges; i++)
    {
      dcost = dcost + edge_info[std::pair{path[i], path[i + 1]}];
    }
  }

  return static_cast<unsigned int>(dcost);
}

void HighlightPlayerPath(std::vector<std::pair<node_t, node_t>> &player_path_edges)
{
  for (std::pair<node_t, node_t> edges : player_path_edges)
  {
    const coord_t &coord_n1 = node_info[edges.first];
    const coord_t &coord_n2 = node_info[edges.second];

    DrawLineEx(coord_n1, coord_n2, line_thickness, GOLD);
  }
};

int main()
{
  const int w{1920}, h{1080}, half_w{w / 2}, half_h{h / 2}, gap{w / 8};
  raylib::Window window{w, h, "Pathfinder"};

  SetTargetFPS(60);

  Graph g;
  add_node(g, 'A', {half_w - gap, half_h});
  add_node(g, 'B', {half_w, half_h});
  add_node(g, 'C', {half_w, half_h - gap});
  add_node(g, 'D', {half_w, half_h + gap});
  add_node(g, 'E', {half_w + gap, half_h + gap});
  add_node(g, 'F', {half_w + gap, half_h});
  add_node(g, 'G', {half_w + (2 * gap), half_h - gap});
  add_double_edge(g, 'A', 'B');
  add_double_edge(g, 'B', 'C');
  add_double_edge(g, 'B', 'D');
  add_double_edge(g, 'C', 'A');
  add_double_edge(g, 'D', 'E');
  add_double_edge(g, 'D', 'A');
  add_double_edge(g, 'E', 'B');
  add_double_edge(g, 'B', 'F');
  add_double_edge(g, 'C', 'F');
  add_double_edge(g, 'C', 'G');
  add_double_edge(g, 'F', 'G');

  int t{60}; // time
  std::vector<node_t> player_path{};

  // Variable player_path_edges, this is done so we can then highlight the player path
  std::vector<std::pair<node_t, node_t>> player_path_edges{};

  node_t start = 'A';
  node_t end = 'G';
  int tokens{2000}, score{}, high_score{}; // try with more/less tokens?

  while (!window.ShouldClose()) // Detect window close button or ESC key
  {
    BeginDrawing();

    // Draw Text onto the screen
    int text_x = 15;
    int text_y = 15;
    int font_size = 30;

    DrawText(TextFormat("Score: %i", score), text_x, text_y, font_size, BLACK);
    text_x += MeasureText(TextFormat("Score: %i", score), font_size) + font_size;

    DrawText(TextFormat("Tokens: %i", tokens), text_x, text_y, font_size, BLACK);
    text_x += MeasureText(TextFormat("Tokens: %i", tokens), font_size) + font_size;

    DrawText(TextFormat("High Score: %i", high_score), text_x, text_y, font_size, GOLD);

    // Highlight Start Node
    const coord_t &coord_startn = node_info[start];
    DrawCircleV(coord_startn, node_radius + 5, GREEN);

    // Highlight End Node
    const coord_t &coord_endn = node_info[end];
    DrawCircleV(coord_endn, node_radius + 5, RED);

    ClearBackground(LIGHTGRAY);

    draw_graph(g);

    // Highlight Player Path
    HighlightPlayerPath(player_path_edges);

    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON))
    {
      if (auto opt = get_nearby_node(GetMousePosition()))
      {
        // Perform a check to see if first node is infact the start node
        std::cout << node_info[*opt] << std::endl;

        if (player_path.empty())
        {
          if (*opt == start)
          {
            player_path.push_back(*opt);
          }
        }
        else
        {
          /*
          Check the neighbours of the last added node, and ensure the new node is one of those,
          this way preventing the player from building a path of unconnected nodes
          */
          std::vector<node_t> neighbours = g.neighbors(player_path.back());
          int check = std::count(neighbours.begin(), neighbours.end(), *opt);
          if (check > 0)
          {
            if (tokens >= edge_info[{player_path.back(), *opt}])
            {
              player_path_edges.push_back({player_path.back(), *opt});
              tokens -= edge_info[{player_path.back(), *opt}];
              player_path.push_back(*opt);
            }
          }
        }
      }
    }

    EndDrawing();
  }

  return 0;
}
