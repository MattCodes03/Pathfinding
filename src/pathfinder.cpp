#include "vec.hpp"
#include "draw-triangle-pro.hpp"
#include "raylib-cpp.hpp"
#include "graph.hpp"
#include "graph-utils.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <cstdlib>
#include <algorithm>

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

		DrawLineEx(coord_n1, coord_n2, line_thickness + 10, GOLD);
	}
};

int main()
{
	const int w{1920}, h{1080}, half_w{w / 2}, half_h{h / 2}, gap{w / 8};
	raylib::Window window{w, h, "Pathfinder"};

	SetTargetFPS(60);

	const node_t graphNodes[7] = {'A', 'B', 'C', 'D', 'E', 'F', 'G'};

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
	int currentTime = t;
	float timeAccum = 0.f;

	std::vector<node_t> player_path{};

	// Variable player_path_edges, this is done so we can then highlight the player path
	std::vector<edge_t> player_path_edges{};

	node_t start = 'A';
	node_t end = 'G';
	int tokens{2000}, score{}, high_score{}; // try with more/less tokens?

	// Load Sound file and Init Raylib Audio
	InitAudioDevice();
	Sound node_clicked = LoadSound("../deps/raylib-cpp-5.0.1/examples/audio/resources/coin.wav");

	while (!window.ShouldClose()) // Detect window close button or ESC key
	{
		timeAccum += GetFrameTime();
		if (timeAccum >= 1.0f)
		{
			timeAccum = 0.f;
			currentTime--;
		}

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
		text_x += MeasureText(TextFormat("High Score: %i", high_score), font_size) + font_size;

		DrawText(TextFormat("Time: %i", currentTime), text_x, text_y, font_size, RED);

		// Highlight Player Path
		HighlightPlayerPath(player_path_edges);

		ClearBackground(LIGHTGRAY);

		draw_graph(g);

		// Highlight Start Node
		const coord_t &coord_startn = node_info[start];
		DrawCircleV(coord_startn, node_radius, GREEN);

		// Highlight End Node
		const coord_t &coord_endn = node_info[end];
		DrawCircleV(coord_endn, node_radius, RED);

		if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON))
		{
			if (auto opt = get_nearby_node(GetMousePosition()))
			{
				// Perform a check to see if first node is infact the start node
				if (player_path.empty())
				{
					if (*opt == start)
					{
						player_path.push_back(*opt);
						PlaySound(node_clicked);
					}
				}
				else
				{
					/*
					Check if node already in path, if it is find its postion in the vector and remove it
					and all connected nodes as the path would now be broken, if the node to be removed was for example the 2nd node clicked.
					*/
					if (std::count(player_path.begin(), player_path.end(), *opt) > 0)
					{
						std::vector<node_t> removed_nodes;
						auto removed_node = std::find(player_path.begin(), player_path.end(), *opt);
						if (removed_node != player_path.end())
						{
							removed_nodes.assign(removed_node, player_path.end());
							player_path.erase(removed_node, player_path.end());
						}

						for (node_t &node : removed_nodes)
						{
							for (const edge_t &edge : player_path_edges)
							{

								if (edge.first == node || edge.second == node)
								{
									// Remove all edges
									player_path_edges.erase(std::remove(player_path_edges.begin(), player_path_edges.end(), edge), player_path_edges.end());

									// Reimburse token cost
									tokens += edge_info[edge];
								}
							};
						}
						removed_nodes.empty();
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
								PlaySound(node_clicked);
							}
						}
					}
				}
			}
		}

		EndDrawing();

		// Check if the player has completed their path before increasing the score and setting the time to 0 to triger game restart
		if (!player_path.empty() && player_path.back() == end)
		{
			if (tokens > 0)
			{
				// Update Player Score
				score += path_cost(astar_pathfind(g, start, end));
				// Check if Highscore needs updating
				if (score > high_score)
				{
					high_score = score;
				}
			}

			currentTime = 0;
		}

		if (currentTime <= 0)
		{
			tokens += score;
			currentTime = t;
			player_path.clear();
			player_path_edges.clear();
			start = graphNodes[rand() % 7];
			end = graphNodes[rand() % 7];
			while (start == end)
			{
				end = graphNodes[rand() % 7];
			}
		}
	}

	return 0;
}
