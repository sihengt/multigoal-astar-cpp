#ifndef GRAPH_MANAGER_HPP
#define GRAPH_MANAGER_HPP
#include <unordered_set>
#include <iostream>
#include <cmath>
#include <limits>

struct Coord3d
{
    int x;
    int y;
    int t;
    Coord3d(int x, int y, int t) : x(x), y(y), t(t)
    {}
};

struct Coord2d
{
    int x;
    int y;
    Coord2d(int x, int y) : x(x), y(y)
    {}
};

struct Node
{
    int index;
    std::pair<int, int> action_to_get_here;
    Node(int index, std::pair<int, int> action) : index(index), action_to_get_here(action) {}
};

class GraphManager
{
    public:
        // Helper functions for converting between a flattened index and either a 2D or 3D grid.
        int coord_to_index_3d(Coord3d &coord);
        Coord3d index_to_coord_3d(int index);
        int coord_to_index_2d(Coord2d &coord);
        Coord2d index_to_coord_2d(int index);

        void add_to_closed(int index) { closed_queue.insert(index); };
        void reset_closed(){ closed_queue.clear(); };

        std::vector<int> get_successors(Coord3d& robot_pose);
        std::vector<Node> get_successors(Coord2d& robot_pose);
        void init_actions(int* dX, int* dY, int num_dirs);

        void populate_l_goals(int* target_traj);

        double compute_heuristic(Coord3d &coord);
        double compute_heuristic(Coord2d &coord);
        int get_c(Coord3d robot_pose);
        int get_c(Coord2d robot_pose);

        bool in_bounds(Coord2d coord);

        GraphManager(int max_x, int max_y, int target_steps, int* map, int collision_thresh, int num_dirs);

        std::vector<int> l_goals;

    private:
        std::unordered_set<int> closed_queue;
        const int min_x = 0;
        const int min_y = 0;
        const int max_x; // max x dimensions of the map
        const int max_y; // max y dimensions of the map
        const int target_steps; // max t dimensions
        const int num_dirs;   // 
        int* map;
        int collision_thresh;
        std::vector<std::pair<int, int>> actions; // assumes 2D
};

#endif // GRAPH_MANAGER_HPP