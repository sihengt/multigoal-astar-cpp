#ifndef GRAPH_MANAGER_HPP
#define GRAPH_MANAGER_HPP
#include <vector>
#include <iostream>
#include <cmath>
#include <limits>
#include <unordered_set>

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

struct Action
{
    int action_x;
    int action_y;
    Action(int action_x, int action_y) : action_x(action_x), action_y(action_y) {}
    Action(std::pair<int, int>& action) : action_x(action.first), action_y(action.second) {}
    Action() : action_x(0), action_y(0) {}
};

struct Node
{
    int index;
    Action action_to_get_here;
    Node(int index, const Action& action) : index(index), action_to_get_here(action) {}
};


class GraphManager
{
    public:
        // Helper functions for converting between a flattened index and either a 2D or 3D grid.
        long long coord_to_index(Coord3d &coord);
        int coord_to_index(Coord2d &coord);
        void index_to_coord(long long index, Coord3d &coord);
        void index_to_coord(int index, Coord2d &coord);

        void add_to_closed(long long index) { closed_queue.insert(index); };
        // void add_to_closed(int index) {closed_queue[index] = true; };

        void get_successors(Coord3d& robot_pose, std::vector<long long>& successors);
        void get_successors(Coord2d& robot_pose, std::vector<int>& successors);
        void init_actions(int* dX, int* dY, int num_dirs);

        void populate_l_goals(int* target_traj, Coord2d& robot_pose);
        void populate_l_goals(int* target_traj, Coord3d& robot_pose);
        int cheybyshev_dist(Coord2d& p1, Coord2d& p2);

        double compute_heuristic(Coord3d &coord);
        double compute_heuristic(Coord2d &coord);
        int get_c(Coord3d &robot_pose);
        int get_c(Coord2d &robot_pose);

        bool in_bounds(Coord2d &coord);

        GraphManager(int max_x, int max_y, int target_steps, int* map, int collision_thresh, int num_dirs);

        std::vector<long long> l_goals;
        const int goal_index;
        std::unordered_set<long long> closed_queue;
        const long long max_x; // max x dimensions of the map
        const long long max_y; // max y dimensions of the map
        const long long target_steps; // max t dimensions
    
    private:
        
        std::vector<Action> actions;
        const long long min_x = 0;
        const long long min_y = 0;

        
        const int num_dirs;   // 
        int* map;
        int collision_thresh;
};

#endif // GRAPH_MANAGER_HPP