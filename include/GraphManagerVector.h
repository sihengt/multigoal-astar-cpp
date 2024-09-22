#ifndef GRAPH_MANAGER_VECTOR_HPP
#define GRAPH_MANAGER_VECTOR_HPP
#include <vector>
#include <iostream>
#include <cmath>
#include <limits>
#include "GraphManager.h"

class GraphManagerVector
{
    public:
        // Helper functions for converting between a flattened index and either a 2D or 3D grid.
        long long coord_to_index(Coord3d &coord);
        int coord_to_index(Coord2d &coord);
        void index_to_coord(long long index, Coord3d &coord);
        void index_to_coord(int index, Coord2d &coord);

        // void add_to_closed(int index) { closed_queue.insert(index); };
        void add_to_closed(int index) {closed_queue[index] = true; };

        void get_successors(Coord3d& robot_pose, std::vector<int>& successors);
        void get_successors(Coord2d& robot_pose, std::vector<int>& successors);
        void init_actions(int* dX, int* dY, int num_dirs);

        int chebyshev_distance(int p1_x, int p2_x, int p1_y, int p2_y);

        void populate_l_goals(int* target_traj, Coord2d& robot_pose);
        void populate_l_goals(int* target_traj, Coord3d& robot_pose);
        void populate_lookup_tables(
            int state_2d_index,
            std::vector<int> &time_of_closest_goal_to_state, // TODO: might be useless
            std::vector<int> &distance_of_closest_goal_to_state, // TODO: might be useless
            std::vector<long long> &closest_goal_to_state);

        double compute_heuristic(Coord3d &coord);
        double compute_heuristic(Coord2d &coord);
        int get_c(Coord3d &robot_pose);
        int get_c(Coord2d &robot_pose);

        bool in_bounds(Coord2d &coord);

        GraphManagerVector(int max_x, int max_y, int target_steps, int* map, int collision_thresh, int num_dirs);

        std::vector<long long> l_goals_3d;
        std::vector<int> l_goals;
        std::vector<bool> closed_queue;
    private:
        std::vector<Action> actions;
        std::vector<int> best_heuristic;
        const int min_x = 0;
        const int min_y = 0;
        const int max_x; // max x dimensions of the map
        const int max_y; // max y dimensions of the map
        const int target_steps; // max t dimensions
        const int num_dirs;   // 
        int* map;
        int collision_thresh;
};

#endif // GRAPH_MANAGER_VECTOR_HPP