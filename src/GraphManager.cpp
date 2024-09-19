#include "GraphManager.h"

// Note: 1-indexed.
int GraphManager::coord_to_index_3d(Coord3d &coord)
{
    int x = coord.x;
    int y = coord.y;
    int t = coord.t;
    return (max_x * max_y * t) + (x - 1) + (max_x * (y - 1));
}

Coord3d GraphManager::index_to_coord_3d(int index)
{
    int t = index / (max_x * max_y);
    index %= (max_x * max_y);
    int y = index / max_x;
    int x = index % max_x;
    return Coord3d(x + 1, y + 1, t);
}

int GraphManager::coord_to_index_2d(Coord2d &coord)
{
    int x = coord.x;
    int y = coord.y;
    return  (x - 1) + (max_x * (y - 1));
}

Coord2d GraphManager::index_to_coord_2d(int index)
{
    int y = index / max_x;
    int x = index % max_x;
    return Coord2d(x + 1, y + 1);
}

void GraphManager::add_to_closed(int index)
{
    closed_queue.insert(index);
}

void GraphManager::init_actions(int* dX, int* dY, int num_dirs)
{
    for (int i=0; i<num_dirs; i++)
        actions.push_back(std::pair<int, int>(dX[i], dY[i]));
}

std::vector<int> GraphManager::get_successors(Coord3d& robot_pose)
{
    std::vector<int> successors;
    int cnt = count(l_goals.begin(), l_goals.end(), coord_to_index_3d(robot_pose)); 

    // We've reached a goal state, push back -1 to signal rest of algorithm.
    if (cnt > 0)
    {
        successors.push_back(-1);
        return successors;
    }
    for (int dir = 0; dir < num_dirs; dir++)
    {
        int new_x = robot_pose.x + actions[dir].first;
        int new_y = robot_pose.y + actions[dir].second;
        Coord3d new_robot_pose(new_x, new_y, robot_pose.t + 1);
        Coord2d new_robot_pose_2d(new_x, new_y);
        int map_index_3d = coord_to_index_3d(new_robot_pose);
        int map_index_2d = coord_to_index_2d(new_robot_pose_2d);
        int cost = map[map_index_2d];
        
        // Checking if cell's cost exceeds collision threshold
        if (cost >= 0 && cost < collision_thresh)
        {
            // Check if successor state is in closed set
            auto iter = closed_queue.find(map_index_3d);
            if (iter == closed_queue.end())
                successors.push_back(map_index_3d);
        }
    }
    return successors;
}

double GraphManager::compute_heuristic(Coord3d& coord)
{
    double heuristic = std::numeric_limits<double>::infinity();

    for (auto goal : l_goals)
    {
        Coord3d goal_coords = index_to_coord_3d(goal);
        double curr_heuristic = std::sqrt(std::pow(coord.x - goal_coords.x, 2) + std::pow(coord.y - goal_coords.y, 2) + std::pow(coord.t - goal_coords.t, 2));
        if (curr_heuristic < heuristic)
        {
            heuristic = curr_heuristic;
        }
    }
    return heuristic;
}

void GraphManager::populate_l_goals(int* target_traj)
{
    for (int i_timestep=0; i_timestep<target_steps; i_timestep++)
        l_goals.push_back(target_traj[i_timestep] + i_timestep * (max_x * max_y));
}

int GraphManager::get_c(Coord3d robot_pose)
{
    Coord2d coord_2d(robot_pose.x, robot_pose.y);
    return map[coord_to_index_2d(coord_2d)];
}

GraphManager::GraphManager(int max_x, int max_y, int target_steps, int* map, int collision_thresh, int num_dirs) : 
    max_x(max_x),
    max_y(max_y),
    target_steps(target_steps),
    map(map),
    collision_thresh(collision_thresh),
    num_dirs(num_dirs)
{}