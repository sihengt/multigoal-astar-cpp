#include "GraphManager.h"

int GraphManager::coord_to_index_3d(Coord3d &coord)
{
    int x = coord.x;
    int y = coord.y;
    int t = coord.t;
    return (max_x * max_y * t) + x + (max_x * y);
}

Coord3d GraphManager::index_to_coord_3d(int index)
{
    int t = index / (max_x * max_y);
    index %= (max_x * max_y);
    int y = index / max_x;
    int x = index % max_x;
    return Coord3d(x, y, t);
}

int GraphManager::coord_to_index_2d(Coord2d &coord)
{
    int x = coord.x;
    int y = coord.y;
    return  x + (max_x * y);
}

Coord2d GraphManager::index_to_coord_2d(int index)
{
    int y = index / max_x;
    int x = index % max_x;
    return Coord2d(x, y);
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

std::vector<int> GraphManager::get_successors(Coord2d& robot_pose)
{
    std::vector<int> successors;
    for (int dir = 0; dir < num_dirs; dir++)
    {
        int new_x = robot_pose.x + actions[dir].first;
        int new_y = robot_pose.y + actions[dir].second;
        Coord2d new_robot_pose(new_x, new_y);
        
        int map_index = coord_to_index_2d(new_robot_pose); 
        int cost = map[map_index];
        if (cost >= 0 && cost < collision_thresh)
            successors.push_back(map_index);
    }
    return successors;
}

GraphManager::GraphManager(int max_x, int max_y, int* map, int collision_thresh, int num_dirs) : 
    max_x(max_x),
    max_y(max_y),
    map(map),
    collision_thresh(collision_thresh),
    num_dirs(num_dirs)
{}