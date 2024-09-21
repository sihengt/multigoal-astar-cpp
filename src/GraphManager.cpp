#include "GraphManager.h"

// Note: 1-indexed.
int GraphManager::coord_to_index(Coord3d &coord)
{
    return (max_x * max_y * coord.t) + (coord.x - 1) + (max_x * (coord.y - 1));
}
int GraphManager::coord_to_index(Coord2d &coord)
{
    return (coord.x - 1) + (max_x * (coord.y - 1));
}
void GraphManager::index_to_coord(int index, Coord3d &coord)
{
    coord.t = index / (max_x * max_y);
    index %= (max_x * max_y);
    coord.y = index / max_x + 1;
    coord.x = index % max_x + 1;
}
void GraphManager::index_to_coord(int index, Coord2d &coord)
{
    coord.y = index / max_x + 1;
    coord.x = index % max_x + 1;
}


bool GraphManager::in_bounds(Coord2d &coord)
{
    // To account for that annoying 1-index.
    if ((coord.x - 1 >= min_x) && 
        (coord.x - 1 <= max_x) &&
        (coord.y - 1 >= min_y) &&
        (coord.y - 1 <= max_y))
    {
        return true;
    }
    return false;
}

void GraphManager::init_actions(int* dX, int* dY, int num_dirs)
{
    for (int i=0; i<num_dirs; i++)
        actions.push_back(Action(dX[i], dY[i]));
}

void GraphManager::get_successors(Coord3d& robot_pose, std::vector<int>& successors)
{
    int current_index, map_index_2d, map_index_3d;
    current_index = coord_to_index(robot_pose);
    int cnt = count(l_goals.begin(), l_goals.end(), current_index); 

    // We've reached a goal state, push back -1 to signal rest of algorithm.
    if (cnt > 0)
    {
        successors.push_back(-1);
        return;
    }
    
    for (int dir = 0; dir < num_dirs; dir++)
    {
        int new_x = robot_pose.x + actions[dir].action_x;
        int new_y = robot_pose.y + actions[dir].action_y;
        
        Coord2d new_robot_pose_2d(new_x, new_y);
        Coord3d new_robot_pose(new_x, new_y, robot_pose.t + 1);
        
        if (!in_bounds(new_robot_pose_2d))
        {
            continue;
        }

        map_index_3d = coord_to_index(new_robot_pose);
        map_index_2d = coord_to_index(new_robot_pose_2d);
        int cost = map[map_index_2d];
        
        // Checking if cell's cost exceeds collision threshold
        if (cost >= 0 && cost < collision_thresh)
        {
            // Check if successor state is in closed set
            // auto iter = closed_queue.find(map_index_3d);
            // if (iter == closed_queue.end())
            //     successors.push_back(map_index_3d);
            if (!closed_queue[map_index_3d])
                successors.push_back(map_index_3d);
        }
    }
}

void GraphManager::get_successors(Coord2d& robot_pose, std::vector<int>& successors)
{   
    std::fill(successors.begin(), successors.end(), -1);
    Coord2d new_robot_pose_2d(0, 0);
    int map_index_2d, cost;
    for (int dir = 0; dir < num_dirs; dir++)
    {
        new_robot_pose_2d.x = robot_pose.x + actions[dir].action_x;
        new_robot_pose_2d.y = robot_pose.y + actions[dir].action_y;

        // Check if the action would take the robot out of bounds.
        if (!in_bounds(new_robot_pose_2d))
            continue;

        map_index_2d = coord_to_index(new_robot_pose_2d);

        // if (closed_queue.find(map_index_2d) != closed_queue.end())
        //     continue;
        if (closed_queue[map_index_2d])
            continue;

        cost = map[map_index_2d];
        
        // Checking if cell's cost exceeds collision threshold
        if (cost >= 0 && cost < collision_thresh)
            successors[dir] = map_index_2d;
    }
}

double GraphManager::compute_heuristic(Coord3d& coord)
{
    double heuristic = std::numeric_limits<double>::infinity();

    for (int goal : l_goals)
    {
        Coord3d goal_coords(0,0,0);
        index_to_coord(goal, goal_coords);
        double curr_heuristic = std::sqrt(std::pow(coord.x - goal_coords.x, 2) + std::pow(coord.y - goal_coords.y, 2) + std::pow(coord.t - goal_coords.t, 2));
        if (curr_heuristic < heuristic)
        {
            heuristic = curr_heuristic;
        }
    }
    return heuristic;
}

int cheybyshev_dist(Coord2d& p1, Coord2d& p2)
{
    return std::max(p2.x - p1.x, p2.y - p1.x);
}

double GraphManager::compute_heuristic(Coord2d& coord)
{

    Coord2d goal_coords(0,0);
    index_to_coord(l_goals[0], goal_coords);
    double heuristic = std::sqrt(std::pow(coord.x - goal_coords.x, 2) + std::pow(coord.y - goal_coords.y, 2));
    return heuristic;
}

void GraphManager::populate_l_goals(int* target_traj, Coord2d& robot_pose)
{
    for (int i_timestep=0; i_timestep<target_steps; i_timestep++)
    {
        Coord2d goal_traj(target_traj[i_timestep], target_traj[i_timestep + target_steps]);
        if (std::max(goal_traj.x - robot_pose.x, goal_traj.y - robot_pose.y) > i_timestep)
            continue;
        l_goals.push_back(coord_to_index(goal_traj));
    }
}

int GraphManager::get_c(Coord3d& robot_pose)
{
    Coord2d coord_2d(robot_pose.x, robot_pose.y);
    return map[coord_to_index(coord_2d)];
}

int GraphManager::get_c(Coord2d& robot_pose)
{
    return map[coord_to_index(robot_pose)];
}


GraphManager::GraphManager(int max_x, int max_y, int target_steps, int* map, int collision_thresh, int num_dirs) : 
    max_x(max_x),
    max_y(max_y),
    target_steps(target_steps),
    map(map),
    collision_thresh(collision_thresh),
    num_dirs(num_dirs),
    closed_queue(max_x * max_y, false)
{}