#include "GraphManagerVector.h"
#include <fstream>
// Note: 1-indexed.
long long GraphManagerVector::coord_to_index(Coord3d &coord)
{
    return (long long)(max_x) * (long long)max_y * (long long)coord.t + 
        (long long)(coord.x - 1) + (long long)(max_x * (coord.y - 1));
}
int GraphManagerVector::coord_to_index(Coord2d &coord)
{
    return (coord.x - 1) + (max_x * (coord.y - 1));
}
void GraphManagerVector::index_to_coord(long long index, Coord3d &coord)
{
    coord.t = index / (max_x * max_y);
    index %= (max_x * max_y);
    coord.y = index / max_x + 1;
    coord.x = index % max_x + 1;
}
void GraphManagerVector::index_to_coord(int index, Coord2d &coord)
{
    coord.y = index / max_x + 1;
    coord.x = index % max_x + 1;
}


bool GraphManagerVector::in_bounds(Coord2d &coord)
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

void GraphManagerVector::init_actions(int* dX, int* dY, int num_dirs)
{
    for (int i=0; i<num_dirs; i++)
        actions.push_back(Action(dX[i], dY[i]));
}

void GraphManagerVector::get_successors(Coord2d& robot_pose, std::vector<int>& successors)
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

void GraphManagerVector::populate_l_goals(int* target_traj, Coord2d& robot_pose)
{
    // std::ofstream outFile("goals_dijk.txt");
    // std::ofstream outFile2("goals_dijk_index.txt");
    int buffer = target_steps/5;
    while (l_goals.empty())
    {
        for (int i_timestep=0; i_timestep<target_steps; i_timestep++)
        {
                Coord3d goal_traj_3d(target_traj[i_timestep], target_traj[i_timestep + target_steps], i_timestep);
                Coord2d goal_traj_2d(goal_traj_3d.x, goal_traj_3d.y);
                if (chebyshev_distance(robot_pose, goal_traj_2d) + buffer > i_timestep)
                    continue;
                // outFile << "(" << goal_traj_3d.x << "," << goal_traj_3d.y << "," << goal_traj_3d.t << ")" << std::endl;
                // outFile2 << coord_to_index(goal_traj_2d) << std::endl;
                
                l_goals.insert(coord_to_index(goal_traj_2d));
                // l_goals_3d.push_back(coord_to_index(goal_traj_3d));
        }
        buffer /= 2;
    }
}

int GraphManagerVector::chebyshev_distance(Coord2d& p1, Coord2d& p2)
{
    return std::max(std::abs(p2.x - p1.x), std::abs(p2.y - p1.y));
}

int GraphManagerVector::get_c(Coord3d& robot_pose)
{
    Coord2d coord_2d(robot_pose.x, robot_pose.y);
    return map[coord_to_index(coord_2d)];
}

int GraphManagerVector::get_c(Coord2d& robot_pose)
{
    return map[coord_to_index(robot_pose)];
}


GraphManagerVector::GraphManagerVector(int max_x, int max_y, int target_steps, int* map, int collision_thresh, int num_dirs) : 
    max_x(max_x),
    max_y(max_y),
    target_steps(target_steps),
    map(map),
    collision_thresh(collision_thresh),
    num_dirs(num_dirs),
    closed_queue(max_x * max_y, false)
{}