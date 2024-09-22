#include "GraphManagerVector.h"

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

void GraphManagerVector::get_successors(Coord3d& robot_pose, std::vector<int>& successors)
{
    std::fill(successors.begin(), successors.end(), -1);
    Coord2d new_robot_pose_2d(0, 0);
    Coord3d new_robot_pose(0, 0, 0);
    int map_index_2d, map_index_3d, cost;
    for (int dir = 0; dir < num_dirs; dir++)
    {
        new_robot_pose_2d.x = robot_pose.x + actions[dir].action_x;
        new_robot_pose_2d.y = robot_pose.y + actions[dir].action_y;
        
        // In 3D
        new_robot_pose.x = new_robot_pose_2d.x;
        new_robot_pose.y = new_robot_pose_2d.y;
        new_robot_pose.t = robot_pose.t + 1;
        
        // Check if new pose is in goal state


        // Check if the action would take the robot out of bounds.
        if (!in_bounds(new_robot_pose_2d))
            continue;

        map_index_2d = coord_to_index(new_robot_pose_2d);
        map_index_3d = coord_to_index(new_robot_pose);

        // if (closed_queue.find(map_index_2d) != closed_queue.end())
        //     continue;
        if (closed_queue[map_index_3d])
            continue;

        cost = map[map_index_2d];
        
        // Checking if cell's cost exceeds collision threshold
        if (cost >= 0 && cost < collision_thresh)
            successors[dir] = map_index_2d;
    }
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

double GraphManagerVector::compute_heuristic(Coord3d& coord)
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

double GraphManagerVector::compute_heuristic(Coord2d& coord)
{

    Coord2d goal_coords(0,0);
    index_to_coord(l_goals[0], goal_coords);
    double heuristic = std::sqrt(std::pow(coord.x - goal_coords.x, 2) + std::pow(coord.y - goal_coords.y, 2));
    return heuristic;
}

void GraphManagerVector::populate_l_goals(int* target_traj, Coord2d& robot_pose)
{
    for (int i_timestep=0; i_timestep<target_steps; i_timestep++)
    {
        Coord3d goal_traj_3d(target_traj[i_timestep], target_traj[i_timestep + target_steps], i_timestep);
        Coord2d goal_traj_2d(goal_traj_3d.x, goal_traj_3d.y);
        if (chebyshev_distance(robot_pose.x, goal_traj_2d.x, robot_pose.y, goal_traj_2d.y) > i_timestep)
            continue;
        l_goals.push_back(coord_to_index(goal_traj_2d));
        l_goals_3d.push_back(coord_to_index(goal_traj_3d));
    }
}

int GraphManagerVector::chebyshev_distance(int p1_x, int p2_x, int p1_y, int p2_y)
{
    return std::max(std::abs(p2_x - p1_x), std::abs(p2_y - p1_y));
}

void GraphManagerVector::populate_lookup_tables(
    int state_2d_index,
    std::vector<int> &time_of_closest_goal_to_state,
    std::vector<int> &distance_of_closest_goal_to_state,
    std::vector<long long> &closest_goal_to_state
)
{
    int smallest_distance = std::numeric_limits<int>::max();
    
    // Irrelevant because we should be overwriting this.
    long long smallest_index = 0;

    Coord2d state_pose_2d(0, 0);
    index_to_coord(state_2d_index, state_pose_2d);
    Coord3d goal_pose_3d(0, 0, 0);
    
    for (long long goal_index_3d : l_goals_3d)
    {
        index_to_coord(goal_index_3d, goal_pose_3d);
        int current_distance = chebyshev_distance(state_pose_2d.x, goal_pose_3d.x, state_pose_2d.y, goal_pose_3d.y);
        if (current_distance < smallest_distance)
        {
            smallest_index = goal_index_3d;
            smallest_distance = current_distance;
        }
    }

    index_to_coord(smallest_index, goal_pose_3d);
    time_of_closest_goal_to_state[state_2d_index] = goal_pose_3d.t;
    distance_of_closest_goal_to_state[state_2d_index] = smallest_distance;
    closest_goal_to_state[state_2d_index] = smallest_index;
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