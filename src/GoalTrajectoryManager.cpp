#include "GoalTrajectoryManager.h"
#include <iostream>


GoalTrajectoryManager::GoalTrajectoryManager(int target_steps, int max_x, int max_y) : 
    target_steps(target_steps),
    max_x(max_x),
    max_y(max_y)
{}

void GoalTrajectoryManager::populate_l_goals(int* target_traj)
{
    // target_traj is the flattened array of x,y. We add time into it here.
    for (int i_timestep=0; i_timestep<target_steps; i_timestep++)
        l_goals.push_back(target_traj[i_timestep] + i_timestep * (max_x * max_y));
}