#ifndef GOAL_TRAJECTORY_MANAGER_H
#define GOAL_TRAJECTORY_MANAGER_H

#include <vector>

class GoalTrajectoryManager
{
    public:
        void populate_l_goals(int* target_traj);
        const std::vector<int>& get_l_goals() const {return l_goals;}
        GoalTrajectoryManager(int target_steps, int max_x, int max_y);

    private:
        std::vector<int> l_goals;
        int target_steps, max_x, max_y;
};

#endif // GOAL_TRAJECTORY_MANAGER_HPP