#ifndef PLANNER_H
#define PLANNER_H

#include "GraphManager.h"
#include "GraphManagerVector.h"
#include "PriorityQueue.h"
struct State;
struct StateComparator;

// Declare the plan function
void planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr
    );

#endif // PLANNER_H