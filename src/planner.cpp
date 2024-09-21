/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "../include/planner.h"
#include <math.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <iostream>
#include <chrono>
#include <limits>

#include "GraphManagerVector.h"
#include "LookupPriorityQueueVector.h"

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

// As a baseline implement multi goal A*
// Every single (x, y, t) is considered a goal.

// Questions
// 1. How do we construct the graph?

void multi_backwards_djikstra(
    LookupPriorityQueueVector &Q,
    GraphManagerVector &gm,
    int start,
    std::vector<int> &heuristic
)
{
    std::vector<int> successors(8, -1);
    
    // Insert goal as start state, with 0 for cost-to-go and 0 for heuristic (cause we djikstra-ing)
    for (int goal : gm.l_goals)
        Q.insert(goal, 0, 0);

    Coord2d current_coord(0, 0);
    Coord2d successor_coord(0, 0);
    int current_state_index, current_g, current_cost;
    while (!Q.empty())
    {
        const State current_state = Q.top();
        current_state_index = current_state.index;
        current_g = current_state.cost_to_go;
        heuristic[current_state_index] = current_g;
        
        // If current_state is the start_state, we've found a solution.
        if (current_state.index == start)
            break;
        Q.pop();

        // Expanded node is optimal and won't be visited again.
        gm.add_to_closed(current_state_index);

        // Getting successors.
        gm.index_to_coord(current_state_index, current_coord);
        gm.get_successors(current_coord, successors);
        current_cost = gm.get_c(current_coord);
        
        int successor_cost_to_goal;
        for (int successor_index : successors)
        {   
            if (successor_index == -1)
                continue;
            
            // For the backward Djikstra, we've established that the cost-to-go is whatever cost + current node's cost.
            successor_cost_to_goal = current_g + current_cost;
            
            // Successor is not in OPEN, so we insert it into the queue.
            if (Q.lookup_table[successor_index].node_ == nullptr)
                Q.insert(successor_index, successor_cost_to_goal, 0);
                
            // Successor is in OPEN, we need to check if it's current cost to go is > previous state's cost to go + cost
            else
            {
                auto succ_state_ptr = Q.lookup_table[successor_index];
                if ((*succ_state_ptr).cost_to_go > successor_cost_to_goal)
                {
                    Q.check_cost_and_update(successor_index, successor_cost_to_goal);
                }
            }
                
        }
    }
}

void multi_goal_astar(
    LookupPriorityQueue &Q,
    GraphManager &gm,
    int start,
    std::vector<int> &optimal_action_to_state,
    std::vector<int> &heuristic
)
{
    std::vector<int> successors(9, -1);

    // Insert goal as start state, with 0 for cost-to-go and 0 for heuristic
    for (int goal : gm.l_goals)
        Q.insert(goal, 0, 0);

    Coord3d current_coord(0, 0, 0);
    Coord3d successor_coord(0, 0, 0);
    Coord2d current_coord_2d(0, 0);
    Coord2d successor_coord_2d(0, 0);

    int current_state_index, current_state_index_2d, successor_index_2d, current_g, current_cost;
    while (!Q.empty())
    {
        const State current_state = Q.top();
        current_state_index = current_state.index;
        current_g = current_state.cost_to_go;
        
        // If current_state is the goal_state, we've found a solution.
        // TODO: how shall we represent our goal state?
        // TODO 2: if current_state = goal_state, break
        if (current_state.index == -1)
            break;

        Q.pop();

        // Expanded node is optimal and won't be visited again.
        gm.add_to_closed(current_state_index);

        // Getting successors.
        gm.index_to_coord(current_state_index, current_coord);
        
        current_coord_2d.x = current_coord.x;
        current_coord_2d.y = current_coord.y;
        current_state_index_2d = gm.coord_to_index(current_coord_2d);
        
        // TODO: test get successors
        gm.get_successors(current_coord, successors);

        int successor_cost_to_go;
        // for (int successor_index : successors)
        for (int action_index = 0; action_index < 9; action_index++)
        {   
            int successor_index = successors[action_index];
            if (successor_index == -1)
                continue;
            
            // For a forward
            gm.index_to_coord(successor_index, successor_coord);
            successor_coord_2d.x = successor_coord.x;
            successor_coord_2d.y = successor_coord.y;
            successor_index_2d = gm.coord_to_index(successor_coord_2d);
            successor_cost_to_go = current_g + gm.get_c(successor_coord);
            
            // Successor is not in OPEN, so we insert it into the queue.
            if (Q.lookup_table[successor_index].node_ == nullptr)
            {
                Q.insert(successor_index, successor_cost_to_go, heuristic[successor_index_2d]);
                optimal_action_to_state[successor_index] = action_index;
            }
                
                
            // Successor is in OPEN, we need to check if it's current cost to go is > previous state's cost to go + cost
            else
            {
                auto succ_state_ptr = Q.lookup_table[successor_index];
                if ((*succ_state_ptr).cost_to_go > successor_cost_to_go)
                {
                    Q.check_cost_and_update(successor_index, successor_cost_to_go);
                }
            }
                
        }
    }

}

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
    )
{
    static bool is_cached = false;
    static std::vector<int> heuristic;
    
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    // 9-connected grid (includes waiting)
    int dX_full[NUMOFDIRS + 1] = {-1, -1, -1,  0,  0,  1, 1, 1, 0};
    int dY_full[NUMOFDIRS + 1] = {-1,  0,  1, -1,  1, -1, 0, 1, 0};

    if (!is_cached)
    {
        Coord2d robot_pose_2d(robotposeX, robotposeY);
        LookupPriorityQueueVector Q_dijkstra(x_size, y_size);
        GraphManagerVector gm_dijkstra(x_size, y_size, target_steps, map, collision_thresh, NUMOFDIRS);

        heuristic.resize(x_size * y_size);
        std::fill(heuristic.begin(), heuristic.end(), std::numeric_limits<int>::max());
        auto start_t = std::chrono::high_resolution_clock::now();
        gm_dijkstra.init_actions(dX, dY, NUMOFDIRS);
        gm_dijkstra.populate_l_goals(target_traj, robot_pose_2d);
        multi_backwards_djikstra(
            Q_dijkstra,
            gm_dijkstra,
            gm_dijkstra.coord_to_index(robot_pose_2d),
            heuristic);
        auto stop_t = std::chrono::high_resolution_clock::now();
        auto duration_t = std::chrono::duration_cast<std::chrono::microseconds>(stop_t - start_t);
        std::cout << "Time taken by Djikstra: " << duration_t.count() << std::endl;
    }

    // Initialization of OPEN (implemented as fibonacci heap priority queue)
    Coord3d robot_pose_3d(robotposeX, robotposeY, curr_time);
    LookupPriorityQueue Q(x_size, y_size, target_steps);
    GraphManager gm(x_size, y_size, target_steps, map, collision_thresh, NUMOFDIRS);
    
    // These actions are the actions that brought you to this state, NOT the actions to take at the state.
    // You can backtrack from the goal state all the way back to the start state to get the best action to take,
    // kind of like MPC.
    std::vector<int> optimal_action_to_state(x_size * y_size * target_steps, -1);
    gm.init_actions(dX_full, dY_full, NUMOFDIRS + 1);
    gm.populate_l_goals(target_traj, robot_pose_3d);
    multi_goal_astar(
        Q,
        gm,
        gm.coord_to_index(robot_pose_3d),
        optimal_action_to_state,
        heuristic
    );

    // int start_index = graph_manager.coord_to_index_2d(robot_pose_2d);
    // int current_index = goalIndex;
    // Action prev_action;
    // while (true)
    // {
    //     prev_action = policy[current_index];
    //     Coord2d current_state = graph_manager.index_to_coord_2d(current_index);
    //     Coord2d previous_state(current_state.x - prev_action.action_x, current_state.y - prev_action.action_y);
    //     current_index = graph_manager.coord_to_index_2d(previous_state);
    //     if (current_index == start_index)
    //     {
    //         break;
    //     }
    // }

    // action_ptr[0] = robotposeX + prev_action.action_x;
    // action_ptr[1] = robotposeY + prev_action.action_y;
    return;
}